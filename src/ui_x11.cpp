// ui_x11.cpp — Valve Howler's X11 LV2 UI (the «NLS Cream» pedal).
//
// This is the window-and-events layer. All painting lives in `ui_draw.h`;
// here lives:
//   - creating a child window inside the one the host provides,
//   - wrapping it in a `cairo_xlib_surface` and loading the bundle fonts,
//   - pumping X events in the `idle` callback (expose / mouse),
//   - turning a knob drag into port writes,
//   - toggling the footswitch (`lv2:enabled`) and opening the variant
//     dropdown,
//   - reflecting on the panel what the host changes on its own
//     (`port_event`).
//
// No Pugl and no SVG renderer: just Xlib + cairo-xlib + FreeType, already
// on the system. The same rig as `TheOmegaCake/src/ui_x11.cpp`, which in
// turn grew out of the 0C family's Pugl scaffolding.
// X11 on purpose (under Wayland it runs via XWayland).
//
// ─────────────────────────────────────────────────────────────────────────────
// THE THREE THINGS THIS FILE MUST GET RIGHT, AND WHY
// ─────────────────────────────────────────────────────────────────────────────
//
// 1. DO NOT REPAINT THE WHOLE PANEL ON EVERY EVENT. That froze Ardour
//    with the 0C-family GUIs: ~100 % of a core per open window. Here the
//    static layer draws ONCE to a surface and gets stamped, with a 30 fps
//    cap. Measured on this machine with `make gui-bench`:
//        full frame without cache ....... 8,744 ms  (26,2 % of a core at 30 fps)
//        live layer with cache .......... 0,069 ms  ( 0,21 %)            => ×126
//
// 2. THE MARGIN IS SUBTRACTED FROM MOUSE COORDINATES. The pedal floats
//    with `UI_MARGIN` of backdrop around it, so the panel origin is NOT
//    the window's. Forget it and the knobs draw in one place and grab in
//    another — with no error raised.
//
// 3. `ui:touch`, THE GAP `DISENO_DE_INTERFACES.md` §6 marks as "the one
//    that hurts, and it is PRODUCT". No workshop GUI implemented it
//    (0 uses in the two audited repos). Without it the host cannot know a
//    gesture began => touch-mode automation and MIDI-learn misbehave. Here
//    the host is told on grab and on release.
//    With the caveat the document itself leaves: the absence is a
//    measured FACT, but that it breaks something concrete in Ardour is an
//    INFERENCE nobody reproduced. Implemented because the spec says so
//    and it costs ten lines, not because the symptom was seen.

#include "ui_draw.h"
#include "nls_variantes.h"
// The port indices, from the ONE place that defines them (see the note at
// the `using` declarations below).
#include "nls_iface.h"

#include <lv2/core/lv2.h>
#include <lv2/ui/ui.h>
#include <lv2/atom/atom.h>
#include <lv2/options/options.h>
#include <lv2/urid/urid.h>

#include <cairo/cairo.h>
#include <cairo/cairo-xlib.h>
#include <X11/Xlib.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <string>

#define NLSC_UI_URI "https://nylarea.com/plugins/valvehowler#ui"

namespace {

// THE PORT INDICES COME FROM `nls_iface.h`, NOT FROM A COPY HERE.
//
// This block used to be its own `enum` with the numbers written out, under a
// comment saying they "must match". A comment cannot make them match: the two
// lists are only equal until someone edits one, and the failure is SILENT —
// the UI keeps writing to an index that now means another port
// -- a rule written twice diverges.
//
// It was about to bite. Removing `oversampling` (index 5), `engine` and
// `seed` renumbers everything behind them: `latency` 6->5,
// `enabled` 7->6, `variant` 8->7. With the old copy still saying
// `PORT_ENABLED = 7`, the footswitch would have written to `variant` — pressing
// bypass would change circuit, with no error anywhere.
//
// The `#include` itself lives with the other includes at the top: pulled in
// HERE it would land inside this anonymous namespace and drag `nlsc::` in with
// it, which is how `nlsc::ui` started shadowing the UI's own namespace.
using nlsc::PORT_DRIVE;
using nlsc::PORT_TONE;
using nlsc::PORT_LEVEL;
using nlsc::PORT_ENABLED;
using nlsc::PORT_VARIANT;

inline double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// A draggable knob: hit centre and radius (panel units) plus its port.
// The geometry is NOT copied here: it comes from `ui_draw.h`, which is
// what draws. House rule — "hit-boxes ALWAYS from the drawing module, so
// drawing and hit-testing CANNOT drift apart".
struct Knob { double cx, cy, r; uint32_t puerto; };

// Drag travel: 200 window px for 0->1. It is what the handoff itself does
// (`(drag.y − e.clientY) / 200`), not a number chosen here.
constexpr double DRAG_PX = 200.0;
constexpr double WHEEL_STEP  = 0.02;

// There is no frame-rate cap: see `ui_idle`. The event loop already batches,
// and a second clock could only take frames away.

double ahora_s()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return double(t.tv_sec) + double(t.tv_nsec) * 1e-9;
}

// --- the variant dropdown ----------------------------------------------------
struct Popup {
    Window           win  = 0;
    cairo_surface_t* surf = nullptr;
    cairo_t*         cr   = nullptr;
    cairo_surface_t* buf   = nullptr;   // off-screen double buffer
    cairo_t*         bufcr = nullptr;
    int  w = 0, h = 0, row_h = 0;
    int  sobre = -1;                    // row under the mouse, or −1
    bool abierto = false;
};

struct UI {
    Display* dpy    = nullptr;
    Window   win    = 0;
    Window   parent = 0;
    Window   root   = 0;
    Visual*  visual = nullptr;
    cairo_surface_t* surf = nullptr;
    cairo_t*         cr   = nullptr;

    // Whole-window double buffer + cached static layer.
    cairo_surface_t* buf   = nullptr;
    cairo_t*         bufcr = nullptr;
    cairo_surface_t* estatica = nullptr;

    int    w = 0, h = 0;
    double scale = 1.0;

    LV2UI_Write_Function write = nullptr;
    LV2UI_Controller     ctl   = nullptr;
    const LV2UI_Touch*   touch = nullptr;

    nlsc::ui::Fonts      fuentes;
    // THE LOGO. It must be loaded HERE and not only in the preview tool: the
    // drawing tolerates a null logo, so a UI that forgets it renders a panel
    // with no logo and raises nothing. What ships is this file, not its twin.
    cairo_surface_t*     logo = nullptr;
    nlsc::ui::PanelState st;
    nlsc::ui::Theme      tema = nlsc::ui::default_theme();
    nlsc::ui::Cache      cache;
    int    variant = 0;

    Knob knobs[3];
    int   n_knobs = 0;

    // THE CACHE KEY: EXACTLY what the static layer draws, not one
    // field more or fewer. More => the expensive backdrop rebuilds for a
    // pixel the cache never held. Fewer => the panel freezes at the old
    // value and the control looks unresponsive — the SILENT failure.
    // Audited by reading `draw_static`: it paints the LED (`on`), the
    // footswitch (`pressed`), the variant name and the dropdown text. It
    // does NOT paint the knob values — `draw_live` does.
    struct Key { int w = -1, h = -1; bool on = false, pressed = false; int variant = -1; };
    Key clave;

    int    arrastre = -1;          // index into `knobs`, or −1
    double arr_y0 = 0.0, arr_v0 = 0.0;

    Popup  popup;
    bool   necesita_pintar = true;
    double ultimo_pintado = 0.0;
};

double knob_value(const UI* ui, int i)
{
    switch (ui->knobs[i].puerto) {
        case PORT_DRIVE: return ui->st.drive;
        case PORT_TONE:  return ui->st.tone;
        case PORT_LEVEL: return ui->st.level;
    }
    return 0.0;
}

void set_knob(UI* ui, int i, double v01)
{
    v01 = clamp01(v01);
    switch (ui->knobs[i].puerto) {
        case PORT_DRIVE: ui->st.drive = v01; break;
        case PORT_TONE:  ui->st.tone  = v01; break;
        case PORT_LEVEL: ui->st.level = v01; break;
    }
    if (ui->write) {
        float pv = float(v01);
        ui->write(ui->ctl, ui->knobs[i].puerto, sizeof(float), 0, &pv);
    }
    ui->necesita_pintar = true;
}

// `ui:touch` — tell the host a gesture begins and ends. It is what makes
// touch-mode automation and MIDI-learn behave.
void notify_touch(UI* ui, uint32_t puerto, bool agarrado)
{
    if (ui->touch && ui->touch->touch)
        ui->touch->touch(ui->touch->handle, puerto, agarrado);
}

// Window coordinates -> PANEL coordinates. This is where the margin is
// subtracted; without it the whole hit map is displaced.
void a_panel(const UI* ui, double px, double py, double* x, double* y)
{
    *x = px / ui->scale - nlsc::ui::UI_MARGIN;
    *y = py / ui->scale - nlsc::ui::UI_MARGIN;
}

int knob_impact(const UI* ui, double px, double py)
{
    double x, y; a_panel(ui, px, py, &x, &y);
    for (int i = 0; i < ui->n_knobs; ++i) {
        const double dx = x - ui->knobs[i].cx, dy = y - ui->knobs[i].cy;
        if (dx * dx + dy * dy <= ui->knobs[i].r * ui->knobs[i].r) return i;
    }
    return -1;
}

// RECTANGLE, not circle. The footswitch is drawn as a plate, so a circular
// hit-test leaves its four corners visible but dead to the click. The drawing
// and the mouse map must come from the SAME constants -- the knobs never had
// this problem because they do; here there were two sources.
// The WELL is used rather than the plate: a foot does not aim finely, and the
// slack around the plate is part of the target the user perceives.
bool hit_footswitch(const UI* ui, double px, double py)
{
    double x, y; a_panel(ui, px, py, &x, &y);
    return x >= nlsc::ui::FS_WELL_X && x <= nlsc::ui::FS_WELL_X + nlsc::ui::FS_WELL_W &&
           y >= nlsc::ui::FS_WELL_Y && y <= nlsc::ui::FS_WELL_Y + nlsc::ui::FS_WELL_H;
}

bool hit_dropdown(const UI* ui, double px, double py)
{
    double x, y; a_panel(ui, px, py, &x, &y);
    return x >= nlsc::ui::DD_X && x <= nlsc::ui::DD_X + nlsc::ui::DD_W &&
           y >= nlsc::ui::DD_Y && y <= nlsc::ui::DD_Y + nlsc::ui::DD_H;
}

void set_variant(UI* ui, int idx)
{
    idx = clampi(idx, 0, nlsc::kNumVariants - 1);
    ui->variant = idx;
    ui->st.variant = nlsc::kVariants[idx].etiqueta;
    if (ui->write) {
        float pv = float(idx);
        ui->write(ui->ctl, PORT_VARIANT, sizeof(float), 0, &pv);
    }
    ui->necesita_pintar = true;
}

void set_enabled(UI* ui, bool on)
{
    ui->st.on = on;
    if (ui->write) {
        float pv = on ? 1.0f : 0.0f;
        ui->write(ui->ctl, PORT_ENABLED, sizeof(float), 0, &pv);
    }
    ui->necesita_pintar = true;
}

// --- the dropdown -----------------------------------------------------------

void popup_paint(UI* ui)
{
    Popup& p = ui->popup;
    if (!p.bufcr) return;
    cairo_t* cr = p.bufcr;
    const nlsc::ui::Theme& th = ui->tema;

    nlsc::ui::set_rgb(cr, th.drop_bot);
    cairo_paint(cr);

    if (ui->fuentes.mono400) cairo_set_font_face(cr, ui->fuentes.mono400);
    else cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL,
                                CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, p.row_h * 0.42);

    for (int r = 0; r < nlsc::kNumVariants; ++r) {
        const double y = r * p.row_h;
        if (r == ui->variant) {
            nlsc::ui::set_rgba(cr, th.drop_top, 0.85);
            cairo_rectangle(cr, 0, y, p.w, p.row_h);
            cairo_fill(cr);
        } else if (r == p.sobre) {
            nlsc::ui::set_rgba(cr, 0xffffff, 0.08);
            cairo_rectangle(cr, 0, y, p.w, p.row_h);
            cairo_fill(cr);
        }
        cairo_text_extents_t e;
        cairo_text_extents(cr, nlsc::kVariants[r].etiqueta, &e);
        nlsc::ui::set_rgba(cr, th.drop_texto, r == ui->variant ? 1.0 : 0.85);
        cairo_move_to(cr, p.row_h * 0.5,
                      y + (p.row_h + e.height) / 2.0 - e.y_bearing - e.height);
        cairo_show_text(cr, nlsc::kVariants[r].etiqueta);
    }

    nlsc::ui::set_rgba(cr, th.print, 0.55);
    cairo_set_line_width(cr, 1.5);
    cairo_rectangle(cr, 0.75, 0.75, p.w - 1.5, p.h - 1.5);
    cairo_stroke(cr);

    cairo_surface_flush(p.buf);
    cairo_set_source_surface(p.cr, p.buf, 0, 0);
    cairo_paint(p.cr);
    cairo_surface_flush(p.surf);
}

void popup_close(UI* ui)
{
    Popup& p = ui->popup;
    if (!p.abierto) return;
    XUngrabPointer(ui->dpy, CurrentTime);
    if (p.bufcr) { cairo_destroy(p.bufcr); p.bufcr = nullptr; }
    if (p.buf)   { cairo_surface_destroy(p.buf); p.buf = nullptr; }
    if (p.cr)    { cairo_destroy(p.cr);   p.cr = nullptr; }
    if (p.surf)  { cairo_surface_destroy(p.surf); p.surf = nullptr; }
    if (p.win)   { XDestroyWindow(ui->dpy, p.win); p.win = 0; }
    p.abierto = false;
    p.sobre = -1;
    XFlush(ui->dpy);
}

void popup_open(UI* ui)
{
    Popup& p = ui->popup;
    if (p.abierto) { popup_close(ui); return; }

    const int scr = DefaultScreen(ui->dpy);
    p.row_h = int(nlsc::ui::DD_H * ui->scale);
    if (p.row_h < 16) p.row_h = 16;
    p.w = int(nlsc::ui::DD_W * ui->scale);
    p.h = p.row_h * nlsc::kNumVariants;
    p.sobre = -1;

    int rx = 0, ry = 0; Window hijo = 0;
    XTranslateCoordinates(ui->dpy, ui->win, ui->root,
                          int((nlsc::ui::DD_X + nlsc::ui::UI_MARGIN) * ui->scale),
                          int((nlsc::ui::DD_Y + nlsc::ui::UI_MARGIN + nlsc::ui::DD_H) * ui->scale),
                          &rx, &ry, &hijo);
    // If it would spill past the bottom of the screen, it opens upwards.
    const int sh = DisplayHeight(ui->dpy, scr);
    if (ry + p.h > sh) ry = ry - int(nlsc::ui::DD_H * ui->scale) - p.h;
    if (ry < 0) ry = 0;

    XSetWindowAttributes attr;
    attr.override_redirect = True;
    attr.background_pixel = BlackPixel(ui->dpy, scr);
    attr.event_mask = ExposureMask | ButtonPressMask | ButtonReleaseMask |
                      PointerMotionMask | LeaveWindowMask;
    p.win = XCreateWindow(ui->dpy, ui->root, rx, ry, p.w, p.h, 0,
                          CopyFromParent, InputOutput, ui->visual,
                          CWOverrideRedirect | CWBackPixel | CWEventMask, &attr);
    p.surf = cairo_xlib_surface_create(ui->dpy, p.win, ui->visual, p.w, p.h);
    p.cr   = cairo_create(p.surf);
    p.buf  = cairo_surface_create_similar(p.surf, CAIRO_CONTENT_COLOR, p.w, p.h);
    p.bufcr = cairo_create(p.buf);

    XMapRaised(ui->dpy, p.win);
    // `owner_events` False: EVERY pointer event goes to the popup in its
    // own coordinates, so a click outside arrives out of range and closes.
    // The grab can FAIL (`AlreadyGrabbed`: the host or another popup holds
    // the pointer). Marking the popup open anyway routes every ButtonPress of
    // the MAIN window through `popup_on_event`, which reads the coordinates as
    // the popup's: a click on the pedal's top-left corner then "selects a
    // row" and writes `PORT_VARIANT` — the user changes circuit by clicking
    // on DRIVE. A failed grab means no
    // popup, not a blind one.
    if (XGrabPointer(ui->dpy, p.win, False,
                     ButtonPressMask | ButtonReleaseMask | PointerMotionMask,
                     GrabModeAsync, GrabModeAsync, None, None, CurrentTime)
        != GrabSuccess) {
        p.abierto = true;      // so `popup_close` tears down what was just built
        popup_close(ui);
        return;
    }
    p.abierto = true;
    XFlush(ui->dpy);
    popup_paint(ui);
}

// Returns true if the dropdown kept the event.
bool popup_on_event(UI* ui, XEvent* ev)
{
    Popup& p = ui->popup;
    if (!p.abierto) return false;

    switch (ev->type) {
        case Expose:
            popup_paint(ui);
            return true;

        case MotionNotify: {
            const int x = ev->xmotion.x, y = ev->xmotion.y;
            int s = -1;
            if (x >= 0 && x < p.w && y >= 0 && y < p.h) s = y / p.row_h;
            // Repaint only if the highlighted row CHANGES: one repaint per
            // pixel of motion is exactly what must be avoided.
            if (s != p.sobre) { p.sobre = s; popup_paint(ui); }
            return true;
        }

        case ButtonPress: {
            const unsigned b = ev->xbutton.button;
            const int x = ev->xbutton.x, y = ev->xbutton.y;
            const bool dentro = (x >= 0 && x < p.w && y >= 0 && y < p.h);
            if (b == Button1) {
                if (dentro) {
                    const int idx = y / p.row_h;
                    if (idx >= 0 && idx < nlsc::kNumVariants) {
                        // The dropdown is no drag, but still a user gesture
                        // on a port: bracketed the same way.
                        notify_touch(ui, PORT_VARIANT, true);
                        set_variant(ui, idx);
                        notify_touch(ui, PORT_VARIANT, false);
                    }
                }
                popup_close(ui);   // inside or out, one click closes
                return true;
            }
            return true;
        }

        default:
            return true;   // while it holds the pointer grab it eats everything
    }
}

// --- painting ----------------------------------------------------------------

void haz_buffers(UI* ui)
{
    if (ui->bufcr)    { cairo_destroy(ui->bufcr); ui->bufcr = nullptr; }
    if (ui->buf)      { cairo_surface_destroy(ui->buf); ui->buf = nullptr; }
    if (ui->estatica) { cairo_surface_destroy(ui->estatica); ui->estatica = nullptr; }
    ui->buf = cairo_surface_create_similar(ui->surf, CAIRO_CONTENT_COLOR, ui->w, ui->h);
    ui->bufcr = cairo_create(ui->buf);
    ui->estatica = cairo_surface_create_similar(ui->surf, CAIRO_CONTENT_COLOR,
                                                ui->w, ui->h);
    ui->clave = UI::Key{};    // invalidate: the surfaces are new
}

// Rebuilds the static layer ONLY if its key changed.
void asegura_estatica(UI* ui)
{
    UI::Key k;
    k.w = ui->w; k.h = ui->h;
    k.on = ui->st.on; k.pressed = ui->st.pressed; k.variant = ui->variant;
    if (k.w == ui->clave.w && k.h == ui->clave.h && k.on == ui->clave.on &&
        k.pressed == ui->clave.pressed && k.variant == ui->clave.variant)
        return;

    cairo_t* cr = cairo_create(ui->estatica);
    cairo_set_antialias(cr, CAIRO_ANTIALIAS_GOOD);
    cairo_scale(cr, ui->scale, ui->scale);
    nlsc::ui::draw_backdrop(cr, nlsc::ui::WINDOW_W, nlsc::ui::WINDOW_H);
    nlsc::ui::draw_sombra(cr, nlsc::ui::UI_MARGIN + 10, nlsc::ui::UI_MARGIN + 8,
                          nlsc::ui::PANEL_W - 20, nlsc::ui::PANEL_H - 16, 26);
    cairo_translate(cr, nlsc::ui::UI_MARGIN, nlsc::ui::UI_MARGIN);
    nlsc::ui::draw_static(cr, ui->st, &ui->fuentes, &ui->cache.tex, ui->scale,
                          nlsc::ui::default_theme(), ui->logo);
    cairo_destroy(cr);
    cairo_surface_flush(ui->estatica);
    ui->clave = k;
}

void paint(UI* ui)
{
    if (!ui->cr || !ui->bufcr) return;
    asegura_estatica(ui);

    cairo_t* cr = ui->bufcr;
    cairo_identity_matrix(cr);
    cairo_set_source_surface(cr, ui->estatica, 0, 0);
    cairo_paint(cr);
    cairo_set_antialias(cr, CAIRO_ANTIALIAS_GOOD);
    cairo_translate(cr, nlsc::ui::UI_MARGIN * ui->scale,
                        nlsc::ui::UI_MARGIN * ui->scale);
    cairo_scale(cr, ui->scale, ui->scale);
    nlsc::ui::draw_live(cr, ui->st, &ui->cache);
    cairo_surface_flush(ui->buf);

    // A single blit to the window: it is never seen half-drawn.
    cairo_identity_matrix(ui->cr);
    cairo_set_source_surface(ui->cr, ui->buf, 0, 0);
    cairo_paint(ui->cr);
    cairo_surface_flush(ui->surf);
    XFlush(ui->dpy);
}

void on_event(UI* ui, XEvent* ev)
{
    switch (ev->type) {
        case Expose:
            if (ev->xexpose.count == 0) ui->necesita_pintar = true;
            break;

        case ConfigureNotify: {
            const int nw = ev->xconfigure.width, nh = ev->xconfigure.height;
            if (nw != ui->w || nh != ui->h) {
                ui->w = nw; ui->h = nh;
                cairo_xlib_surface_set_size(ui->surf, nw, nh);
                haz_buffers(ui);
                ui->necesita_pintar = true;
            }
            break;
        }

        case ButtonPress: {
            const unsigned b = ev->xbutton.button;
            const double px = ev->xbutton.x, py = ev->xbutton.y;
            if (b == Button1) {
                // ORDER MATTERS: the first hit keeps the click. Tested
                // from smallest to largest box and NONE overlaps another —
                // checked against `ui_draw.h`'s geometry: DRIVE's and
                // TONE's centres sit 144 units apart with radii summing 86,
                // and the footswitch (y=494±51) and dropdown (y=622..660)
                // do not touch.
                const int i = knob_impact(ui, px, py);
                if (i >= 0) {
                    ui->arrastre = i;
                    ui->arr_y0   = py;
                    ui->arr_v0   = knob_value(ui, i);
                    notify_touch(ui, ui->knobs[i].puerto, true);
                } else if (hit_footswitch(ui, px, py)) {
                    ui->st.pressed = true;          // it SINKS while held down
                    ui->necesita_pintar = true;
                } else if (hit_dropdown(ui, px, py)) {
                    popup_open(ui);
                }
            } else if (b == Button4 || b == Button5) {
                const int i = knob_impact(ui, px, py);
                if (i >= 0) {
                    const double step = (b == Button4) ? WHEEL_STEP : -WHEEL_STEP;
                    // The wheel is a gesture too: bracketed with touch, or
                    // the host sees an orphan write.
                    notify_touch(ui, ui->knobs[i].puerto, true);
                    set_knob(ui, i, knob_value(ui, i) + step);
                    notify_touch(ui, ui->knobs[i].puerto, false);
                }
            }
            break;
        }

        case ButtonRelease:
            if (ev->xbutton.button == Button1) {
                if (ui->arrastre >= 0) {
                    notify_touch(ui, ui->knobs[ui->arrastre].puerto, false);
                    ui->arrastre = -1;
                }
                if (ui->st.pressed) {
                    ui->st.pressed = false;
                    // The footswitch toggles ON RELEASE, and only released
                    // on top: dragging away and releasing cancels, like any
                    // button.
                    if (hit_footswitch(ui, ev->xbutton.x, ev->xbutton.y)) {
                        notify_touch(ui, PORT_ENABLED, true);
                        set_enabled(ui, !ui->st.on);
                        notify_touch(ui, PORT_ENABLED, false);
                    }
                    ui->necesita_pintar = true;
                }
            }
            break;

        case MotionNotify:
            if (ui->arrastre >= 0) {
                const double dy = ui->arr_y0 - ev->xmotion.y;
                set_knob(ui, ui->arrastre, ui->arr_v0 + dy / (DRAG_PX * ui->scale));
            }
            break;

        default:
            break;
    }
}

// --- LV2UI entry points -------------------------------------------------------

LV2UI_Handle instantiate(const LV2UI_Descriptor*, const char*, const char* bundle_path,
                         LV2UI_Write_Function write_function,
                         LV2UI_Controller controller, LV2UI_Widget* widget,
                         const LV2_Feature* const* features)
{
    Window parent = 0;
    const LV2UI_Resize* resize_host = nullptr;
    const LV2UI_Touch*  touch = nullptr;
    const LV2_Options_Option* options = nullptr;
    const LV2_URID_Map* map = nullptr;

    for (int i = 0; features && features[i]; ++i) {
        if (!std::strcmp(features[i]->URI, LV2_UI__parent))
            parent = static_cast<Window>(reinterpret_cast<uintptr_t>(features[i]->data));
        else if (!std::strcmp(features[i]->URI, LV2_UI__resize))
            resize_host = static_cast<const LV2UI_Resize*>(features[i]->data);
        else if (!std::strcmp(features[i]->URI, LV2_UI__touch))
            touch = static_cast<const LV2UI_Touch*>(features[i]->data);
        else if (!std::strcmp(features[i]->URI, LV2_OPTIONS__options))
            options = static_cast<const LV2_Options_Option*>(features[i]->data);
        else if (!std::strcmp(features[i]->URI, LV2_URID__map))
            map = static_cast<const LV2_URID_Map*>(features[i]->data);
    }

    auto* ui = new UI();
    ui->write = write_function;
    ui->ctl   = controller;
    ui->touch = touch;

    // `ui:scaleFactor`: the host SAYS what scale it wants the window at
    // on HiDPI screens. The workshop used to deduce it from Cairo's
    // matrix, which is guessing. If the host does not pass it, 1,0 and go.
    ui->scale = 1.0;
    if (options && map) {
        const LV2_URID urid_scale = map->map(map->handle, LV2_UI__scaleFactor);
        const LV2_URID urid_float  = map->map(map->handle, LV2_ATOM__Float);
        for (const LV2_Options_Option* o = options; o->key; ++o) {
            if (o->key == urid_scale && o->type == urid_float && o->value) {
                const float f = *static_cast<const float*>(o->value);
                if (f > 0.1f && f < 8.0f) ui->scale = double(f);
            }
        }
    }

    ui->w = int(nlsc::ui::WINDOW_W * ui->scale);
    ui->h = int(nlsc::ui::WINDOW_H * ui->scale);

    ui->dpy = XOpenDisplay(nullptr);
    if (!ui->dpy) { delete ui; return nullptr; }

    const int screen = DefaultScreen(ui->dpy);
    ui->visual = DefaultVisual(ui->dpy, screen);
    ui->root   = RootWindow(ui->dpy, screen);
    ui->parent = parent ? parent : ui->root;

    ui->win = XCreateSimpleWindow(ui->dpy, ui->parent, 0, 0,
                                  (unsigned)ui->w, (unsigned)ui->h, 0,
                                  BlackPixel(ui->dpy, screen),
                                  BlackPixel(ui->dpy, screen));
    if (!ui->win) { XCloseDisplay(ui->dpy); delete ui; return nullptr; }

    XSelectInput(ui->dpy, ui->win,
                 ExposureMask | StructureNotifyMask | ButtonPressMask |
                 ButtonReleaseMask | PointerMotionMask | ButtonMotionMask);

    ui->surf = cairo_xlib_surface_create(ui->dpy, ui->win, ui->visual, ui->w, ui->h);
    ui->cr   = cairo_create(ui->surf);
    haz_buffers(ui);

    // The fonts travel INSIDE the bundle: loaded from there so the panel
    // looks the same in any host, whatever it has installed.
    ui->fuentes = nlsc::ui::load_fonts(bundle_path ? bundle_path : "");
    // The logo travels in the bundle beside the fonts, by the same path.
    ui->logo = nlsc::ui::load_logo(bundle_path ? bundle_path : "");
    ui->cache   = nlsc::ui::make_cache(ui->scale);

    ui->knobs[0] = { nlsc::ui::DRIVE_CX, nlsc::ui::DRIVE_CY, nlsc::ui::DRIVE_R, PORT_DRIVE };
    ui->knobs[1] = { nlsc::ui::LEVEL_CX, nlsc::ui::LEVEL_CY, nlsc::ui::LEVEL_R, PORT_LEVEL };
    ui->knobs[2] = { nlsc::ui::TONE_CX,  nlsc::ui::TONE_CY,  nlsc::ui::TONE_R,  PORT_TONE  };
    ui->n_knobs = 3;

    ui->st.variant = nlsc::kVariants[ui->variant].etiqueta;

    XMapWindow(ui->dpy, ui->win);
    XFlush(ui->dpy);

    if (resize_host && resize_host->ui_resize)
        resize_host->ui_resize(resize_host->handle, ui->w, ui->h);

    *widget = reinterpret_cast<LV2UI_Widget>(static_cast<uintptr_t>(ui->win));
    return ui;
}

void cleanup(LV2UI_Handle handle)
{
    auto* ui = static_cast<UI*>(handle);
    popup_close(ui);
    nlsc::ui::free_cache(ui->cache);
    nlsc::ui::free_fonts(ui->fuentes);
    nlsc::ui::free_logo(ui->logo);
    if (ui->bufcr)    cairo_destroy(ui->bufcr);
    if (ui->buf)      cairo_surface_destroy(ui->buf);
    if (ui->estatica) cairo_surface_destroy(ui->estatica);
    if (ui->cr)       cairo_destroy(ui->cr);
    if (ui->surf)     cairo_surface_destroy(ui->surf);
    if (ui->win)      XDestroyWindow(ui->dpy, ui->win);
    if (ui->dpy)      XCloseDisplay(ui->dpy);
    delete ui;
}

void port_event(LV2UI_Handle handle, uint32_t port, uint32_t buffer_size,
                uint32_t format, const void* buffer)
{
    if (format != 0 || buffer_size < sizeof(float)) return;
    auto* ui = static_cast<UI*>(handle);
    const float v = *static_cast<const float*>(buffer);
    switch (port) {
        case PORT_DRIVE:   ui->st.drive = clamp01(v); break;
        case PORT_TONE:    ui->st.tone  = clamp01(v); break;
        case PORT_LEVEL:   ui->st.level = clamp01(v); break;
        case PORT_ENABLED: ui->st.on    = (v >= 0.5f); break;
        case PORT_VARIANT: {
            const int idx = clampi(int(v + 0.5f), 0, nlsc::kNumVariants - 1);
            ui->variant = idx;
            ui->st.variant = nlsc::kVariants[idx].etiqueta;
            break;
        }
        default: return;
    }
    ui->necesita_pintar = true;
}

// LV2's `idle` interface: pump X events and repaint when due.
int ui_idle(LV2UI_Handle handle)
{
    auto* ui = static_cast<UI*>(handle);
    if (!ui->dpy) return 1;
    while (XPending(ui->dpy)) {
        XEvent ev;
        XNextEvent(ui->dpy, &ev);
        if (ui->popup.abierto &&
            (ev.xany.window == ui->popup.win || ev.type == ButtonPress ||
             ev.type == ButtonRelease || ev.type == MotionNotify)) {
            popup_on_event(ui, &ev);          // the popup holds the pointer grab
        } else {
            on_event(ui, &ev);
        }
    }
    // THERE IS NO FRAME-RATE CAP HERE, and that is deliberate: knobs moved in
    // visible steps while there was one, because TWO limiters were beating
    // against each other. The loop above already drains EVERY pending event and
    // paints ONCE, so the batching a cap claims to add is already done. All a
    // cap adds is a second clock, and since the host calls `ui_idle` at ITS own
    // rate the two rates beat: with the host calling every 40 ms and a cap
    // demanding 33,3 since the last paint, one turn in two is skipped. That is
    // the visible step -- not the ports, which are continuous 0..1, nor the
    // drag, which is `dy/200` and unquantised.
    //
    // => A redundant limiter is not neutral: it can only remove frames, and it
    // costs more the faster the host calls.
    if (ui->necesita_pintar) {
        ui->necesita_pintar = false;
        ui->ultimo_pintado = ahora_s();
        paint(ui);
    }
    return 0;
}

const LV2UI_Idle_Interface idle_iface = { ui_idle };

const void* extension_data(const char* uri)
{
    if (!std::strcmp(uri, LV2_UI__idleInterface)) return &idle_iface;
    return nullptr;
}

const LV2UI_Descriptor ui_descriptor = {
    NLSC_UI_URI,
    instantiate,
    cleanup,
    port_event,
    extension_data,
};

} // namespace

extern "C" LV2_SYMBOL_EXPORT
const LV2UI_Descriptor* lv2ui_descriptor(uint32_t index)
{
    return (index == 0) ? &ui_descriptor : nullptr;
}
