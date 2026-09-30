// ui_x11.cpp — Valve Howler's X11 LV2 UI.
//
// This is the window-and-events layer. All painting lives in `ui_photo.h`
// (the photoreal panel: Blender renders composited with Cairo); here lives:
//   - creating a child window inside the one the host provides,
//   - wrapping it in a `cairo_xlib_surface`, loading the panel images and the
//     bundle fonts,
//   - pumping X events in the `idle` callback (expose / mouse),
//   - turning a knob drag into port writes,
//   - toggling the footswitch (`lv2:enabled`) and opening the variant
//     dropdown,
//   - reflecting on the panel what the host changes on its own
//     (`port_event`),
//   - the two short animations: the plates crossfade on bypass and the
//     treadle travels.
//
// No Pugl and no SVG renderer: just Xlib + cairo-xlib + FreeType, already
// on the system. X11 on purpose (under Wayland it runs via XWayland).
//
// Design constraints:
//
// 1. No repaint per event. Painting happens once per idle turn, and only
//    when something changed or an animation is running; the images are
//    scaled once per scale factor (`ui_photo.h`), never per paint.
//
// 2. Hit zones come from the same projection as the images. Mouse positions
//    are converted to ASSET pixels (the renders' grid) and tested against
//    `ui_photo_layout.h`, which the Blender scene generates: nothing here is
//    measured off a picture.
//
// 3. `ui:touch`: the host is told when a gesture on a knob begins and ends,
//    which touch-mode automation and MIDI-learn rely on (the LV2 UI spec).

#include "ui_photo.h"
#include "ui_fonts.h"
#include "nls_variantes.h"
// The port indices, from the one place that defines them (see the note at
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
#include <X11/keysym.h>
#include <X11/Xutil.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#define NLSC_UI_URI "https://nylarea.com/plugins/valvehowler#ui"

namespace {

// The port indices come from `nls_iface.h`, not from a local copy that could
// diverge at a renumbering. The `#include` sits with the
// others at the top: inside this anonymous namespace it would drag `nlsc::` in
// and shadow the UI's own namespace.
using nlsc::PORT_DRIVE;
using nlsc::PORT_TONE;
using nlsc::PORT_LEVEL;
using nlsc::PORT_ENABLED;
using nlsc::PORT_VARIANT;

// NaN-safe: fmax/fmin return the non-NaN operand, so a NaN lands on 0.
inline double clamp01(double v) { return std::fmin(1.0, std::fmax(0.0, v)); }
inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

namespace photo = nlsc::ui::photo;

// The port of each knob, in the order `photo::KNOBS` keeps them (drive, level,
// tone). The geometry is not here: it is the scene's projection.
constexpr uint32_t KNOB_PORT[3] = { PORT_DRIVE, PORT_LEVEL, PORT_TONE };

double seconds_now()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// Drag travel: 200 logical px for 0->1, the same law as the panel design.
constexpr double DRAG_PX = 200.0;
constexpr double WHEEL_STEP  = 0.02;

// There is no frame-rate cap: see `ui_idle`.


// --- the variant dropdown ----------------------------------------------------
struct Popup {
    Window           win  = 0;
    cairo_surface_t* surf = nullptr;
    cairo_t*         cr   = nullptr;
    cairo_surface_t* buf   = nullptr;   // off-screen double buffer
    cairo_t*         bufcr = nullptr;
    int  w = 0, h = 0, row_h = 0;
    int  hover = -1;                    // row under the mouse, or −1
    bool is_open = false;
    Window focus_at_open = 0;           // who had the keyboard focus when it opened
    int  frame = 0;                     // width of the bevelled frame around the rows
};

struct UI {
    Display* dpy    = nullptr;
    Window   win    = 0;
    Window   parent = 0;
    Window   root   = 0;
    Visual*  visual = nullptr;
    int      depth  = 0;                // the parent's; the popup uses it too
    Colormap cmap   = 0;                // ours, when the parent's visual is not the default
    cairo_surface_t* surf = nullptr;
    cairo_t*         cr   = nullptr;

    // Whole-window double buffer: the panel is composed here, then blitted once.
    cairo_surface_t* buf   = nullptr;
    cairo_t*         bufcr = nullptr;

    int    w = 0, h = 0;
    double scale = 1.0;          // the host's `ui:scaleFactor`: the window's size at open
    double fit = 0.5;            // window pixels per asset pixel: the pedal fitted to the window
    int    ox = 0, oy = 0;       // where the fitted pedal starts in the window (centred)

    LV2UI_Write_Function write = nullptr;
    LV2UI_Controller     ctl   = nullptr;
    const LV2UI_Touch*   touch = nullptr;

    nlsc::ui::Fonts      fonts_;
    photo::Assets        assets;     // as loaded from the bundle, for the current variant's green
    std::string          photo_dir;  // the bundle's photo/
    photo::Scaled        scaled;     // the images at this window's scale

    // What the panel shows. Knob values are the ports' (0..1).
    double knob[3] = { 0.5, 0.5, 0.5 };   // drive, level, tone
    bool   knob_dirty[3] = {};            // moved, not yet written to the port
    bool   on = true;                     // `enabled`
    bool   pressed = false;               // the treadle held down
    int    variant = 0;
    bool   display_hover = false;         // the pointer is over the variant display

    // The two animations, driven by the clock (not by how often the host idles).
    double lit = 1.0, lit_from = 1.0, lit_t0 = -1.0;          // plate crossfade
    double tread = 0.0, tread_from = 0.0, tread_t0 = -1.0;    // treadle travel

    int    drag = -1;          // index into `knob`, or −1
    double drag_y0 = 0.0, drag_v0 = 0.0;

    Popup  popup;
    bool   needs_paint = true;
};

double knob_value(const UI* ui, int i) { return ui->knob[i]; }

// A drag moves the knob on every motion event, but the port is written once per
// idle turn with the latest value: the host gets the same final value
// without a write per pixel of mouse travel.
void set_knob(UI* ui, int i, double v01)
{
    ui->knob[i] = clamp01(v01);
    ui->knob_dirty[i] = true;
    ui->needs_paint = true;
}

// Writes what moved. Called once per idle turn, and before a gesture's closing
// `touch`, so the host always sees the last value inside the gesture.
void flush_knobs(UI* ui)
{
    for (int i = 0; i < 3; ++i) {
        if (!ui->knob_dirty[i]) continue;
        ui->knob_dirty[i] = false;
        if (ui->write) {
            float pv = float(ui->knob[i]);
            ui->write(ui->ctl, KNOB_PORT[i], sizeof(float), 0, &pv);
        }
    }
}

// `ui:touch` — tell the host a gesture begins and ends. It is what makes
// touch-mode automation and MIDI-learn behave.
void notify_touch(UI* ui, uint32_t port_idx, bool held)
{
    if (ui->touch && ui->touch->touch)
        ui->touch->touch(ui->touch->handle, port_idx, held);
}

// Window pixels per ASSET pixel: the images are rendered at twice the logical size.
// The pedal keeps its proportions and fills the window as far as it can: the host may
// resize it. Recomputed on every size change.
double asset_scale(const UI* ui) { return ui->fit; }

// The same factor as a UI scale (1 = the logical 450 x 675), for sizes given in logical px.
double ui_scale(const UI* ui) { return ui->fit * 2.0; }

void update_fit(UI* ui)
{
    const double f = std::fmin(ui->w / photo::ASSET_W, ui->h / photo::ASSET_H);
    ui->fit = f > 0.0 ? f : ui->scale * 0.5;
    ui->ox = int((ui->w - std::ceil(photo::ASSET_W * ui->fit)) / 2.0);
    ui->oy = int((ui->h - std::ceil(photo::ASSET_H * ui->fit)) / 2.0);
    if (ui->ox < 0) ui->ox = 0;
    if (ui->oy < 0) ui->oy = 0;
}

// Window coordinates -> ASSET coordinates, the grid the hit zones are given in.
void to_asset(const UI* ui, double px, double py, double* x, double* y)
{
    *x = (px - ui->ox) / asset_scale(ui);
    *y = (py - ui->oy) / asset_scale(ui);
}

int knob_impact(const UI* ui, double px, double py)
{
    double x, y; to_asset(ui, px, py, &x, &y);
    return photo::knob_at(x, y);
}

// The treadle's own box: a foot does not aim finely, and the box already includes
// a little of the frame around the plate.
bool hit_footswitch(const UI* ui, double px, double py)
{
    double x, y; to_asset(ui, px, py, &x, &y);
    return photo::in_box(photo::TREADLE_HIT, x, y);
}

bool hit_dropdown(const UI* ui, double px, double py)
{
    double x, y; to_asset(ui, px, py, &x, &y);
    return photo::in_box(photo::DISPLAY_BEZEL, x, y);
}

// Start an animation from wherever it is now, so a reversal mid-way does not jump.
void start_fade(UI* ui)
{
    ui->lit_from = ui->lit;
    ui->lit_t0 = seconds_now();
    ui->needs_paint = true;
}

void start_treadle(UI* ui)
{
    ui->tread_from = ui->tread;
    ui->tread_t0 = seconds_now();
    ui->needs_paint = true;
}

// Advance both animations to `now`; true while either still moves.
bool animate(UI* ui, double now)
{
    bool moving = false;
    const double lit_to = ui->on ? 1.0 : 0.0;
    if (ui->lit_t0 >= 0.0) {
        const double t = (now - ui->lit_t0) / photo::FADE_S;
        if (t >= 1.0) { ui->lit = lit_to; ui->lit_t0 = -1.0; }
        else { ui->lit = ui->lit_from + (lit_to - ui->lit_from) * std::fmax(0.0, t); moving = true; }
    }
    const double tread_to = ui->pressed ? 1.0 : 0.0;
    if (ui->tread_t0 >= 0.0) {
        const double t = (now - ui->tread_t0) / photo::TREADLE_S;
        if (t >= 1.0) { ui->tread = tread_to; ui->tread_t0 = -1.0; }
        else { ui->tread = ui->tread_from + (tread_to - ui->tread_from) * std::fmax(0.0, t); moving = true; }
    }
    return moving;
}

void set_variant(UI* ui, int idx)
{
    idx = clampi(idx, 0, nlsc::kNumVariants - 1);
    ui->variant = idx;
    if (ui->write) {
        float pv = float(idx);
        ui->write(ui->ctl, PORT_VARIANT, sizeof(float), 0, &pv);
    }
    ui->needs_paint = true;
}

void set_enabled(UI* ui, bool on)
{
    ui->on = on;
    if (ui->write) {
        float pv = on ? 1.0f : 0.0f;
        ui->write(ui->ctl, PORT_ENABLED, sizeof(float), 0, &pv);
    }
    start_fade(ui);
}

// --- the dropdown -----------------------------------------------------------

// The list continues the variant display: dark glass inside a bevelled frame like
// the display's own, the dot-matrix face in the readout's lit green.
void popup_paint(UI* ui)
{
    Popup& p = ui->popup;
    if (!p.bufcr) return;
    cairo_t* cr = p.bufcr;
    const int green = photo::green_of_circuit(nlsc::kVariants[ui->variant].circuit);
    const unsigned lit = photo::READOUT_HEX[green];
    const double fw = p.frame;

    // the frame: dark grey bevel, a light edge on top, a dark one on the inside
    photo::set_hex(cr, 0x2b2c2e, 1.0);
    cairo_paint(cr);
    photo::set_hex(cr, 0x55575a, 1.0);
    cairo_rectangle(cr, 0, 0, p.w, 1.0);
    cairo_fill(cr);
    photo::set_hex(cr, 0x151516, 1.0);
    cairo_rectangle(cr, 0, p.h - 1.0, p.w, 1.0);
    cairo_fill(cr);
    photo::set_hex(cr, 0x050505, 1.0);
    cairo_rectangle(cr, fw - 1.0, fw - 1.0, p.w - 2.0 * fw + 2.0, p.h - 2.0 * fw + 2.0);
    cairo_fill(cr);
    // the glass
    photo::set_hex(cr, 0x0b0c0c, 1.0);
    cairo_rectangle(cr, fw, fw, p.w - 2.0 * fw, p.h - 2.0 * fw);
    cairo_fill(cr);

    if (ui->fonts_.doto) cairo_set_font_face(cr, ui->fonts_.doto);
    else cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL,
                                CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, p.row_h * 0.66);
    // Doto's dots are thinner than a pixel at list size, so with a fill alone
    // antialiasing never reaches the colour and the text reads dark. Stroking
    // the outline too makes every dot a whole pixel.
    const double dot_stroke = photo::DOT_STROKE_PX * ui_scale(ui);

    for (int r = 0; r < nlsc::kNumVariants; ++r) {
        const double y = fw + r * p.row_h;
        if (r == ui->variant || r == p.hover) {
            photo::set_hex(cr, lit, r == ui->variant ? 0.22 : 0.10);
            cairo_rectangle(cr, fw, y, p.w - 2.0 * fw, p.row_h);
            cairo_fill(cr);
        }
        std::string label = nlsc::kVariants[r].label;
        for (char& c : label) c = char(std::toupper(static_cast<unsigned char>(c)));
        cairo_text_extents_t e;
        cairo_text_extents(cr, label.c_str(), &e);
        photo::set_hex(cr, lit, r == ui->variant || r == p.hover ? 1.0 : 0.85);
        cairo_move_to(cr, (p.w - e.width) / 2.0 - e.x_bearing,
                      y + p.row_h / 2.0 - e.height / 2.0 - e.y_bearing);
        cairo_text_path(cr, label.c_str());
        cairo_set_line_width(cr, dot_stroke);
        cairo_stroke_preserve(cr);
        cairo_fill(cr);
    }

    cairo_surface_flush(p.buf);
    cairo_set_source_surface(p.cr, p.buf, 0, 0);
    cairo_paint(p.cr);
    cairo_surface_flush(p.surf);
}

void popup_close(UI* ui)
{
    Popup& p = ui->popup;
    if (!p.is_open) return;
    XUngrabPointer(ui->dpy, CurrentTime);
    XUngrabKeyboard(ui->dpy, CurrentTime);
    if (p.bufcr) { cairo_destroy(p.bufcr); p.bufcr = nullptr; }
    if (p.buf)   { cairo_surface_destroy(p.buf); p.buf = nullptr; }
    if (p.cr)    { cairo_destroy(p.cr);   p.cr = nullptr; }
    if (p.surf)  { cairo_surface_destroy(p.surf); p.surf = nullptr; }
    if (p.win)   { XDestroyWindow(ui->dpy, p.win); p.win = 0; }
    p.is_open = false;
    p.hover = -1;
    XFlush(ui->dpy);
}

void popup_open(UI* ui)
{
    Popup& p = ui->popup;
    if (p.is_open) { popup_close(ui); return; }

    const int scr = DefaultScreen(ui->dpy);
    const photo::Box& gl = photo::DISPLAY;          // the black screen
    const photo::Box& bz = photo::DISPLAY_BEZEL;    // its frame
    const double f = asset_scale(ui);
    p.row_h = int(30.0 * ui_scale(ui));
    if (p.row_h < 18) p.row_h = 18;
    p.frame = int(std::lround(4.0 * ui_scale(ui)));
    p.w = int(std::lround((gl.x1 - gl.x0) * f));
    p.h = p.row_h * nlsc::kNumVariants + 2 * p.frame;
    p.hover = -1;

    // Just under the display's frame, exactly as wide as its black screen.
    int rx = 0, ry = 0; Window child = 0;
    XTranslateCoordinates(ui->dpy, ui->win, ui->root,
                          ui->ox + int(std::lround(gl.x0 * f)), ui->oy + int(bz.y1 * f),
                          &rx, &ry, &child);
    // If it would spill past the bottom of the screen, it opens upwards.
    const int sh = DisplayHeight(ui->dpy, scr);
    if (ry + p.h > sh) ry = ry - int((bz.y1 - bz.y0) * f) - p.h;
    if (ry < 0) ry = 0;

    // The popup is a child of the root but uses the panel's visual, so it needs
    // that visual's depth and colormap too: under a 32-bit ARGB parent the
    // root's depth would not match the visual (BadMatch).
    XSetWindowAttributes attr{};
    attr.override_redirect = True;
    attr.background_pixel = BlackPixel(ui->dpy, scr);
    attr.border_pixel = 0;
    attr.event_mask = ExposureMask | ButtonPressMask | ButtonReleaseMask |
                      PointerMotionMask | LeaveWindowMask | KeyPressMask;
    unsigned long mask = CWOverrideRedirect | CWBackPixel | CWBorderPixel | CWEventMask;
    if (ui->cmap) { attr.colormap = ui->cmap; mask |= CWColormap; }
    p.win = XCreateWindow(ui->dpy, ui->root, rx, ry, p.w, p.h, 0,
                          ui->depth, InputOutput, ui->visual, mask, &attr);
    p.surf = cairo_xlib_surface_create(ui->dpy, p.win, ui->visual, p.w, p.h);
    p.cr   = cairo_create(p.surf);
    p.buf  = cairo_surface_create_similar(p.surf, CAIRO_CONTENT_COLOR, p.w, p.h);
    p.bufcr = cairo_create(p.buf);

    XMapRaised(ui->dpy, p.win);
    // `owner_events` False: every pointer event goes to the popup in its
    // own coordinates, so a click outside arrives out of range and closes.
    // The grab can fail (`AlreadyGrabbed`: the host or another popup holds
    // the pointer). An open popup routes every ButtonPress of the main window
    // through `popup_on_event`, which reads the coordinates as the popup's,
    // so a click on the panel would select a row; hence a failed grab means
    // no popup at all.
    if (XGrabPointer(ui->dpy, p.win, False,
                     ButtonPressMask | ButtonReleaseMask | PointerMotionMask,
                     GrabModeAsync, GrabModeAsync, None, None, CurrentTime)
        != GrabSuccess) {
        p.is_open = true;      // so `popup_close` tears down what was just built
        popup_close(ui);
        return;
    }
    p.is_open = true;
    int revert = 0;
    XGetInputFocus(ui->dpy, &p.focus_at_open, &revert);
    // The keyboard too, so Escape closes the list. A failed keyboard grab
    // leaves the popup usable by mouse.
    XGrabKeyboard(ui->dpy, p.win, False, GrabModeAsync, GrabModeAsync, CurrentTime);
    XFlush(ui->dpy);
    popup_paint(ui);
}

// Returns true if the dropdown kept the event.
bool popup_on_event(UI* ui, XEvent* ev)
{
    Popup& p = ui->popup;
    if (!p.is_open) return false;

    switch (ev->type) {
        case Expose:
            popup_paint(ui);
            return true;

        case MotionNotify: {
            const int x = ev->xmotion.x, y = ev->xmotion.y;
            int s = -1;
            if (x >= p.frame && x < p.w - p.frame && y >= p.frame && y < p.h - p.frame)
                s = (y - p.frame) / p.row_h;
            // Repaint only if the highlighted row changes, not per pixel of
            // motion.
            if (s != p.hover) { p.hover = s; popup_paint(ui); }
            return true;
        }

        case ButtonPress: {
            const unsigned b = ev->xbutton.button;
            const int x = ev->xbutton.x, y = ev->xbutton.y;
            // The frame is not a row: (y - frame) / row_h would truncate a click
            // on the top edge to row 0.
            const bool inside = (x >= p.frame && x < p.w - p.frame &&
                                 y >= p.frame && y < p.h - p.frame);
            if (b == Button1) {
                if (inside) {
                    const int idx = (y - p.frame) / p.row_h;
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

        case KeyPress:
            if (XLookupKeysym(&ev->xkey, 0) == XK_Escape) popup_close(ui);
            return true;

        default:
            return true;   // while it holds the pointer grab it eats everything
    }
}

// --- painting ----------------------------------------------------------------

void make_buffers(UI* ui)
{
    if (ui->bufcr)    { cairo_destroy(ui->bufcr); ui->bufcr = nullptr; }
    if (ui->buf)      { cairo_surface_destroy(ui->buf); ui->buf = nullptr; }
    ui->buf = cairo_surface_create_similar(ui->surf, CAIRO_CONTENT_COLOR, ui->w, ui->h);
    ui->bufcr = cairo_create(ui->buf);
}

photo::View view_of(const UI* ui)
{
    photo::View v;
    v.drive = ui->knob[0]; v.level = ui->knob[1]; v.tone = ui->knob[2];
    v.green = photo::green_of_circuit(nlsc::kVariants[ui->variant].circuit);
    v.lit = ui->lit;
    v.treadle = ui->tread;
    // In bypass the display is dark, and lights while hovered or
    // while the list is open, so a variant is never changed blind.
    v.display_lit = ui->on || ui->display_hover || ui->popup.is_open;
    v.variant = nlsc::kVariants[ui->variant].label;
    v.face = ui->fonts_.doto;
    return v;
}

void paint(UI* ui)
{
    if (!ui->cr || !ui->bufcr) return;
    const photo::View v = view_of(ui);
    if (ui->assets.green != v.green) {       // the variant changed circuit: the other paint
        photo::free_scaled(ui->scaled);
        photo::free_assets(ui->assets);
        ui->assets = photo::load_assets(ui->photo_dir, v.green);
    }
    photo::ensure_scaled(ui->scaled, ui->assets, asset_scale(ui));

    cairo_t* cr = ui->bufcr;
    cairo_identity_matrix(cr);
    cairo_set_antialias(cr, CAIRO_ANTIALIAS_GOOD);
    if (ui->ox > 0 || ui->oy > 0) {          // the margins around the fitted pedal
        photo::set_hex(cr, photo::BACKDROP_HEX, 1.0);
        cairo_paint(cr);
    }
    cairo_translate(cr, ui->ox, ui->oy);     // integer: the frames stay on the plate's pixel grid
    photo::draw_panel(cr, ui->scaled, v);
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
            if (ev->xexpose.count == 0) ui->needs_paint = true;
            break;

        case ConfigureNotify: {
            const int nw = ev->xconfigure.width, nh = ev->xconfigure.height;
            if (nw != ui->w || nh != ui->h) {
                ui->w = nw; ui->h = nh;
                update_fit(ui);
                cairo_xlib_surface_set_size(ui->surf, nw, nh);
                make_buffers(ui);
                ui->needs_paint = true;
            }
            break;
        }

        case ButtonPress: {
            const unsigned b = ev->xbutton.button;
            const double px = ev->xbutton.x, py = ev->xbutton.y;
            if (b == Button1) {
                // No zone overlaps another (the scene's projection: DRIVE's and
                // TONE's centres sit 210 asset px apart with radii summing 151,
                // and the display bezel ends ~270 px above the treadle), so
                // the order of the tests does not decide anything.
                const int i = knob_impact(ui, px, py);
                if (i >= 0) {
                    ui->drag = i;
                    ui->drag_y0   = py;
                    ui->drag_v0   = knob_value(ui, i);
                    notify_touch(ui, KNOB_PORT[i], true);
                } else if (hit_footswitch(ui, px, py)) {
                    ui->pressed = true;             // it sinks while held down
                    start_treadle(ui);
                } else if (hit_dropdown(ui, px, py)) {
                    popup_open(ui);
                }
            } else if (b == Button4 || b == Button5) {
                const int i = knob_impact(ui, px, py);
                if (i >= 0) {
                    const double step = (b == Button4) ? WHEEL_STEP : -WHEEL_STEP;
                    // The wheel is a gesture too: bracketed with touch, or
                    // the host sees an orphan write.
                    notify_touch(ui, KNOB_PORT[i], true);
                    set_knob(ui, i, knob_value(ui, i) + step);
                    flush_knobs(ui);
                    notify_touch(ui, KNOB_PORT[i], false);
                }
            }
            break;
        }

        case ButtonRelease:
            if (ev->xbutton.button == Button1) {
                if (ui->drag >= 0) {
                    flush_knobs(ui);
                    notify_touch(ui, KNOB_PORT[ui->drag], false);
                    ui->drag = -1;
                }
                if (ui->pressed) {
                    ui->pressed = false;
                    start_treadle(ui);
                    // The footswitch toggles on release, and only released
                    // on top: dragging away and releasing cancels, like any
                    // button.
                    if (hit_footswitch(ui, ev->xbutton.x, ev->xbutton.y)) {
                        notify_touch(ui, PORT_ENABLED, true);
                        set_enabled(ui, !ui->on);
                        notify_touch(ui, PORT_ENABLED, false);
                    }
                }
            }
            break;

        case MotionNotify:
            if (ui->drag >= 0 && !(ev->xmotion.state & Button1Mask)) {
                // The button is up and the release never arrived (another
                // client grabbed the pointer mid-drag): end the drag here, or
                // the knob stays stuck to the mouse and the host's touch
                // bracket is never closed.
                flush_knobs(ui);
                notify_touch(ui, KNOB_PORT[ui->drag], false);
                ui->drag = -1;
            } else if (ui->drag >= 0) {
                const double dy = ui->drag_y0 - ev->xmotion.y;
                set_knob(ui, ui->drag, ui->drag_v0 + dy / (DRAG_PX * ui_scale(ui)));
            } else {
                const bool over = hit_dropdown(ui, ev->xmotion.x, ev->xmotion.y);
                if (over != ui->display_hover) { ui->display_hover = over; ui->needs_paint = true; }
            }
            break;

        case LeaveNotify:
            if (ui->display_hover) { ui->display_hover = false; ui->needs_paint = true; }
            break;

        case UnmapNotify:          // the pedal's window hidden: its list goes with it
            popup_close(ui);
            break;

        default:
            break;
    }
}

// --- LV2UI entry points -------------------------------------------------------

// X errors. Xlib's default handler exit()s the whole process -- the host -- on
// any protocol error, and a parent window whose visual we do not match (a
// 32-bit ARGB parent) or one the server does not know raises one. While any
// instance of this UI is alive, a handler records errors on our connections and
// hands every other error to whatever handler was installed before (the host's).
// The handler is process-wide by Xlib's design, so it is installed with the
// first instance and restored with the last.
// NLSC_UI_PARENT_VISUAL=0 builds a plain window with the default visual and no
// error handler; it is not used by the plugin.
#ifndef NLSC_UI_PARENT_VISUAL
#define NLSC_UI_PARENT_VISUAL 1
#endif

#if NLSC_UI_PARENT_VISUAL
int (*g_prev_x_handler)(Display*, XErrorEvent*) = nullptr;
int  g_ui_instances = 0;
Display* g_ui_displays[16] = {};
bool g_x_error = false;

int ui_x_error(Display* d, XErrorEvent* e)
{
    for (Display* ours : g_ui_displays)
        if (ours == d) { g_x_error = true; return 0; }
    return g_prev_x_handler ? g_prev_x_handler(d, e) : 0;
}

void track_display(Display* d, bool add)
{
    for (Display*& slot : g_ui_displays)
        if (add ? slot == nullptr : slot == d) { slot = add ? d : nullptr; return; }
}
#endif

void cleanup(LV2UI_Handle handle);

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

    // `ui:scaleFactor`: the host states what scale it wants the window at
    // on HiDPI screens. If the host does not pass it, 1.0.
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

    ui->w = int(std::ceil(photo::LOGICAL_W * ui->scale));
    ui->h = int(std::ceil(photo::LOGICAL_H * ui->scale));
    update_fit(ui);

    ui->dpy = XOpenDisplay(nullptr);
    if (!ui->dpy) { delete ui; return nullptr; }

    const int screen = DefaultScreen(ui->dpy);
    ui->visual = DefaultVisual(ui->dpy, screen);
    ui->depth  = DefaultDepth(ui->dpy, screen);
    ui->root   = RootWindow(ui->dpy, screen);
    ui->parent = parent ? parent : ui->root;

#if NLSC_UI_PARENT_VISUAL
    if (g_ui_instances++ == 0) { g_x_error = false; g_prev_x_handler = XSetErrorHandler(ui_x_error); }
    track_display(ui->dpy, true);
    g_x_error = false;
    // The window takes the parent's visual and depth, and so does the cairo
    // surface: a child of another depth cannot be drawn into its parent.
    XWindowAttributes pa;
    if (XGetWindowAttributes(ui->dpy, ui->parent, &pa) && !g_x_error) {
        ui->visual = pa.visual;
        ui->depth  = pa.depth;
    }
    XSetWindowAttributes wa{};
    unsigned long mask = CWBackPixel | CWBorderPixel;
    wa.background_pixel = 0;
    wa.border_pixel = 0;
    if (ui->visual != DefaultVisual(ui->dpy, screen)) {
        ui->cmap = XCreateColormap(ui->dpy, ui->root, ui->visual, AllocNone);
        wa.colormap = ui->cmap;
        mask |= CWColormap;
    }
    ui->win = XCreateWindow(ui->dpy, ui->parent, 0, 0, (unsigned)ui->w, (unsigned)ui->h, 0,
                            ui->depth, InputOutput, ui->visual, mask, &wa);
    XSync(ui->dpy, False);
    if (!ui->win || g_x_error) {   // e.g. a parent the server does not know
        ui->win = 0;
        cleanup(ui);
        return nullptr;
    }
#else
    ui->win = XCreateSimpleWindow(ui->dpy, ui->parent, 0, 0,
                                  (unsigned)ui->w, (unsigned)ui->h, 0,
                                  BlackPixel(ui->dpy, screen),
                                  BlackPixel(ui->dpy, screen));
    if (!ui->win) { XCloseDisplay(ui->dpy); delete ui; return nullptr; }
#endif

    // Size hints for a host that resizes the plugin window: keep the pedal's
    // proportions, never below half its size. The drawing fits whatever size it gets.
    if (XSizeHints* sh = XAllocSizeHints()) {
        sh->flags = PMinSize | PAspect | PBaseSize;
        sh->min_width  = int(photo::LOGICAL_W / 2);
        sh->min_height = int(photo::LOGICAL_H / 2);
        sh->base_width = ui->w; sh->base_height = ui->h;
        sh->min_aspect.x = sh->max_aspect.x = int(photo::ASSET_W);
        sh->min_aspect.y = sh->max_aspect.y = int(photo::ASSET_H);
        XSetWMNormalHints(ui->dpy, ui->win, sh);
        XFree(sh);
    }

    XSelectInput(ui->dpy, ui->win,
                 ExposureMask | StructureNotifyMask | ButtonPressMask |
                 ButtonReleaseMask | PointerMotionMask | ButtonMotionMask |
                 LeaveWindowMask);

    ui->surf = cairo_xlib_surface_create(ui->dpy, ui->win, ui->visual, ui->w, ui->h);
    ui->cr   = cairo_create(ui->surf);
    make_buffers(ui);

    // The fonts and the panel images travel inside the bundle: loaded from
    // there so the panel looks the same in any host, whatever it has installed.
    const std::string bundle = bundle_path ? bundle_path : "";
    ui->fonts_  = nlsc::ui::load_fonts(bundle);
    ui->photo_dir = (bundle.empty() ? std::string(".") : bundle) + "/photo";   // loaded at the first paint

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
    photo::free_scaled(ui->scaled);
    photo::free_assets(ui->assets);
    nlsc::ui::free_fonts(ui->fonts_);
    if (ui->bufcr)    cairo_destroy(ui->bufcr);
    if (ui->buf)      cairo_surface_destroy(ui->buf);
    if (ui->cr)       cairo_destroy(ui->cr);
    if (ui->surf)     cairo_surface_destroy(ui->surf);
    if (ui->win)      XDestroyWindow(ui->dpy, ui->win);
    if (ui->cmap)     XFreeColormap(ui->dpy, ui->cmap);
    if (ui->dpy) {
        XSync(ui->dpy, False);
        XCloseDisplay(ui->dpy);
#if NLSC_UI_PARENT_VISUAL
        track_display(ui->dpy, false);
        if (--g_ui_instances == 0) XSetErrorHandler(g_prev_x_handler);
#endif
    }
    delete ui;
}

void port_event(LV2UI_Handle handle, uint32_t port, uint32_t buffer_size,
                uint32_t format, const void* buffer)
{
    if (format != 0 || buffer_size < sizeof(float)) return;
    auto* ui = static_cast<UI*>(handle);
    const float v = *static_cast<const float*>(buffer);
    // A non-finite value from the host is ignored: it would reach the knob
    // drawing (an out-of-range knurl index, and a cairo context stuck in an
    // error state so the window never repaints) and be written back to the
    // DSP on the next drag.
    if (!std::isfinite(v)) return;
    switch (port) {
        // A knob with a write still pending keeps the user's newer value.
        case PORT_DRIVE:   if (!ui->knob_dirty[0]) ui->knob[0] = clamp01(v); break;
        case PORT_LEVEL:   if (!ui->knob_dirty[1]) ui->knob[1] = clamp01(v); break;
        case PORT_TONE:    if (!ui->knob_dirty[2]) ui->knob[2] = clamp01(v); break;
        case PORT_ENABLED:
            if ((v >= 0.5f) != ui->on) { ui->on = (v >= 0.5f); start_fade(ui); }
            break;
        case PORT_VARIANT: {
            // Clamped as a float first: int() of a value past INT_MAX is UB.
            const float vc = std::fmin(float(nlsc::kNumVariants - 1), std::fmax(0.0f, v));
            const int idx = clampi(int(vc + 0.5f), 0, nlsc::kNumVariants - 1);
            ui->variant = idx;
            break;
        }
        default: return;
    }
    ui->needs_paint = true;
}

// LV2's `idle` interface: pump X events and repaint when due.
int ui_idle(LV2UI_Handle handle)
{
    auto* ui = static_cast<UI*>(handle);
    if (!ui->dpy) return 1;
    while (XPending(ui->dpy)) {
        XEvent ev;
        XNextEvent(ui->dpy, &ev);
        if (ui->popup.is_open &&
            (ev.xany.window == ui->popup.win || ev.type == ButtonPress ||
             ev.type == ButtonRelease || ev.type == MotionNotify)) {
            popup_on_event(ui, &ev);          // the popup holds the pointer grab
        } else {
            on_event(ui, &ev);
        }
    }
    // There is deliberately no frame-rate cap: the loop above already drains
    // every pending event and paints once per turn. A cap would be a second
    // clock beating against the host's idle rate and skipping turns, which
    // shows as knobs moving in visible steps.
    //
    // The list closes when the keyboard focus moves away from where it was when
    // it opened: an override-redirect list has no window manager to hide it, so
    // after switching application it would float over whatever is in front.
    // Polled, not an event: under XWayland the grabs do not stop a Wayland
    // window taking the focus, and the FocusOut they raise also fires when the
    // grab is merely refused.
    if (ui->popup.is_open) {
        Window fw = 0; int revert = 0;
        XGetInputFocus(ui->dpy, &fw, &revert);
        if (fw != ui->popup.focus_at_open) popup_close(ui);
    }
    flush_knobs(ui);
    if (animate(ui, seconds_now())) ui->needs_paint = true;
    if (ui->needs_paint) {
        ui->needs_paint = false;
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
