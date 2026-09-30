// The photoreal panel: pre-rendered Blender images composited with Cairo.
//
// No X in here, so the preview tool draws exactly what the host shows. The images
// are rendered at twice the logical size (HiDPI), and scaled once per scale factor
// into a cache, never per paint.
//
// Layers, back to front:
//   plate      the whole top for the variant's green and the `enabled` state, knobs
//              at rest and the variant display empty; a change of state crossfades
//   knobs      one frame of each knob's strip, pasted through a soft circular mask
//              over its base circle (the paint around it is the plate's)
//   treadle    one frame of the treadle strip (rest -> pressed)
//   readout    the variant name and the caret, drawn live in the dot-matrix face
#pragma once

#include "ui_photo_layout.h"

#include <cairo.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <vector>

namespace nlsc {
namespace ui {
namespace photo {

inline constexpr double LOGICAL_W = ASSET_W / 2.0;   // the window at scale 1
inline constexpr double LOGICAL_H = ASSET_H / 2.0;
inline constexpr int GREEN_OD9 = 0, GREEN_OD8 = 1;
inline constexpr const char* GREEN_NAME[2] = { "od9", "od8" };
inline constexpr unsigned GREEN_HEX[2] = { 0x6BC75C, 0x48B180 };   // the box paints
inline constexpr unsigned BACKDROP_HEX = 0x161618;   // around the pedal: the colour its renders fade to
// The lit readout's dots as the scene renders them (median of the lit pixels).
inline constexpr unsigned READOUT_HEX[2] = { 0x91FF7E, 0x6AF4B3 };
inline constexpr double KNOB_MIN_DEG = -135.0, KNOB_MAX_DEG = 135.0; // the scale dots' travel
inline constexpr double FADE_S = 0.120;              // plate crossfade on bypass
inline constexpr double TREADLE_S = 0.080;           // treadle travel, down or up

// The circuit decides the paint: the OD-8 rows are OD-8 green, the OD-9 rows OD-9 green.
inline int green_of_circuit(int circuit) { return circuit == 0 ? GREEN_OD8 : GREEN_OD9; }

// --- the images, as loaded from the bundle --------------------------------------

// One green at a time: only one is ever on screen, and decoding both would cost ~60 MB and
// ~290 ms when opening the window. The other loads when the variant changes circuit.
struct Assets {
    int green = -1;
    cairo_surface_t* plate[2] = {};      // [0 active, 1 bypass]
    cairo_surface_t* knob[3]  = {};      // [drive, level, tone]: frames stacked vertically
    cairo_surface_t* treadle  = nullptr;
    bool ok = false;
};

inline cairo_surface_t* load_png(const std::string& path)
{
    cairo_surface_t* s = cairo_image_surface_create_from_png(path.c_str());
    if (cairo_surface_status(s) != CAIRO_STATUS_SUCCESS) { cairo_surface_destroy(s); return nullptr; }
    return s;
}

inline void free_assets(Assets& a)
{
    for (auto* s : a.plate) if (s) cairo_surface_destroy(s);
    for (auto* s : a.knob)  if (s) cairo_surface_destroy(s);
    if (a.treadle) cairo_surface_destroy(a.treadle);
    a = Assets{};
}

// `dir` is the bundle's `photo/` directory. A missing or unreadable image leaves `ok` false and
// the UI draws a plain placeholder: a panel with a hole in it would look like the plugin.
inline Assets load_assets(const std::string& dir, int g)
{
    Assets a;
    a.green = g;
    bool ok = true;
    for (int s = 0; s < 2; ++s) {
        a.plate[s] = load_png(dir + "/plate_" + GREEN_NAME[g] + (s ? "_bypass" : "_active") + ".png");
        ok &= a.plate[s] != nullptr &&
              cairo_image_surface_get_width(a.plate[s]) == int(ASSET_W) &&
              cairo_image_surface_get_height(a.plate[s]) == int(ASSET_H);
    }
    for (int k = 0; k < 3; ++k) {
        const KnobGeo& kg = KNOBS[k];
        a.knob[k] = load_png(dir + "/knob_" + GREEN_NAME[g] + "_" + kg.name + ".png");
        ok &= a.knob[k] != nullptr &&
              cairo_image_surface_get_width(a.knob[k]) == kg.x1 - kg.x0 &&
              cairo_image_surface_get_height(a.knob[k]) == (kg.y1 - kg.y0) * kg.frames;
    }
    a.treadle = load_png(dir + "/treadle.png");
    ok &= a.treadle != nullptr &&
          cairo_image_surface_get_width(a.treadle) == TREADLE_X1 - TREADLE_X0 &&
          cairo_image_surface_get_height(a.treadle) == (TREADLE_Y1 - TREADLE_Y0) * TREADLE_FRAMES;
    a.ok = ok;
    return a;
}

// --- the images, scaled to the window ------------------------------------------

// `f` = window pixels per asset pixel (0.5 at scale 1), for the green the assets hold.
struct Scaled {
    double f = -1.0;
    int    green = -1;
    cairo_surface_t* plate[2] = {};
    std::vector<cairo_surface_t*> knob[3];
    std::vector<cairo_surface_t*> treadle;
    int kx[3] = {}, ky[3] = {};          // integer window position of each knob frame
    int tx = 0, ty = 0;
};

inline void free_scaled(Scaled& s)
{
    for (auto*& p : s.plate) if (p) cairo_surface_destroy(p);
    for (auto& v : s.knob) for (auto* p : v) cairo_surface_destroy(p);
    for (auto* p : s.treadle) cairo_surface_destroy(p);
    s = Scaled{};
}

// One region of a source image, resampled onto its own surface whose top-left sits at the
// integer window pixel (px0, py0). Each window pixel samples the source at the same place the
// whole-plate resample does, so a pasted frame lines up with the plate under it.
inline cairo_surface_t* resample(cairo_surface_t* src, double sx, double sy, double sw, double sh,
                                 double ax, double ay, double f, int px0, int py0, int pw, int ph)
{
    cairo_surface_t* sub = cairo_surface_create_for_rectangle(src, sx, sy, sw, sh);
    cairo_surface_t* dst = cairo_image_surface_create(CAIRO_FORMAT_RGB24, pw, ph);
    cairo_t* cr = cairo_create(dst);
    cairo_translate(cr, -px0, -py0);
    cairo_scale(cr, f, f);
    cairo_set_source_surface(cr, sub, ax, ay);
    cairo_pattern_set_filter(cairo_get_source(cr), f == 1.0 ? CAIRO_FILTER_NEAREST : CAIRO_FILTER_GOOD);
    cairo_pattern_set_extend(cairo_get_source(cr), CAIRO_EXTEND_PAD);
    cairo_paint(cr);
    cairo_destroy(cr);
    cairo_surface_destroy(sub);
    cairo_surface_flush(dst);
    return dst;
}

inline void ensure_scaled(Scaled& s, const Assets& a, double f)
{
    if (!a.ok || (s.f == f && s.green == a.green)) return;
    free_scaled(s);
    s.f = f; s.green = a.green;
    const int W = int(std::ceil(ASSET_W * f)), H = int(std::ceil(ASSET_H * f));
    for (int st = 0; st < 2; ++st)
        s.plate[st] = resample(a.plate[st], 0, 0, ASSET_W, ASSET_H, 0, 0, f, 0, 0, W, H);
    for (int k = 0; k < 3; ++k) {
        const KnobGeo& kg = KNOBS[k];
        const int w = kg.x1 - kg.x0, h = kg.y1 - kg.y0;
        s.kx[k] = int(std::floor(kg.x0 * f)); s.ky[k] = int(std::floor(kg.y0 * f));
        const int pw = int(std::ceil(kg.x1 * f)) - s.kx[k], ph = int(std::ceil(kg.y1 * f)) - s.ky[k];
        for (int i = 0; i < kg.frames; ++i)
            s.knob[k].push_back(resample(a.knob[k], 0, double(i * h), w, h, kg.x0, kg.y0,
                                         f, s.kx[k], s.ky[k], pw, ph));
    }
    const int tw = TREADLE_X1 - TREADLE_X0, th = TREADLE_Y1 - TREADLE_Y0;
    s.tx = int(std::floor(TREADLE_X0 * f)); s.ty = int(std::floor(TREADLE_Y0 * f));
    const int pw = int(std::ceil(TREADLE_X1 * f)) - s.tx, ph = int(std::ceil(TREADLE_Y1 * f)) - s.ty;
    for (int i = 0; i < TREADLE_FRAMES; ++i)
        s.treadle.push_back(resample(a.treadle, 0, double(i * th), tw, th, TREADLE_X0, TREADLE_Y0,
                                     f, s.tx, s.ty, pw, ph));
}

// --- what is drawn ---------------------------------------------------------------

struct View {
    double drive = 0.5, level = 0.5, tone = 0.5;   // knob positions, 0..1
    int    green = GREEN_OD9;
    double lit = 1.0;          // 1 active plate, 0 bypass plate; in between while crossfading
    double treadle = 0.0;      // 0 rest .. 1 pressed
    bool   display_lit = true; // the readout shows (active, hovered or the list open)
    const char* variant = "";
    cairo_font_face_t* face = nullptr;   // the dot-matrix face; null falls back to monospace
};

inline int knob_frame(const KnobGeo& kg, double v01)
{
    const double deg = KNOB_MIN_DEG + (KNOB_MAX_DEG - KNOB_MIN_DEG) * std::clamp(v01, 0.0, 1.0);
    const double t = (deg - kg.deg_first) / (kg.deg_last - kg.deg_first);
    return std::clamp(int(std::lround(t * (kg.frames - 1))), 0, kg.frames - 1);
}

inline void set_hex(cairo_t* cr, unsigned hex, double alpha)
{
    cairo_set_source_rgba(cr, ((hex >> 16) & 0xff) / 255.0, ((hex >> 8) & 0xff) / 255.0,
                          (hex & 0xff) / 255.0, alpha);
}

// The readout: a backlit dot-matrix line in the colour the scene renders it, with a faint glow
// (the text's own outline widened at low alpha). The dots are stroked as well as filled: at
// scale 1 they are thinner than a pixel and antialiasing never reaches the colour, so a
// fill alone reads dark. `DOT_STROKE_PX` per unit of UI scale is the same the variant list uses.
// Font size of the readout, in scene mm: calibrated against the scene's own render of the
// display (ink height 21 asset px on the OD-9 plate), not chosen by eye.
inline constexpr double TEXT_MM = 3.66;
inline constexpr double DOT_STROKE_PX = 0.9;

inline void draw_readout(cairo_t* cr, const View& v, double f)
{
    const Box& d = DISPLAY;
    const double w = (d.x1 - d.x0) * f;
    const double mm = w / 54.6;                       // the glass is 54.6 mm wide in the scene
    const double cx = (d.x0 + d.x1) / 2.0 * f, cy = (d.y0 + d.y1) / 2.0 * f;
    const unsigned g = READOUT_HEX[v.green];
    std::string label = v.variant;
    for (char& c : label) c = char(std::toupper(static_cast<unsigned char>(c)));

    cairo_save(cr);
    if (v.face) cairo_set_font_face(cr, v.face);
    else cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, TEXT_MM * mm);
    cairo_text_extents_t e;
    cairo_text_extents(cr, label.c_str(), &e);
    const double max_w = w - 12.0 * mm;
    if (e.width > max_w) {
        cairo_set_font_size(cr, TEXT_MM * mm * max_w / e.width);
        cairo_text_extents(cr, label.c_str(), &e);
    }
    const double tx = cx - 2.0 * mm - e.width / 2.0 - e.x_bearing;
    const double ty = cy - e.height / 2.0 - e.y_bearing;
    cairo_move_to(cr, tx, ty);
    cairo_text_path(cr, label.c_str());
    // caret at the right end of the window
    const double kx = cx + (w / 2.0) - 3.2 * mm;
    cairo_move_to(cr, kx - 1.3 * mm, cy - 0.7 * mm);
    cairo_line_to(cr, kx + 1.3 * mm, cy - 0.7 * mm);
    cairo_line_to(cr, kx, cy + 0.8 * mm);
    cairo_close_path(cr);
    cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
    set_hex(cr, g, 0.07);
    cairo_set_line_width(cr, 0.6 * mm);
    cairo_stroke_preserve(cr);
    set_hex(cr, g, 1.0);
    cairo_set_line_width(cr, DOT_STROKE_PX * f * 2.0);    // f * 2 = the UI scale
    cairo_stroke_preserve(cr);
    cairo_fill(cr);
    cairo_restore(cr);
}

// The whole panel, at `f` window pixels per asset pixel, onto `cr` in window pixels.
inline void draw_panel(cairo_t* cr, const Scaled& s, const View& v)
{
    if (s.f <= 0.0) {                       // assets missing: a flat placeholder, never a half panel
        cairo_set_source_rgb(cr, 0.086, 0.086, 0.094);
        cairo_paint(cr);
        return;
    }
    const double f = s.f;
    cairo_save(cr);
    // plates: bypass underneath, active on top at `lit`
    cairo_set_source_surface(cr, s.plate[1], 0, 0);
    cairo_paint(cr);
    if (v.lit > 0.0) {
        cairo_set_source_surface(cr, s.plate[0], 0, 0);
        cairo_paint_with_alpha(cr, std::min(v.lit, 1.0));
    }
    // knobs, each through a soft circular mask over its base circle
    const double values[3] = { v.drive, v.level, v.tone };
    for (int k = 0; k < 3; ++k) {
        const KnobGeo& kg = KNOBS[k];
        cairo_set_source_surface(cr, s.knob[k][size_t(knob_frame(kg, values[k]))], s.kx[k], s.ky[k]);
        const double r_in = (kg.r + 1.0) * f, r_out = (kg.r + 3.0) * f;
        cairo_pattern_t* m = cairo_pattern_create_radial(kg.cx * f, kg.cy * f, 0, kg.cx * f, kg.cy * f, r_out);
        cairo_pattern_add_color_stop_rgba(m, 0.0, 0, 0, 0, 1);
        cairo_pattern_add_color_stop_rgba(m, r_in / r_out, 0, 0, 0, 1);
        cairo_pattern_add_color_stop_rgba(m, 1.0, 0, 0, 0, 0);
        cairo_mask(cr, m);
        cairo_pattern_destroy(m);
    }
    // treadle
    const int ti = std::clamp(int(std::lround(v.treadle * (TREADLE_FRAMES - 1))), 0, TREADLE_FRAMES - 1);
    cairo_set_source_surface(cr, s.treadle[size_t(ti)], s.tx, s.ty);
    cairo_paint(cr);
    cairo_restore(cr);
    if (v.display_lit) draw_readout(cr, v, f);
}

// Hit tests in ASSET pixels (window pixels / f).
inline int knob_at(double ax, double ay)
{
    for (int k = 0; k < 3; ++k) {
        const double dx = ax - KNOBS[k].cx, dy = ay - KNOBS[k].cy;
        if (dx * dx + dy * dy <= KNOBS[k].r * KNOBS[k].r) return k;
    }
    return -1;
}

inline bool in_box(const Box& b, double ax, double ay)
{
    return ax >= b.x0 && ax <= b.x1 && ay >= b.y0 && ay <= b.y1;
}

}  // namespace photo
}  // namespace ui
}  // namespace nlsc
