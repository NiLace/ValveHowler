// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// The photoreal panel: pre-rendered Blender images composited with Cairo.
//
// No X in here, so the preview tool draws exactly what the host shows. The images
// are rendered at twice the logical size (HiDPI), and scaled once per scale factor
// into a cache, never per paint. Decoding and scaling run on a few threads, one image
// per job: each image is decoded or scaled by exactly the code of the serial path, so
// the result does not depend on how the jobs fall on the threads.
//
// Layers, back to front:
//   plate      the whole top for the variant's green and the `enabled` state, knobs
//              at rest and the variant display empty; a change of state crossfades
//   knobs      one frame of each knob's strip, pasted through a soft circular mask
//              over its base circle (the paint around it is the plate's)
//   treadle    one frame of the treadle strip (rest -> pressed)
//   hq         the HQ button, active state only (it lights only with the pedal on):
//              its pressed frames unlit, then its light over them (the `hq` port,
//              faded), both at the active plate's alpha; at rest and unlit the plate
//              shows it as it is
//   readout    the variant name and the caret, drawn live in the dot-matrix face
#pragma once

#include "ui_photo_layout.h"

#include <cairo.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace nlsc {
namespace ui {
namespace photo {

inline constexpr double LOGICAL_W = ASSET_W / 2.0;   // the window at scale 1
inline constexpr double LOGICAL_H = ASSET_H / 2.0;
inline constexpr int GREEN_VH9 = 0, GREEN_VH8 = 1;
inline constexpr const char* GREEN_NAME[2] = { "vh9", "vh8" };
inline constexpr unsigned GREEN_HEX[2] = { 0x6BC75C, 0x48B180 };   // the box paints
inline constexpr unsigned BACKDROP_HEX = 0x161618;   // around the pedal: the colour its renders fade to
// The lit readout's dots as the scene renders them (median of the lit pixels).
inline constexpr unsigned READOUT_HEX[2] = { 0x91FF7E, 0x6AF4B3 };
inline constexpr double KNOB_MIN_DEG = -135.0, KNOB_MAX_DEG = 135.0; // the scale dots' travel
inline constexpr double FADE_S = 0.120;              // plate crossfade on bypass
inline constexpr double TREADLE_S = 0.080;           // treadle travel, down or up
inline constexpr double HQ_PRESS_S = 0.050;          // the HQ button's travel, down or up
inline constexpr double HQ_GLOW_S = 0.100;           // the HQ light's fade, on or off

// The circuit decides the paint: the VH-8 rows are VH-8 green, the VH-9 rows VH-9 green.
inline int green_of_circuit(int circuit) { return circuit == 0 ? GREEN_VH8 : GREEN_VH9; }

// --- a few threads over a list of independent jobs -------------------------------

inline constexpr unsigned MAX_THREADS = 6;

// Runs job(0..n-1), each once, on up to MAX_THREADS threads (the caller's among them), and
// returns when all are done. The jobs are taken in list order, so a list puts its longest
// first. A thread that cannot be created leaves its share to the others; `threaded` false
// runs them all on the caller's thread (the reference path of the gates).
inline void run_jobs(int n, const std::function<void(int)>& job, bool threaded = true)
{
    std::atomic<int> next{ 0 };
    auto worker = [&] { for (int i; (i = next.fetch_add(1)) < n;) job(i); };
    unsigned nt = threaded ? std::min(std::max(1u, std::thread::hardware_concurrency()), MAX_THREADS) : 1u;
    nt = std::min(nt, unsigned(std::max(n, 1)));
    std::vector<std::thread> t;
    for (unsigned i = 1; i < nt; ++i) {
        try { t.emplace_back(worker); } catch (...) { break; }
    }
    worker();
    for (auto& th : t) th.join();
}

// --- the images, as loaded from the bundle --------------------------------------

// PNG decodes since the process started: the gates read it to tell a decode from a cache hit.
inline std::atomic<unsigned> g_png_decodes{ 0 };

// One green at a time: only one is ever on screen, and decoding both would cost ~60 MB when
// opening the window. The other loads when the variant changes circuit.
struct Assets {
    int green = -1;
    cairo_surface_t* plate[2] = {};      // [0 active, 1 bypass]
    cairo_surface_t* knob[3]  = {};      // [drive, level, tone]: frames stacked vertically
    cairo_surface_t* treadle  = nullptr;
    cairo_surface_t* hq[2]    = {};      // [unlit, lit]: frames rest -> pressed, the active state's
    bool ok = false;
};

inline cairo_surface_t* load_png(const std::string& path)
{
    g_png_decodes.fetch_add(1);
    cairo_surface_t* s = cairo_image_surface_create_from_png(path.c_str());
    if (cairo_surface_status(s) != CAIRO_STATUS_SUCCESS) { cairo_surface_destroy(s); return nullptr; }
    return s;
}

inline void free_assets(Assets& a)
{
    for (auto* s : a.plate) if (s) cairo_surface_destroy(s);
    for (auto* s : a.knob)  if (s) cairo_surface_destroy(s);
    if (a.treadle) cairo_surface_destroy(a.treadle);
    for (auto* s : a.hq)    if (s) cairo_surface_destroy(s);
    a = Assets{};
}

// One image to decode: its file, where it goes, and the size it must have.
struct LoadJob {
    std::string name;
    cairo_surface_t** dst;
    int w, h;
};

// Every image of green `g`, into `a`'s slots. The knob strips are most of the pixels, so they
// come first, then the plates. The knobs and the treadle are the same in both greens (one set of
// files) and are left out when `share` (the caller takes them from the other green's set).
inline std::vector<LoadJob> load_list(Assets& a, int g, bool share = false)
{
    std::vector<LoadJob> l;
    if (!share)
        for (int k = 0; k < 3; ++k) {
            const KnobGeo& kg = KNOBS[k];
            l.push_back({ std::string("knob_") + kg.name + ".png", &a.knob[k],
                          kg.x1 - kg.x0, (kg.y1 - kg.y0) * kg.frames });
        }
    for (int s = 0; s < 2; ++s)
        l.push_back({ std::string("plate_") + GREEN_NAME[g] + (s ? "_bypass" : "_active") + ".png", &a.plate[s],
                      int(ASSET_W), int(ASSET_H) });
    for (int h = 0; h < 2; ++h)
        l.push_back({ std::string("hq_") + GREEN_NAME[g] + (h ? "_lit" : "_active") + ".png", &a.hq[h],
                      HQ_X1 - HQ_X0, (HQ_Y1 - HQ_Y0) * HQ_FRAMES });
    if (!share)
        l.push_back({ "treadle.png", &a.treadle, TREADLE_X1 - TREADLE_X0, (TREADLE_Y1 - TREADLE_Y0) * TREADLE_FRAMES });
    return l;
}

inline bool sized(cairo_surface_t* s, int w, int h)
{
    return s && cairo_image_surface_get_width(s) == w && cairo_image_surface_get_height(s) == h;
}

// `dir` is the bundle's `photo/` directory. A missing or unreadable image leaves `ok` false and
// the UI draws a plain placeholder: a panel with a hole in it would look like the plugin.
// `other`, when it holds the other green's images, lends its knobs and treadle (a second
// reference to the same surfaces) instead of decoding them again.
inline Assets load_assets(const std::string& dir, int g, const Assets* other = nullptr, bool threaded = true)
{
    Assets a;
    a.green = g;
    const bool share = other && other->ok && other->treadle;
    const std::vector<LoadJob> l = load_list(a, g, share);
    run_jobs(int(l.size()), [&](int i) { *l[size_t(i)].dst = load_png(dir + "/" + l[size_t(i)].name); }, threaded);
    if (share) {
        a.treadle = cairo_surface_reference(other->treadle);
        for (int k = 0; k < 3; ++k) a.knob[k] = cairo_surface_reference(other->knob[k]);
    }
    bool ok = sized(a.treadle, TREADLE_X1 - TREADLE_X0, (TREADLE_Y1 - TREADLE_Y0) * TREADLE_FRAMES);
    for (int k = 0; k < 3; ++k)
        ok &= sized(a.knob[k], KNOBS[k].x1 - KNOBS[k].x0, (KNOBS[k].y1 - KNOBS[k].y0) * KNOBS[k].frames);
    for (const LoadJob& j : l) ok &= sized(*j.dst, j.w, j.h);
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
    std::vector<cairo_surface_t*> hq[2];
    int kx[3] = {}, ky[3] = {};          // integer window position of each knob frame
    int tx = 0, ty = 0;
    int hx = 0, hy = 0;
};

inline void free_scaled(Scaled& s)
{
    for (auto*& p : s.plate) if (p) cairo_surface_destroy(p);
    for (auto& v : s.knob) for (auto* p : v) cairo_surface_destroy(p);
    for (auto* p : s.treadle) cairo_surface_destroy(p);
    for (auto& v : s.hq) for (auto* p : v) cairo_surface_destroy(p);
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

// The longest jobs first, as in load_assets: the knobs' 64 frames each, then the plates. Each
// job reads its own source image and writes its own surfaces: no cairo object is shared
// between two threads.
inline void ensure_scaled(Scaled& s, const Assets& a, double f, bool threaded = true)
{
    if (!a.ok || (s.f == f && s.green == a.green)) return;
    free_scaled(s);
    s.f = f; s.green = a.green;
    const int W = int(std::ceil(ASSET_W * f)), H = int(std::ceil(ASSET_H * f));
    for (int k = 0; k < 3; ++k) {
        s.kx[k] = int(std::floor(KNOBS[k].x0 * f)); s.ky[k] = int(std::floor(KNOBS[k].y0 * f));
    }
    s.tx = int(std::floor(TREADLE_X0 * f)); s.ty = int(std::floor(TREADLE_Y0 * f));
    s.hx = int(std::floor(HQ_X0 * f)); s.hy = int(std::floor(HQ_Y0 * f));
    auto job = [&](int i) {
        if (i < 3) {
            const int k = i;
            const KnobGeo& kg = KNOBS[k];
            const int w = kg.x1 - kg.x0, h = kg.y1 - kg.y0;
            const int pw = int(std::ceil(kg.x1 * f)) - s.kx[k], ph = int(std::ceil(kg.y1 * f)) - s.ky[k];
            for (int fr = 0; fr < kg.frames; ++fr)
                s.knob[k].push_back(resample(a.knob[k], 0, double(fr * h), w, h, kg.x0, kg.y0,
                                             f, s.kx[k], s.ky[k], pw, ph));
        } else if (i < 5) {
            const int st = i - 3;
            s.plate[st] = resample(a.plate[st], 0, 0, ASSET_W, ASSET_H, 0, 0, f, 0, 0, W, H);
        } else if (i >= 6) {
            const int h = i - 6;
            const int w = HQ_X1 - HQ_X0, fh = HQ_Y1 - HQ_Y0;
            const int pw = int(std::ceil(HQ_X1 * f)) - s.hx, ph = int(std::ceil(HQ_Y1 * f)) - s.hy;
            for (int fr = 0; fr < HQ_FRAMES; ++fr)
                s.hq[h].push_back(resample(a.hq[h], 0, double(fr * fh), w, fh, HQ_X0, HQ_Y0,
                                           f, s.hx, s.hy, pw, ph));
        } else {
            const int tw = TREADLE_X1 - TREADLE_X0, th = TREADLE_Y1 - TREADLE_Y0;
            const int pw = int(std::ceil(TREADLE_X1 * f)) - s.tx, ph = int(std::ceil(TREADLE_Y1 * f)) - s.ty;
            for (int fr = 0; fr < TREADLE_FRAMES; ++fr)
                s.treadle.push_back(resample(a.treadle, 0, double(fr * th), tw, th, TREADLE_X0, TREADLE_Y0,
                                             f, s.tx, s.ty, pw, ph));
        }
    };
    run_jobs(8, job, threaded);
}

// --- what is drawn ---------------------------------------------------------------

struct View {
    double drive = 0.5, level = 0.5, tone = 0.5;   // knob positions, 0..1
    int    green = GREEN_VH9;
    double lit = 1.0;          // 1 active plate, 0 bypass plate; in between while crossfading
    double treadle = 0.0;      // 0 rest .. 1 pressed
    double hq_press = 0.0;     // the HQ button: 0 rest .. 1 pressed
    double hq_glow = 0.0;      // its light: 0 off .. 1 on (the `hq` port, faded)
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
// display (ink height 21 asset px on the VH-9 plate), not chosen by eye.
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
    // HQ, active state only: its pressed frames unlit, then its light over them
    {
        const int hf = std::clamp(int(std::lround(v.hq_press * (HQ_FRAMES - 1))), 0, HQ_FRAMES - 1);
        const double a = std::min(std::max(v.lit, 0.0), 1.0);
        if (a > 0.0 && hf > 0) {              // at rest the plate shows it unlit
            cairo_set_source_surface(cr, s.hq[0][size_t(hf)], s.hx, s.hy);
            cairo_paint_with_alpha(cr, a);
        }
        const double gl = std::min(std::max(v.hq_glow, 0.0), 1.0);
        if (a > 0.0 && gl > 0.0) {
            cairo_set_source_surface(cr, s.hq[1][size_t(hf)], s.hx, s.hy);
            cairo_paint_with_alpha(cr, a * gl);
        }
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
