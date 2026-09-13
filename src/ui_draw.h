// ui_draw.h — Valve Howler's front panel, drawn in pure Cairo.
//
// A BY-HAND port of the design handoff
// (`gui/The Omega Cake Pedal Design/design_handoff_nls_cream/PedalNLSCream.jsx`)
// to Cairo calls. No SVG renderer in between, so the GUI drags no
// dependency beyond Cairo (+ FreeType for the embedded fonts, see
// `ui_fonts.h`).
//
// The coordinate system is THE SAME as the SVG viewBox: 400 × 700, origin
// top-left, growing downwards. The caller scales the context to map that
// nominal size onto the real surface.
//
// SAME ARCHITECTURE as `TheOmegaCake/src/ui_draw.h`, and the handoff
// itself says it: "the knobs are interchangeable between the two pedals'
// codebases". What is NOT shared is the file: two pedals with two looks,
// and a common file would end up an `if (pedal)` in every layer.
//
// THE KNOB CONTRACT, literal from the handoff: value 0..1 -> rotation
// −150°..+150° (300° of travel, classic potentiometer). THE LIGHTING IS
// STATIC: only the knurled skirt and the index rotate. That is what makes
// it look like a real knob and not a spinning decal.
//
// ─────────────────────────────────────────────────────────────────────────────
// TWO LAYERS, AND IT IS NO PREMATURE OPTIMISATION: IT IS A GATE
// ─────────────────────────────────────────────────────────────────────────────
// The 0C-family GUIs FROZE Ardour, and the cause is measured, not
// assumed: repainting the WHOLE panel on every port event cost 12-20 ms
// per frame => ~100 % of a core per open window. Here it would be worse
// still: this panel's backdrop carries TWO window-sized noise masks.
//
//   `draw_static()` — box, enamel, grain, wear, silkscreen, LED,
//                     footswitch, dropdown, and of each knob whatever sits
//                     UNDER the knurl (marks, contact shadow, base). Drawn
//                     ONCE to a surface and stamped. Depends only on
//                     (size, `on`, `pressed`, `variant`) => that is the
//                     cache KEY.
//   `draw_live()`   — only the three knobs from the waist up: rotating
//                     knurl, skirt, cap and index.
//
// The cache key carries EXACTLY what the static layer draws, no more,
// no less. Too much rebuilds the expensive backdrop for a pixel that
// never changes; too little leaves the panel FROZEN at the old value,
// which reads as an unresponsive control. Re-audit the key whenever
// something changes layer.
//
// `draw_panel()` = static + live, and exists so the PNG preview uses
// the same path as the host. It bit once already: the chassis moved into
// the cache, the host was updated, the preview was forgotten => PNGs
// shipped without the chassis for several commits, silently.

#pragma once

#include "ui_fonts.h"

#include <cairo/cairo.h>
#include "ui_tokens.h"
#include "ui_logo.h"
#include <cmath>
#include <cstdint>
#include <cstring>

// GRAIN parameters, exposed so they can be SWEPT. The handoff asks for
// `feTurbulence baseFrequency="0.9"` + alpha 0,05, but `feTurbulence` is
// Perlin noise with the SVG standard's exact generator and this is
// value-noise of its own: copying the two CSS numbers does not reproduce
// the texture. What gets matched is the STATISTICS (σ and
// autocorrelation) against the prototype's render, and for that both
// parameters must move and be measured.
// Same lesson as "`mix-blend-mode` ≠ direct alpha": when the token
// carries a mechanism Cairo lacks, its number is NOT transferable.
#ifndef NLSC_GRAIN_PERIOD
#define NLSC_GRAIN_PERIOD 1.111    // panel units (1/0,9)
#endif
#ifndef NLSC_GRAIN_ALPHA
#define NLSC_GRAIN_ALPHA 0.045
#endif

namespace nlsc {
namespace ui {

// Nominal panel size, as the SVG viewBox.
inline constexpr double PANEL_W = 400.0;
inline constexpr double PANEL_H = 700.0;

// Knob travel (handoff): angle = −150 + 300·value.
inline constexpr double KNOB_MIN_DEG = -150.0;
inline constexpr double KNOB_MAX_DEG =  150.0;

// Geometry of the three knobs and the footswitch, in panel units. Lives
// here and not in the X11 code: the mouse hit map must come from THE SAME
// source as the drawing, or a knob paints in one place and grabs in
// another.
// THE KNOB GROUP MOVES AS ONE. It is a single offset rather than three new
// coordinates: typing three numbers by hand is exactly how a relation gets
// broken by accident, and nobody would see it — the panel would still draw.
//
// Because the mouse map comes off these same constants (see below), the
// hit-testing moves with it. Anyone who shifts the knobs with a `translate` in
// the drawing instead of touching this gets knobs that look up here and respond
// down there.
// The LABELS are not here, so they add the same `KNOBS_DY` by hand on their own
// line. They are the one piece of the group that does not travel by itself.
//
// Margin: the graduation marks reach `DRIVE_CY − (R+12)` and the body starts at
// y=12 => with DY = −22 they sit at y=32, i.e. 20 px of edge.
// It is declared as HOW FAR IT RISES rather than as a signed offset: a `-D`
// carrying parentheses and a minus sign does not survive the shell when
// sweeping candidates, and a sweep that fails to compile is obvious — one that
// compiles the SAME binary four times is not.

inline constexpr double KNOBS_DY = -(NLSC_KNOBS_UP);

inline constexpr double DRIVE_CX = 102.0, DRIVE_CY = 112.0 + KNOBS_DY, DRIVE_R = 46.0;
inline constexpr double LEVEL_CX = 298.0, LEVEL_CY = 112.0 + KNOBS_DY, LEVEL_R = 46.0;
inline constexpr double TONE_CX  = 200.0, TONE_CY  = 218.0 + KNOBS_DY, TONE_R  = 40.0;

// The footswitch: centre (200,494), the whole assembly scaled ×1,12
// about its own centre => the outer ring's effective radius is 46·1,12.
// There is no `FS_CX/FS_CY/FS_SCALE/FS_R_HIT`: they were the centre and radius
// of a round stomp switch, and the plate is described by `FS_WELL_*` and
// `FS_PL_*` below. They were deleted rather than left behind: a constant that
// no longer describes anything is the one the next reader uses by mistake, and
// it compiles just the same.

// The variant dropdown (the SVG's `foreignObject`, its inner `select`
// measuring 290×38).
// The well is anchored by its BOTTOM edge (602), not its top: the dropdown sits
// right below it and does not move, while above it is the logo, which does want
// to grow. That makes `NLSC_WELL_H` a lever on space rather than a change of
// position.
inline constexpr double FS_WELL_BOTTOM = 602.0;
inline constexpr double FS_WELL_H = NLSC_WELL_H;
inline constexpr double FS_WELL_X = 56.0,  FS_WELL_Y = FS_WELL_BOTTOM - FS_WELL_H;
inline constexpr double FS_WELL_W = 288.0;
inline constexpr double FS_GAP    = 11.0;   // holgura pletina-pozo
inline constexpr double FS_PL_X = FS_WELL_X + FS_GAP, FS_PL_Y = FS_WELL_Y + FS_GAP;
inline constexpr double FS_PL_W = FS_WELL_W - 2 * FS_GAP, FS_PL_H = FS_WELL_H - 2 * FS_GAP;

inline constexpr double DD_X = 55.0, DD_Y = 622.0, DD_W = 290.0, DD_H = 38.0;

// The variant name and its rules: the big italic silkscreen.
inline constexpr double VAR_BASELINE = 600.0;
inline constexpr double VAR_LINEA_Y  = 587.0;

// THE PEDAL FLOATS, NOT EDGE-TO-EDGE — an EXPRESS preference: "the
// panel floating over the backdrop with drop shadow and rounded corners,
// NOT edge-to-edge. A `ui_preview` that fills to the border looks flat
// next to the reference".
//
// How to achieve it is also spelled out, because the first attempt got
// it wrong: GROW THE WINDOW, never shrink the panel. The pedal still
// measures exactly 400×700; what grows is the gap around it.
//
// It applies in the HOST, not just the capture. So mouse coordinates
// carry the margin subtracted — forget it and the knobs grab displaced.
//
// The backdrop is the approved prototype page's own
// (`radial-gradient(ellipse at 50% 30%, #26292e 0%, #131518 70%)`): not
// invented — it is what the designer put behind the pedal.
inline constexpr double UI_MARGIN = 18.0;
inline constexpr double WINDOW_W = PANEL_W + 2 * UI_MARGIN;
inline constexpr double WINDOW_H = PANEL_H + 2 * UI_MARGIN;

// ---------------------------------------------------------------------------
// Theme: every panel colour in one place. Changing the struct repaints
// the whole pedal (the handoff already anticipates per-variant re-skins).
// Colours are 0xRRGGBB; opacities go separately, where the SVG used
// `opacity`. `default_theme()` == the handoff's «NLS Cream» tokens.
// ---------------------------------------------------------------------------
struct Theme {
    // Chassis and painted body (enamelled green).
    unsigned rim;                                  // painted aluminium rim
    unsigned body0, body1, body2, body3;           // body radial, 4 stops
    unsigned edge_luz, edge_sombra;                // left and right edges
    unsigned filo;                                 // light border edge
    unsigned filo_oscuro;                          // dark inner line
    unsigned desgaste;                             // wear patches

    unsigned print;                                // all the silkscreen
    unsigned print_on;                             // the NAME while `enabled` is on
    unsigned print_glow;                           // its halo
    unsigned rule_on;                              // the two rules while lit

    // LED.
    unsigned led0, led1, led2;                     // lit dome
    unsigned led_off0, led_off1;                   // unlit dome
    unsigned led_glow;                             // halo (lit only)

    // Knob: skirt, knurl, metal cap, index.
    unsigned skirt0, skirt1, skirt2, skirt3;
    unsigned rib0, rib1, rib2, rib3;
    unsigned cap0, cap1, cap2, cap3;
    unsigned indicador;

    // Footswitch: washer, hex chrome, thread, cap.
    unsigned washer0, washer1, washer2;
    unsigned chrome0, chrome1, chrome2, chrome3;
    unsigned thread0, thread1, thread2, thread3;
    unsigned capflat0, capflat1, capflat2, capflat3;

    // Dropdown.
    unsigned drop_top, drop_bot, drop_texto;
};

inline Theme default_theme()
{
    Theme t{};
    t.rim = 0x0e2f17;
    t.body0 = 0x63d472; t.body1 = 0x42b256; t.body2 = 0x2e8440; t.body3 = 0x1d5c2c;
    t.edge_luz = 0xeafff0; t.edge_sombra = 0x062810;
    t.filo = 0xeafff0; t.filo_oscuro = 0x0b3517;
    t.desgaste = 0xdfe8d9;
    t.print = 0xf2f5ec;
    // THE LIT NAME IS BLUE, NOT RED. Red on green is the worst pair there is
    // for protanopia and deuteranopia, so the family's POWER token (`#ff4a3a`)
    // is unusable against the green chassis. The blue comes from the Okabe-Ito
    // colour-blind-safe palette, which this family already uses for its meters.
    // The five state tokens live in `ui_tokens.h`, with no dependencies, so
    // that `make decisiones` can publish the value actually compiled.
    t.print_on   = NLSC_NAME_ON;
    t.print_glow = NLSC_NAME_GLOW;
    t.rule_on    = NLSC_RULE_ON;
    t.led0 = 0xffd9d4; t.led1 = 0xff5040; t.led2 = 0xa80f06;
    t.led_off0 = 0x7a2a24; t.led_off1 = 0x3c0f0b;
    t.led_glow = 0xff4a38;
    t.skirt0 = 0x3c3e42; t.skirt1 = 0x232529; t.skirt2 = 0x121316; t.skirt3 = 0x050607;
    t.rib0 = 0x040405; t.rib1 = 0x2e3034; t.rib2 = 0x3a3c41; t.rib3 = 0x08090a;
    t.cap0 = 0xeef0f3; t.cap1 = 0xc4c8ce; t.cap2 = 0x8e939b; t.cap3 = 0x61666e;
    t.indicador = 0xf4f6f2;
    t.washer0 = 0xffffff; t.washer1 = 0xf7f6f1; t.washer2 = 0xdcdad2;
    t.chrome0 = 0xf8f9fb; t.chrome1 = 0xc9cdd6; t.chrome2 = 0x82878f; t.chrome3 = 0x41454d;
    t.thread0 = 0x5a5e65; t.thread1 = 0x33363b; t.thread2 = 0x1e2024; t.thread3 = 0x43464c;
    t.capflat0 = 0xe3e5e8; t.capflat1 = 0xc2c6cb; t.capflat2 = 0xa9adb4; t.capflat3 = 0x989ca3;
    t.drop_top = 0x1c5f2c; t.drop_bot = 0x123f1d; t.drop_texto = 0xeaf5e6;
    return t;
}

// --- colour and utilities ---------------------------------------------------

inline void set_rgb(cairo_t* cr, unsigned hex)
{
    cairo_set_source_rgb(cr, ((hex >> 16) & 0xff) / 255.0,
                             ((hex >> 8)  & 0xff) / 255.0,
                             ( hex        & 0xff) / 255.0);
}

inline void set_rgba(cairo_t* cr, unsigned hex, double a)
{
    cairo_set_source_rgba(cr, ((hex >> 16) & 0xff) / 255.0,
                              ((hex >> 8)  & 0xff) / 255.0,
                              ( hex        & 0xff) / 255.0, a);
}

// LOGO HALO. The logo is a MASK, not text, so the trick of fattening the
// stroke does not apply: the same mask is stacked, offset, in RINGS around its
// place, from the widest and faintest inwards. Same principle as the name's
// halo — Cairo does not blur, it is faked by stacking — but with translation
// instead of line width.
//
// Eight positions per ring: with four the cross shows, and with sixteen it
// costs twice as much with no visible difference.
inline void draw_logo_bold(cairo_t* cr, cairo_surface_t* logo,
                           double x, double y, double lado,
                           unsigned color, double alfa, double grosor);

// Paints the logo fitted into a square, keeping its aspect ratio.
inline void draw_logo(cairo_t* cr, cairo_surface_t* logo,
                      double x, double y, double lado, unsigned color, double alfa)
{
    if (!logo) return;
    const double iw = cairo_image_surface_get_width(logo);
    const double ih = cairo_image_surface_get_height(logo);
    if (iw <= 0 || ih <= 0) return;
    // Fitted by the LARGER side and not centred: because the mask arrives
    // cropped to the ink, `x`/`y` are the top-left corner OF THE STROKE and
    // `lado` its larger dimension. Centring here would put invisible slack back
    // in, which is exactly the defect the crop just removed.
    const double k = (iw >= ih) ? lado / iw : lado / ih;
    cairo_save(cr);
    cairo_translate(cr, x, y);
    cairo_scale(cr, k, k);
    cairo_pattern_t* pat = cairo_pattern_create_for_surface(logo);
    cairo_pattern_set_filter(pat, CAIRO_FILTER_GOOD);
    set_rgba(cr, color, alfa);
    cairo_mask(cr, pat);
    cairo_pattern_destroy(pat);
    cairo_restore(cr);
}

// FATTENING THE STROKE, which is NOT a halo. The logo is a thin-line mask and
// at panel size it gets lost against the brushed chassis. Drawing it eight times
// offset by a HAIR and at FULL alpha DILATES the stroke: the line thickens, the
// drawing does not smear and the contrast rises. The difference from the halo is
// the alpha: there it decays, here it does not — there is light around it there,
// thickness here.
inline void draw_logo_bold(cairo_t* cr, cairo_surface_t* logo,
                           double x, double y, double lado,
                           unsigned color, double alfa, double grosor)
{
    if (!logo) return;
    if (grosor > 0.0) {
        for (int k = 0; k < 8; ++k) {
            const double ang = k * (M_PI / 4.0);
            draw_logo(cr, logo, x + std::cos(ang) * grosor, y + std::sin(ang) * grosor,
                      lado, color, alfa);
        }
    }
    draw_logo(cr, logo, x, y, lado, color, alfa);
}

// THE LOGO IS A BACKGROUND LAYER, and that is what unblocks its size. While it
// was drawn INSIDE the identity block it was capped by that band — the TONE
// marks above, the footswitch well below — and stuck at 162 px: asking for 174
// produced a render byte-identical to the one at 162. Drawing it BEFORE the
// knobs stops it competing with them: it grows upwards and the knobs pass over
// it, which is what a separate layer means.
//
// It is anchored at the BOTTOM rather than at its centre: the lower edge is the
// one with a hard neighbour (the well), so "bigger" means raising the top edge
// and leaving the bottom still.
// TOP edge of the logo box, in ONE place. It is used by the logo drawing and by
// the vertical centring of the text, which is aligned with the logo's centre:
// if each computed it separately, changing the size would part them with nobody
// seeing it.
inline constexpr double logo_top() { return FS_WELL_Y - NLSC_BAND_MARGIN - NLSC_LOGO_SIDE; }
inline constexpr double logo_cy()  { return logo_top() + NLSC_LOGO_SIDE * 0.5; }

// THE STROKE'S REAL WIDTH, whose absence left 74 px of PHANTOM AIR.
// `NLSC_LOGO_SIDE` is the LARGER side, and this drawing is taller than it is
// wide (676x882 of ink) => at 186 tall it is **142 wide**. Computing the text's
// box from 186 pushed it 44 px to the right of where the drawing ends, and the
// text came out small with no visible reason: the space it thought it had was
// not the space it had.
// A constant cannot fix this because it depends on the FILE: a different logo
// has a different aspect ratio. It is measured off the mask.
inline double logo_ancho(cairo_surface_t* logo)
{
    if (!logo) return NLSC_LOGO_SIDE;
    const double iw = cairo_image_surface_get_width(logo);
    const double ih = cairo_image_surface_get_height(logo);
    if (iw <= 0 || ih <= 0) return NLSC_LOGO_SIDE;
    const double k = (iw >= ih) ? NLSC_LOGO_SIDE / iw : NLSC_LOGO_SIDE / ih;
    return iw * k;
}

inline void draw_logo_capa(cairo_t* cr, cairo_surface_t* logo,
                           unsigned color, double alfa)
{
    // The character is drawn at `FILL` of its box and CENTRED in it, so its
    // centre stays `logo_cy()` — which is what the text aligns to. Anchoring it
    // at the bottom would throw it out against the lettering.
    // Horizontally it is flush LEFT in the box, not centred: centring left half
    // the spare air on each side — with `FILL` at 0,84 that is 15 px of nothing
    // on the left — and it also stopped `NLSC_LOGO_X` being the edge of the
    // stroke, which is what it claims to be.
    // Vertically it IS centred, and that is not an inconsistency. The box's
    // vertical centre is `logo_cy()`, which the text aligns to; off-centring
    // there would break that pair, while horizontally there is nothing to
    // align.
    const double lado = NLSC_LOGO_SIDE * NLSC_LOGO_FILL;
    const double sobra = NLSC_LOGO_SIDE - lado;
    draw_logo_bold(cr, logo, NLSC_LOGO_X, logo_top() + sobra * 0.5,
                   lado, color, alfa, NLSC_LOGO_BOLD);
}


inline void add_stop(cairo_pattern_t* p, double off, unsigned hex, double a = 1.0)
{
    cairo_pattern_add_color_stop_rgba(p, off, ((hex >> 16) & 0xff) / 255.0,
                                              ((hex >> 8)  & 0xff) / 255.0,
                                              ( hex        & 0xff) / 255.0, a);
}

// Rounded rectangle (equivalent to the SVG's `rect rx=ry=r`).
inline void rounded_rect(cairo_t* cr, double x, double y, double w, double h, double r)
{
    const double k = M_PI / 180.0;
    if (r <= 0.0) { cairo_rectangle(cr, x, y, w, h); return; }
    cairo_new_sub_path(cr);
    cairo_arc(cr, x + w - r, y + r,     r, -90 * k,   0 * k);
    cairo_arc(cr, x + w - r, y + h - r, r,   0 * k,  90 * k);
    cairo_arc(cr, x + r,     y + h - r, r,  90 * k, 180 * k);
    cairo_arc(cr, x + r,     y + r,     r, 180 * k, 270 * k);
    cairo_close_path(cr);
}

// SVG GRADIENTS USE `objectBoundingBox`, AND THAT IS NO DETAIL.
//
// In that system the object's box is a UNIT SQUARE, so a radial gradient
// with `r=105%` over a 372×676 box is not a circle: it is a 390×710
// ELLIPSE. Cairo only does circles, so the anisotropy enters through the
// pattern's MATRIX.
//
// This function's version in `TheOmegaCake/src/ui_draw.h` does
// `rr = fr*bw` — i.e. assumes a square box. There it holds (its gradients
// sit on circles); here it does NOT: the pedal body is nearly twice as
// tall as wide, and with that approximation the green gradient comes out
// visibly wrong.
//
// The matrix maps user space -> pattern space:
//     (x,y) -> ((x−bx)/bw, (y−by)/bh)
inline void obb_pattern(cairo_pattern_t* p, double bx, double by, double bw, double bh)
{
    cairo_matrix_t m;
    cairo_matrix_init(&m, 1.0 / bw, 0.0, 0.0, 1.0 / bh, -bx / bw, -by / bh);
    cairo_pattern_set_matrix(p, &m);
}

// Radial in `objectBoundingBox`: focus = centre (the SVG default).
inline cairo_pattern_t* radial_obb(double bx, double by, double bw, double bh,
                                   double fcx, double fcy, double fr)
{
    cairo_pattern_t* p = cairo_pattern_create_radial(fcx, fcy, 0.0, fcx, fcy, fr);
    obb_pattern(p, bx, by, bw, bh);
    return p;
}

// Linear in `objectBoundingBox` (x1,y1)->(x2,y2), all in [0,1].
inline cairo_pattern_t* linear_obb(double bx, double by, double bw, double bh,
                                    double x1, double y1, double x2, double y2)
{
    cairo_pattern_t* p = cairo_pattern_create_linear(x1, y1, x2, y2);
    obb_pattern(p, bx, by, bw, bh);
    return p;
}

// --- text -------------------------------------------------------------------
//
// Drawn with the embedded FreeType faces (`ui_fonts.h`) so the panel
// looks the same in any host. If a face is missing, `face` is null and it
// falls back to a Cairo "toy" font at the requested family/weight.

// NO HINTING — NEITHER METRICS NOR OUTLINE. It is what matches the
// typography to the prototype, and it came from LOOKING at the diff, not
// from knowing it:
//
//   · Cairo by default applies FULL hinting: it snaps vertical strokes to
//     the pixel grid. Glyphs come out thicker and crunchier than the
//     browser's, which renders unhinted. Visible zooming the TONE label.
//   · `hint_metrics` ON rounds each glyph's ADVANCE to an integer. With
//     manual letter-spacing that ACCUMULATES: 16 glyphs of "SMOOTH
//     OVERDRIVE" gave 221,5 units of width against the prototype's 220,5,
//     spreading half a pixel to each end.
//
// The panel draws at a fixed, known size, not at 9 px in a list: here
// hinting buys no legibility, it only drifts from the design.
inline void opciones_de_fuente(cairo_t* cr)
{
    cairo_font_options_t* o = cairo_font_options_create();
    cairo_get_font_options(cr, o);
    cairo_font_options_set_hint_style(o, CAIRO_HINT_STYLE_NONE);
    cairo_font_options_set_hint_metrics(o, CAIRO_HINT_METRICS_OFF);
    cairo_font_options_set_antialias(o, CAIRO_ANTIALIAS_GRAY);
    cairo_set_font_options(cr, o);
    cairo_font_options_destroy(o);
}

inline void select_font(cairo_t* cr, cairo_font_face_t* face,
                        const char* familia, cairo_font_slant_t inclin,
                        cairo_font_weight_t peso, double size)
{
    opciones_de_fuente(cr);
    if (face) cairo_set_font_face(cr, face);
    else      cairo_select_font_face(cr, familia, inclin, peso);
    cairo_set_font_size(cr, size);
}

// Total width of an ASCII string with `spacing` between glyphs (no
// trailing gap).
inline double spaced_width(cairo_t* cr, const char* s, double spacing)
{
    const size_t n = std::strlen(s);
    double total = 0.0;
    for (size_t i = 0; i < n; ++i) {
        char ch[2] = { s[i], 0 };
        cairo_text_extents_t e;
        cairo_text_extents(cr, ch, &e);
        total += e.x_advance + (i + 1 < n ? spacing : 0.0);
    }
    return total;
}

// Centred text with MANUAL letter-spacing: Cairo's text API lacks it and
// the design depends on it. Drawn glyph by glyph. ASCII (byte by byte),
// which is all this panel's labels use.
//
// `x_svg` IS THE SVG ANCHOR, NOT THE OPTICAL CENTRE — and the
// difference was MEASURED (diff against the approved prototype):
//
//     "SMOOTH OVERDRIVE" ... 19,44 / 255 mean difference, with the SAME
//                            font, SAME size and SAME spacing
//
// The cause: `letter-spacing` adds the gap BEHIND the last glyph too, and
// `text-anchor="middle"` centres the width WITH that gap inside => the
// text lands half a spacing to the LEFT. Centring optically — the
// "correct" way — breaks the design: at spacing 6 that is 3 px of drift.
//
// Doing it this way is what makes the handoff table DIRECTLY USABLE.
// The designer ALREADY compensated where true optical centring was
// wanted: knob labels carry `x = cx + spacing/2` (102 -> 103,75), which
// under this rule gives exactly the optical centre. Labels at bare
// `x = 200` — the name, the subtitle, the variant — land displaced ON
// PURPOSE. => Pass the handoff's `x` values as-is and do NOT "fix" them.
inline void text_spaced(cairo_t* cr, const char* s, double x_svg, double baseline,
                        double size, double spacing, unsigned hex, double a,
                        cairo_font_face_t* face, const char* familia,
                        cairo_font_slant_t inclin, cairo_font_weight_t peso)
{
    select_font(cr, face, familia, inclin, peso, size);
    const double total = spaced_width(cr, s, spacing) + spacing;   // + trailing gap
    double pen = x_svg - total / 2.0;
    set_rgba(cr, hex, a);
    const size_t n = std::strlen(s);
    for (size_t i = 0; i < n; ++i) {
        char ch[2] = { s[i], 0 };
        cairo_text_extents_t e;
        cairo_text_extents(cr, ch, &e);
        cairo_move_to(cr, pen, baseline);
        cairo_show_text(cr, ch);
        pen += e.x_advance + spacing;
    }
}

// THE BODY THAT FILLS A WIDTH, growing OR shrinking.
//
// `fit_size` only SHRINKS: it keeps something from overflowing. This is the
// opposite, and it comes from two requirements that together stop being two
// adjustments by eye and become an EQUATION: the lettering should be as large
// as possible, and the right edge of the two-line block must line up with the
// right edge of the well. The body is the one that makes the widest line
// measure exactly the distance from the text to that edge.
//
// => The text size is therefore TIED to the geometry: move the well or grow the
// logo and the letters recompute themselves instead of quietly going out of
// square.
//
// Width is nearly linear in the body, so one proportional step and three of
// fine adjustment are enough; it is capped from above so that an enormous box
// does not produce absurd type.
inline double fill_size(cairo_t* cr, const char* a, const char* b, double objetivo,
                        double spacing, double cuerpo_max, cairo_font_face_t* face,
                        const char* familia, cairo_font_slant_t inclin,
                        cairo_font_weight_t peso)
{
    auto ancho = [&](double c) {
        select_font(cr, face, familia, inclin, peso, c);
        double w = spaced_width(cr, a, spacing) + spacing;
        if (b) w = std::max(w, spaced_width(cr, b, spacing) + spacing);
        return w;
    };
    double c = cuerpo_max;
    double w = ancho(c);
    if (w <= 0.0) return c;
    c *= objetivo / w;
    for (int i = 0; i < 3; ++i) {
        w = ancho(c);
        if (w <= 0.0) break;
        c *= objetivo / w;
    }
    return std::min(c, cuerpo_max);
}

// THE SIZE THAT FITS, IN ONE FUNCTION. Both the bounded text and its HALO use
// it, which is why it was extracted: written twice, the halo would come out at
// the nominal size while the text shrinks — a halo bigger than its own letters,
// and no error anywhere.
inline double fit_size(cairo_t* cr, const char* s, double cuerpo, double spacing,
                       double max_width, cairo_font_face_t* face, const char* familia,
                       cairo_font_slant_t inclin, cairo_font_weight_t peso)
{
    select_font(cr, face, familia, inclin, peso, cuerpo);
    double w = spaced_width(cr, s, spacing) + spacing;   // same rule as the SVG
    // One proportional step then a fine adjust: width is nearly linear in
    // the size, so this converges in two rounds and does not iterate blindly.
    if (w > max_width) {
        cuerpo *= max_width / w;
        for (int i = 0; i < 8; ++i) {
            select_font(cr, face, familia, inclin, peso, cuerpo);
            w = spaced_width(cr, s, spacing) + spacing;
            if (w <= max_width) break;
            cuerpo *= 0.98;
        }
    }
    return cuerpo;
}

// THE LIT NAME'S HALO.
//
// Cairo has NO blur, so it is FAKED the same way the panel's shadow is: by
// STACKING outline strokes of the text, from the widest and faintest to the
// narrowest and densest. It is the idiom this file already uses; a single-stroke
// halo reads as a border, not as light.
//
// It ALWAYS goes under the fill, never instead of it: the glow carries the
// "lit" and the fill's sharpness is what keeps the text legible. That is why
// the lit state can afford a luminous colour against the green even though it
// contrasts poorly in light-dark terms: what separates the two states is the
// HALO, not the hue.
inline void text_spaced_glow(cairo_t* cr, const char* s, double x_svg, double baseline,
                             double size, double spacing, unsigned hex, double a,
                             cairo_font_face_t* face, const char* familia,
                             cairo_font_slant_t inclin, cairo_font_weight_t peso,
                             double radio, int capas)
{
    if (capas < 1 || radio <= 0.0) return;
    select_font(cr, face, familia, inclin, peso, size);
    const double total = spaced_width(cr, s, spacing) + spacing;
    cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    for (int k = capas; k >= 1; --k) {
        const double t = double(k) / double(capas);       // 1 = the widest
        // Alpha grows inwards with the SQUARE: linear leaves a flat decal of a
        // halo instead of a decay of light.
        const double alfa = a * (1.0 - t) * (1.0 - t) * 0.55 + a * 0.06;
        cairo_set_line_width(cr, radio * t * 2.0);
        set_rgba(cr, hex, alfa);
        double pen = x_svg - total / 2.0;
        const size_t n = std::strlen(s);
        for (size_t i = 0; i < n; ++i) {
            char ch[2] = { s[i], 0 };
            cairo_text_extents_t e;
            cairo_text_extents(cr, ch, &e);
            cairo_move_to(cr, pen, baseline);
            cairo_text_path(cr, ch);
            pen += e.x_advance + spacing;
        }
        cairo_stroke(cr);
    }
}

// Same, but BOUNDED to a maximum width: if it does not fit, the size
// drops until it does, and the occupied width is returned.
//
// It exists for a concrete, measured reason: the handoff says «NLS
// CREAM» at 46 px, and THIS plugin's plate says VALVE HOWLER — two more
// letters inside the same 312 px of silkscreen. Hard-wiring a size would
// leave the name spilling off the pedal as soon as a word changes, and
// spilling raises no error: it just looks wrong.
inline double text_spaced_fit(cairo_t* cr, const char* s, double cx, double baseline,
                              double size, double spacing, double max_width,
                              unsigned hex, double a, cairo_font_face_t* face,
                              const char* familia, cairo_font_slant_t inclin,
                              cairo_font_weight_t peso)
{
    double cuerpo = size;
    cuerpo = fit_size(cr, s, cuerpo, spacing, max_width, face, familia, inclin, peso);
    select_font(cr, face, familia, inclin, peso, cuerpo);
    const double w = spaced_width(cr, s, spacing) + spacing;
    text_spaced(cr, s, cx, baseline, cuerpo, spacing, hex, a, face, familia, inclin, peso);
    return w;
}

// --- textures: grain and wear ------------------------------------------------
//
// Cairo has NO `feTurbulence`. The handoff asks for two noise layers
// over the paint (fine grain at `baseFrequency 0.9` with alpha 0,05, and
// low-frequency wear patches at 0,012/0,02 with opacity 0,10); without
// them the green reads as flat plastic instead of used enamelled metal.
//
// They are synthesised with DETERMINISTIC value noise (integer hash, no
// `rand()`), once, into A8 masks at the surface's pixel size. Two
// intended consequences: the panel draws identically on every repaint — a
// grain that changes per `expose` looks like snow — and costs nothing per
// frame.
//
// Generated at PIXEL size, not nominal: generating at 400×700 and
// letting Cairo scale would blur the grain to double size in the scale-2
// preview.
struct Textures {
    cairo_surface_t* grano    = nullptr;   // A8, per-pixel white noise
    cairo_surface_t* desgaste = nullptr;   // A8, low-frequency patches
    cairo_surface_t* cepillo  = nullptr;   // A8, ANISOTROPIC: brushed anodising
    cairo_surface_t* cepillo2 = nullptr;   // A8, the dark half of the brushing
    int w = 0, h = 0;
};

// Integer hash -> [0,1). Deterministic and stateless (the same pixel
// always yields the same value, on any machine).
inline double hash01(uint32_t x, uint32_t y, uint32_t sem)
{
    uint32_t h = x * 374761393u + y * 668265263u + sem * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return double(h) / 4294967296.0;
}

// Value noise with smooth interpolation, on a grid of pitch (px,py).
inline double noise_value(double x, double y, double px, double py, uint32_t sem)
{
    const double gx = x / px, gy = y / py;
    const double fx = std::floor(gx), fy = std::floor(gy);
    const double tx = gx - fx, ty = gy - fy;
    // Hermite smoothing: without it the grid's edges show.
    const double sx = tx * tx * (3.0 - 2.0 * tx);
    const double sy = ty * ty * (3.0 - 2.0 * ty);
    const uint32_t ix = uint32_t(int32_t(fx) + 4096), iy = uint32_t(int32_t(fy) + 4096);
    const double v00 = hash01(ix,     iy,     sem);
    const double v10 = hash01(ix + 1, iy,     sem);
    const double v01 = hash01(ix,     iy + 1, sem);
    const double v11 = hash01(ix + 1, iy + 1, sem);
    return (v00 * (1 - sx) + v10 * sx) * (1 - sy) + (v01 * (1 - sx) + v11 * sx) * sy;
}

inline Textures make_textures(int w, int h, double scale)
{
    Textures t;
    if (w <= 0 || h <= 0) return t;
    t.w = w; t.h = h;

    t.grano = cairo_image_surface_create(CAIRO_FORMAT_A8, w, h);
    if (cairo_surface_status(t.grano) == CAIRO_STATUS_SUCCESS) {
        unsigned char* d = cairo_image_surface_get_data(t.grano);
        const int stride = cairo_image_surface_get_stride(t.grano);
        // GRAIN IS MEASURED IN PANEL UNITS, NOT PIXELS. `feTurbulence`
        // works in the SVG's user space, so `baseFrequency 0.9` is a
        // period of 1/0,9 = 1,11 UNITS — 2,22 pixels at scale 2.
        // Generating per device pixel, as the first version did, gives a
        // grain twice as fine as the design asks the moment the window is
        // not at scale 1. Visible in the prototype diff, not by eye.
        // Two octaves, as `numOctaves="2"`.
        const double periodo = NLSC_GRAIN_PERIOD * scale;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                double n = 0.0, amp = 0.5, f = 1.0;
                for (int o = 0; o < 2; ++o) {
                    n += amp * noise_value(x, y, periodo / f, periodo / f, 1u + uint32_t(o));
                    amp *= 0.5; f *= 2.0;
                }
                // Two-octave noise with weights 1/2 and 1/4 sums to at
                // most 0,75: rescaled to [0,1] so the grain's statistics
                // match the SVG's (mean 0,5 => mean effective alpha 0,025
                // with the `feColorMatrix`'s 0,05).
                n /= 0.75;
                if (n < 0.0) n = 0.0;
                if (n > 1.0) n = 1.0;
                d[y * stride + x] = (unsigned char)(n * 255.0);
            }
        }
        cairo_surface_mark_dirty(t.grano);
    }

    // ANODISED BRUSHING. This departs from the design brief ON PURPOSE: the
    // brief specifies ENAMELLED aluminium — paint over metal — and this is
    // anodised metal, that is, colour INSIDE the oxide with the brush grain
    // visible. Do not "fix" it back by comparing against the brief.
    //
    // The trick is for the noise to be ANISOTROPIC: a fine period in X and a
    // long one in Y gives VERTICAL grain, which is how a pedal enclosure is
    // brushed (lengthwise). It reuses the same deterministic noise as the
    // grain, so it stays identical on every repaint and costs nothing per
    // frame: it lives in the static layer, which is cached.
    for (int pasada = 0; pasada < 2; ++pasada) {
        cairo_surface_t** dst = pasada ? &t.cepillo2 : &t.cepillo;
        *dst = cairo_image_surface_create(CAIRO_FORMAT_A8, w, h);
        if (cairo_surface_status(*dst) != CAIRO_STATUS_SUCCESS) continue;
        unsigned char* d = cairo_image_surface_get_data(*dst);
        const int stride = cairo_image_surface_get_stride(*dst);
        const double px = 1.35 * scale;    // thin ACROSS the grain
        const double py = 110.0 * scale;   // long ALONG the grain
        const uint32_t sem = pasada ? 41u : 17u;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                double n = 0.0, amp = 0.62, f = 1.0;
                for (int o = 0; o < 3; ++o) {
                    n += amp * noise_value(x, y, px / f, py / f, sem + uint32_t(o));
                    amp *= 0.45; f *= 2.3;
                }
                n /= 1.05;
                // Squared so the grain comes out NARROW and separated
                // rather than a uniform haze: real brushing is fine lines
                // over a smooth surface, not texture everywhere. Without
                // this the chassis looks like cloth, not metal.
                n = n * n;
                if (n < 0.0) n = 0.0;
                if (n > 1.0) n = 1.0;
                d[y * stride + x] = (unsigned char)(n * 255.0);
            }
        }
        cairo_surface_mark_dirty(*dst);
    }

    t.desgaste = cairo_image_surface_create(CAIRO_FORMAT_A8, w, h);
    if (cairo_surface_status(t.desgaste) == CAIRO_STATUS_SUCCESS) {
        unsigned char* d = cairo_image_surface_get_data(t.desgaste);
        const int stride = cairo_image_surface_get_stride(t.desgaste);
        // The handoff's periods (1/0,012 and 1/0,02 panel units) carry
        // the scale, so a patch measures the same on the pedal at any
        // resolution.
        const double px = (1.0 / 0.012) * scale, py = (1.0 / 0.02) * scale;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                // Three octaves, as `numOctaves="3"`.
                double n = 0.0, amp = 0.5, fx = 1.0;
                for (int o = 0; o < 3; ++o) {
                    n += amp * noise_value(x, y, px / fx, py / fx, 7u + uint32_t(o));
                    amp *= 0.5; fx *= 2.0;
                }
                // The handoff's `feColorMatrix`: alpha = 0,9·n − 0,52 =>
                // only the crests let paint through, which is what makes
                // "patches" instead of a uniform haze.
                double a = 0.9 * n - 0.52;
                if (a < 0.0) a = 0.0;
                if (a > 1.0) a = 1.0;
                d[y * stride + x] = (unsigned char)(a * 255.0);
            }
        }
        cairo_surface_mark_dirty(t.desgaste);
    }
    return t;
}

inline void free_textures(Textures& t)
{
    if (t.grano)    cairo_surface_destroy(t.grano);
    if (t.desgaste) cairo_surface_destroy(t.desgaste);
    if (t.cepillo)  cairo_surface_destroy(t.cepillo);
    if (t.cepillo2) cairo_surface_destroy(t.cepillo2);
    t = Textures{};
}

// --- the knob ----------------------------------------------------------------
//
// Knurled skirt + brushed-metal cap, a port of the handoff's `Knob`
// component. Only layers 4 (knurl) and 9 (index) rotate: the lights
// stay put.
// SPLIT IN TWO BY THE LAYER EACH PIECE BELONGS TO, not by position.
// `draw_knob_static` is what sits UNDER the knurl and does not depend on
// the value; `draw_knob_live` is from there up. Cutting "through the
// middle of the figure" is what once left a `cairo_save`+`clip` without
// its `restore` in the workshop and erased half a panel.
inline void draw_knob_static(cairo_t* cr, double cx, double cy, double r,
                             const Theme& th, bool on)
{
    // 1. The 11 graduation marks, every 30°, at radius r+12. They do NOT
    //    rotate.
    // THE DOTS LIGHT UP WITH THE PEDAL, and the labels DRIVE / TONE / LEVEL do
    // not. The reason for not lighting everything is hierarchy: light the whole
    // panel and there is no focus left, so the state reads WORSE because
    // nothing contrasts with anything. The dots work because they are small:
    // they read as light without stealing the name's place.
    for (int i = 0; i <= 10; ++i) {
        const double a = (-150.0 + 30.0 * i) * M_PI / 180.0;
        const double px = cx + std::sin(a) * (r + 12.0);
        const double py = cy - std::cos(a) * (r + 12.0);
        // NO HALO here. Glow on everything distracts and reads as kitsch, so
        // it stays on the name alone, which is the focus; everything else just
        // changes colour.
        set_rgba(cr, on ? th.rule_on : th.print, 0.88);
        cairo_arc(cr, px, py, 1.9, 0, 2 * M_PI);
        cairo_fill(cr);
    }

    // 2. Contact shadow: ellipse rx=r+5, ry=r+2, 6 px below centre.
    {
        cairo_pattern_t* ao = radial_obb(cx - (r + 5), cy + 6 - (r + 2),
                                        2 * (r + 5), 2 * (r + 2), 0.5, 0.5, 0.5);
        add_stop(ao, 0.00, 0x000000, 0.50);
        add_stop(ao, 0.60, 0x000000, 0.28);
        add_stop(ao, 1.00, 0x000000, 0.00);
        cairo_save(cr);
        cairo_translate(cr, cx, cy + 6);
        cairo_scale(cr, r + 5, r + 2);
        cairo_arc(cr, 0, 0, 1.0, 0, 2 * M_PI);
        cairo_restore(cr);
        cairo_set_source(cr, ao);
        cairo_fill(cr);
        cairo_pattern_destroy(ao);
    }

    // 3. Black base.
    set_rgb(cr, 0x0c0d0f);
    cairo_arc(cr, cx, cy, r, 0, 2 * M_PI);
    cairo_fill(cr);
}

// `draw_knob_live` LIVED HERE and was DELETED on purpose when the
// cache went in. Its body moved to `draw_knurl` / `draw_disc` /
// `draw_pointer`, the three pieces the cache needs separately. Keeping
// it "just in case" would have left two descriptions of the same knob
// diverging at the first tweak — and the stale copy does not fail: it
// draws wrong.

// --- the LED ------------------------------------------------------------------
inline void draw_led(cairo_t* cr, bool on, const Theme& th)
{
    const double cx = 200.0, cy = 52.0;
    if (on) {
        cairo_pattern_t* g = radial_obb(cx - 30, cy - 30, 60, 60, 0.5, 0.5, 0.5);
        add_stop(g, 0.00, th.led_glow, 0.80);
        add_stop(g, 0.55, th.led_glow, 0.22);
        add_stop(g, 1.00, th.led_glow, 0.00);
        cairo_arc(cr, cx, cy, 30, 0, 2 * M_PI);
        cairo_set_source(cr, g);
        cairo_fill(cr);
        cairo_pattern_destroy(g);
    }
    set_rgb(cr, 0x0a0b0c);                       // the chassis hole
    cairo_arc(cr, cx, cy, 10.5, 0, 2 * M_PI);
    cairo_fill(cr);
    {
        cairo_pattern_t* d = radial_obb(cx - 7.5, cy - 7.5, 15, 15, 0.42, 0.38, 0.60);
        if (on) { add_stop(d, 0.00, th.led0); add_stop(d, 0.35, th.led1); add_stop(d, 1.00, th.led2); }
        else    { add_stop(d, 0.00, th.led_off0); add_stop(d, 1.00, th.led_off1); }
        cairo_arc(cr, cx, cy, 7.5, 0, 2 * M_PI);
        cairo_set_source(cr, d);
        cairo_fill(cr);
        cairo_pattern_destroy(d);
    }
    // The specular dot, always: it is plastic, it shines when off too.
    set_rgba(cr, 0xffffff, 0.85);
    cairo_save(cr);
    cairo_translate(cr, 197.5, 49.5);
    cairo_scale(cr, 2.4, 1.8);
    cairo_arc(cr, 0, 0, 1.0, 0, 2 * M_PI);
    cairo_restore(cr);
    cairo_fill(cr);
}

// --- the 3PDT footswitch ------------------------------------------------------
//
// Concentric at (200,494), the whole group scaled ×1,12 about its own
// centre. `pressed` sinks the cap into the panel: scale 0,945 towards its
// centre, darkened. From the handoff: NOT translated downwards — it
// SINKS.
inline void hex_path(cairo_t* cr, double cx, double cy, double r)
{
    for (int i = 0; i < 6; ++i) {
        const double a = (i * 60.0 - 90.0) * M_PI / 180.0;
        const double x = cx + r * std::cos(a), y = cy + r * std::sin(a);
        if (i == 0) cairo_move_to(cr, x, y); else cairo_line_to(cr, x, y);
    }
    cairo_close_path(cr);
}

// Scales a hex colour by a factor (k<1 darkens it). Used for the plate's
// PRESSED state: one factor over the whole ramp keeps the relation between its
// five stops, which is what makes it still read as the same metal rather than
// as a different colour.
inline unsigned mezcla(unsigned hex, double k)
{
    auto c = [&](int desp) {
        double v = double((hex >> desp) & 0xff) * k;
        if (v < 0.0) v = 0.0;
        if (v > 255.0) v = 255.0;
        return unsigned(v + 0.5);
    };
    return (c(16) << 16) | (c(8) << 8) | c(0);
}

// Deterministic noise for the footswitch textures. Never `rand()`: a
// texture that changes on every `expose` reads as snow, not as metal.
struct Lcg {
    uint32_t e;
    explicit Lcg(uint32_t sem) : e(sem) {}
    uint32_t u32() { e = e * 1664525u + 1013904223u; return e; }
    double  u01() { return (u32() >> 8) / 16777216.0; }
    double  rango(double a, double b) { return a + (b - a) * u01(); }
};

// A HINGED FOOTSWITCH PLATE, not a round stomp switch.
//
// PROVENANCE, because it matters: what is reproduced is the REAL HARDWARE of
// the pedal this plugin models — its hinged plate — from photographs of the
// unit itself. Screenshots of other people's plugins were LOOKED AT to
// understand the shape, which is what the rules here allow; no graphic asset
// and no measurement was taken from them. They agree with each other because
// they portray the same object.
//
// The shape, read off the photographs:
//   · rounded-corner plate, COLD brushed aluminium, wider than it is tall
//     (about 1,6:1)
//   · TWO groups of horizontal ribs — one above, one below — separated by a
//     smooth polished band. It is NOT a uniform grille: a uniform one is what
//     makes a plate read as plastic
//   · a bevel that catches light at the top left and darkens at the bottom right
//   · sunk into a black WELL with visible slack around it: without the well the
//     plate looks like a sticker rather than a part that moves
// The plate's geometry lives ABOVE, with the rest of the panel geometry:
// `draw_logo_capa` needs it, and that is drawn long before the footswitch.


// A group of ribs: `n` engraved grooves, each one a dark line with its bright
// reflection just below it. That pair is what makes it ENGRAVED and not
// painted.
inline void estrias(cairo_t* cr, double x0, double x1, double y0, int n, double paso)
{
    for (int i = 0; i < n; ++i) {
        const double y = y0 + i * paso;
        cairo_set_line_width(cr, 1.0);
        set_rgba(cr, 0x3f4a4f, 0.55);
        cairo_move_to(cr, x0, y);       cairo_line_to(cr, x1, y);       cairo_stroke(cr);
        set_rgba(cr, 0xffffff, 0.42);
        cairo_move_to(cr, x0, y + 1.1); cairo_line_to(cr, x1, y + 1.1); cairo_stroke(cr);
    }
}

inline void draw_footswitch(cairo_t* cr, bool pressed, const Theme& th)
{
    (void)th;
    // -- the WELL: dark recess in the chassis ---------------------------
    {
        rounded_rect(cr, FS_WELL_X, FS_WELL_Y, FS_WELL_W, FS_WELL_H, 8);
        cairo_pattern_t* g = linear_obb(FS_WELL_X, FS_WELL_Y, FS_WELL_W, FS_WELL_H,
                                        0.0, 0.0, 0.0, 1.0);
        add_stop(g, 0.00, 0x05100a, 1.0);
        add_stop(g, 0.45, 0x0b1a11, 1.0);
        add_stop(g, 1.00, 0x122417, 1.0);
        cairo_set_source(cr, g); cairo_fill(cr); cairo_pattern_destroy(g);
    }
    // Inner shadow at the top: this is what says the well is SUNK.
    {
        cairo_save(cr);
        rounded_rect(cr, FS_WELL_X, FS_WELL_Y, FS_WELL_W, FS_WELL_H, 8);
        cairo_clip(cr);
        cairo_pattern_t* g = linear_obb(FS_WELL_X, FS_WELL_Y, FS_WELL_W, 26,
                                        0.0, 0.0, 0.0, 1.0);
        add_stop(g, 0.00, 0x000000, 0.75);
        add_stop(g, 1.00, 0x000000, 0.00);
        cairo_set_source(cr, g);
        cairo_rectangle(cr, FS_WELL_X, FS_WELL_Y, FS_WELL_W, 26);
        cairo_fill(cr); cairo_pattern_destroy(g);
        cairo_restore(cr);
    }
    // WELL TEXTURE. A flat black is a HOLE, not a surface. On the real
    // hardware the bottom of the well is rough paint, so it carries fine
    // speckle — light and dark — plus a little coarser grain. Deterministic.
    {
        cairo_save(cr);
        rounded_rect(cr, FS_WELL_X, FS_WELL_Y, FS_WELL_W, FS_WELL_H, 8);
        cairo_clip(cr);
        Lcg r(77712026u);
        for (int i = 0; i < 900; ++i) {
            const double x = r.rango(FS_WELL_X, FS_WELL_X + FS_WELL_W);
            const double y = r.rango(FS_WELL_Y, FS_WELL_Y + FS_WELL_H);
            const bool claro = r.u01() < 0.58;
            set_rgba(cr, claro ? 0x7f9a88 : 0x000000, r.rango(0.05, 0.16));
            cairo_arc(cr, x, y, r.rango(0.35, 1.05), 0, 2 * M_PI);
            cairo_fill(cr);
        }
        // And a few larger specks, since rough paint is not uniform.
        for (int i = 0; i < 70; ++i) {
            const double x = r.rango(FS_WELL_X, FS_WELL_X + FS_WELL_W);
            const double y = r.rango(FS_WELL_Y, FS_WELL_Y + FS_WELL_H);
            set_rgba(cr, 0x9db3a5, r.rango(0.05, 0.11));
            cairo_arc(cr, x, y, r.rango(1.0, 2.1), 0, 2 * M_PI);
            cairo_fill(cr);
        }
        cairo_restore(cr);
    }

    // Light edge along the bottom of the well: the chassis lip.
    set_rgba(cr, 0xbfe8c9, 0.16);
    cairo_set_line_width(cr, 1.2);
    rounded_rect(cr, FS_WELL_X + 0.6, FS_WELL_Y + 0.6, FS_WELL_W - 1.2, FS_WELL_H - 1.2, 8);
    cairo_stroke(cr);

    // When pressed the plate DROPS and its shadow flattens. 1,5 px is enough:
    // more and it looks like it sinks into the chassis instead of pivoting.
    const double dy = pressed ? 1.5 : 0.0;
    const double px = FS_PL_X, py = FS_PL_Y + dy, pw = FS_PL_W, ph = FS_PL_H;

    // The plate's cast shadow inside the well.
    {
        cairo_save(cr);
        rounded_rect(cr, FS_WELL_X, FS_WELL_Y, FS_WELL_W, FS_WELL_H, 8);
        cairo_clip(cr);
        set_rgba(cr, 0x000000, pressed ? 0.42 : 0.55);
        rounded_rect(cr, px - 2.0, py + (pressed ? 2.0 : 4.5), pw + 4.0, ph, 10);
        cairo_fill(cr);
        cairo_restore(cr);
    }

    // ── the PLATE: cold brushed aluminium ───────────────────────────────
    {
        rounded_rect(cr, px, py, pw, ph, 9);
        cairo_pattern_t* g = linear_obb(px, py, pw, ph, 0.0, 0.0, 0.0, 1.0);
        const double k = pressed ? 0.88 : 1.0;
        // WIDENED ramp. The first went from 0xe4 to 0x84 — 57 levels — and
        // came out milky; this one goes from 0xf2 to 0x5c, i.e. **150 levels**,
        // with the lower half clearly darker, which is what the photograph of
        // the hardware shows.
        add_stop(g, 0.00, mezcla(0xf2f6f8, k), 1.0);
        add_stop(g, 0.14, mezcla(0xd2d9dd, k), 1.0);
        add_stop(g, 0.46, mezcla(0x99a3a9, k), 1.0);
        add_stop(g, 0.62, mezcla(0x77828a, k), 1.0);
        add_stop(g, 0.84, mezcla(0xa7b0b6, k), 1.0);
        add_stop(g, 1.00, mezcla(0x5c666d, k), 1.0);
        cairo_set_source(cr, g); cairo_fill(cr); cairo_pattern_destroy(g);
    }

    cairo_save(cr);
    rounded_rect(cr, px, py, pw, ph, 9);
    cairo_clip(cr);

    // Soft diagonal sheen, so the metal does not come out flat.
    {
        cairo_pattern_t* g = linear_obb(px, py, pw, ph, 0.0, 0.0, 1.0, 0.7);
        add_stop(g, 0.00, 0xffffff, 0.20);
        add_stop(g, 0.42, 0xffffff, 0.05);
        add_stop(g, 0.70, 0x000000, 0.05);
        add_stop(g, 1.00, 0x000000, 0.13);
        cairo_set_source(cr, g);
        cairo_rectangle(cr, px, py, pw, ph); cairo_fill(cr);
        cairo_pattern_destroy(g);
    }

    // HORIZONTAL BRUSHING OF THE ALUMINIUM. A clean ramp with no
    // microstructure reads as vector illustration rather than as metal.
    // It runs HORIZONTALLY, perpendicular to the chassis's: the plate is a
    // separate part, pressed on its own, and its grain follows the long axis —
    // as in the photograph. A grain continuing the chassis's would fuse the two
    // into one piece.
    {
        Lcg r(31415926u);
        for (int i = 0; i < 260; ++i) {
            const double y  = py + r.u01() * ph;
            const double x0 = px + r.rango(0.0, pw * 0.35);
            const double x1 = x0 + r.rango(pw * 0.18, pw * 0.95);
            const bool claro = r.u01() < 0.5;
            cairo_set_line_width(cr, r.rango(0.35, 0.95));
            set_rgba(cr, claro ? 0xffffff : 0x232b30, r.rango(0.04, 0.13));
            cairo_move_to(cr, x0, y); cairo_line_to(cr, x1 > px + pw ? px + pw : x1, y);
            cairo_stroke(cr);
        }
    }

    // THE TWO GROUPS OF RIBS, with the smooth band between them. The
    // proportions come from photographs of the hardware: the upper group is the
    // denser one.
    {
        // Inset from the sides: at 7,5 % they ran almost edge to edge, and on
        // the real hardware both groups leave a side margin. That margin is
        // part of what makes it read as a pressed part.
        const double mx0 = px + pw * 0.135, mx1 = px + pw * 0.865;
        estrias(cr, mx0, mx1, py + ph * 0.130, 10, ph * 0.031);
        estrias(cr, mx0, mx1, py + ph * 0.690, 7, ph * 0.036);
    }
    cairo_restore(cr);

    // Bevel: light at the top left, shadow at the bottom right.
    set_rgba(cr, 0xffffff, 0.55);
    cairo_set_line_width(cr, 1.3);
    rounded_rect(cr, px + 0.8, py + 0.8, pw - 1.6, ph - 1.6, 9);
    cairo_stroke(cr);
    set_rgba(cr, 0x1b2226, 0.80);
    cairo_set_line_width(cr, 1.4);
    rounded_rect(cr, px + 2.1, py + 2.1, pw - 4.2, ph - 4.2, 8);
    cairo_stroke(cr);
}

// --- the whole panel ----------------------------------------------------------

// The live values the panel needs. Knobs are normalised to [0,1].
struct PanelState {
    double drive = 0.62;
    double tone  = 0.50;
    double level = 0.55;
    bool   on    = true;             // `enabled`: lights the LED and its halo
    bool   pressed = false;          // the footswitch, while the mouse is down
    const char* variant = "OD-8 W Tone";
};

// Where the two rules hugging the variant name end. The JSX measures it
// with `getBBox()`; here with `cairo_text_extents`. Exposed because the
// X11 code does not need it, but the guided preview does.
struct VariantFrame { double izq, der; };

inline VariantFrame variant_frame(cairo_t* cr, const char* nombre, double cuerpo,
                                    double spacing, cairo_font_face_t* face)
{
    select_font(cr, face, "sans-serif", CAIRO_FONT_SLANT_ITALIC,
                CAIRO_FONT_WEIGHT_BOLD, cuerpo);
    // The SVG's same rule: the width carries the trailing gap and the
    // anchor is 200. The JSX measures with `getBBox()`, which returns the
    // GLYPHS' box (no trailing gap), so the left edge is the displaced
    // anchor's.
    const double w_svg = spaced_width(cr, nombre, spacing) + spacing;
    const double w = w_svg - spacing;                    // the glyphs' bbox
    const double x = 200.0 - w_svg / 2.0;
    const double gap = 16.0;                       // from the handoff
    VariantFrame m;
    m.izq = x - gap; if (m.izq < 62.0)  m.izq = 62.0;
    m.der = x + w + gap; if (m.der > 338.0) m.der = 338.0;
    return m;
}

// Paints the whole panel into `cr`. The caller sets the context so that
// (0,0)..(PANEL_W,PANEL_H) lands on the target surface. `fonts` may be
// null (toy fonts) and so may `tex` (no grain or wear).
// `logo` is a PARAMETER and not part of `PanelState`: the state is port
// VALUES, and putting a graphic resource in there mixes two things with
// different lifetimes — the state changes per sample, the logo lives as long as
// the window.
inline void draw_static(cairo_t* cr, const PanelState& st, const Fonts* fonts = nullptr,
                        const Textures* tex = nullptr, double scale = 1.0,
                        const Theme& th = default_theme(),
                        cairo_surface_t* logo = nullptr)
{
    // `archivo800` is not used here any more: it was the product name, which
    // the panel no longer draws. The face stays loaded in `Fonts` because the
    // name may come back.
    // `archivo700i` (the italic) is not used here either: it was the variant
    // legend, now removed. The face stays loaded because `variant_frame()`
    // measures it for anyone who wants it on another pedal.
    cairo_font_face_t* barlow600   = fonts ? fonts->barlow600   : nullptr;
    cairo_font_face_t* mono400     = fonts ? fonts->mono400     : nullptr;

    const double W = PANEL_W, H = PANEL_H;

    // ── box: painted aluminium ──────────────────────────────────────────
    set_rgb(cr, th.rim);
    rounded_rect(cr, 10, 8, W - 20, H - 16, 26);
    cairo_fill(cr);

    // The body radial: cx 40 %, cy 18 %, r 105 % of a 372×676 box =>
    // ELLIPSE, not circle (see `obb_pattern`).
    {
        cairo_pattern_t* b = radial_obb(14, 12, W - 28, H - 24, 0.40, 0.18, 1.05);
        add_stop(b, 0.00, th.body0);
        add_stop(b, 0.38, th.body1);
        add_stop(b, 0.74, th.body2);
        add_stop(b, 1.00, th.body3);
        rounded_rect(cr, 14, 12, W - 28, H - 24, 22);
        cairo_set_source(cr, b);
        cairo_fill(cr);
        cairo_pattern_destroy(b);
    }
    // EVERY BODY LAYER IS CLIPPED TO THE BODY. The two side edges used to be
    // TALL rectangles with radii of their own (5 and 7) running from y=15 to
    // y=H-15, while the body's corner curves with radius 22 from (14,12). At
    // the top-left corner the body starts at y~22,9 for x=17, so the edge
    // painted about 8 px OUTSIDE the body: that is the spike.
    // => The fix is NOT to tune each radius by hand — that breaks again the
    // moment somebody moves an inset — but to CLIP to the body outline, which
    // is the only figure that defines where the chassis ends.
    cairo_save(cr);
    rounded_rect(cr, 14, 12, W - 28, H - 24, 22);
    cairo_clip(cr);

    // Enamel: sheen on top, darkened below.
    {
        cairo_pattern_t* e = linear_obb(14, 12, W - 28, H - 24, 0.0, 0.0, 0.0, 1.0);
        add_stop(e, 0.00, 0xffffff, 0.34);
        add_stop(e, 0.10, 0xffffff, 0.10);
        add_stop(e, 0.30, 0xffffff, 0.00);
        add_stop(e, 0.82, 0x000000, 0.00);
        add_stop(e, 1.00, 0x000000, 0.28);
        rounded_rect(cr, 14, 12, W - 28, H - 24, 22);
        cairo_set_source(cr, e);
        cairo_fill(cr);
        cairo_pattern_destroy(e);
    }
    // Edges: light on the left, shadow on the right.
    {
        cairo_pattern_t* l = linear_obb(17, 15, 10, H - 30, 0.0, 0.0, 1.0, 0.0);
        add_stop(l, 0.00, th.edge_luz, 0.55);
        add_stop(l, 1.00, th.edge_luz, 0.00);
        rounded_rect(cr, 17, 15, 10, H - 30, 5);
        cairo_set_source(cr, l);
        cairo_fill(cr);
        cairo_pattern_destroy(l);
    }
    {
        cairo_pattern_t* d = linear_obb(W - 31, 15, 14, H - 30, 1.0, 0.0, 0.0, 0.0);
        add_stop(d, 0.00, th.edge_sombra, 0.50);
        add_stop(d, 1.00, th.edge_sombra, 0.00);
        rounded_rect(cr, W - 31, 15, 14, H - 30, 7);
        cairo_set_source(cr, d);
        cairo_fill(cr);
        cairo_pattern_destroy(d);
    }
    // The clip is released BEFORE the stroked edges: a `stroke` over the same
    // outline would come out at half width if it were still clipped.
    cairo_restore(cr);

    set_rgba(cr, th.filo, 0.35);
    cairo_set_line_width(cr, 1.4);
    rounded_rect(cr, 14, 12, W - 28, H - 24, 22);
    cairo_stroke(cr);
    set_rgba(cr, th.filo_oscuro, 0.55);
    cairo_set_line_width(cr, 1.0);
    rounded_rect(cr, 15.5, 13.5, W - 31, H - 27, 21);
    cairo_stroke(cr);

    // Grain and wear, clipped to the body. The masks are in screen
    // pixels, so they apply with the IDENTITY matrix and the clip in
    // panel coordinates: painting an A8 mask with the scaled context
    // would interpolate it and turn the grain to mush.
    if (tex && (tex->grano || tex->desgaste || tex->cepillo)) {
        // THE MASKS STAMP AT DEVICE RESOLUTION, 1:1. Painting them
        // with the scaled context INTERPOLATES them: the grain comes out
        // blurred at double size — the failure once shipped with the knob
        // skirts ("cache at device resolution"). Hence: clip in panel
        // coordinates, identity matrix for the stamp, and the panel
        // origin converted to pixels with `cairo_user_to_device` — which
        // also absorbs the window margin.
        double ox = 0.0, oy = 0.0;
        cairo_user_to_device(cr, &ox, &oy);
        cairo_save(cr);
        rounded_rect(cr, 14, 12, W - 28, H - 24, 22);
        cairo_clip(cr);
        cairo_identity_matrix(cr);
        if (tex->grano) {
            set_rgba(cr, 0xffffff, NLSC_GRAIN_ALPHA);
            cairo_mask_surface(cr, tex->grano, ox, oy);
        }
        if (tex->desgaste) {
            set_rgba(cr, th.desgaste, 0.10);
            cairo_mask_surface(cr, tex->desgaste, ox, oy);
        }
        // The brushing goes AFTER the grain and the wear: it is the layer that
        // rules the reading of metal. Two passes, light and dark, because a
        // light-only grain reads as dust rather than as a groove.
        if (tex->cepillo) {
            set_rgba(cr, 0xffffff, 0.16);
            cairo_mask_surface(cr, tex->cepillo, ox, oy);
        }
        if (tex->cepillo2) {
            set_rgba(cr, 0x001a08, 0.17);
            cairo_mask_surface(cr, tex->cepillo2, ox, oy);
        }
        cairo_restore(cr);
    }
    (void)scale;

    // ── knobs (the UNDERSIDE) and labels ────────────────────────────────
    // The logo, UNDER the knobs. The order IS the design here.
    draw_logo_capa(cr, logo, st.on ? unsigned(NLSC_LOGO_ON) : th.print, 1.0);

    draw_knob_static(cr, DRIVE_CX, DRIVE_CY, DRIVE_R, th, st.on);
    draw_knob_static(cr, LEVEL_CX, LEVEL_CY, LEVEL_R, th, st.on);
    draw_knob_static(cr, TONE_CX,  TONE_CY,  TONE_R,  th, st.on);
    // THE LABELS ALWAYS STAY WHITE, while the dots do light up.
    // => A dot is a scale mark, and lighting it reads as light; a lit WORD
    // reads as a warning, and DRIVE is not warning about anything.
    const unsigned etiqueta_col = th.print;
    text_spaced(cr, "DRIVE", 103.75, 199 + KNOBS_DY, 22, 3.5, etiqueta_col, 1.0,
                barlow600, "sans-serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    text_spaced(cr, "LEVEL", 299.75, 199 + KNOBS_DY, 22, 3.5, etiqueta_col, 1.0,
                barlow600, "sans-serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    // TONE goes ABOVE its knob (baseline 152), not below: the tone
    // knob sits in the centre and a label below would hit the silkscreen.
    text_spaced(cr, "TONE",  201.75, 152 + KNOBS_DY, 22, 3.5, etiqueta_col, 1.0,
                barlow600, "sans-serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);

    // NO LED. The lit state is carried by the name and the dots instead.
    // `draw_led()` is kept in this file on purpose: it is the family's
    // component and the next pedal may want it.

    // ── the IDENTITY BLOCK: logo + two lines ────────────────────────────
    //
    // THE PRODUCT NAME IS NOT DRAWN. The panel carries the logo on the left
    // and, to its right, two lines of description.
    // The name stays alive where it matters: in the `.ttl` (`lv2:name`), which
    // is how the host lists the plugin. What is removed is the SILKSCREEN.
    // The logo carries the identity better than the text it replaces.
    const unsigned nombre_col = st.on ? th.print_on : th.print;
    {
        // THE BAND IS DERIVED, not written down. With the rules removed, its
        // limits are the objects that really surround it: above, how far the
        // TONE knob's graduation marks reach; below, the edge of the footswitch
        // well. Moving the knobs or the well therefore repositions the identity
        // block on its own, and there are no two places that can disagree.
        const double MARGEN = NLSC_BAND_MARGIN;
        const double banda_y0 = TONE_CY + TONE_R + 12.0 + MARGEN;
        const double banda_y1 = FS_WELL_Y - MARGEN;
        // The text is centred on the LOGO'S CENTRE, not the band's: since the
        // logo became a layer anchored at the bottom the two centres no longer
        // coincide, and the one that rules visually is the drawing's.
        const double cy = logo_cy();
        const double alto = banda_y1 - banda_y0;

        // The logo never overflows the band: it takes the smaller of what was
        // asked for and what fits. Asking for 150 with a band of 140 must not
        // clip the drawing silently, which is exactly what happened at 144 px.
        const double lado = std::min(double(NLSC_LOGO_SIDE), alto);

        // The logo follows the state but with ITS OWN colour, not the
        // lettering's, and with no halo: the glow stays on the text, which is
        // the focus.
        // The LOGO is not drawn here any more: it is a LAYER that goes before
        // the knobs, so it can grow upwards behind them.
        (void)lado;

        // DECOUPLED from the logo. See `NLSC_TEXT_X`: moving the drawing no
        // longer moves or resizes the text.
        (void)logo_ancho;
        const double tx = NLSC_TEXT_X;
        // The text block ENDS where the footswitch well ends, so its width is
        // not slack but a TARGET, and the body comes out of it.
        // `NLSC_TEXT_MAX` only prevents an absurdity if that space ever becomes
        // enormous.
        const double derecha = FS_WELL_X + FS_WELL_W;
        const double ancho_obj = derecha - tx;
        const double esp = 2.0;
        const double cuerpo = fill_size(cr, "SMOOTH", "OVERDRIVE", ancho_obj, esp,
                                        NLSC_TEXT_MAX, barlow600, "sans-serif",
                                        CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
        const double interlinea = cuerpo;

        // Vertically centred on the two lines' real INK, not by eye.
        select_font(cr, barlow600, "sans-serif", CAIRO_FONT_SLANT_NORMAL,
                    CAIRO_FONT_WEIGHT_BOLD, cuerpo);
        cairo_text_extents_t e1, e2;
        cairo_text_extents(cr, "SMOOTH", &e1);
        cairo_text_extents(cr, "OVERDRIVE", &e2);
        const double base1 = cy - (e1.y_bearing + interlinea + e2.y_bearing + e2.height) * 0.5;

        // The two lines are CENTRED ON EACH OTHER, not left-aligned. The axis
        // is the centre of the WIDER line, so the short line stays centred
        // under the long one whatever body ends up fitting.
        const double eje = tx + ancho_obj * 0.5;

        // The block is stretched VERTICALLY until it matches the logo's
        // height. Both are centred on the same axis, so matching heights is
        // what makes them read as a pair rather than as a drawing with a label
        // beside it.
        const double alto_bloque = (interlinea + e2.y_bearing + e2.height) - e1.y_bearing;
        const double alto_logo   = NLSC_LOGO_SIDE * NLSC_LOGO_FILL;
        const double estira = (alto_bloque > 1.0)
            ? std::min(double(NLSC_TEXT_STRETCH_MAX), std::max(1.0, alto_logo / alto_bloque))
            : 1.0;

        auto linea = [&](const char* t, double base) {
            select_font(cr, barlow600, "sans-serif", CAIRO_FONT_SLANT_NORMAL,
                        CAIRO_FONT_WEIGHT_BOLD, cuerpo);
            const double centro = eje;
            if (st.on)
                text_spaced_glow(cr, t, centro, base, cuerpo, esp, th.print_glow, 0.95,
                                 barlow600, "sans-serif", CAIRO_FONT_SLANT_NORMAL,
                                 CAIRO_FONT_WEIGHT_BOLD, NLSC_NAME_GLOW_R * 0.75, 7);
            text_spaced(cr, t, centro, base, cuerpo, esp, nombre_col, 1.0,
                        barlow600, "sans-serif", CAIRO_FONT_SLANT_NORMAL,
                        CAIRO_FONT_WEIGHT_BOLD);
        };
        // The stretch is applied about `cy`, not about the origin: scaling
        // without translating would move the block down as much as it grows it,
        // and the centring just computed would be lost.
        cairo_save(cr);
        cairo_translate(cr, 0.0, cy);
        cairo_scale(cr, 1.0, estira);
        cairo_translate(cr, 0.0, -cy);
        linea("SMOOTH",    base1);
        linea("OVERDRIVE", base1 + interlinea);
        cairo_restore(cr);
    }

    draw_footswitch(cr, st.pressed, th);

    // NO VARIANT LEGEND. The dropdown already shows the variant name, so a
    // second copy above it would be the same string in two places. The dropdown
    // is the ONLY citation of it.
    // `variant_frame()` stays here: it measures the gap where such a legend
    // would go, for whoever wants one on another pedal in the family.

    // ── desplegable ──────────────────────────────────────────────────────
    {
        // THE DROPDOWN'S DROP SHADOW, once missing entirely. The
        // handoff carries `box-shadow: inset 0 1px 0 rgba(255,255,255,
        // 0.12), 0 2px 7px rgba(0,0,0,0.4)` — only the `inset` half had
        // been ported, not the outer one, which is what seats the box on
        // the paint. Visible in the prototype zoom as a dark band under
        // the bottom edge.
        // The alpha is NOT the CSS's: the gaussian blur spreads the
        // energy and the stacking concentrates it => ~55 % of nominal
        // (0,4 × 0,55 ≈ 0,22).
        for (int i = 7; i >= 1; --i) {
            const double e = i * 1.0;
            const double a2 = 0.22 * (1.0 - double(i - 1) / 7.0) / 7.0 * 2.0;
            set_rgba(cr, 0x000000, a2);
            rounded_rect(cr, DD_X - e, DD_Y - e + 2.0, DD_W + 2 * e, DD_H + 2 * e, 4 + e);
            cairo_fill(cr);
        }
        cairo_pattern_t* g = linear_obb(DD_X, DD_Y, DD_W, DD_H, 0.0, 0.0, 0.0, 1.0);
        add_stop(g, 0.0, th.drop_top);
        add_stop(g, 1.0, th.drop_bot);
        rounded_rect(cr, DD_X, DD_Y, DD_W, DD_H, 4);
        cairo_set_source(cr, g);
        cairo_fill(cr);
        cairo_pattern_destroy(g);
        // THE BORDER GOES INSIDE, and this is MEASURED, not style. A
        // CSS `border` paints INSIDE the box (form controls are
        // `border-box`); a `cairo_stroke` paints CENTRED on the path.
        // With the path on the exact edge, the 1,5 stroke spills 0,75
        // outwards and the whole box sits half a pixel high.
        // Row profile against the prototype: its top border occupies
        // y = 622,0..623,5 (i.e. INWARD from 622); this one occupied
        // 621,25..622,75. The box measures exactly 290×38 in both — that
        // part was already right.
        set_rgba(cr, th.print, 0.50);
        cairo_set_line_width(cr, 1.5);
        rounded_rect(cr, DD_X + 0.75, DD_Y + 0.75, DD_W - 1.5, DD_H - 1.5, 3.25);
        cairo_stroke(cr);
        // Light inner edge (`inset 0 1px 0 rgba(255,255,255,0.12)`), JUST
        // inside the border: measured at y ≈ 623,5..624,5.
        set_rgba(cr, 0xffffff, 0.12);
        cairo_set_line_width(cr, 1.0);
        cairo_move_to(cr, DD_X + 4, DD_Y + 2);
        cairo_line_to(cr, DD_X + DD_W - 4, DD_Y + 2);
        cairo_stroke(cr);
        // Baseline 645,5 measured on the prototype (its ink occupies
        // y=637,5..645,0; this one used to land half a unit lower).
        if (st.variant)
            text_spaced(cr, st.variant, DD_X + DD_W / 2.0, DD_Y + 23.5, 12, 2.0,
                        th.drop_texto, 1.0, mono400, "monospace",
                        CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
        // THE CHEVRON IS THE ▼ GLYPH AT 9 px, NOT A TRIANGLE BY EYE.
        // Its real ink, measured on the prototype, is 4,5 × 4,5 centred at
        // (326,25 · 641,25) — half as wide as the triangle once drawn at
        // `DD_X+DD_W−14`, which is where CSS puts the `span`'s RIGHT
        // border, not its centre. Drawn by hand because the glyph is not
        // in the bundle fonts and a fifth family will not be added for an
        // arrowhead.
        set_rgb(cr, th.drop_texto);
        const double gx = 326.25, gy = 641.25;
        cairo_move_to(cr, gx - 2.25, gy - 2.25);
        cairo_line_to(cr, gx + 2.25, gy - 2.25);
        cairo_line_to(cr, gx,        gy + 2.25);
        cairo_close_path(cr);
        cairo_fill(cr);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LIVE-LAYER CACHE — measured before writing it, not assumed
// ─────────────────────────────────────────────────────────────────────────────
// Profile of `draw_knob_live` × 3, image surface, this machine
// (`harness/perfil_gui.cpp`):
//
//     draw_knob_live FULL x3 ........... 2,506 ms
//       the 10 ribs x3 ................. 1,488 ms   (59 %)
//       skirt + cap + arcs x3 .......... 0,806 ms   (32 %)
//       the index x3 ................... 0,023 ms   ( 1 %)
//
// => 91 % of the live frame goes to two things that do NOT depend on the
// value continuously, hence cacheable:
//
//  · skirt + cap + arcs do not depend on the value AT ALL. It is an
//    opaque disc of radius r−3 sitting ON TOP of the knurl, so it cannot
//    go in the static layer: cached apart, stamped between the two live
//    layers.
//  · the knurl has ORDER-10 SYMMETRY (ribs every 36°) => rotating 360° is
//    rotating 36°. `kFases` phases are pre-drawn within those 36° and the
//    nearest is stamped. With 12 phases the maximum error is 1,5° of
//    knurl, on a pattern whose period is 36°: invisible.
//
// PRE-DRAWN, NOT ROTATED. Stamping ONE surface with a rotation
// resamples it and the knurl comes out blurred — the failure once shipped
// in the 0C family ("cache at device resolution"). Each phase is DRAWN by
// Cairo at its angle, so it comes out sharp.
//
// Which is why the cache builds in DEVICE PIXELS and the scale is part
// of its key: a cache in logical units stamped through a `cairo_scale`
// interpolates — the same failure in another disguise.
//
// Direct drawing still exists and is the FALLBACK path: if the cache
// could not be built, `draw_live` paints the same, only dearer. A knob
// without its skirt is not an acceptable option.

inline constexpr int kKnurlPhases = 12;      // 36° / 12 = 3° per phase
inline constexpr int kMaxKnobs      = 3;

struct KnobCache {
    double r = 0.0;
    double lado = 0.0;                                  // crop size, in units
    cairo_surface_t* disco = nullptr;                   // skirt + cap + arcs
    cairo_surface_t* moleteado[kKnurlPhases] = { nullptr };
};

struct Cache {
    Textures   tex;
    KnobCache knobs[kMaxKnobs];
    int    n = 0;
    double scale = 1.0;
    bool   ok = false;
};

// Draws ONLY the ribs, at a given angle, centred at (cx,cy).
inline void draw_knurl(cairo_t* cr, double cx, double cy, double r,
                             double rad, const Theme& th)
{
    // The pattern is the SAME for all ten: in the pre-rotation frame
    // the ten ribs occupy the same rectangle. Creating it ten times was
    // wasted work.
    const double x = cx - 7.5, y = cy - r - 2.5, w = 15.0, h = r * 0.44;
    cairo_pattern_t* rib = linear_obb(x, y, w, h, 0.0, 0.0, 1.0, 0.0);
    add_stop(rib, 0.00, th.rib0);
    add_stop(rib, 0.45, th.rib1);
    add_stop(rib, 0.55, th.rib2);
    add_stop(rib, 1.00, th.rib3);
    for (int i = 0; i < 10; ++i) {
        cairo_save(cr);
        cairo_translate(cr, cx, cy);
        cairo_rotate(cr, rad + (i * 36.0 + 18.0) * M_PI / 180.0);
        cairo_translate(cr, -cx, -cy);
        rounded_rect(cr, x, y, w, h, 6.5);
        cairo_set_source(cr, rib);
        cairo_fill_preserve(cr);
        set_rgba(cr, 0x000000, 0.5);
        cairo_set_line_width(cr, 0.6);
        cairo_stroke(cr);
        cairo_restore(cr);
    }
    cairo_pattern_destroy(rib);
}

// Draws the skirt + cap + arcs disc (what covers the knurl from inside).
inline void draw_disc(cairo_t* cr, double cx, double cy, double r, const Theme& th)
{
    const double capR = r * 0.64;
    // 5. Skirt (VERTICAL gradient: what gives the cylinder its volume).
    {
        cairo_pattern_t* sk = linear_obb(cx - (r - 3), cy - (r - 3),
                                        2 * (r - 3), 2 * (r - 3), 0.0, 0.0, 0.0, 1.0);
        add_stop(sk, 0.00, th.skirt0);
        add_stop(sk, 0.24, th.skirt1);
        add_stop(sk, 0.60, th.skirt2);
        add_stop(sk, 1.00, th.skirt3);
        cairo_arc(cr, cx, cy, r - 3, 0, 2 * M_PI);
        cairo_set_source(cr, sk);
        cairo_fill(cr);
        cairo_pattern_destroy(sk);
    }
    // 6. Cap step.
    set_rgb(cr, 0x08090a);
    cairo_arc(cr, cx, cy, capR + 3.5, 0, 2 * M_PI);
    cairo_fill(cr);
    // 7. FLAT metal cap, DIAGONAL gradient (light from top-left).
    // The handoff is explicit: NO radial shine, no concentric rings.
    // It is flat brushed aluminium, not a chromed dome.
    {
        cairo_pattern_t* cap = linear_obb(cx - capR, cy - capR, 2 * capR, 2 * capR,
                                         0.0, 0.0, 0.35, 1.0);
        // WIDENED RAMP, WITH A KNEE. Four soft stops from 0xee to 0x61 — 141
        // levels spread like a cloud — read as a drawing. Aluminium has a KNEE,
        // a short jump where the surface goes from reflecting light to
        // reflecting shadow; without it the ramp reads as an illustrator's
        // gradient.
        add_stop(cap, 0.00, 0xfdfefe);
        add_stop(cap, 0.22, th.cap0);
        add_stop(cap, 0.40, th.cap1);
        add_stop(cap, 0.48, 0xa8adb5);
        add_stop(cap, 0.56, 0x7d828b);       // the knee
        add_stop(cap, 0.74, th.cap2);
        add_stop(cap, 1.00, 0x4a4f57);
        cairo_arc(cr, cx, cy, capR, 0, 2 * M_PI);
        cairo_set_source(cr, cap);
        cairo_fill(cr);
        cairo_pattern_destroy(cap);

        // BRUSHING on the cap. The design brief asks for "flat brushed
        // aluminium" and the cap was drawn flat without the brushing: adding
        // the grain is not departing from the brief, it is finishing it.
        // LINEAR, not concentric: the brief expressly forbids rings ("no radial
        // shine, no concentric rings"), and besides, a circular grain on a part
        // that ROTATES would read as movement.
        // Deterministic and clipped to the disc, like everything else.
        if (NLSC_CAP_BRUSH > 0.0) {
            cairo_save(cr);
            cairo_arc(cr, cx, cy, capR - 1.0, 0, 2 * M_PI);
            cairo_clip(cr);
            uint32_t e = 0x9E3779B9u ^ uint32_t(int(capR * 16.0));
            auto u01 = [&]() { e = e * 1664525u + 1013904223u; return (e >> 8) / 16777216.0; };
            const int n = int(capR * 2.2);
            for (int i = 0; i < n; ++i) {
                const double t = u01();
                const double off = (t * 2.0 - 1.0) * capR;
                const double largo = capR * (0.55 + 0.45 * u01());
                const bool claro = u01() < 0.5;
                cairo_set_line_width(cr, 0.4 + 0.5 * u01());
                set_rgba(cr, claro ? 0xffffff : 0x2b2f35,
                         NLSC_CAP_BRUSH * (0.45 + 0.55 * u01()));
                // Grain at 20 degrees, aligned with the light's diagonal.
                const double a = 0.349;
                const double dx = std::cos(a) * largo, dy = std::sin(a) * largo;
                const double ox = -std::sin(a) * off, oy = std::cos(a) * off;
                cairo_move_to(cr, cx + ox - dx, cy + oy - dy);
                cairo_line_to(cr, cx + ox + dx, cy + oy + dy);
                cairo_stroke(cr);
            }
            cairo_restore(cr);
        }
    }
    // 8. Dark ring + light inner edge + light arc top-left.
    set_rgba(cr, 0x000000, 0.55);
    cairo_set_line_width(cr, 1.2);
    cairo_arc(cr, cx, cy, capR, 0, 2 * M_PI);
    cairo_stroke(cr);
    set_rgba(cr, 0xffffff, 0.45);
    cairo_set_line_width(cr, 1.0);
    cairo_arc(cr, cx, cy, capR - 1.4, 0, 2 * M_PI);
    cairo_stroke(cr);
    // The SVG's arc («A capR*0.8 ... 0 0 1») is centred on the knob's own
    // centre: its endpoints fall at 0,772 and 0,778 of capR. Drawn as a
    // Cairo arc between those two angles — the same with less trig.
    set_rgba(cr, 0xffffff, 0.50);
    cairo_set_line_width(cr, 2.0);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    cairo_new_path(cr);
    cairo_arc(cr, cx, cy, capR * 0.775,
              std::atan2(-0.46, -0.62), std::atan2(-0.74, 0.24));
    cairo_stroke(cr);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT);
}

// The index: dark groove below, light line on top. Never cached — it
// costs 0,023 ms and is the ONLY thing that truly moves with the value.
inline void draw_pointer(cairo_t* cr, double cx, double cy, double r,
                          double rad, const Theme& th)
{
    const double capR = r * 0.64;
    cairo_save(cr);
    cairo_translate(cr, cx, cy);
    cairo_rotate(cr, rad);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    const double y0 = -capR * 0.12, y1 = -capR + 4.0;
    set_rgba(cr, 0x0a0a0a, 0.6);
    cairo_set_line_width(cr, 4.6);
    cairo_move_to(cr, 0, y0); cairo_line_to(cr, 0, y1);
    cairo_stroke(cr);
    set_rgb(cr, th.indicador);
    cairo_set_line_width(cr, 3.0);
    cairo_move_to(cr, 0, y0); cairo_line_to(cr, 0, y1);
    cairo_stroke(cr);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT);
    cairo_restore(cr);
}

inline void free_knob_cache(KnobCache& m)
{
    if (m.disco) { cairo_surface_destroy(m.disco); m.disco = nullptr; }
    for (int i = 0; i < kKnurlPhases; ++i)
        if (m.moleteado[i]) { cairo_surface_destroy(m.moleteado[i]); m.moleteado[i] = nullptr; }
    m.r = 0.0;
}

// Builds ONE knob's cache at `escala` pixels per unit. The crop is
// square and covers up to r+3 (a rib's outer edge).
inline bool make_knob_cache(KnobCache& m, double r, double scale, const Theme& th)
{
    free_knob_cache(m);
    m.r = r;
    m.lado = 2.0 * (r + 4.0);
    const int px = int(std::ceil(m.lado * scale));
    const double c = m.lado / 2.0;                 // centre within the crop

    m.disco = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, px, px);
    if (cairo_surface_status(m.disco) != CAIRO_STATUS_SUCCESS) return false;
    { cairo_t* c2 = cairo_create(m.disco);
      cairo_scale(c2, scale, scale);
      cairo_set_antialias(c2, CAIRO_ANTIALIAS_GOOD);
      draw_disc(c2, c, c, r, th);
      cairo_destroy(c2); }

    for (int i = 0; i < kKnurlPhases; ++i) {
        m.moleteado[i] = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, px, px);
        if (cairo_surface_status(m.moleteado[i]) != CAIRO_STATUS_SUCCESS) return false;
        cairo_t* c2 = cairo_create(m.moleteado[i]);
        cairo_scale(c2, scale, scale);
        cairo_set_antialias(c2, CAIRO_ANTIALIAS_GOOD);
        const double rad = (36.0 * i / kKnurlPhases) * M_PI / 180.0;
        draw_knurl(c2, c, c, r, rad, th);
        cairo_destroy(c2);
    }
    return true;
}

inline Cache make_cache(double scale, const Theme& th = default_theme())
{
    Cache k;
    k.scale = scale;
    k.tex = make_textures(int(PANEL_W * scale), int(PANEL_H * scale), scale);
    const double radios[kMaxKnobs] = { DRIVE_R, LEVEL_R, TONE_R };
    k.ok = true;
    for (int i = 0; i < kMaxKnobs; ++i)
        if (!make_knob_cache(k.knobs[i], radios[i], scale, th)) k.ok = false;
    k.n = kMaxKnobs;
    return k;
}

inline void free_cache(Cache& k)
{
    free_textures(k.tex);
    for (int i = 0; i < kMaxKnobs; ++i) free_knob_cache(k.knobs[i]);
    k = Cache{};
}

// Stamps a cache surface centred at panel coordinates (cx,cy), AT DEVICE
// RESOLUTION (identity matrix) so it does not interpolate.
inline void stamp(cairo_t* cr, cairo_surface_t* s, double cx, double cy,
                    double lado, double scale)
{
    double dx = cx - lado / 2.0, dy = cy - lado / 2.0;
    cairo_user_to_device(cr, &dx, &dy);
    cairo_save(cr);
    cairo_identity_matrix(cr);
    // Round to the pixel: half a pixel of offset would interpolate again
    // and wash out the knurl — exactly what the cache avoids.
    cairo_set_source_surface(cr, s, std::floor(dx + 0.5), std::floor(dy + 0.5));
    cairo_pattern_set_filter(cairo_get_source(cr), CAIRO_FILTER_NEAREST);
    cairo_paint(cr);
    cairo_restore(cr);
    (void)scale;
}

// The LIVE layer: only the three knobs from the waist up. It is all that
// repaints while dragging, which is why the frame costs what it costs.
//
// With `k` it stamps the cache; without it (or if it could not be built)
// it draws direct, which
// is the FALLBACK path and yields exactly the same pixel bar the knurl's
// phase rounding.
inline void draw_live(cairo_t* cr, const PanelState& st, const Cache* k = nullptr,
                      const Theme& th = default_theme())
{
    const double cx[kMaxKnobs] = { DRIVE_CX, LEVEL_CX, TONE_CX };
    const double cy[kMaxKnobs] = { DRIVE_CY, LEVEL_CY, TONE_CY };
    const double rr[kMaxKnobs] = { DRIVE_R,  LEVEL_R,  TONE_R  };
    const double vv[kMaxKnobs] = { st.drive, st.level, st.tone };

    for (int i = 0; i < kMaxKnobs; ++i) {
        const double grad = KNOB_MIN_DEG + vv[i] * (KNOB_MAX_DEG - KNOB_MIN_DEG);
        const double rad  = grad * M_PI / 180.0;
        if (k && k->ok && k->knobs[i].disco) {
            const KnobCache& m = k->knobs[i];
            // The phase within the knurl's 36° period.
            double fase = std::fmod(grad, 36.0);
            if (fase < 0.0) fase += 36.0;
            int idx = int(fase / 36.0 * kKnurlPhases + 0.5) % kKnurlPhases;
            if (m.moleteado[idx])
                stamp(cr, m.moleteado[idx], cx[i], cy[i], m.lado, k->scale);
            stamp(cr, m.disco, cx[i], cy[i], m.lado, k->scale);
        } else {
            draw_knurl(cr, cx[i], cy[i], rr[i], rad, th);
            draw_disc(cr, cx[i], cy[i], rr[i], th);
        }
        draw_pointer(cr, cx[i], cy[i], rr[i], rad, th);
    }
}

// The COMPLETE panel, in one path. Exists so the PNG preview and the
// host draw exactly the same: when the host caches the static layer and
// the preview calls something else, the PNGs stop reflecting the plugin
// and nobody notices until someone opens it in Ardour.
inline void draw_panel(cairo_t* cr, const PanelState& st, const Fonts* fonts = nullptr,
                       const Textures* tex = nullptr, double scale = 1.0,
                       const Theme& th = default_theme(),
                       cairo_surface_t* logo = nullptr)
{
    draw_static(cr, st, fonts, tex, scale, th, logo);
    draw_live(cr, st, nullptr, th);
}

// --- window backdrop and pedal shadow -----------------------------------------

// The prototype page's radial backdrop, painted over the WHOLE window.
// In WINDOW coordinates, not panel: it is the layer under the pedal.
inline void draw_backdrop(cairo_t* cr, double w, double h)
{
    cairo_pattern_t* b = radial_obb(0, 0, w, h, 0.50, 0.30, 0.72);
    add_stop(b, 0.00, 0x26292e);
    add_stop(b, 0.70, 0x131518);
    add_stop(b, 1.00, 0x0d0f11);
    cairo_rectangle(cr, 0, 0, w, h);
    cairo_set_source(cr, b);
    cairo_fill(cr);
    cairo_pattern_destroy(b);
}

// The pedal's drop shadow (`drop-shadow(0 30px 55px rgba(0,0,0,.65))`).
//
// Cairo has NO blur, so it is FAKED by stacking growing rounded
// rectangles with falling alpha — many faint layers, not two thick ones,
// which is what gives a soft halo instead of a hard edge.
// And the alpha is NOT the CSS's: gaussian blur SPREADS the energy and
// stacking CONCENTRATES it. The workshop's measured equivalence was ~55 %
// of the nominal alpha, whence this 0,36 (0,65 × 0,55).
inline void draw_sombra(cairo_t* cr, double x, double y, double w, double h, double r)
{
    constexpr int CAPAS = 14;
    for (int i = CAPAS; i >= 1; --i) {
        const double e = i * 1.6;                       // reach ~22 px
        const double a = 0.36 * (1.0 - double(i - 1) / CAPAS) / CAPAS * 2.2;
        set_rgba(cr, 0x000000, a);
        rounded_rect(cr, x - e, y - e + 4.0, w + 2 * e, h + 2 * e, r + e);
        cairo_fill(cr);
    }
}

} // namespace ui
} // namespace nlsc
