#pragma once
// ui_logo.h — the pedal's logo, loaded from the BUNDLE like the fonts.
//
// IT IS PAINTED AS A MASK, NOT AS AN IMAGE. The PNG is black line art, and
// blitting it as-is would leave a BLACK drawing on the green chassis, which is
// not what screen printing looks like. What is used is its coverage as a MASK,
// filled with the screen-print colour, so the logo comes out of the same cream
// as the rest of the panel and reads as ink on the box rather than a sticker.
// => Useful side effect: the logo's colour is decided by the THEME, so it can
// follow state like any other ink.
//
// `draw_logo` does NOT live here but in `ui_draw.h`: it uses `set_rgba`, which
// is defined there. This header keeps loading and freeing, which is the part
// that does not depend on the drawing.
//
// The PNG is 1024x1024 and is drawn at ~104: it is scaled on the fly with a
// GOOD filter, and since it lives in the STATIC layer that happens once per
// cache rebuild, not once per frame.
#include <cairo/cairo.h>
#include <string>

namespace nlsc {
namespace ui {

// THE MASK IS DERIVED FROM LUMINANCE, NOT FROM THE ALPHA CHANNEL.
//
// `Logo.png` has NO transparency: it is three RGB channels, and the
// "transparency checkerboard" is PAINTED INTO the image (pixel (5,5) is grey
// 204). Using its alpha as a mask gives a SOLID SQUARE, because to Cairo the
// whole rectangle is opaque.
//
// => An A8 mask is built here instead: ink where the pixel is DARK. The
// checkerboard sits between 204 and 255 and the line art near 0, so a
// threshold at 170 separates them with room to spare and the ramp keeps the
// stroke's antialiasing. This also survives the file being replaced by one
// with real alpha: dark line art on a light ground still reads the same.
inline cairo_surface_t* load_logo(const std::string& dir)
{
    const std::string base = dir.empty() ? "." : dir;
    cairo_surface_t* src = cairo_image_surface_create_from_png((base + "/Logo.png").c_str());
    if (!src) return nullptr;
    if (cairo_surface_status(src) != CAIRO_STATUS_SUCCESS) {
        cairo_surface_destroy(src);
        return nullptr;   // without a logo the rest still draws: this does not sink the GUI
    }
    const int w = cairo_image_surface_get_width(src);
    const int h = cairo_image_surface_get_height(src);
    cairo_surface_flush(src);
    const unsigned char* sd = cairo_image_surface_get_data(src);
    const int ss = cairo_image_surface_get_stride(src);
    const bool con_alfa = cairo_image_surface_get_format(src) == CAIRO_FORMAT_ARGB32;

    cairo_surface_t* m = cairo_image_surface_create(CAIRO_FORMAT_A8, w, h);
    if (cairo_surface_status(m) != CAIRO_STATUS_SUCCESS) { cairo_surface_destroy(src); return nullptr; }
    unsigned char* md = cairo_image_surface_get_data(m);
    const int ms = cairo_image_surface_get_stride(m);

    const double UMBRAL = 170.0, RAMPA = 95.0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const unsigned char* px = sd + y * ss + x * 4;   // BGRA/BGRX in memory
            const double b = px[0], g = px[1], r = px[2], a = con_alfa ? px[3] : 255.0;
            // If the file DOES carry alpha it is honoured: a transparent pixel
            // is not ink, however dark its RGB.
            const double lum = 0.299 * r + 0.587 * g + 0.114 * b;
            double t = (UMBRAL - lum) / RAMPA;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;
            md[y * ms + x] = (unsigned char)(t * (a / 255.0) * 255.0);
        }
    }
    cairo_surface_mark_dirty(m);
    cairo_surface_destroy(src);

    // IT IS CROPPED TO THE INK BOX.
    //
    // The file is a 1024x1024 canvas with the drawing inside and a LOT of empty
    // margin: measured, the ink occupies 676x882 at offset (203, 74) -- 203 px
    // of air on the left alone, 20 % of the width. Both consequences are bad:
    //   · the X that is asked for positions the CANVAS, not the stroke, so
    //     "move it left" has no precise answer;
    //   · asking for a 210-wide logo gives a 138x181 drawing, because the fit
    //     looks at the canvas side. The requested size is not the visible size.
    //
    // => Cropping here makes `NLSC_LOGO_X` the left edge OF THE STROKE and
    // `NLSC_LOGO_SIDE` its real height, so both controls mean what they say.
    // It also survives the file being replaced by one with a different margin:
    // the box is measured again.
    int x0 = w, y0 = h, x1 = -1, y1 = -1;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            if (md[y * ms + x] > 8) {
                if (x < x0) x0 = x;
                if (x > x1) x1 = x;
                if (y < y0) y0 = y;
                if (y > y1) y1 = y;
            }
    if (x1 < x0 || y1 < y0) return m;          // all empty: returned as it is
    const int cw = x1 - x0 + 1, ch = y1 - y0 + 1;
    if (cw == w && ch == h) return m;          // already tight

    cairo_surface_t* c = cairo_image_surface_create(CAIRO_FORMAT_A8, cw, ch);
    if (cairo_surface_status(c) != CAIRO_STATUS_SUCCESS) { cairo_surface_destroy(c); return m; }
    unsigned char* cd = cairo_image_surface_get_data(c);
    const int cs = cairo_image_surface_get_stride(c);
    for (int y = 0; y < ch; ++y)
        for (int x = 0; x < cw; ++x)
            cd[y * cs + x] = md[(y + y0) * ms + (x + x0)];
    cairo_surface_mark_dirty(c);
    cairo_surface_destroy(m);
    return c;
}

inline void free_logo(cairo_surface_t*& s)
{
    if (s) { cairo_surface_destroy(s); s = nullptr; }
}

}  // namespace ui
}  // namespace nlsc
