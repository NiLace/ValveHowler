// ui_fonts.h — loads the embedded TTFs into Cairo faces, via FreeType.
//
// The «NLS Cream» panel is typographic: Archivo for the name and the
// variant, Barlow Condensed for the knob labels, Space Mono for the
// dropdown. The host is NOT trusted to have those families installed
// (it usually does not). The `.ttf` files travel INSIDE the LV2 bundle
// and load from there, so the panel draws the same on any machine.
// fontconfig is skipped entirely.
//
// Each `cairo_font_face_t` owns its `FT_Face` through a user-data
// destructor, so destroying the face also frees FreeType's. The shared
// `FT_Library` lives in the struct and closes in `free_fonts()`.
//
// Copied from `TheOmegaCake/src/ui_fonts.h`, adapted to four other
// families. The SAME technique, not a redesign: a fix to the FreeType
// lifetime handling must land in both.

#pragma once

#include <cairo/cairo.h>
#include <cairo/cairo-ft.h>
#include <ft2build.h>
#include FT_FREETYPE_H

#include <string>

namespace nlsc {
namespace ui {

// The four faces the panel needs. Any may stay null if its file is
// missing; the drawing code then falls back to a Cairo "toy" font, which
// fontconfig resolves to something close. It degrades, it does not crash.
struct Fonts {
    FT_Library ft = nullptr;
    cairo_font_face_t* archivo800  = nullptr;  // «VALVE HOWLER»
    cairo_font_face_t* archivo700i = nullptr;  // variant name (italic)
    cairo_font_face_t* barlow600   = nullptr;  // knob labels + subtitle
    cairo_font_face_t* mono400     = nullptr;  // dropdown
    bool ok = false;                            // true only if all four loaded
};

// Key the FT_Face hangs from on its Cairo face, for destruction.
inline cairo_user_data_key_t* ft_face_key()
{
    static cairo_user_data_key_t key;
    return &key;
}

// A wrapper instead of `reinterpret_cast`-ing `FT_Done_Face`: Cairo
// wants a `void(*)(void*)` and FreeType gives an `FT_Error(*)(FT_Face)`.
// Converting between incompatible function types and CALLING through the
// converted pointer is UB, and GCC flags it with `-Wcast-function-type`.
// The repo builds with `-Wall -Wextra`, and a GUI that adds a new warning
// teaches everyone to ignore warnings.
extern "C" inline void suelta_ft_face(void* p)
{
    if (p) FT_Done_Face(static_cast<FT_Face>(p));
}

inline cairo_font_face_t* load_face(FT_Library ft, const std::string& path)
{
    FT_Face face = nullptr;
    if (FT_New_Face(ft, path.c_str(), 0, &face) != 0) return nullptr;
    cairo_font_face_t* cf = cairo_ft_font_face_create_for_ft_face(face, 0);
    if (cairo_font_face_status(cf) != CAIRO_STATUS_SUCCESS) {
        cairo_font_face_destroy(cf);
        FT_Done_Face(face);
        return nullptr;
    }
    // The FT_Face's life is tied to the Cairo face: destroying it calls
    // FT_Done_Face. If that wiring fails, clean up by hand.
    if (cairo_font_face_set_user_data(cf, ft_face_key(), face, suelta_ft_face)
        != CAIRO_STATUS_SUCCESS) {
        cairo_font_face_destroy(cf);
        FT_Done_Face(face);
        return nullptr;
    }
    return cf;
}

// ONE FT_Library PER PROCESS, OPENED ONCE AND NEVER CLOSED — and this is a
// crash fix, not tidiness.
//
// `cairo_font_face_destroy()` does not destroy the face: it drops OUR reference.
// Cairo keeps its own inside a PROCESS-WIDE scaled-font cache, so the user-data
// callback that calls `FT_Done_Face` does NOT run when we ask it to. Closing the
// library right afterwards therefore tears it down UNDER faces cairo is still
// holding, and the next instance dies inside `cairo_text_extents` ->
// `FT_Set_Transform` on a dangling face.
//
// That sequence is a host reopening the plugin window: instantiate, cleanup,
// instantiate. Measured — same binary, one env var apart: with
// `FT_Done_FreeType`, SIGSEGV on the second instantiate; without it, clean.
//
// The library is a few KB and its lifetime cannot be shorter than a cache we do
// not own, so it lasts as long as the process. What still gets released properly
// is each FACE, whenever cairo drops it — safe now, because the library outlives
// them. `make gui-touch` walks the path and is the gate.
inline FT_Library shared_ft()
{
    static FT_Library ft = nullptr;
    static bool tried = false;
    if (!tried) {
        tried = true;
        if (FT_Init_FreeType(&ft) != 0) ft = nullptr;
    }
    return ft;
}

// Loads the four faces from `dir` — the running LV2 bundle's path, or
// `assets/fonts` for the standalone preview.
inline Fonts load_fonts(const std::string& dir)
{
    Fonts f;
    f.ft = shared_ft();
    if (!f.ft) return f;
    const std::string base = dir.empty() ? "." : dir;
    f.archivo800  = load_face(f.ft, base + "/Archivo-800.ttf");
    f.archivo700i = load_face(f.ft, base + "/Archivo-700-Italic.ttf");
    f.barlow600   = load_face(f.ft, base + "/BarlowCondensed-600.ttf");
    f.mono400     = load_face(f.ft, base + "/SpaceMono-400.ttf");
    f.ok = f.archivo800 && f.archivo700i && f.barlow600 && f.mono400;
    return f;
}

inline void free_fonts(Fonts& f)
{
    // Drop OUR reference to each face. Cairo may hold its own for a while
    // longer, and that is fine: the FT_Face dies with the last reference, and
    // the library that owns it is process-wide (`shared_ft`).
    //
    // The library is NOT closed here. It used to be, and it was a crash on
    // the second open of the window — the reasoning is on `shared_ft`.
    if (f.archivo800)  cairo_font_face_destroy(f.archivo800);
    if (f.archivo700i) cairo_font_face_destroy(f.archivo700i);
    if (f.barlow600)   cairo_font_face_destroy(f.barlow600);
    if (f.mono400)     cairo_font_face_destroy(f.mono400);
    f = Fonts{};
}

} // namespace ui
} // namespace nlsc
