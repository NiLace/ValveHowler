// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// ui_fonts.h — loads the embedded TTFs into Cairo faces, via FreeType.
//
// The panel's lettering is in its images; the one face drawn live is Doto, the
// dot-matrix face of the variant display and its list. The host system is not
// assumed to have it installed: the `.ttf` travels inside the LV2 bundle and
// loads from there, so the panel draws the same on any machine.
// fontconfig is skipped entirely.
//
// Each `cairo_font_face_t` owns its `FT_Face` through a user-data
// destructor, so destroying the face also frees FreeType's once Cairo lets it
// go. The `FT_Library` is shared by the process and never closed (see
// `shared_ft()`): Cairo keeps faces cached past our destroy.

#pragma once

#include <cairo/cairo.h>
#include <cairo/cairo-ft.h>
#include <ft2build.h>
#include FT_FREETYPE_H

#include <string>

namespace nlsc {
namespace ui {

// The face the panel draws live. It may stay null if its file is missing; the
// drawing code then falls back to a Cairo "toy" font, which fontconfig resolves
// to something close. It degrades, it does not crash.
struct Fonts {
    FT_Library ft = nullptr;
    cairo_font_face_t* doto = nullptr;   // the variant display and its list
    bool ok = false;                     // true only if the face loaded
};

// Key the FT_Face hangs from on its Cairo face, for destruction.
inline cairo_user_data_key_t* ft_face_key()
{
    static cairo_user_data_key_t key;
    return &key;
}

// A wrapper instead of `reinterpret_cast`-ing `FT_Done_Face`: Cairo
// wants a `void(*)(void*)` and FreeType gives an `FT_Error(*)(FT_Face)`.
// Converting between incompatible function types and calling through the
// converted pointer is UB, and GCC flags it with `-Wcast-function-type`.
extern "C" inline void release_ft_face(void* p)
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
    if (cairo_font_face_set_user_data(cf, ft_face_key(), face, release_ft_face)
        != CAIRO_STATUS_SUCCESS) {
        cairo_font_face_destroy(cf);
        FT_Done_Face(face);
        return nullptr;
    }
    return cf;
}

// One FT_Library per process, opened once and never closed.
//
// `cairo_font_face_destroy()` does not destroy the face: it drops our reference.
// Cairo keeps its own inside a process-wide scaled-font cache, so the user-data
// callback that calls `FT_Done_Face` does not run when we ask it to. Closing the
// library then would tear it down under faces cairo still holds, and the next
// instance (a host reopening the plugin window) would crash inside
// `cairo_text_extents` -> `FT_Set_Transform` on a dangling face.
//
// The library is a few KB and its lifetime cannot be shorter than a cache we do
// not own, so it lasts as long as the process. Each face is still released
// whenever cairo drops it.
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

// Loads the face from `dir`: the running LV2 bundle's path, or
// `assets/fonts` for the standalone preview.
inline Fonts load_fonts(const std::string& dir)
{
    Fonts f;
    f.ft = shared_ft();
    if (!f.ft) return f;
    const std::string base = dir.empty() ? "." : dir;
    f.doto = load_face(f.ft, base + "/Doto-Round-Bold.ttf");
    f.ok = f.doto != nullptr;
    return f;
}

inline void free_fonts(Fonts& f)
{
    // Drop our reference to the face. Cairo may hold its own for a while
    // longer, and that is fine: the FT_Face dies with the last reference, and
    // the library that owns it is process-wide and not closed here (see
    // `shared_ft`).
    if (f.doto) cairo_font_face_destroy(f.doto);
    f = Fonts{};
}

} // namespace ui
} // namespace nlsc
