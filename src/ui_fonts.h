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
// Each `cairo_font_face_t` owns a PRIVATE `FT_Library` holding its one `FT_Face`,
// hung on the face as user data whose destructor is FreeType's own
// `FT_Done_FreeType`. Cairo keeps faces in a process-wide cache past our destroy,
// possibly after the host has closed the editor and unloaded this module (built
// with -fvisibility=hidden, it really unloads); when Cairo finally drops the face,
// FreeType frees the library and the face, and nothing calls back into this
// module (ValveHowler#11: with a destructor in the module the host jumped into
// unmapped code; gate `make ui-unload`).
//
// `FT_Done_FreeType` is `FT_Error(*)(FT_Library)` called as Cairo's
// `void(*)(void*)`: a conversion the C++ standard leaves undefined and the x86-64
// System V ABI makes exact (the pointer travels in the same register, the int
// result is discarded). A wrapper of the right type would be code in THIS
// module, which is what crashed.
//
// NLSC_UI_FONTS_FT_CALLBACK=1 restores the old fonts (one shared library, a
// release wrapper in the module), for the gate's negative arm only.

#pragma once

#ifndef NLSC_UI_FONTS_FT_CALLBACK
#define NLSC_UI_FONTS_FT_CALLBACK 0
#endif

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
    cairo_font_face_t* doto = nullptr;   // the variant display and its list
    bool ok = false;                     // true only if the face loaded
};

#if NLSC_UI_FONTS_FT_CALLBACK
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

inline cairo_font_face_t* load_face_ft(FT_Library ft, const std::string& path)
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

#endif

// Key the private FT_Library hangs from on its Cairo face.
inline cairo_user_data_key_t* ft_library_key()
{
    static cairo_user_data_key_t key;
    return &key;
}

// One face from one bundled file, owning its own FreeType library (see the top).
inline cairo_font_face_t* load_face(const std::string& path)
{
#if NLSC_UI_FONTS_FT_CALLBACK
    FT_Library ft = shared_ft();
    return ft ? load_face_ft(ft, path) : nullptr;
#else
    FT_Library lib = nullptr;
    if (FT_Init_FreeType(&lib) != 0) return nullptr;
    FT_Face face = nullptr;
    if (FT_New_Face(lib, path.c_str(), 0, &face) != 0) { FT_Done_FreeType(lib); return nullptr; }
    cairo_font_face_t* cf = cairo_ft_font_face_create_for_ft_face(face, 0);
    if (cairo_font_face_status(cf) != CAIRO_STATUS_SUCCESS) {
        cairo_font_face_destroy(cf);
        FT_Done_FreeType(lib);
        return nullptr;
    }
    if (cairo_font_face_set_user_data(cf, ft_library_key(), lib,
                                      reinterpret_cast<cairo_destroy_func_t>(
                                          reinterpret_cast<void (*)()>(&FT_Done_FreeType)))
        != CAIRO_STATUS_SUCCESS) {
        cairo_font_face_destroy(cf);
        FT_Done_FreeType(lib);
        return nullptr;
    }
    return cf;
#endif
}

// Loads the face from `dir`: the running LV2 bundle's path, or
// `assets/fonts` for the standalone preview.
inline Fonts load_fonts(const std::string& dir)
{
    Fonts f;
    const std::string base = dir.empty() ? "." : dir;
    f.doto = load_face(base + "/Doto-Round-Bold.ttf");
    f.ok = f.doto != nullptr;
    return f;
}

inline void free_fonts(Fonts& f)
{
    // Drop our reference to the face. Cairo may hold its own for a while
    // longer, and that is fine: the face's private library, and its FT_Face,
    // go with the last reference, by FreeType's own code.
    if (f.doto) cairo_font_face_destroy(f.doto);
    f = Fonts{};
}

} // namespace ui
} // namespace nlsc
