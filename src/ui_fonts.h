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
// possibly after the host has closed the editor and unloaded this module (the
// shipped module is linked -z nodelete and stays mapped; a build without it,
// with -fvisibility=hidden, really unloads, and the fonts must not depend on
// which); when Cairo finally drops the face,
// FreeType frees the library and the face, and nothing calls back into this
// module (with a destructor in the module the host jumps into
// unmapped code; gate `make ui-unload`).
//
// `FT_Done_FreeType` is `FT_Error(*)(FT_Library)` called as Cairo's
// `void(*)(void*)`: a conversion the C++ standard leaves undefined and the x86-64
// System V ABI makes exact (the pointer travels in the same register, the int
// result is discarded). A wrapper of the right type would be code in THIS
// module, which is what crashed.

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
    cairo_font_face_t* doto = nullptr;   // the variant display and its list
    bool ok = false;                     // true only if the face loaded
};


// Key the private FT_Library hangs from on its Cairo face.
inline cairo_user_data_key_t* ft_library_key()
{
    static cairo_user_data_key_t key;
    return &key;
}

// One face from one bundled file, owning its own FreeType library (see the top).
inline cairo_font_face_t* load_face(const std::string& path)
{
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
    // FreeType's own function as the destroy callback, not a wrapper of ours:
    // see the top of this file.
    if (cairo_font_face_set_user_data(cf, ft_library_key(), lib,
                                      reinterpret_cast<cairo_destroy_func_t>(
                                          reinterpret_cast<void (*)()>(&FT_Done_FreeType)))
        != CAIRO_STATUS_SUCCESS) {
        cairo_font_face_destroy(cf);
        FT_Done_FreeType(lib);
        return nullptr;
    }
    return cf;
}

struct SharedFace {
    std::string path;
    cairo_font_face_t* face = nullptr;
    ~SharedFace() { if (face) cairo_font_face_destroy(face); }
};

inline cairo_font_face_t* shared_face(const std::string& path)
{
    static SharedFace s;
    if (!s.face || s.path != path) {
        if (s.face) cairo_font_face_destroy(s.face);
        s.face = load_face(path);
        s.path = path;
    }
    return s.face ? cairo_font_face_reference(s.face) : nullptr;
}

inline Fonts load_fonts(const std::string& dir)
{
    Fonts f;
    const std::string base = dir.empty() ? "." : dir;
    f.doto = shared_face(base + "/Doto-Round-Bold.ttf");
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
