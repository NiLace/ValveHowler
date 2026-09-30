// GENERATED FILE — DO NOT EDIT BY HAND.
//
// Where the photoreal GUI's parts land in its renders, in ASSET pixels (the renders' own
// grid, top-left origin), projected from the Blender scene. Regenerate with
// gui/scene/build_assets.sh then gui/scene/pack_assets.py.
#pragma once

namespace nlsc {
namespace ui {
namespace photo {

inline constexpr double ASSET_W = 900;
inline constexpr double ASSET_H = 1350;

// A knob: its base circle (the hit zone and the paste mask) and its strip's box.
struct KnobGeo {
    const char* name;
    double cx, cy, r;            // base circle
    int    x0, y0, x1, y1;       // strip frame box
    int    frames;
    double deg_first, deg_last;  // pointer angle of the first and last frame
};

inline constexpr KnobGeo KNOBS[3] = {
    { "drive", 281.99, 363.60, 81.00, 196, 278, 367, 449, 64, -135.00, 135.00 },
    { "level", 618.01, 363.60, 81.00, 533, 278, 704, 449, 64, -135.00, 135.00 },
    { "tone", 450.00, 490.34, 69.75, 376, 416, 524, 566, 64, -135.00, 135.00 },
};

struct Box { double x0, y0, x1, y1; };

inline constexpr int TREADLE_X0 = 226, TREADLE_Y0 = 942, TREADLE_X1 = 674, TREADLE_Y1 = 1237;
inline constexpr int TREADLE_FRAMES = 5;
inline constexpr Box TREADLE_HIT = { 230.19, 946.71, 669.81, 1231.21 };
inline constexpr Box DISPLAY = { 204.11, 588.68, 695.89, 665.25 };
inline constexpr Box DISPLAY_BEZEL = { 193.33, 577.88, 706.67, 676.05 };
inline constexpr Box LABEL_PLATE = { 232.40, 712.09, 667.60, 920.70 };

}  // namespace photo
}  // namespace ui
}  // namespace nlsc
