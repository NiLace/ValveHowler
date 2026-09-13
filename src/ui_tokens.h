#pragma once
// ui_tokens.h — the VISUAL TOKENS whose value is a design decision.
//
// WHY THEY LIVE HERE AND NOT IN `ui_draw.h`. They were born inside the drawing
// file, and the decision gate rejected them with good reason: that gate demands
// a PROBE that publishes the value actually compiled, and the probe cannot
// include `ui_draw.h` because it drags Cairo in.
//
// => Separating them is not a detour to please a gate: a design token is not
// drawing code. Nothing is included here, so the same probe that publishes the
// DSP switches publishes these, and a silent change to any of them shows up in
// `make test`.
//
// These values are design choices made looking at renders, not technical
// adjustments. The reason for each one is recorded in `docs/DECISIONES.tsv`.

// The core of the lit name. Deliberately light: against the green chassis what
// separates is LUMINANCE, not hue — a red and a sky blue of luminosity close to
// the green's both failed before this one.
#ifndef NLSC_NAME_ON
#define NLSC_NAME_ON 0xbfe4ff
#endif

// The halo around it. Dark: the glow carries the "lit" and the core carries the
// legibility, which is why they can be different colours.
#ifndef NLSC_NAME_GLOW
#define NLSC_NAME_GLOW 0x005c92
#endif

// Halo radius, in panel units. Chosen from a render of four widths
// (11 / 14 / 17 / 21).
#ifndef NLSC_NAME_GLOW_R
#define NLSC_NAME_GLOW_R 17.0
#endif

// The graduation dots and the knob labels while the pedal is lit. SOLID, with
// no halo: glow on everything distracts and reads as kitsch.
#ifndef NLSC_RULE_ON
#define NLSC_RULE_ON 0x0072b2
#endif

// How far the WHOLE three-knob group rises, now that there is no LED.
// One constant on purpose: three coordinates edited by hand is how a relation
// breaks with nobody seeing it. The mouse map follows it on its own.
#ifndef NLSC_KNOBS_UP
#define NLSC_KNOBS_UP 22.0
#endif

// There are no blue horizontal rules any more; they were removed so the logo
// and the lettering could grow. With them went `NLSC_RULE_TOP_Y`: the identity
// band is no longer bounded by two lines but by the real geometry — the TONE
// knob's marks above and the footswitch well below — so it is DERIVED in the
// drawing instead of being written here.
// `NLSC_RULE_ON` stays alive: it is the blue of the graduation DOTS.

// The logo, to the left of the identity block: the side of its square box and
// its left margin.
#ifndef NLSC_LOGO_X
#define NLSC_LOGO_X 48.0
#endif
#ifndef NLSC_LOGO_SIDE
#define NLSC_LOGO_SIDE 186.0
#endif

// There is no `NLSC_TEXT_GAP`. It was the separation between logo and text, and
// it died when the two were DECOUPLED: since the text starts at `NLSC_TEXT_X`
// there is no separation left to compute. It was deleted rather than left at
// zero — a live constant that describes nothing is the one the next reader
// touches expecting it to do something.

// Margin of the identity band against its hard neighbours: the TONE knob's
// marks above and the footswitch well below.
#ifndef NLSC_BAND_MARGIN
#define NLSC_BAND_MARGIN 4.0
#endif

// STROKE THICKENING: zero. It was tried and rejected. The mechanism is kept at
// 0 rather than deleted, because the halo was measured and rejected too, so
// whoever wants more contrast next knows both of these routes have already been
// tried and turned down.
#ifndef NLSC_LOGO_BOLD
#define NLSC_LOGO_BOLD 0.0
#endif

// The logo's colour when the pedal is lit. Unlit it uses the cream of the
// screen print, like the rest of the ink.
// This is the third route tried against the same complaint, that the drawing
// contrasts poorly. The other two — the HALO and the stroke thickening — were
// rejected.
// => This one works because it attacks the right axis: against a mid green what
// separates is LUMINANCE, and neither of the others touched it. One added light
// around the stroke, the other more ink of the same hue.
// This dark, the logo is told apart by DARKNESS rather than by colour. Good for
// colour blindness; in exchange it converses less with the dots and the text.
#ifndef NLSC_LOGO_ON
#define NLSC_LOGO_ON 0x002e4a
#endif

// Height of the footswitch WELL. It is anchored at the BOTTOM (see
// `FS_WELL_Y`): shrinking it frees space ABOVE, which is where the logo eats
// from, and leaves the lower edge still, which is the one the dropdown sits
// against.
// A real footswitch plate is about 1,6:1. With the well at 184 ours comes out
// at 1,64; flattening it further moves away from that proportion, and that is a
// product decision rather than a free adjustment.
#ifndef NLSC_WELL_H
#define NLSC_WELL_H 184.0
#endif

// HOW MUCH OF ITS BOX THE CHARACTER FILLS.
//
// These are two different things and used to be one: `NLSC_LOGO_SIDE` is the
// BOX the logo reserves — the text's position and body come off it, because the
// text starts where that box ends — and this is how much of that box the
// DRAWING occupies. Separating them allows the character to shrink WITHOUT
// moving the text or changing its size.
#ifndef NLSC_LOGO_FILL
#define NLSC_LOGO_FILL 0.84
#endif

// VERTICAL STRETCH OF THE TEXT.
//
// The text's body is fixed by its WIDTH — it has to end at the edge of the well
// — so all the available height went unused. Rather than raising the body
// (which would push it off the panel to the right) it is stretched vertically
// only, which is what uses that space.
// This is a CAP, not a fixed value: the real stretch is computed so the
// two-line block ends up as tall as the LOGO, and this number only stops an odd
// geometry from producing deformed letters.
#ifndef NLSC_TEXT_STRETCH_MAX
#define NLSC_TEXT_STRETCH_MAX 1.6
#endif

// WHERE THE TEXT STARTS, and why it is a number rather than a formula.
//
// It used to come from `NLSC_LOGO_X + stroke_width + a gap`, so the text hung
// off the logo. That served to DERIVE its body once — the requirement being
// that its right edge line up with the well's — but it had an awkward side
// effect: every retouch of the drawing moved and resized text that was already
// approved.
//
// => The two are DECOUPLED. This value is what the formula produced with the
// logo at x=48, that is, the approved state, frozen. The body is still computed
// to reach the edge of the well; what stops depending on the logo is WHERE it
// starts.
// The price, written down so nobody is surprised: if the logo is ever moved a
// long way, air can open up between it and the text, and this has to be
// adjusted by hand.
#ifndef NLSC_TEXT_X
#define NLSC_TEXT_X 220.55782312925171
#endif

// Cap on the text body. It only prevents an absurdity if the gap between the
// text and the edge of the well ever becomes enormous; in today's geometry it
// does not bite.
#ifndef NLSC_TEXT_MAX
#define NLSC_TEXT_MAX 72.0
#endif

// Strength of the brushing on the knob CAP. The design brief asks for flat
// brushed aluminium and the cap was drawn flat without the brushing.
#ifndef NLSC_CAP_BRUSH
#define NLSC_CAP_BRUSH 0.085
#endif
