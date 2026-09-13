// nls_variantes.h — THE circuit-variant table. Single source of truth.
//
// WHY IT LIVES IN ITS OWN HEADER AND NOT IN THE PLUGIN: TWO different
// binaries need it. The DSP (`nls_valvehowler.cpp`) wants the resistor values;
// the GUI (`ui_x11.cpp`) wants the dropdown labels — and they are two .so
// files the host loads separately. With the table written into each, adding
// a variant in one place and not the other leaves the dropdown offering a
// circuit that does not exist — or hiding one that does — and raises NO
// error: a rule written twice diverging, with two binaries in between, which
// is the hard-to-see version.
//
// The third copy is the `.ttl` (`lv2:scalePoint`), which cannot include a
// header. Its labels must be THE SAME strings as here; that is the one
// duplication left, and it is noted in the `.ttl` itself.
//
// ─────────────────────────────────────────────────────────────────────────────
// WHERE THE NUMBERS COME FROM — which is what decides if this table may grow
// ─────────────────────────────────────────────────────────────────────────────
// `docs/VARIANTES.md` audits the three axes that would separate one model
// from another:
//
//   §1 output resistors .......... COMPLETE, three independent sources
//   §2 each model's opamp ........ EMPTY (no macromodel for any of them)
//   §3 the model 5's bias ........ EMPTY ("some components" is not a value)
//
// => This table is EXACTLY axis §1 and cannot have more rows than that axis
// has distinct circuits. The three §1 sources: Keen's table (verified against
// the text), Wampler's big book — which marks them on a PHOTO OF THE BOARD,
// `808-1` = 100 Ω and `808-2` = 10 kΩ, p. 255 — and the netlist transcribed
// from the original Eagle files, which carries the factory 808.
//
// MODELS 9, 9RI AND 5 SHARE ONE ROW because in our model they are
// ELECTRICALLY IDENTICAL: they share the 470/100k pair, and the only thing
// that would separate them is the opamp (axis §2, empty). Offering three
// modes that sound exactly the same would be lying to the user.
//
// MODEL 10 IS ABSENT, and it is not an oversight: it needs a NEW ELEMENT
// in the `RA` position — topology, not a value — and its value is disputed
// between sources (Keen says 1 kΩ, others 220 Ω; `VERIFICACION_NETLIST.md`
// §13). Writing a number there would be filling a gap the sources do not
// give.
//
// WHEN ADDING A VARIANT, IT GOES AT THE END. The port carries an INDEX, so
// inserting in the middle changes the circuit under every saved session.

#pragma once

namespace nlsc {

// TWO TABLES, AND THE SPLIT IS LOAD-BEARING.
//
// A CIRCUIT is what §1 above audits: a pair of resistors, and with it a whole
// generated coefficient bank. A VARIANT is a ROW OF THE SELECTOR the user
// sees, and it is a PAIR: (circuit, knob law). They used to be the same
// thing, and while every row was a different circuit nothing distinguished
// them.
//
// WHY THEY MUST NOT STAY MERGED: `nls_juegos.h` carries a `static_assert`
// on the table's size whose whole job is to refuse to compile when a row is
// announced without its bank. Bumping that number to fit a row that is NOT a
// new circuit would switch off the guard for the next row that IS one —
// disarming a gate to let through the case it was not built for.
//
// => The assert now counts CIRCUITS, which is what it always meant, and rows
// may grow without touching it.
struct Circuit {
    double rout_ser;        // `R14`, in series towards the output
    // This is `R15`, the OUTPUT LOAD after the
    // coupling cap — NOT Q2's emitter. `R13` (the emitter) is 10 k in both
    // models, so the two variants share Q2's quiescent point and the
    // transistor stays in class A in both. What this resistor sets is the
    // output divider with `R14`, worth 0,10 dB — Keen's "admittedly very
    // subtle". The old text described a modelling defect, and everything it
    // justified (cut-off, -14,4 dB of shape, the Newton subsystem's +52 % of
    // CPU) fell with it. Primary source and measurements:
    // `docs/OD9_SHUNT_NODE_SIZING.md`.
    double rout_shunt;
};

// THE TONE KNOB'S LAW — position -> physical parameter.
//
// It is NOT a circuit change and no null can see it: the null runs on the
// PHYSICAL parameter and this lives on top of it
// value. What it decides is which part
// of the pot's physical travel the user is given.
//
//   Pot   the real pot's G taper — FAITHFUL to the hardware, and the
//         default. Validated against a capture of the OD-9's original within 0,7 dB.
//   Even  derived by INVERTING the measured brightness so that dB come out
//         even across the knob. Measured on the product: the worst 0,1 step
//         goes from 11,03 dB to 2,79, and the last tenth of travel stops
//         carrying 67,2 % of the range. Deliberately NOT faithful — it is
//         re-voicing, and it is offered as its own row, never as a default.
enum class ToneLaw { Pot = 0, Even = 1 };

struct Variant {
    const char* etiqueta;   // what the GUI shows AND the `lv2:scalePoint` label
    int         circuit;    // index into `kCircuits`
    ToneLaw     tone_law;
};

// Labels are ALREADY UPPERCASE, with the exact spelling that gets drawn.
// The design handoff calls `toUpperCase()` in the view; doing it here keeps
// the GUI and the `.ttl` from ending up with two spellings of one model.
// DO NOT CROSS THESE VALUES BETWEEN VARIANTS — MEASURED.
//
// They were made overridable to ask the circuit which of the two resistors
// carries the 808's input ceiling. That experiment is INVALID, and the
// measurement shows it by itself: giving the 808 BOTH of the 9RI's resistors,
// the ANMR at 1,0 V reads +14,23 dB, while the real 9RI reads −27,13. That
// is 41 dB between "the 9RI" and "the 808 wearing the 9RI's numbers".
//
// => A VARIANT IS NOT TWO RESISTORS: IT IS A COEFFICIENT BANK. Each variant
// carries its own, generated from ngspice on ITS netlist — the variant gate
// measures that they differ in 36 of 38 (drive rest), 58 of 95 (fijos_s) and
// 31 of 91 (level). Changing the resistor without regenerating the bank
// leaves the model INCOHERENT, with no error raised: just a null 40 dB
// worse.
//
// => To try a different resistor, regenerate the whole variant
// (`gen_variantes.sh`) — do not edit this number.
//
// Without a -D on the command line these evaluate to EXACTLY the usual
// values and the product .so comes out bit-identical, which is the control
// that this parametrisation changes nothing by itself.
#ifndef NLSC_V808_RSER
#  define NLSC_V808_RSER 100.0
#endif
#ifndef NLSC_V808_RSHUNT
#  define NLSC_V808_RSHUNT 10e3
#endif
#ifndef NLSC_V9RI_RSER
#  define NLSC_V9RI_RSER 470.0
#endif
#ifndef NLSC_V9RI_RSHUNT
#  define NLSC_V9RI_RSHUNT 100e3
#endif

inline constexpr Circuit kCircuits[] = {
    { NLSC_V808_RSER, NLSC_V808_RSHUNT },   // 0 — the factory 808
    { NLSC_V9RI_RSER, NLSC_V9RI_RSHUNT },   // 1 — the OD-9
};
inline constexpr int kNumCircuits = int(sizeof(kCircuits) / sizeof(kCircuits[0]));

inline constexpr Variant kVariants[] = {
    { "OD-8 W Tone",           0, ToneLaw::Pot  },
    { "OD-9 W Tone",           1, ToneLaw::Pot  },
    // THE SAME 808, WITH THE TONE KNOB's TRAVEL REDISTRIBUTED. A product
    // decision, and it is re-voicing, not fidelity: the circuit, the bank and
    // the null are the 808's, byte for byte — only the position -> parameter
    // law changes. It goes in ITS OWN ROW precisely because the faithful rows
    // must keep the hardware's taper.
    { "OD-8 L Tone",      0, ToneLaw::Even },
    // AND THE SAME LAW ON THE OTHER CIRCUIT. One table serves both: stage 3
    // does not depend on the variant (measured: under 0,0004 dB when `R14`/`R15`
    // change), so there is no `_v9ri` law and there must not be one.
    // But that is stage 3's TRANSFER, not the whole chain — `R14`/`R15` DO
    // move stage 4 —, so the split is MEASURED on the `.so` for both rows and
    // not inherited: a measurement formula does not travel between variants.
    { "OD-9 L Tone",          1, ToneLaw::Even },
};
inline constexpr int kNumVariants = int(sizeof(kVariants) / sizeof(kVariants[0]));

// A ROW's CIRCUIT, in ONE place. Everything that wants the resistors goes
// through here — the core, the DK harnesses and the cascade — so the
// row -> circuit translation exists once and cannot drift.
inline constexpr const Circuit& circuit_of(int row)
{
    return kCircuits[kVariants[row].circuit];
}

} // namespace nlsc
