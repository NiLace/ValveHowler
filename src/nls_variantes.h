// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// nls_variantes.h — the circuit-variant table. Single source of truth.
//
// It lives in its own header because two binaries need it: the DSP
// (`nls_valvehowler.cpp`) wants the resistor values and the GUI (`ui_x11.cpp`)
// wants the dropdown labels, and the host loads them as two separate .so
// files. One table keeps the dropdown from offering a circuit the DSP lacks.
//
// The `.ttl` (`lv2:scalePoint`) cannot include a header; its labels must be
// the same strings as here, as noted in the `.ttl` itself.
//
// Where the numbers come from. Three axes would separate one model from
// another:
//
//   1. output resistors .......... documented by three independent sources
//   2. each model's opamp ........ no macromodel for any of them
//   3. the model 5's bias ........ not given as values by any source
//
// This table covers axis 1 only, and has one circuit per distinct resistor
// pair. The axis-1 sources: Keen's table, Wampler's book (which marks them on
// a photo of the board, `808-1` = 100 Ω and `808-2` = 10 kΩ, p. 255) and the
// netlist transcribed from the original Eagle files, which carries the
// factory 808.
//
// Models 9, 9RI and 5 share one row because in this model they are
// electrically identical: they share the 470/100k pair, and only the opamp
// (axis 2) would separate them.
//
// Model 10 is absent: it needs a new element in the `RA` position (topology,
// not a value), and its value is disputed between sources (Keen gives 1 kΩ,
// others 220 Ω).
//
// A new variant goes at the end. The port carries an index, so inserting in
// the middle would change the circuit under every saved session.

#pragma once

namespace nlsc {

// Two tables, and the split is load-bearing.
//
// A circuit is what axis 1 above covers: a pair of resistors, and with it a
// whole generated coefficient bank. A variant is a row of the selector the
// user sees, and it is a pair: (circuit, knob law).
//
// `nls_juegos.h` checks its list of banks against `kCircuits`, so a circuit
// announced without its bank does not compile. Rows that only change a knob
// law reuse a circuit, so they may grow without a new bank.
struct Circuit {
    double rout_ser;        // `R14`, in series towards the output
    // `R15`, the output load after the coupling cap, not Q2's emitter. `R13`
    // (the emitter) is 10 k in both models, so the two variants share Q2's
    // quiescent point and the transistor stays in class A in both. This
    // resistor sets the output divider with `R14`, worth 0,10 dB (Keen:
    // "admittedly very subtle").
    double rout_shunt;
};

// The tone knob's law: position -> physical parameter.
//
// It is not a circuit change: the circuit runs on the physical parameter and
// this law sits on top of it, deciding how the pot's physical travel is laid
// out along the knob.
//
//   Pot   the real pot's G taper, faithful to the hardware, and the
//         default. Agrees with a capture of the OD-9's original within 0,7 dB.
//   Even  derived by inverting the measured brightness so that dB come out
//         even across the knob: the worst 0,1 step is 2,79 dB instead of
//         11,03 dB, and the last tenth of travel no longer carries 67,2 % of
//         the range. Deliberately not faithful: it is re-voicing, offered as
//         its own row, never as a default.
enum class ToneLaw { Pot = 0, Even = 1 };

struct Variant {
    const char* label;   // what the GUI shows AND the `lv2:scalePoint` label
    int         circuit;    // index into `kCircuits`
    ToneLaw     tone_law;
};

// The label is the one spelling of a row: the `.ttl`'s scalePoint and the GUI
// both take it from here (the GUI upper-cases it for the dot-matrix display),
// so the two cannot end up with two spellings of one model.
//
// These values must not be crossed between variants. A variant is not two
// resistors but a coefficient bank, generated from ngspice on its own
// netlist; changing a resistor here without regenerating the bank leaves the
// model incoherent with no error raised. To try a different resistor,
// regenerate the whole variant's bank. Without a -D on the command line these
// macros evaluate to the values below.
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
    // The same 808, with the tone knob's travel redistributed. Re-voicing,
    // not fidelity: the circuit and the bank are the 808's; only the
    // position -> parameter law changes. It is its own row because the
    // faithful rows keep the hardware's taper.
    { "OD-8 L Tone",      0, ToneLaw::Even },
    // The same law on the other circuit. One table serves both: stage 3
    // does not depend on the variant (under 0,0004 dB when `R14`/`R15`
    // change), so there is no `_v9ri` law.
    { "OD-9 L Tone",          1, ToneLaw::Even },
};
inline constexpr int kNumVariants = int(sizeof(kVariants) / sizeof(kVariants[0]));

// A row's circuit, for callers that start from a selector row (the DK
// harnesses). The core translates with `kVariants[row].circuit` and the
// cascade reads `kCircuits[J::idx]` directly.
inline constexpr const Circuit& circuit_of(int row)
{
    return kCircuits[kVariants[row].circuit];
}

} // namespace nlsc
