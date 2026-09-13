// Stage 2 — gain and clipping (IC1A + D1/D2), with the IDEAL opamp.
//
// WHY THE IDEAL OPAMP, AND WHY THAT IS NOT A HACK
// -----------------------------------------------
// The discarded cascade split by the SIGNAL, an unjustified cut: if two
// blocks load each other, splitting them loses something. A survey of the
// cheap TS models found every one of them splits by something else --
// THE VIRTUAL GROUND:
//
//   "The IDEAL opamp is what decouples the circuit. With infinite gain and a
//    virtual ground, the input network, the feedback network and the clipper
//    stop seeing each other […] It is not an approximate cascade of a
//    coupled network: it is an exact consequence of assuming the ideal
//    opamp."
//
// => With the ideal opamp, `n7` is a voltage source: whatever stage 3 draws
// through `R7` does NOT change its value. That is why stages 2 and 3 can be
// separated without approximating anything BEYOND the opamp itself, and why
// this cascade is a different architecture from the one already measured.
//
// AND WHAT THAT COSTS, WRITTEN BEFORE MEASURING (a prediction)
// ----------------------------------------------------------------
// The real opamp is an NJM4558: GBW 3 MHz, Avol 1e5. At max drive the
// closed-loop gain at 1 kHz is ≈69 (Rf = 551 kΩ against |Zin| ≈ 8 kΩ), and
// the open-loop gain there is 3000 => loop gain ≈43, i.e. a 2,3 % gain error,
// which is −33 dB. I expect THAT term to dominate this stage's residual,
// above the shared rail and the backward coupling. Written beforehand so the
// result can contradict it.
//
// THE EQUATIONS
// -------------
// With `v(n5) = v(n4)` imposed by the ideal opamp, the only KCL left is node
// `n5`'s, into which the opamp injects nothing. The unknowns are the output
// and the diodes' two internal nodes (the 1N914 model's `RS`):
//
//     F1 (KCL at n5) : (v5−v7)/Rf + i_C4 + (v5−va1)/RS − i_j2 + i_leg + v5/RCM
//     F2 (KCL at a1) : i_j1(va1−v7) − (v5−va1)/RS
//     F3 (KCL at a2) : i_j2(va2−v5) − (v7−va2)/RS
//
// - `Rf = R6 + RGAIN` in series (nothing else hangs off n1).
// - `i_leg` is the current through the C3+R5 leg towards VR. With `v5`
//   KNOWN, it falls out explicitly: it does not enter the Newton.
// - `RCM` (the opamp model's 500 MΩ) is NOT decoration: without it this
//   stage's rest comes out with `v7 = v5`, and the real circuit holds
//   4,65 mV between the two. That would be 4,65 mV of DC fed into the chain.
//
// The diode model is THE SAME OBJECT the DK engine uses (`mna::DModel` +
// `mna::Engine::diode`), junction charge included, same trapezoidal
// discretisation. Deliberate: if stage 2 carried its own device model, the
// residual would measure that difference and blame it on the split.

#pragma once

// Two consumption levers, both ON by default. Together they
// take 4,73 % off the cascade (15/15 paired rounds) with the three yardsticks
// unchanged: null -70,5 dB, ANMR -15,3 dB, in-band alias -61,5 dB. The audio
// residual against the previous build is -129 dB, which is the reciprocal
// -vs- division rounding floor and nothing else.
//
//   NLSC_E2_HOIST       hoist the per-sample constants into `izar()`
//   NLSC_E2_CHARGE_CACHE reuse the last Newton diode evaluation for the charge
//
// Set either to 0 to opt out. They are defined HERE, before first use: at the
// bottom of the file the preprocessor read them as 0 and the switch silently
// did not exist.
#ifndef NLSC_E2_PAIR_EXP
#define NLSC_E2_PAIR_EXP 0
#endif
#ifndef NLSC_E2_HOIST
#define NLSC_E2_HOIST 1
#endif

// STAGE 2 SOLVES BY TABLE — SHIPPED, for less CPU at −96,5 dB.
//
// The Newton of this stage reduces algebraically to `P·w + Q·id(w) = R` with `P`
// and `Q` constant between knob changes, so `w` is a function of ONE scalar and
// can be tabulated. Four tables share one index — `w`, `ID(w)`, `q(w)`, `q(−w)`
// — one warp and one division, 131 kB, and the path evaluates NO diode at all.
//
// What it buys, paired against the shipped Newton, 15 pairs per point, both
// variants, 87 of 90 pairs agreeing in sign: **−4,58 % of the WHOLE PLUGIN at
// drive 0, rising to −7,32 % at drive 1**. The citable figure is the
// unfavourable end, −4,6 %: the saving grows with the drive because the table
// costs a constant while the Newton iterates more as the diode conducts.
//
// What it costs: its own residual against the Newton is **−96,5 dB**, which is
// 34 dB under the worst port cell, and against a −62 dB null moves it 0,0015 dB
// — three orders below the printed decimal. Measured on the ngspice grid rather
// than argued: **0 of 36 over the bar in BOTH arms** on the 9RI at 4× (8-Sep)
// and on the 808 (7-Sep), with the null moving ≤0,1 dB in all 36.
//
// Its SECOND budget is paid too, and that was the blocker: building the table
// ran on the audio thread and cost 36,1 % of a block on every knob move. The
// fill now advances per SAMPLE — a function of the sample index, so it does not
// see how the host slices — and a knob move is back to **0,4 % of a block**.
// While the table is half built, `w` comes from the Newton, so there is no new
// path and no new test. A table is paid for twice: once per sample, and once in
// whatever control INVALIDATES it.
//
// It NEEDS `NLSC_E2_HOIST`, which is why it sits here: without hoisting, the
// coefficients `P` and `Q` do not exist. There is an `#error` for it below.
// And the default is EXPLICIT on purpose. It was used as a bare `#if`, which
// evaluates to 0 when undefined -- a switch whose default nobody wrote down,
// and this repository has paid for that twice.
//
// THE TABLE LIGHTS UP TWO GATES, and neither cap may be re-baselined here:
//
//   · `make cache-rodajas` -- stage 4's sliced-fill bit-identity goes to 8192
//     of 8192 samples differing, worst 6,962e-06. Diagnosed: it is the
//     INTERACTION of two deferred fills, not the table's arithmetic. Filling
//     this one in a single go (`BINS_POR_MUESTRA=8192`) makes that gate
//     BIT-IDENTICAL again.
//   · `test_passthrough` -- "moving a knob does not reset the state".
//     Criterion `jump <= 20 x control`: with the table 9,239e-07 against
//     2,980e-08 (fails by 1,55x), without it 5,215e-07 against 2,086e-07
//     (passes comfortably). The absolute JUMP grows 1,77x (-110 -> -105,5 dB
//     over an rms of 0,175); what trips is the RATIO, because the table shrinks
//     the CONTROL 7x by not iterating the Newton. The threshold is relative to
//     something the lever itself moves.
//
// AND THE STEP IS THE NEWTON'S ERROR, NOT THE TABLE'S. The hypothesis was that
// the switching step is the table's INTERPOLATION residual, so raising the
// resolution would drop it to second order. Refuted: from 4096 to 16384 bins
// the jump reads 9,239e-07 · 9,537e-07 · 9,686e-07, i.e. FLAT over a 4x range,
// and if anything it rises. A residual that does not move when refined is not
// an approximation error.
//
// The next suspect does move, and saturates, which is the signature of having
// found the term that rules -- sweeping `NLSC_E2_TOL_ABS/REL`:
//     1e-6/1e-5 (what ships) -> 9,239e-07   fails (cap 5,96e-07)
//     1e-9/1e-8              -> 2,831e-07   passes
//     1e-12/1e-11            -> 2,831e-07   (identical: saturated)
//
// => If the INEXACT one were the table, tightening the Newton would not move
// the difference. It moves it 3,3x (-10,3 dB) and then stops => the table is
// MORE exact than the Newton that ships, and what shows on a switch is the
// NEWTON's error. That also reframes the -96,5 dB the decision was taken on:
// it is a MUTUAL difference and most of it belongs to the Newton, so the table
// is not a fidelity COST.
//
// THE FILL SCHEDULE IS THE TABLE'S PROPERTY, not the Newton's, and raising it
// passes both gates without touching the shipped tolerance. Measured at the
// SHIPPED tolerance, load cache on, N=4096, moving only
// `NLSC_E2_TABLA_BINS_POR_MUESTRA` (each arm with its own md5):
//
//   bins/sample   jump         block bit-identity
//   1             62 ULP       worst 6,962e-06
//   2 … 64        53 ULP       —
//   128           35 ULP       worst 1,892e-06
//   4096           2 ULP       worst 2,446e-09
//   4097           2 ULP       BIT-IDENTICAL
//
// The bit-identity threshold is EXACT and explainable: **4097 = `kTablaN + 1`**,
// i.e. the WHOLE table in one sample. One bin fewer and it is not.
// => Both gates go green AT ONCE with the all-at-once fill -- which is exactly
// what the per-sample schedule removed in order to lower the peak -- and the
// price is what that work bought: **49,91 % of a worst block against 17,51 %**,
// over a baseline of 15,89 %. Any bins-per-sample value that passes fits INSIDE
// one worst block, so it inherits that peak: there is no middle option.
// => It is a PRODUCT decision with a price.
//
// TWO RESERVATIONS ABOUT THE INSTRUMENT, before quoting any number above:
//  · They are all INTEGER counts of the float32 ULP (2^-26 = 1,4901e-08): the
//    whole sweep lives between 2 and 65 counts, so a ratio like "/3,80" carries
//    +-0,5 ULP in its denominator and does NOT support a claim of second order.
//    Score the RMS of the window, not the maximum -- stale load, for instance,
//    is 0,82 dB in RMS and not the 1,86 the maximum gives.
//  · `control` sits on the rounding floor and USED TO DEPEND on which sections
//    had run BEFORE it in the same binary: with no warm-up pass it read 5 ULP
//    instead of 2, and with that the SAME `.so` passed. `test_passthrough` now
//    throws away a warm-up pass before the three it measures, so the reading no
//    longer depends on execution order. What the measurement says, which is not
//    the same as the fix: the deciding arm (delta 1e-07) gives 2 ULP with and
//    without warming, and the bulk moves 142 -> 139 ULP, so the sensitivity to
//    order is REAL but was not biting in this build. It is fixed anyway: a gate
//    whose verdict can depend on order decides nothing, biting today or not.
//    Negative arm: `NLSC_PT_SIN_CALENTAR=1` skips it.
#ifndef NLSC_E2_TABLA
#define NLSC_E2_TABLA 1      // SHIPPED
#endif

// The table's RESOLUTION, a parameter so it can be swept.
// It lives HERE, outside every `#if`, because the first version sat next to
// `kTablaN` -- which is INSIDE `#if NLSC_E2_TABLA`. With the table off the
// `#define` never happened, the decision probe printed the NAME instead of the
// value, and the gate went red. A `#define` below its use switches the function
// off in silence.
// It costs MEMORY and fill time (4 x (N+1) x 8 B = 131 kB), not per-sample
// clock. Raising it buys NOTHING: measured flat from 4096 to 16384.
// THE TABLE NEEDS THE HOIST, AND THIS GUARD HAS TO BE ABLE TO FIRE.
// Without `izar()` the coefficients `P` and `Q` the table is a function of do
// not exist. It goes HERE, outside every conditional, because written next to
// its use it sat INSIDE `#if NLSC_E2_HOIST` and was therefore unreachable: with
// the hoist off the preprocessor skipped it and the compiler complained about a
// function that does not exist. A guard inside the region it watches watches
// nothing.
#if (NLSC_E2_TABLA || NLSC_E2_MAP_PROBE) && !NLSC_E2_HOIST
#error "NLSC_E2_TABLA needs NLSC_E2_HOIST: without the hoist, the table's P and Q coefficients do not exist. Compile with -DNLSC_E2_TABLA=0 to switch the hoist off."
#endif

// THE TABLE'S BOX, PARAMETRISED so it can be switched OFF and the damage seen.
// At 1 (what ships) the table refuses to answer over the stretch whose ends it
// fabricates rather than solves, and there it falls back to the Newton. At 0 it
// returns to the earlier behaviour, which leaves the engine MUTE forever after
// a square-wave abuse: that is this guard's negative arm, and `build/abuso`
// fires it.
// At guitar level it NEVER fires (0 of 192.000 measured samples), so turning it
// on does not change the audio that ships — and that is checked by IDENTITY of
// the output, not assumed.
#ifndef NLSC_E2_TABLA_CAJA
#define NLSC_E2_TABLA_CAJA 1
#endif

#ifndef NLSC_E2_TABLA_N
#define NLSC_E2_TABLA_N 4096
#endif
#ifndef NLSC_E2_CHARGE_E3
#define NLSC_E2_CHARGE_E3 0
#endif
#ifndef NLSC_E2_CHARGE_CACHE
#define NLSC_E2_CHARGE_CACHE 1
#endif

// HOW MANY BINS OF STAGE 2's TABLE ARE SOLVED PER SAMPLE.
//
// It goes ABOVE and not next to its use: a `#define` below where it is read
// leaves the `#else` in place and switches the function off IN SILENCE.
//
// It is a COMPROMISE between how far the PEAK rises while the table fills and
// how long the table takes to come back, chosen by MEASURING the worst block of
// 256 internal samples (= 64 @ 48 kHz), which is what decides an xrun:
//
//   no table ............................. 15,89 %   <- the reference
//   all at once, not deferred ............ 49,91 %
//   deferred,  8 bins/sample ............. 34,39 %   (back in  2,7 ms)
//   deferred,  4 bins/sample ............. 23,36 %   (back in  5,3 ms)
//   deferred,  2 bins/sample ............. 19,98 %   (back in 10,7 ms)
//   deferred,  1 bin  per sample ......... 17,51 %   (back in 21,3 ms)
//
// **THE ALL-AT-ONCE FILL IS WHAT SHIPS, and this table is no longer what
// decides.** What made it irrelevant is that none of its rows measures **a knob
// that does NOT stop**: under automation `apply_knobs()` invalidates the table
// on EVERY block, and there the deferred fill costs **4,88 % of a core median
// against 3,00 baseline** — i.e. MORE than having no table — while leaving BOTH
// gates red. With `NLSC_E2_TABLA_ESPERA` in front, the all-at-once fill only
// happens once the knob has stopped, and automating costs **3,40** again with
// both gates green.
// => The 49,91 % peak is still true and is now paid **once per gesture**, not
// once per block.
//
// AND MIND THE MEASUREMENT, BECAUSE THE FIRST INSTRUMENT ANSWERED A DIFFERENT
// QUESTION: the AVERAGE over the fill stretch comes out **the same for 1 and
// for 16** (12,6 against 13,0 core points), because the TOTAL work is constant
// — 4097 bins — and all that changes is how it is SPREAD. An average cannot see
// a peak height, and what decides an xrun is the WORST BLOCK.
// THE TOLERANCE EACH BIN OF THE TABLE IS SOLVED TO.
//
// It used to be NAILED at 1e-12 inside `avanza_tabla`, and a nailed constant
// CANNOT BE MEASURED. It is the same Newton and the same `ID_d` as the audio
// loop, which stops at `kTolAbs + kTolRel*|x|` = 1e-6 + 1e-5|x|; here it stops
// six orders tighter because the fill is paid ONCE per knob turn and not
// 192.000 times a second. At the default the `.so` comes out bit-identical.
//
// AND THE USEFUL HALF IS A CORRECTION. It was written that loosening this
// "moves nothing", and therefore that the table's accuracy does not come from
// here. **That negative arm never activated**: it was loosened only to 1e-4 and
// only at N=16384, the FINEST grid, where the continuation step between bins is
// 32x smaller — so of course nothing moved. Loosened properly it moves a lot:
// `1e-2` at N=16384 raises the step from 2 to **36 ULP**, and at N=512 from
// 1211 to **17065**. The correct sentence is "the SHIPPED tolerance has ~10
// orders of margin", which is a measured fact, not "this does not matter".
// => A negative arm that never activates **refutes nothing and looks like a
// success**, and the axis to move when loosening it is the one that makes the
// continuation step LARGE: low N.
// It lives HERE, outside every `#if`, for the reason this file gives twice
// already: a `#define` below its use switches the function off in silence.
#ifndef NLSC_E2_TABLA_FILL_TOL
#define NLSC_E2_TABLA_FILL_TOL 1e-12
#endif

// THE WAIT: NOTHING IS BUILT WHILE THE KNOB IS MOVING.
//
// THE PROBLEM, AND IT IS MEASURED. The table is invalidated as soon as
// `apply_knobs()` sees a different value, that is, **on every block** while a
// host automation lane moves a knob. With the ALL-AT-ONCE fill — the only one
// that turns both gates green — that means rebuilding it entirely every block:
// median **15,66 % of a core against 3,00 % baseline, 5,22x**, and sustained,
// not a peak. With the deferred fill it costs 1,63x and also **more than having
// no table at all**, because it throws the fill away every block and runs the
// Newton anyway.
//
// THE PATTERN THAT FIXES IT IS NOT INVENTED HERE: two regimes with different
// demands, where the GESTURE asks not to cost and REST asks for accuracy.
// Applied: while the knob moves **nothing is built** and the Newton solves (that
// is, exactly what ships today); once it has been still for `kEspera` samples,
// the table is built in ONE go.
//
// THE WAIT IS COUNTED IN INTERNAL SAMPLES, NOT IN BLOCKS. Same reason the fill
// advances per sample: counted in blocks, WHEN the table appears depends on how
// the host slices and block invariance breaks. In samples, the schedule is a
// function of the SAMPLE INDEX and does not see the slicing.
//
// SIZING. Under automation `apply_knobs()` restarts the count every block, so
// it is enough for the wait to be LONGER than the block for it never to reach
// zero. 2048 internal samples are 512 host samples at 4x, i.e. 10,7 ms: that
// covers 64/128/256/512, and the case it does NOT cover — blocks of 1024 — is
// precisely the cheap one, because there a whole fill (~0,48 ms) lands in a
// period of 21,3 ms.
// On releasing the knob the table takes those 10,7 ms to appear. Over that
// stretch the engine that ships today runs, so no fidelity is lost: the saving
// is delayed.
// THE NEWTON'S TOLERANCE **DURING THE WAIT ONLY**.
//
// This does NOT touch the tolerance of the path that SHIPS. With the table in
// place that path **does not run in steady state**: the Newton only executes
// while the user is gesturing. This is a SEPARATE adjustment, for a stretch
// lasting 10,7 ms per gesture.
//
// WHAT FOR: `test_passthrough` compares the same knob change applied from the
// start against applied halfway, and during the wait the transient arm runs the
// NEWTON while the other is already on the table => the difference between the
// two shows, 53 ULP against a cap of 40. Tightening the Newton over that
// stretch brings it down to 19 ULP.
// Default 0 = use the usual tolerance, and the `.so` comes out bit-identical.
// The switch is an INTEGER and separate from the values: `#if` only does
// INTEGER arithmetic, so `#if NLSC_E2_ESPERA_TOL_ABS > 0` with `1e-9` does not
// compile. And seeing the compile fail is not enough: a harness that does not
// rebuild ran the OLD `.so` and published a number from it.
#ifndef NLSC_E2_ESPERA_TOL
#define NLSC_E2_ESPERA_TOL 0
#endif
#ifndef NLSC_E2_ESPERA_TOL_ABS
#define NLSC_E2_ESPERA_TOL_ABS 1e-9
#endif
#ifndef NLSC_E2_ESPERA_TOL_REL
#define NLSC_E2_ESPERA_TOL_REL 1e-8
#endif

#ifndef NLSC_E2_TABLA_ESPERA
#define NLSC_E2_TABLA_ESPERA 2048   // SHIPPED
#endif

#ifndef NLSC_E2_TABLA_BINS_POR_MUESTRA
#define NLSC_E2_TABLA_BINS_POR_MUESTRA (NLSC_E2_TABLA_N + 1)   // DE GOLPE: la tabla ENTERA
#endif

// ORACLE `n9` INJECTION — MEASUREMENT ONLY.
//
// WHY: `NLSC_E2_CHARGE_E3` models the load stage 3 hangs off `n7` as `R7`
// in series with (`C5` ∥ `R8`) — and that is NOT the whole tone network,
// which is what really hangs off `n9`. That load is the DOMINANT term of
// what was there (turning it off raises the fixed part from 40,83 to
// 192,88 µV), so its remainder is the natural candidate.
//
// Given the ORACLE's `n9`, the load current becomes EXACT without writing a
// new model:  `i = (v7 − n9)·G7` => `Yeq = G7` and `Ieq = G7·n9`. It is an
// UPPER BOUND of what modelling the load well would buy, not a model — same
// treatment as `n4`-oracle / `n7`-oracle / `n19`-oracle.
//
// Behind a macro and OFF by default: it puts a branch in the hot path and
// the shipped .so must come out bit-identical. Checked with the md5.
#ifndef NLSC_E2_N9_ORACLE
#define NLSC_E2_N9_ORACLE 0
#endif

// NEWTON ITERATION PROBE. MEASUREMENT ONLY, OFF BY
// DEFAULT, and the product `.so` is checked bit-identical with it off.
//
// WHY IT EXISTS: `Stage2::process` is 40,55 % of the cascade and nobody knew how
// many Newton iterations it runs. `samples_` is incremented OUTSIDE the loop,
// so the counters that were already here answer "did it converge", never "how
// much work". That number dimensions every other consumption lever: at 1,2
// iterations there is nothing to trim, at 3-4 there is.
//
// The counter sits just BEFORE the convergence test, so it counts the pass
// that converges as well. Placing it after the `break` would under-count by
// exactly one per sample — the shortcut-counter trap this workshop has already
// paid for once.
#ifndef NLSC_E2_ITER_PROBE
#define NLSC_E2_ITER_PROBE 0
#endif

// NEWTON SEED PREDICTOR — the lever C1 pointed at.
//
// Stage 2 seeds Newton with the PREVIOUS SAMPLE's `w` and nothing else, which
// is predictor order 0. The DK has the same axis measured on its own solver:
// order 0 -> 1,936 iterations, order 1 -> 1,025, order 2 -> 1,008. Stage 2 was
// running the top step of that ladder, and `make iter-e2` puts it at 3,5.
//
// Order 1 is a linear extrapolation of the last two samples: two subtractions
// and two doubles of state, once per sample, against whole Newton passes each
// of which costs two diode evaluations (an `exp` and a `pow` apiece).
//
//   0  what ships today: the previous sample, untouched
//   1  linear    w = 2*w[n-1] - w[n-2]
//   2  quadratic w = 3*w[n-1] - 3*w[n-2] + w[n-3]
//
// The converged root is unchanged to within the stopping tolerance, so this
// is not a model change -- but the DK's own note records that the predictor
// leaves a trace in the last bits, so it is measured, not assumed. The history
// is cleared at the end of `reposo()` for the same reason `dk::prepare()`
// erases it: the DC settling loop would otherwise extrapolate into sample 0.
//
// THE WARNING INHERITED FROM THE DK DID NOT SURVIVE BEING RE-MEASURED HERE.
// It read: "the order HIGH is not free -- on the DK the quadratic seed ran 2,7 %
// SLOWER despite fewer iterations". True there, FALSE in stage 2, where order 2
// is the fastest of the three. A warning is a claim as much as a figure is, and
// it inherits the engine it was taken on.
//
// Measured, order 2 against order 0, paired on the product `.so`:
// 2,037 Newton passes per sample against 2,801, -14,93 % of a core (21 of 21,
// p = 9,54e-07), against a control on the DK engine -- which does not carry this
// lever -- of +0,16 %, 9 of 21, p = 0,664. Fidelity untouched: own residual
// -127,72 dB with the output open, ANMR unchanged on the worst cell, and 216
// cells identical against ngspice across both variants.
//
// The price is real and small: two extra doubles of state per instance, and a
// history guard without which the first samples extrapolate from the DC solve.
#ifndef NLSC_E2_PREDICTOR
#define NLSC_E2_PREDICTOR 2
#endif

#include <cmath>

#include "nls_mna.h"

namespace nlsc {

class Stage2 {
public:
    // `gain` is the netlist's PHYSICAL parameter (0..1), not a knob position.
    // `reinit = false` re-tunes the matrices WITHOUT dropping the
    // capacitors' state: what the real circuit does when you turn a pot.
    // With `true` the rest point is solved again — what startup or a RATE
    // change needs.
    void prepare(double fs, double gain, double vr, double v5_rest,
                 double eps = 1e-3, bool reinit = true)
    {
        h_    = 1.0 / fs;
        // PER-ELEMENT BILINEAR (Germain & Werner). Each reactive element gets its
        // own step `T'_l = alpha_l * h` instead of one trapezoidal rule for all.
        // In this companion form `h` only enters through the conductance, so this
        // is the whole change and it costs NOTHING at run time: it is computed
        // once here. With every alpha at 1.0 the arithmetic is a multiply by one,
        // so the engine is bit-identical -- that is the control.
        // Why here and not on the DK: measured, stage 2's excess
        // over the DK is a DISCRETISATION error that grows towards Nyquist (same
        // harmonic order gives +4.8 dB at 4x and -0.7 dB at 8x), and this is the
        // technique whose whole point is that it pays near Nyquist.
        g2h_  = 2.0 / (h_ * kA_QD);
        vr_   = vr;
        Rf_   = 51e3 + 500e3 * gain + eps;
        G4_   = 2.0 * kC4 / (h_ * kA_C4);
        // C3+R5 leg: the trapezoidal leaves an equivalent series resistance.
        invG3_ = h_ * kA_C3 / (2.0 * kC3);
        Gleg_ = 1.0 / (kR5 + invG3_);
        // Stage 3's load: R7 in series with (C5 to ground ∥ R8 to the rail).
        // Trapezoidal for C5, like everything else.
#if NLSC_E2_CHARGE_E3
        G7_ = 1.0 / kR7;
        G8_ = 1.0 / kR8;
        G5_ = 2.0 * kC5 / (h_ * kA_C5);
        Gsum_ = G7_ + G5_ + G8_;
#endif
#if NLSC_E2_CHARGE_E3
        if (reinit) { cv5c_ = vr; ci5c_ = 0.0; }   // at rest `n9` sits on the rail
#endif

        d_.derive();
        jd_.init(d_.CJO, d_.VJ, d_.M, d_.FC);
        nvtD_  = d_.N * mna::VT;
        vcritD_ = nvtD_ * std::log(nvtD_ / (std::sqrt(2.0) * d_.IS));

        mna::OModel o_;
        gm_ = o_.gm(); Rp_ = o_.rp(); imax_ = o_.imax();
        inv_imax_ = 1.0 / imax_;
        Gcp_ = 2.0 * o_.CP / (h_ * kA_CP);

#if NLSC_E2_HOIST
        hoist();
#endif
        if (reinit) rest(v5_rest);
        else           v5_0_ = v5_rest;   // for the explicit `reset()` afterwards
    }

    void reset() { rest(v5_0_); }

    // THE SHARED RAIL, AS AN INPUT. `VR` is not AC ground: `C11` leaves it
    // ~34 Ω at 98 Hz and every stage feeds it signal current through `R4`,
    // `R8` and `R5`, so it follows the signal (measured: 10 % of `n5` at
    // 98 Hz). And the branch that sets this stage's gain goes precisely TO
    // VR. Holding it constant is what the cascade gets wrong.
    void set_vr(double vr) { vr_ = vr; }
#if NLSC_E2_N9_ORACLE
    // Measurement only: the oracle's ABSOLUTE `n9` for the coming sample.
    void set_n9_orac(double n9) { n9_orac_ = n9; use_n9_ = true; }
#endif

    // `n11` (the C3-R5 junction) contributes 2,0 % of the current that moves
    // the rail, and only this stage knows it.
    double n11() const { return n11_; }
    // Probe: the three nodes the stage solves, so that WHERE a zero is born
    // can be seen. Not product.
    double probe_v5() const { return v5_; }
    double probe_v7() const { return v7_; }
    double probe_v2() const { return v2_; }

    // Input and output in ABSOLUTE VOLTAGE (volts to ground): `v5` is what
    // stage 1 puts on `n4`, and the return is `n7`.
    // THE COST LEVERS, separable on purpose.
    //
    // Stage 2 is 80,9 % of the cascade's cost (measured,
    // `harness/coste_etapas.cpp`), so everything worth winning lives here.
    // Two things make it expensive and both can be removed:
    //
    //   -DNLSC_E2_NO_RS     `RS` = 0 => the diodes' two internal nodes
    //                        vanish and the Newton goes from 3x3 to SCALAR.
    //   -DNLSC_E2_CHARGE_TT   removes ONLY the depletion (`CJO`), where the
    //                        single `pow` lives. Diffusion (`TT`) stays and
    //                        costs two multiplies, reusing the exponential
    //                        already computed.
    //   -DNLSC_E2_NO_CHARGE  removes the junction charge ENTIRELY (`CJO`
    //                        and `TT`) => one exponential per iteration
    //                        instead of two `diode()` calls, one state less.
    //
    // Kept SEPARATE because attribution matters: if the combination loses
    // fidelity, one must know which of the two loses it.
    // And `SIN_CARGA` used to mix TWO levers in one: the time-expensive
    // one (the depletion's `pow`) and the fidelity-bearing one (diffusion,
    // dominant in a CONDUCTING diode). Hence the intermediate mode: without
    // it there is no telling which of the two costs the 15,2 dB measured
    // when both were removed together.
    //
    // What is NOT done, ruled out by earlier measurement: closing the pair
    // in CLOSED FORM (Lambert W / omega, what the published models do). It
    // requires dropping the subdominant exponential, and that is worth
    // −30,5 dB at max drive (`docs/LAMBERT_W_NO_VALE.md` §2) — worse than
    // the whole cascade's residual, so it would dominate. The full `sinh`
    // stays; only HOW it is solved changes.
// (`NLSC_E2_CHARGE_E3`'s default lives in the header block at the top, with its
// siblings — it used to be defined HERE, 76 lines below its first use, which is
// the exact silent-zero trap that block warns about.)
#if defined(NLSC_E2_OPAMP_LIN)
    // ---- FINITE-gain but LINEAR opamp: back to SCALAR ----
    //
    // WHY IT IS ALLOWED: the real opamp fixes the residual (−36,8 ->
    // −60,5 dB together with the rail) but adds two unknowns and a 3x3
    // Newton costing 49 % extra. And its two NONlinearities — the slew limit
    // and the rail clipping — NEVER act with real material: measured by
    // sweeping the level, `satA` does not fire even at +24 dBFS.
    //
    // With their nonlinearity removed, the pole node solves by hand:
    //
    //     v2 = [gm·(x − v5) + s2] / Y2        (linear in v5)
    //     v7 = v2 + Ro·iF(w)                  (no clipping, v3 = v2)
    //  =>  v5 = [w + (gm·x + s2)/Y2 + Ro·iF(w)] / (1 + gm/Y2)
    //
    // i.e. `v5` is an EXPLICIT function of `w`, and `n5`'s KCL is again ONE
    // scalar equation. Real-opamp fidelity at scalar-path cost.
    //
    // And it carries a GUARD: how many times the slew limit WOULD have
    // acted is counted. A hypothesis unchecked on the real run is a
    // hypothesis, not a measurement.
    // THE RESIDUAL'S BODY, IN ONE PLACE.
    //
    // It is extracted because the TABLE path has to evaluate `F` at TWO points
    // — `w = 0`, which gives the index, and the `w` the table returns — and the
    // alternative was writing the formula out again. A rule written twice
    // DIVERGES, and here it would diverge in silence: both copies would keep
    // compiling and the null would only move once somebody touched one of them.
    //
    // It is NOT a re-derivation: it is THE SAME TEXT, moved. That is why the
    // substitution is signed off with bit-identity of the audio rather than
    // with a fidelity measurement — 107 cells (72 of grid at 48 kHz, 16 at 44,1
    // and 96, the 2 and 8 factors and the bypass) bit-identical, with two
    // negative arms that DO fire (drive 0,5 against 0,5001, and variant 0
    // against 1).
    //
    // `id` and `gd` arrive already evaluated because they are the only costly
    // part: the caller decides whether they come from `diode_par` or, at
    // `w = 0`, from state arithmetic — there the antiparallel pair sees the
    // same thing on both sides, so currents and charges cancel to EXACTLY ZERO
    // and no transcendental is needed.
    struct Cuerpo { double F, dF, v5; };

#if NLSC_E2_HOIST
    inline Cuerpo residuo(double w, double id, double gd,
                          double x, double c, double s2) const
#else
    inline Cuerpo residuo(double w, double id, double gd,
                          double x, double c, double s2, double Y2) const
#endif
    {
        double v5;
        const double iC4 = G4_ * (w - cv4_) - ci4_;
#if NLSC_E2_HOIST
        const double iF  = w * invRf_ + iC4 + id;
        const double giF = invRf_ + G4_ + gd;
#else
        const double iF  = w / Rf_ + iC4 + id;
        const double giF = 1.0 / Rf_ + G4_ + gd;
#endif

        // THE RAIL CLIPPING THAT WAS MISSING.
        //
        // The linearised version removed the opamp's two nonlinearities
        // "because they never act" — true at guitar level, but the
        // clipping is NOT optional: without it the output has no bound.
        // Measured: 10 V of input peak had the plugin delivering 635 kV.
        // The full model clips `v3` (what `Ro` sees), NOT `v2`, which
        // keeps integrating; same here.
        //
        // With `v2` inside the rails:  v5 = k*(w + c + Ro*iF)  (as before)
        // With `v2` outside:           v5 = w + Vsat + Ro*iF
        // Both are EXPLICIT in `w`, so the Newton stays scalar.
#if NLSC_E2_CHARGE_E3
        // WITH STAGE 3's LOAD. The current `n7` delivers is
        //     i_carga = Yeq*v7 - Ieq        (linear in v7)
        // so  v7*(1 - Ro*Yeq) = v2 + Ro*iF - Ro*Ieq, and `v5` stays
        // EXPLICIT in `w`: the scalar path is preserved, which is the
        // reason to do it this way and not with one more Newton.
        // Algebraic control: with Yeq = 0 it gives D = 1 and kL = k,
        //    the old formula. Checked by compiling with the macro at 0.
        v5 = kL_ * (w * D_ + c + kRO * iF + kRO * Ieq_);
        double dv5 = kL_ * (D_ + kRO * giF);
#else
        // `k` was RENAMED to `kL_` when stage 3's charge arrived, and this
        // branch — the `NLSC_E2_CHARGE_E3 == 0` one — kept the old name, so
        // the "linear opamp" control had not COMPILED since then. Nobody
        // noticed because nothing built it.
        // The rename is exact, not an approximation: with `Yeq_ = 0` you get
        // `D_ = 1` and `Ieq_ = 0`, and the branch above reduces literally to
        // this one. Its own algebraic comment says so.
        v5 = kL_ * (w + c + kRO * iF);
        double dv5 = kL_ * (1.0 + kRO * giF);
#endif
#if NLSC_E2_HOIST
        const double v2_lin = (gm_ * (x - v5) + s2) * invY2_;
#else
        const double v2_lin = (gm_ * (x - v5) + s2) / Y2;
#endif
        if (v2_lin <= kVeeSat || v2_lin >= kVccSat) {
            const double vsat = (v2_lin <= kVeeSat) ? kVeeSat : kVccSat;
#if NLSC_E2_CHARGE_E3
#if NLSC_E2_HOIST
            v5  = w + (vsat + kRO * iF + kRO * Ieq_) * invD_;
            dv5 = 1.0 + kRO * giF * invD_;
#else
            v5  = w + (vsat + kRO * iF + kRO * Ieq_) / D_;
            dv5 = 1.0 + kRO * giF / D_;
#endif
#else
            v5  = w + vsat + kRO * iF;
            dv5 = 1.0 + kRO * giF;
#endif
        }

        const double i_leg = Gleg_ * (v5 - vr_ - cv3_)
                           - Gleg_ * invG3_ * ci3_;
        const double F  = iF + i_leg + v5 * kInvRCM + (v5 - x) * kInvRIN;
        const double dF = giF + (Gleg_ + kInvRCM + kInvRIN) * dv5;
        return Cuerpo{F, dF, v5};
    }
    double process(double x)
    {
#if NLSC_E2_HOIST
        // HOISTED INTO `prepare()`.
        //
        // `Y2`, `k`, `Yeq_`, `D_` and `kL_` depend only on `prepare()`
        // constants (`Rp_`, `Gcp_`, `gm_`, `G5_`/`G7_`/`G8_`) and were being
        // recomputed 192 000 times per second. And the divisions by `Y2`,
        // `Gsum_` and `Rf_` are by constants => precomputed reciprocal.
        // Measured with `perf annotate`: DIVISIONS are 18,8 % of this
        // function and the transcendentals 0 % (inlined) — the hot spot was
        // not where it was assumed.
        // Multiplying by the reciprocal is NOT bit-identical to dividing
        // (1 ULP). Measured with all THREE yardsticks, not assumed.
        // A bare `Y2` no longer exists here: on hoisting, this branch
        // moved to `kY2_`/`invY2_` and the variable went orphan
        // (`-Wunused-variable`). Deleted rather than silenced with
        // `[[maybe_unused]]`: if it is ever needed, the compiler shouts —
        // which is what the attribute would mute.
        const double s2 = Gcp_ * cvcp_ + icp_;
        // `k = kY2_` used to live here and fed only a `k*gm_/(1+0)*0` term in
        // `v2_` — a product that is always zero but that the compiler may not
        // fold (NaN/inf semantics). Term and variable removed.
        const double c  = (gm_ * x + s2) * invY2_;
#else
        const double Y2 = 1.0 / Rp_ + Gcp_;
        const double s2 = Gcp_ * cvcp_ + icp_;
        const double c  = (gm_ * x + s2) / Y2;
#endif
#if NLSC_E2_CHARGE_E3
        // Load seen at `n7`: R7 in series with (C5 to ground ∥ R8 to rail).
        //   KCL at n9:  (v7-v9)*G7 = G5*(v9-cv5c) - ci5c + (v9-vr)*G8
        //   => v9 = [v7*G7 + s5 + vr*G8] / Gsum
        //   => i_carga = (v7-v9)*G7 = Yeq*v7 - Ieq
#if NLSC_E2_N9_ORACLE
        // The EXACT load: `i = (v7 − n9)·G7` with the oracle's `n9`.
        // `Yeq` stops depending on `Gsum` and becomes plain `G7`, so `D_`
        // and `kL_` — hoisted assuming the other `Yeq` — must be redone.
        // `C5`'s state keeps updating below with the MODELLED `v9`: not
        // read under this flag, but left running so the rest of the block
        // is the same code and no second difference appears.
        // And it sits behind `use_n9_`, not the bare macro: if merely
        // compiling the flag already changed the model, the instrumented
        // binary could not serve as its own control in `modelado` mode.
        if (use_n9_) {
            Yeq_ = G7_;
            Ieq_ = G7_ * n9_orac_;
#if NLSC_E2_HOIST
            D_    = 1.0 + kRO * Yeq_;
            invD_ = 1.0 / D_;
            kL_   = 1.0 / (D_ + gm_ / Y2_);
#endif
        } else {
            const double s5 = G5_ * cv5c_ + ci5c_;
#if NLSC_E2_HOIST
            Yeq_ = G7_ * (1.0 - G7_ * invGsum_);
            Ieq_ = G7_ * (s5 + vr_ * G8_) * invGsum_;
            D_    = 1.0 + kRO * Yeq_;
            invD_ = 1.0 / D_;
            kL_   = 1.0 / (D_ + gm_ / Y2_);
#else
            Yeq_ = G7_ * (1.0 - G7_ / Gsum_);
            Ieq_ = G7_ * (s5 + vr_ * G8_) / Gsum_;
#endif
        }
#else
        const double s5 = G5_ * cv5c_ + ci5c_;
#if NLSC_E2_HOIST
        Ieq_ = G7_ * (s5 + vr_ * G8_) * invGsum_;   // `Yeq_`, `D_` and `kL_`: in prepare()
#else
        Yeq_ = G7_ * (1.0 - G7_ / Gsum_);
        Ieq_ = G7_ * (s5 + vr_ * G8_) / Gsum_;
#endif
#endif
        // THE SIGN: `iF` is defined as current ENTERING `n7`
        // (`v7 = v2 + Ro*iF`), and the load DRAWS current => subtract.
        // With the sign flipped the measurement worsened 4,5 dB — exactly
        // what doubling an error instead of removing it does.
#if !NLSC_E2_HOIST
        D_   = 1.0 + kRO * Yeq_;
        kL_  = 1.0 / (D_ + gm_ / Y2);
#endif
#endif
        double w = v5_ - v7_;
#if NLSC_E2_PREDICTOR >= 1
        // High order ONLY with enough history: starting a quadratic
        // from two samples (one of them the rest) throws the seed far away
        // and COSTS iterations instead of saving them. The same guard the
        // DK carries, for the same reason.
        const double w_actual = w;
#if NLSC_E2_PREDICTOR >= 2
        if (n_pred_ >= 2)      w = 3.0 * w - 3.0 * wp1_ + wp2_;
        else if (n_pred_ >= 1) w = 2.0 * w - wp1_;
#else
        if (n_pred_ >= 1)      w = 2.0 * w - wp1_;
#endif
        wp2_ = wp1_;
        wp1_ = w_actual;
        if (n_pred_ < 2) ++n_pred_;
#endif
        double v5 = v5_;
        bool convergio = false;
#if NLSC_E2_ITER_PROBE
        int iters = 0;
#endif

#if NLSC_E2_MAP_PROBE
        {
            // `K` = the STATE term `id(w)` drags along (the previous sample's
            // charges and currents). It is obtained by EVALUATING THE SAME
            // expression at `w = 0`, where the `w`-dependent part is zero
            // because the pair is ANTIPARALLEL and symmetric: `id(0) = K`.
            // This is not re-deriving algebra: it is the same `diode` call with
            // the same `g2h_` and the same state, at a known point.
            const mna::Engine::DOut z1 = mna::Engine::diode(d_, jd_, 0.0, kCharge);
            const mna::Engine::DOut z2 = mna::Engine::diode(d_, jd_, 0.0, kCharge);
            const double K = (z1.i + g2h_ * (z1.q - dq1_) - di1_)
                           - (z2.i + g2h_ * (z2.q - dq2_) - di2_);
            if (kfin_n_ < kMapaTope) kfin_[kfin_n_++] = K;
        }
#endif
#if NLSC_E2_CHARGE_CACHE && !defined(NLSC_E2_NO_CHARGE)
        // THE CHARGE IS CACHED FROM THE LAST NEWTON STEP.
        //
        // The state update used to call `diode()` TWO MORE times per
        // sample, just to read `.q` — and each call is one `exp` and one
        // `pow` (the depletion). The Newton has ALREADY evaluated them:
        // all that changes is that its `w` is the one BEFORE the last
        // `dw`, and the loop exits precisely when `|dw| < tol`.
        // That `tol` is the one above and has moved: at 1e-6 abs and
        // 1e-5 rel, over a charge of a few pF, this is ~1e-17 C (was
        // ~1e-23 with the original tolerance).
        // Still far below what the solver accepts as converged, but THIS
        // approximation's margin is set by the tolerance: if it ever
        // relaxes further, this limit relaxes with it — re-examine.
        // => The COMPOUND effect of the two IS measured: every yardstick of
        // the tolerance change ran with this cache on, which is how it
        // ships.
        // Not a new approximation: it is NOT repeating a computation
        // already done at a precision the solver declared sufficient.
        mna::Engine::DOut o1u{}, o2u{};
#endif
#if NLSC_E2_TABLA
        // THE INDEX, WITHOUT A SINGLE TRANSCENDENTAL.
        //
        // The residual at `w = 0` IS `-R'`, the table's index. That used to
        // cost a WHOLE pass of the body — with its `diode_par`, i.e. its
        // exponential and its division — and it was why this path took TWO
        // passes against the Newton's 1,336, that is, why it COST MORE.
        //
        // It is no longer needed, and not because of an approximation: at
        // `w = 0` the two diodes of the ANTIPARALLEL pair see exactly the same
        // thing, so in the subtraction that forms `id` their currents and their
        // charges cancel to EXACTLY ZERO in floating point (`e = 1` =>
        // `IS*(1-1) = 0`; and with the constant depletion that ships,
        // `q = cj0*0 = 0`). All that survives is the STATE term, which is
        // arithmetic already done.
        //
        // It is written with the SAME order of operations as the body — the
        // leading `0.0 +` and `0.0 -` are not decoration: they are the `o1.i`
        // and `o1.q` that cancel, and they are there so the substitution is
        // bit-identical rather than "almost". The rest of `F` is not rewritten:
        // `residuo()` evaluates it, and that is the same text the Newton runs.
        //
        // `gd = 0` because it only feeds `dF`, and here `dF` is discarded: the
        // table does not take a Newton step, it returns the solved `w`.
        // Advance the fill BEFORE using it, so the schedule is a function of
        // the sample index and not of the host's slicing.
        avanza_relleno_por_muestra();
        bool usa_tabla = tb_lista_;
        if (usa_tabla) {
            const double id0 = (0.0 + g2h_ * (0.0 - dq1_) - di1_)
                             - (0.0 + g2h_ * (0.0 - dq2_) - di2_);
            const Cuerpo c0 = residuo(0.0, id0, 0.0, x, c, s2);
            // AND HERE THE TRANSCENDENTALS END. `id` is recomposed as
            // `ID(w) + id0`: the first term is a function of `w` alone and
            // comes from the table, the second is the state term already
            // computed two lines above. The decomposition is algebraic and
            // exact — it is the body's own subtraction regrouped — but NOT
            // bit-identical, because the order of the additions changes; so it
            // is measured rather than signed off.
            bool en_caja = false;
            const Tab tb = tabla_en(-c0.F, en_caja);
            if (en_caja || !NLSC_E2_TABLA_CAJA) {
                w         = tb.w;
                id_tabla_ = tb.ID + id0;
                q1_tabla_ = tb.q1;
                q2_tabla_ = tb.q2;
            } else {
                // Outside the box it does NOT answer: it falls back to the
                // Newton, which is still compiled and is the same fallback
                // stage 4's cache uses.
                usa_tabla = false;
                ++fuera_caja_;
            }
        }
#endif
#if NLSC_E2_TABLA && NLSC_E2_TABLA_ESPERA > 0 && NLSC_E2_ESPERA_TOL
        const bool en_espera_ = (tb_espera_ > 0);
        const double tol_abs_ = en_espera_ ? kEspTolAbs : kTolAbs;
        const double tol_rel_ = en_espera_ ? kEspTolRel : kTolRel;
#endif
        for (int it = 0; it < kMaxIt; ++it) {
            double id, gd;
#if NLSC_E2_TABLA
            // THE TABLE PATH DOES NOT EVALUATE THE DIODE. That is the whole
            // point: `id` arrives recomposed from above and both charges are
            // already there, so not one exponential and not one division by the
            // pair is left here. `gd` only feeds `dF`, which this path
            // discards.
            // The choice is made at RUN TIME, not at compile time: while the
            // table fills, something has to solve, and that is the Newton below
            // — the same fallback stage 4's cache uses when its point falls
            // outside the box. That is why the deferred fill needs neither a
            // new path nor a new test.
            if (usa_tabla) {
            id = id_tabla_;
            gd = 0.0;
#if NLSC_E2_CHARGE_CACHE
            o1u.q = q1_tabla_;
            o2u.q = q2_tabla_;
#endif
            } else {
#endif
#if defined(NLSC_E2_NO_CHARGE)
            const double e = std::exp(w * d_.inv_nvt), ei = 1.0 / e;
            id = d_.IS * (e - ei);
            gd = d_.IS * (e + ei) * d_.inv_nvt;
#else
#if NLSC_E2_PAIR_EXP
            mna::Engine::DOut o1, o2;
            mna::Engine::diode_par(d_, jd_, w, kCharge, o1, o2);
#else
            const mna::Engine::DOut o1 = mna::Engine::diode(d_, jd_,  w, kCharge);
            const mna::Engine::DOut o2 = mna::Engine::diode(d_, jd_, -w, kCharge);
#endif
#if NLSC_E2_CHARGE_CACHE
            o1u = o1; o2u = o2;
#endif
            id = (o1.i + g2h_ * (o1.q - dq1_) - di1_)
               - (o2.i + g2h_ * (o2.q - dq2_) - di2_);
            gd = (o1.g + g2h_ * o1.c) + (o2.g + g2h_ * o2.c);
#endif
#if NLSC_E2_TABLA
            }
#endif
#if NLSC_E2_HOIST
            const Cuerpo cb = residuo(w, id, gd, x, c, s2);
#else
            const Cuerpo cb = residuo(w, id, gd, x, c, s2, Y2);
#endif
            const double F = cb.F, dF = cb.dF;
            v5 = cb.v5;
#if NLSC_E2_MAP_PROBE
            // IMPLICIT-MAP PROBE. It records the TRIPLE `(w, id, F)` of EVERY
            // iteration, which is all that is needed to check from outside
            // whether the equation has the form
            //     F = P*w + Q*id - R      with P and Q constant,
            // without re-deriving a single line of algebra here: re-derived,
            // the probe could agree with the same mistake instead of with the
            // code.
            // It goes INSIDE the loop on purpose: the Newton evaluates points
            // it then rejects, and those are precisely the ones that give the
            // second equation.
            if (mapa_n_ < kMapaTope) {
                mapa_[mapa_n_].muestra = samples_;
                mapa_[mapa_n_].w  = w;
                mapa_[mapa_n_].id = id;
                mapa_[mapa_n_].F  = F;
                ++mapa_n_;
            }
#endif
#if NLSC_E2_TABLA
            (void)dF;
            // The iteration counter is incremented HERE as well: its
            // invariant — "bin 0 stays at zero, because the loop runs at least
            // once" — is what catches a new path skipping the probe, and it
            // caught this one first time.
            // What it counts is REAL: the table takes ONE pass of the body (the
            // index no longer costs any) against the Newton's 1,336. That does
            // NOT authorise publishing a saving: fewer passes is not less
            // clock, and the figure comes from a paired measurement, not from
            // this count.
            if (usa_tabla) {
#if NLSC_E2_ITER_PROBE
                ++iters;
#endif
                (void)it;
                convergio = true;
                break;                                    // everything below is already set
            }
#endif
            const double dw = -F / dF;
#if NLSC_E2_TABLA && NLSC_E2_TABLA_ESPERA > 0 && NLSC_E2_ESPERA_TOL
            // While the table waits for the knob to stop, the Newton is the
            // only thing solving: there it can be asked for more, because the
            // price is paid per GESTURE and not per sample. `tb_espera_` does
            // not change inside the loop, so the selection is hoisted out of
            // it.
            const double tol = tol_abs_ + tol_rel_ * std::fabs(x);
#else
            const double tol = kTolAbs + kTolRel * std::fabs(x);
#endif
            // THE LIMITER MUST BE SYMMETRIC, and it was not.
            //
            // `pnjlim` is SPICE's junction limiting and bounds the FORWARD
            // excursion. Here the diodes are an ANTIPARALLEL pair: the
            // return one sees `-w`, so with the limiter applied only to `w`
            // the negative half went unprotected. Measured: above 3,0 V of
            // input peak, `w` ran away to −4,65 V — impossible across two
            // silicon diodes — and 78 % of samples left Newton unconverged,
            // output collapsed, cost TRIPLED.
            // The telltale signature is the SIGN: it always escaped
            // towards the unlimited side.
            const double wn = w + dw;
            w = (wn >= 0.0) ?  mna::Engine::pnjlim( wn,  w, nvtD_, vcritD_)
                            : -mna::Engine::pnjlim(-wn, -w, nvtD_, vcritD_);
#if NLSC_E2_ITER_PROBE
            ++iters;   // BEFORE the test: the pass that converges is work too
#endif
            if (std::fabs(dw) < tol) { convergio = true; break; }
        }
#if NLSC_E2_ITER_PROBE
        iter_total_ += iters;
        if (iters > iter_max_) iter_max_ = iters;
        ++iter_hist_[iters];   // bin 0 must stay at ZERO: the loop always runs >= 1 pass
#endif
        // DIAGNOSTIC PROBE: how many samples leave the loop UNCONVERGED,
        // and how far the diode pair's voltage strays. It exists because the
        // cascade DIVERGES above +9,5 dBFS and one must know whether it is
        // the Newton or the model before fixing anything.
#if NLSC_E2_MAP_PROBE
        // This sample's CONVERGED `w`, which is what a table would have to
        // return. It is stored separately because the loop's last point is the
        // `w` from BEFORE the final step, not the solution.
        if (wfin_n_ < kMapaTope) wfin_[wfin_n_++] = w;
#endif
        if (!convergio) ++no_conv_;
        if (std::fabs(w) > std::fabs(w_ext_)) w_ext_ = w;
        ++samples_;
        v5_ = v5;
        v7_ = v5 - w;
#if NLSC_E2_HOIST
        v2_ = (gm_ * (x - v5) + s2) * invY2_;
#else
        v2_ = (gm_ * (x - v5) + s2) / Y2;
#endif

        // THESE TWO PROBES COUNT, THEY DO NOT BOUND — and that is the whole
        // mechanism behind the engine going mute. Both say "GUARD" and neither
        // guards anything: they are the witness that this APPROXIMATION is
        // outside its range, and nobody was reading them.
        //
        // This path (`NLSC_E2_OPAMP_LIN`) models a LINEAR opamp. The reference
        // 3x3 version carries the two non-linearities that hold it in — the
        // current-limited transconductance, `imax*tanh(gm*(x-v5)/imax)`, and
        // the clip against the rails, `v3 = clamp(v2, kVeeSat, kVccSat)` — and
        // here they are NOT. Its range is the region where the opamp does not
        // saturate; an approximation carries its range with it.
        //
        // MEASURED with an 880 Hz sine at 3 V and the table OFF: from sample
        // ~33 both count on EVERY sample, i.e. the approximation lives outside
        // its range permanently. With no `tanh` and no clamp, nothing holds
        // `v2`: the output PINS (it moves in the 5th decimal while the input
        // travels 0,9 V), `v2` goes to -373 V, then +1160, +2236, -2,6e7, and
        // at sample 216 **NaN**. `process()`'s belt turns the NaN into EXACTLY
        // ZERO, and since the internal state is already NaN everything that
        // comes in afterwards leaves as zero: MUTE FOREVER.
        // The Newton is NOT to blame: `no_conv` does not fire until AFTER the
        // state is already broken (sample 212 of 215).
        //
        // And that is why the TABLE cures it, although it was not shipped for
        // this: it returns `w` from a grid over the warp `u = R'/(S+|R'|)`,
        // bounded by construction. Same signal, same point: `max|v7|` goes from
        // **539.646 V to 7,44** and there is no NaN.
        // Do not retire the table assuming this regime stays covered.
        // GUARD: would the slew limit have acted?
        if (std::fabs(gm_ * (x - v5)) > imax_) ++slew_;
        // GUARD: did the output HARD CLAMP against the rails actually fire?
        // Same reason as the slew guard next to it -- a hypothesis that is never
        // checked on the real run is not a hypothesis. Added while
        // hunting the 4.8 dB of alias stage 2 hands the DK at 4x: the clamp is a
        // discontinuity and a discontinuity has infinite harmonic order, so
        // whether it fires at all is the first thing to know.
        {
#if NLSC_E2_HOIST
            const double v2p = (gm_ * (x - v5) + s2) * invY2_;
#else
            const double v2p = (gm_ * (x - v5) + s2) / Y2;
#endif
            if (v2p <= kVeeSat || v2p >= kVccSat) ++sat_;
        }

#if NLSC_E2_CHARGE_E3
        // C5's state, with the `v9` that follows from the solved `v7`.
        {
            const double s5b = G5_ * cv5c_ + ci5c_;
#if NLSC_E2_HOIST
            const double v9  = (v7_ * G7_ + s5b + vr_ * G8_) * invGsum_;
#else
            const double v9  = (v7_ * G7_ + s5b + vr_ * G8_) / Gsum_;
#endif
            ci5c_ = G5_ * (v9 - cv5c_) - ci5c_;
            cv5c_ = v9;
        }
#endif
        ci4_ = G4_ * (w - cv4_) - ci4_;  cv4_ = w;
        const double i_leg = Gleg_ * (v5 - vr_ - cv3_)
                           - Gleg_ * invG3_ * ci3_;
        cv3_ = cv3_ + invG3_ * (i_leg + ci3_);  ci3_ = i_leg;
        n11_ = v5 - cv3_;
        icp_ = Gcp_ * (v2_ - cvcp_) - icp_;  cvcp_ = v2_;
        if (kCharge) {
#if NLSC_E2_CHARGE_CACHE && !defined(NLSC_E2_NO_CHARGE)
            const double q1 = o1u.q, q2 = o2u.q;   // already evaluated by the Newton
            di1_ = g2h_ * (q1 - dq1_) - di1_;  dq1_ = q1;
            di2_ = g2h_ * (q2 - dq2_) - di2_;  dq2_ = q2;
#else
            const double q1 = mna::Engine::diode(d_, jd_,  w, kCharge).q;
            di1_ = g2h_ * (q1 - dq1_) - di1_;  dq1_ = q1;
            const double q2 = mna::Engine::diode(d_, jd_, -w, kCharge).q;
            di2_ = g2h_ * (q2 - dq2_) - di2_;  dq2_ = q2;
#endif
        }
        return v7_;
    }

    long slew_ejercido() const { return slew_; }
    long sat_ejercido() const { return sat_; }
#elif defined(NLSC_E2_OPAMP_REAL)
    // ---- path with the REAL OPAMP inside the loop ----
    //
    // WHY: measured, the cascade's residual is NOT the shared rail or the
    // clipping — it is assuming the IDEAL opamp, and it scales with the
    // closed-loop gain the drive pot sets. A post-filter cannot fix it
    // (level-dependent), so the finite gain must live INSIDE the solve.
    //
    // The whole NJM4558 macromodel is used, the SAME one the DK engine has:
    // `Rin`/`Rcm` at the input, current-LIMITED transconductance (the slew
    // mechanism), dominant pole `Rp`‖`Cp`, rail clipping and `Ro` at the
    // output. Unknowns: `v5`, `v7` and the pole's internal node.
    //
    // WHAT IS OMITTED, AND ITS SIZE: the load stage 3 hangs off `n7`. The
    // feedback taps AFTER `Ro`, so the loop corrects that drop up to the
    // loop gain: 75 Ω × (v/1,7 kΩ) ÷ T≈31 => ~−51 dB, below what is being
    // measured. If the residual ever drops past that, this omission becomes
    // the ceiling — and then stage 3's input admittance goes in, which is
    // LINEAR and known, so it would NOT break unidirectionality.
    double process(double x)
    {
        for (int it = 0; it < kMaxIt; ++it) {
            const double w = v5_ - v7_;
            double id, gd;
#if defined(NLSC_E2_NO_CHARGE)
            const double e = std::exp(w * d_.inv_nvt), ei = 1.0 / e;
            id = d_.IS * (e - ei);
            gd = d_.IS * (e + ei) * d_.inv_nvt;
#else
            const mna::Engine::DOut o1 = mna::Engine::diode(d_, jd_,  w, kCharge);
            const mna::Engine::DOut o2 = mna::Engine::diode(d_, jd_, -w, kCharge);
            id = (o1.i + g2h_ * (o1.q - dq1_) - di1_)
               - (o2.i + g2h_ * (o2.q - dq2_) - di2_);
            gd = (o1.g + g2h_ * o1.c) + (o2.g + g2h_ * o2.c);
#endif
            const double iC4 = G4_ * (w - cv4_) - ci4_;
            const double iF  = w / Rf_ + iC4 + id;          // corriente n5 -> n7
            const double giF = 1.0 / Rf_ + G4_ + gd;

            const double i_leg = Gleg_ * (v5_ - vr_ - cv3_)
                               - Gleg_ * invG3_ * ci3_;

            // current-LIMITED transconductance = the slew mechanism
            const double arg = gm_ * (x - v5_) * inv_imax_;
            const double t   = std::tanh(arg);
            const double ign = imax_ * t;
            const double dign = gm_ * (1.0 - t * t);

            const double iCp = Gcp_ * (v2_ - cvcp_) - icp_;
            // clipping against the rails: inside them v3 = v2
            const bool   dentro = (v2_ > kVeeSat && v2_ < kVccSat);
            const double v3 = dentro ? v2_ : (v2_ <= kVeeSat ? kVeeSat : kVccSat);

            const double F1 = iF + i_leg + v5_ * kInvRCM + (v5_ - x) * kInvRIN;
            const double F2 = v2_ / Rp_ + iCp - ign;
            const double F3 = (v7_ - v3) * kInvRO - iF;

            const double J11 = giF + Gleg_ + kInvRCM + kInvRIN, J12 = -giF, J13 = 0.0;
            const double J21 = dign, J22 = 0.0, J23 = 1.0 / Rp_ + Gcp_;
            const double J31 = -giF, J32 = kInvRO + giF, J33 = dentro ? -kInvRO : 0.0;

            // 3x3 by direct elimination
            const double det = J11 * (J22 * J33 - J23 * J32)
                             - J12 * (J21 * J33 - J23 * J31)
                             + J13 * (J21 * J32 - J22 * J31);
            const double d5 = -(F1 * (J22 * J33 - J23 * J32)
                              - J12 * (F2 * J33 - J23 * F3)
                              + J13 * (F2 * J32 - J22 * F3)) / det;
            const double d7 = -(J11 * (F2 * J33 - J23 * F3)
                              - F1 * (J21 * J33 - J23 * J31)
                              + J13 * (J21 * F3 - F2 * J31)) / det;
            const double d2 = -(J11 * (J22 * F3 - F2 * J32)
                              - J12 * (J21 * F3 - F2 * J31)
                              + F1 * (J21 * J32 - J22 * J31)) / det;

            v5_ += d5; v2_ += d2;
            const double v7n = v7_ + d7;
            v7_ = v5_ - mna::Engine::pnjlim(v5_ - v7n, w, nvtD_, vcritD_);

            const double tol = kTolAbs + kTolRel * std::fabs(x);
            if (std::fabs(d5) < tol && std::fabs(d7) < tol && std::fabs(d2) < tol) break;
        }

        const double w = v5_ - v7_;
        ci4_ = G4_ * (w - cv4_) - ci4_;  cv4_ = w;
        const double i_leg = Gleg_ * (v5_ - vr_ - cv3_)
                           - Gleg_ * invG3_ * ci3_;
        cv3_ = cv3_ + invG3_ * (i_leg + ci3_);  ci3_ = i_leg;
        n11_ = v5_ - cv3_;
        icp_ = Gcp_ * (v2_ - cvcp_) - icp_;  cvcp_ = v2_;
        if (kCharge) {
            const double q1 = mna::Engine::diode(d_, jd_,  w, kCharge).q;
            di1_ = g2h_ * (q1 - dq1_) - di1_;  dq1_ = q1;
            const double q2 = mna::Engine::diode(d_, jd_, -w, kCharge).q;
            di2_ = g2h_ * (q2 - dq2_) - di2_;  dq2_ = q2;
        }
        return v7_;
    }
#elif defined(NLSC_E2_NO_RS)
    // ---- SCALAR path: one unknown, `w = v(n5) − v(n7)` ----
    double process(double v5)
    {
        const double i_leg = Gleg_ * (v5 - vr_ - cv3_) - Gleg_ * invG3_ * ci3_;
        const double i_rcm = v5 * kInvRCM;
        double w = v5 - v7_;

        for (int it = 0; it < kMaxIt; ++it) {
            double id, gd;
#if defined(NLSC_E2_NO_CHARGE)
            // Without charge the pair is exactly `2·IS·sinh(w/nVt)`, so one
            // exponential suffices: `e − 1/e`. It is the SAME expression as
            // two charge-less `diode()` calls, not an approximation.
            const double e = std::exp(w * d_.inv_nvt), ei = 1.0 / e;
            id = d_.IS * (e - ei);
            gd = d_.IS * (e + ei) * d_.inv_nvt;
#else
            const mna::Engine::DOut o1 = mna::Engine::diode(d_, jd_,  w, kCharge);
            const mna::Engine::DOut o2 = mna::Engine::diode(d_, jd_, -w, kCharge);
            id = (o1.i + g2h_ * (o1.q - dq1_) - di1_)
               - (o2.i + g2h_ * (o2.q - dq2_) - di2_);
            gd = (o1.g + g2h_ * o1.c) + (o2.g + g2h_ * o2.c);
#endif
            const double iC4 = G4_ * (w - cv4_) - ci4_;
            const double F  = w / Rf_ + iC4 + id + i_leg + i_rcm;
            const double dF = 1.0 / Rf_ + G4_ + gd;
            const double dw = -F / dF;
            const double tol = kTolAbs + kTolRel * std::fabs(v5);
            // The junction limits as in SPICE: without this a cold start
            // overflows the exponential and the Newton never comes back.
            w = mna::Engine::pnjlim(w + dw, w, nvtD_, vcritD_);
            if (std::fabs(dw) < tol) break;
        }
        v7_ = v5 - w;

        ci4_ = G4_ * (w - cv4_) - ci4_;
        cv4_ = w;
        cv3_ = cv3_ + invG3_ * (i_leg + ci3_);
        ci3_ = i_leg;
        n11_ = v5 - cv3_;
#if !defined(NLSC_E2_NO_CHARGE)
        const double q1 = mna::Engine::diode(d_, jd_,  w, kCharge).q;
        di1_ = g2h_ * (q1 - dq1_) - di1_;  dq1_ = q1;
        const double q2 = mna::Engine::diode(d_, jd_, -w, kCharge).q;
        di2_ = g2h_ * (q2 - dq2_) - di2_;  dq2_ = q2;
#endif
        return v7_;
    }
#else
    // ---- reference path: 3x3, with `RS` and junction charge ----
    double process(double v5)
    {
        // --- C3 + R5 leg towards VR: explicit, `v5` is known ---
        // Norton of the series: the current leaving n5 into the leg.
        const double i_leg = Gleg_ * (v5 - vr_ - cv3_) - Gleg_ * invG3_ * ci3_;

        // --- RCM current and linear term, constants inside the Newton ---
        const double i_rcm = v5 * kInvRCM;

        for (int it = 0; it < kMaxIt; ++it) {
            const double vd1 = va1_ - v7_;     // D1's junction (anode at n5)
            const double vd2 = va2_ - v5;      // D2's junction (anode at n7)
            const mna::Engine::DOut o1 = mna::Engine::diode(d_, jd_, vd1, kCharge);
            const mna::Engine::DOut o2 = mna::Engine::diode(d_, jd_, vd2, kCharge);

            const double ij1 = o1.i + g2h_ * (o1.q - dq1_) - di1_;
            const double ij2 = o2.i + g2h_ * (o2.q - dq2_) - di2_;
            const double g1  = o1.g + g2h_ * o1.c;
            const double g2  = o2.g + g2h_ * o2.c;

            const double iC4 = G4_ * ((v5 - v7_) - cv4_) - ci4_;

            const double F1 = (v5 - v7_) / Rf_ + iC4 + (v5 - va1_) * kInvRS
                              - ij2 + i_leg + i_rcm;
            const double F2 = ij1 - (v5 - va1_) * kInvRS;
            const double F3 = ij2 - (v7_ - va2_) * kInvRS;

            const double J11 = -1.0 / Rf_ - G4_;
            const double J12 = -kInvRS;
            const double J13 = -g2;
            const double J21 = -g1,      J22 = g1 + kInvRS;
            const double J31 = -kInvRS,  J33 = g2 + kInvRS;

            // Hand elimination: F2 and F3 each touch one unknown besides
            // v7, so the 3x3 reduces to a single division.
            const double den = J11 - J12 * J21 / J22 - J13 * J31 / J33;
            const double num = -F1 + J12 * F2 / J22 + J13 * F3 / J33;
            const double d7  = num / den;
            const double d1  = (-F2 - J21 * d7) / J22;
            const double d2  = (-F3 - J31 * d7) / J33;

            const double v7n = v7_ + d7;
            // Junctions limit as in SPICE: without this, a cold start
            // overflows the exponential and the Newton never comes back.
            const double vd1n = mna::Engine::pnjlim((va1_ + d1) - v7n, vd1, nvtD_, vcritD_);
            const double vd2n = mna::Engine::pnjlim((va2_ + d2) - v5,  vd2, nvtD_, vcritD_);
            v7_  = v7n;
            va1_ = vd1n + v7n;
            va2_ = vd2n + v5;

            // Tolerance RELATIVE to the node's own scale: an absolute
            // threshold on a magnitude whose scale you do not set is either
            // unreachable or trivial.
            const double tol = kTolAbs + kTolRel * std::fabs(v5);
            if (std::fabs(d7) < tol && std::fabs(d1) < tol && std::fabs(d2) < tol)
                break;
        }

        // --- state update (trapezoidal, same as the engine) ---
        const double vc4 = v5 - v7_;
        ci4_ = G4_ * (vc4 - cv4_) - ci4_;
        cv4_ = vc4;

        // The leg's state is C3's voltage, i.e. v(n5) − v(n11), integrated
        // with the SAME trapezoidal the engine uses for its twelve
        // capacitors: v <- v + (h/2C)·(i + i_prev).
        cv3_ = cv3_ + invG3_ * (i_leg + ci3_);
        ci3_ = i_leg;

        if (kCharge) {
            const double q1 = mna::Engine::diode(d_, jd_, va1_ - v7_, kCharge).q;
            di1_ = g2h_ * (q1 - dq1_) - di1_;  dq1_ = q1;
            const double q2 = mna::Engine::diode(d_, jd_, va2_ - v5, kCharge).q;
            di2_ = g2h_ * (q2 - dq2_) - di2_;  dq2_ = q2;
        }

        return v7_;
    }
#endif

private:
    // Netlist values (harness/spice/full.inc). NOT constants copied
    // from a binary: they are the circuit's components.
    // Per-element bilinear alphas. 1.0 = plain trapezoidal = bit-identical.
    // Overridable from the build so a sweep needs no source edit.
#ifndef NLSC_E2_A_C3
#define NLSC_E2_A_C3 1.0
#endif
#ifndef NLSC_E2_A_C4
#define NLSC_E2_A_C4 1.0
#endif
#ifndef NLSC_E2_A_C5
#define NLSC_E2_A_C5 1.0
#endif
#ifndef NLSC_E2_A_CP
#define NLSC_E2_A_CP 1.0
#endif
#ifndef NLSC_E2_A_QD
#define NLSC_E2_A_QD 1.0
#endif
    static constexpr double kA_C3 = NLSC_E2_A_C3;   // C3, the R5 leg
    static constexpr double kA_C4 = NLSC_E2_A_C4;   // C4, 51 pF across the feedback
    static constexpr double kA_C5 = NLSC_E2_A_C5;   // C5, stage-3 load
    static constexpr double kA_CP = NLSC_E2_A_CP;   // the opamp compensation cap
    static constexpr double kA_QD = NLSC_E2_A_QD;   // the diode junction charge
    double invG3_ = 0.0;                            // h*alpha/(2*C3)
    static constexpr double kC4  = 51e-12;
    static constexpr double kC3  = 0.047e-6;
    static constexpr double kR5  = 4.7e3;
    // STAGE 3's LOAD ON `n7`. From the netlist:
    //     R7 n7 n9 1k · C5 n9 0 0.22u · R8 n9 vr 10k
    // Stage 2 used to solve as if `n7` fed NOTHING, and that was the
    // missing term: the `correct()` comment in `nls_cascada.h` already
    // The prescription was "put the finite gain inside the loop (AND WITH IT
    // STAGE 3's LINEAR LOAD ON n7)", and only the first half had been done.
    // It deliberately does NOT include the TONE network (RTONEA/RTONEB,
    // >=20 k): at high frequency `C5` dominates the load and that is where
    // the error is worst. If measurement says more is missing, it gets
    // added — but first measure what this captures.
    // OFF BY DEFAULT until measurement says it ships: it is a MODEL change
    // in the shipped engine. Enabled with -DNLSC_E2_CHARGE_E3=1.
    static constexpr double kR7  = 1.0e3;
    static constexpr double kC5  = 0.22e-6;
    static constexpr double kR8  = 10.0e3;
    // ONE definition, not a copy. This used to be a literal 0,568 sitting a
    // file away from `NLSC_DIODE_RS`, which is exactly how two copies of a
    // value diverge: editing the diode would have moved the DK and left the
    // cascade on the old part, and nothing would have failed.
    static constexpr double kRS  = NLSC_DIODE_RS;  // the MA150's RS
    static constexpr double kInvRS  = 1.0 / kRS;
    static constexpr double kInvRCM = 1.0 / 500e6; // macromodel Rcm2
    static constexpr double kInvRIN = 1.0 / 5e6;   // Rin between the inputs
    // `Ro` = 0 is a DISCRIMINATOR, not a model. The F1 path's declared
    // omission is the load stage 3 hangs off `n7`; it was estimated at
    // −51 dB using the 1 kHz LOOP gain (2140), but at 5 kHz that gain is
    // 8,2, and then the drop across `Ro` leaves ~−42 dB — the order of the
    // floor being measured. With `Ro` ≈ 0 the output is rigid and the load
    // CANNOT matter: if the null does not improve, the omitted load was not
    // the ceiling.
#if defined(NLSC_E2_RO0)
    static constexpr double kInvRO  = 1.0 / 1e-3;
#else
    static constexpr double kInvRO  = 1.0 / 75.0;  // macromodel Ro
#endif
    static constexpr double kVccSat = 9.0 - 1.5;   // clipping against the rails
    static constexpr double kVeeSat = 0.0 + 1.5;
    // THREE JUNCTION-CHARGE MODES, not two. The charge's two terms do NOT
    // cost the same: DEPLETION carries a `pow` (the assembly's most
    // expensive transcendental) and DIFFUSION is two multiplies reusing the
    // exponential already computed above. Dropping them together — what
    // `NLSC_E2_NO_CHARGE` did — mixes a COST lever with a FIDELITY lever and
    // forbids attribution.
    //   (nothing)            -> 1, full
    //   -DNLSC_E2_CHARGE_TT   -> 2, diffusion ONLY: saves the `pow`
    //   -DNLSC_E2_NO_CHARGE  -> 0, none
    // An earlier default was 3 — DEPLETION WITHOUT DIFFUSION. Measured then:
    // dropping diffusion cost 0,1 dB of null (−60,6 -> −60,5) and saved
    // −10,6 % of the cascade's clock (paired ratio against the engine
    // 0,6209 -> 0,5552, 21/21). Not just the two multiplies: the `TT` term
    // also forces two EXTRA `diode()` calls per sample in the state update,
    // each with its exponential.
    // Modes 4 and 5 (explicit linearised depletion) EXISTED and were
    // retired: once linearisation went global (`NLSC_JUNC_LIN`, default)
    // they were exact duplicates of 1 and 3, and a duplicated mode is a
    // place where two measurements say the same thing by different routes
    // and nobody knows.
// DIFFUSION IS BACK IN, AND IT IS THE DEFAULT.
//
// Dropping it was an optimisation, judged against the null, which read it as
// "0 % of the error for 10.6 % of the clock". The null cannot see it: the
// diffusion capacitance is proportional to the diode current, so it exists only
// while the diodes conduct and it acts near Nyquist. It was the WHOLE of the
// 4.8 dB of alias stage 2 was handing the DK at 4x.
//
// Restoring it, measured on the shipped path: the pinched-harmonic ANMR -- the
// only criterion that can fail -- goes from -10.9 dB (margin +0.9) to -15.3
// (margin +5.3), landing on the DK's -15.4; the gap against the DK closes from
// +4.8 dB to -0.1 at 0.50 and from +6.6 to +0.1 at 0.45, where it peaked; and
// the null improves from -68.0 to -68.2. Cost, paired with the DK as an
// in-process control: +12.6 % of the cascade.
//
// Keep the opt-outs for measurement only. Modes 0 and 2 cost 15.2 dB.
#if defined(NLSC_E2_NO_CHARGE)
    static constexpr int    kCharge = 0;   // none — costs 15,2 dB
#elif defined(NLSC_E2_CHARGE_TT)
    static constexpr int    kCharge = 2;   // diffusion only — costs 15,2 dB
#elif defined(NLSC_E2_NO_DIFFUSION)
    static constexpr int    kCharge = 3;   // depletion alone: the FORMER default
#else
    static constexpr int    kCharge = 1;   // DEFAULT: depletion + diffusion
#endif
#if NLSC_E2_PREDICTOR >= 1
    // Predictor history. `n_pred_` counts how many VALID samples sit behind:
    // without that guard the first extrapolation leaves from rest.
    double wp1_ = 0.0, wp2_ = 0.0;
    int    n_pred_ = 0;
#endif
    // Convergence-probe counters (not product).
    long   no_conv_ = 0, samples_ = 0;
    double w_ext_ = 0.0;
public:
    long   no_conv() const   { return no_conv_; }
    long   samples() const  { return samples_; }
    double w_extremo() const { return w_ext_; }
    void   reset_probe()
    {
        no_conv_ = samples_ = 0; w_ext_ = 0.0;
#if NLSC_E2_ITER_PROBE
        reset_iter();
#endif
    }
private:
// THE ITERATION CAP — AND AT `=1` IT IS NO COST PROBE.
//
// Added to put a number on a NON-ITERATIVE scheme's ceiling (N1): if one
// step costs one iteration, forcing 1 would give the saving. It does NOT
// work, and fails the expensive way: at `=1` the Newton does not
// converge, the engine goes MUTE (rms 0,00000 over 5 s of DI) and the
// clock comes out +27,74 % SLOWER — a broken model is dearer, not
// cheaper.
// And `isfinite` on the output says True: silence is finite, so
// hunting `nan` does not catch it. The only signal was the RMS.
// => N1's ceiling CANNOT be measured this way: it takes a one-step scheme
// that CONVERGES, i.e. implementing it. What does hold is the profile
// bound, written in `docs/TODO.md`.
#ifndef NLSC_E2_ITER_CAP
#  define NLSC_E2_ITER_CAP 40
#endif
    static constexpr int    kMaxIt = NLSC_E2_ITER_CAP;
    // THE STOPPING CRITERION. An inherited tolerance is a cost lever: it
    // gets written once, early, and is never revisited while everything else in
    // the file is measured.
    //
    // These asked for 1e-12 absolute plus 1e-11 relative -- nine orders tighter
    // than the DK's 1e-3 V step -- and `make iter-e2` measured what that costs:
    // 3,5 Newton passes per sample against the DK's 1,159, which with quadratic
    // convergence is about two extra passes.
    //
    // The relaxation is NOT scored on the null. A badly resolved solver is
    // paid in ALIAS before it is paid in residual: at 1e-4/1e-3 the damage is
    // 24,8 dB of ANMR against only 14,0 dB of null, so the null under-reports
    // it. And the DK's own setting does not transplant -- it lands stage 2 on
    // the DK's iteration count and FAILS the port bar.
    //
    // 1e-6/1e-5 is what ships. It got here in TWO steps, and the second only
    // became possible because of the quadratic Newton seed: a better seed lands
    // the first step nearer the root, so stopping earlier costs less. Measured
    // on the same arm, predictor 0 against predictor 2: at 1e-4/1e-3 the damage
    // goes from ANMR -4,5 / null -46,0 to -14,7 / -58,1, i.e. the seed buys back
    // 10,2 and 12,1 dB. A tolerance is therefore NOT a property of the solver
    // alone -- it is a property of the pair (seed, tolerance), and re-opening one
    // re-opens the other.
    //
    // What ships was measured whole before it shipped: 1,400 Newton passes per
    // sample against 2,037, -11,37 % of a core paired (21 of 21, p = 9,54e-07)
    // with the DK engine as a control at +0,00 %, own residual -94,73 dB against
    // the tighter build, the 100-point ANMR grid IDENTICAL (14 cells better, 86
    // tied, none worse) and against ngspice 0 of 36 above the -60 bar in both
    // variants, medians and worsts equal to the digit.
    //
    // The scoring yardstick is the ANMR, never the null: a badly resolved
    // solver is paid in ALIAS before it is paid in residual.
    // 1e-4/1e-3 is still NOT transplantable -- it fails the port bar.
    //
    // => Still overridable, so the sweep stays reproducible and either previous
    // setting can be selected without editing this file.
#ifndef NLSC_E2_TOL_ABS
#define NLSC_E2_TOL_ABS 1e-6
#endif
#ifndef NLSC_E2_TOL_REL
#define NLSC_E2_TOL_REL 1e-5
#endif
    static constexpr double kTolAbs = NLSC_E2_TOL_ABS;
    static constexpr double kTolRel = NLSC_E2_TOL_REL;

#if NLSC_E2_ITER_PROBE
    // Newton iteration histogram. `iter_hist_[k]` = samples that spent
    // exactly `k` passes; bin 0 is an INVARIANT that has to stay at zero, and
    // the harness checks it.
    long iter_total_ = 0;
    long iter_hist_[kMaxIt + 1] = {0};
    int  iter_max_ = 0;
public:
    long iter_total()      const { return iter_total_; }
    int  iter_max()        const { return iter_max_; }
    long iter_hist(int k)  const { return (k >= 0 && k <= kMaxIt) ? iter_hist_[k] : -1; }
    static constexpr int iter_tope() { return kMaxIt; }
    void reset_iter()
    {
        iter_total_ = 0; iter_max_ = 0;
        for (int k = 0; k <= kMaxIt; ++k) iter_hist_[k] = 0;
    }
private:
#endif

#if NLSC_E2_HOIST
    // HOISTED OUT OF THE PER-SAMPLE LOOP, in a consumption campaign.
    //
    // `Y2`, `k`, `Yeq_`, `D_` and `kL_` depend only on `Rp_`, `Gcp_`, `gm_`,
    // `Rf_` and `G5_`/`G7_`/`G8_`, and were being recomputed 192.000 times per
    // second. The divisions by `Y2`, `Gsum_` and `Rf_` are by constants too, so
    // they become precomputed reciprocals. Measured with `perf annotate`:
    // divisions are 18,8 % of this function and transcendentals 0 % (inlined) —
    // the cost was not where it was assumed to be.
    //
    // WHY THIS IS A METHOD AND NOT A BLOCK INSIDE `prepare()`.
    // `reposo()` deliberately zeroes `Gcp_` (and `G4_`, `Gleg_`, `g2h_`) to
    // solve the DC operating point, then restores them. A value hoisted once in
    // `prepare()` does NOT see that temporary mutation, so the quiescent point
    // is solved with the wrong `Y2` and the plugin starts from a wrong state.
    // Measured before the fix: the steady state is unchanged (all three
    // yardsticks blind) but the first 0,1 s carry an error of 0,389 absolute.
    // => every writer of an input must call `izar()` afterwards.
    //
    // Multiplying by the reciprocal is NOT bit-identical to dividing (1 ULP).
    // It is measured with the three yardsticks, not assumed.
#if NLSC_E2_MAP_PROBE || NLSC_E2_TABLA
public:
    // `ID(w)`: the part of `id(w)` that depends ONLY on `w`, i.e. `id` minus
    // its state term. It is written with the SAME calls and the same `g2h_` as
    // the loop, rather than re-derived, so it cannot diverge from it. It
    // satisfies `ID(0) = 0` by construction: at `w = 0` both diodes of the pair
    // see the same thing.
    double ID(double w) const
    {
        const mna::Engine::DOut a1 = mna::Engine::diode(d_, jd_,  w, kCharge);
        const mna::Engine::DOut a2 = mna::Engine::diode(d_, jd_, -w, kCharge);
        return (a1.i + g2h_ * a1.q) - (a2.i + g2h_ * a2.q);
    }

    // `ID(w)` AND ITS DERIVATIVE, with ONE evaluation of the pair. The
    // derivative is the same expression the audio loop calls `gd`: written with
    // the SAME calls and the same `g2h_` as `ID`, so the two cannot diverge.
    // The sign: the return diode is evaluated at `-w`, so the chain rule
    // contributes `-1`, and since its contribution to `ID` already enters as a
    // subtraction, the two derivatives ADD. That is what `gd` does in the
    // loop.
    void ID_d(double w, double& valor, double& deriv) const
    {
        const mna::Engine::DOut a1 = mna::Engine::diode(d_, jd_,  w, kCharge);
        const mna::Engine::DOut a2 = mna::Engine::diode(d_, jd_, -w, kCharge);
        valor = (a1.i + g2h_ * a1.q) - (a2.i + g2h_ * a2.q);
        deriv = (a1.g + g2h_ * a1.c) + (a2.g + g2h_ * a2.c);
    }
private:
#endif

#if NLSC_E2_MAP_PROBE || NLSC_E2_TABLA
// The `NLSC_E2_HOIST` guard used to live HERE and COULD NOT FIRE: this whole
// block is inside a `#if NLSC_E2_HOIST`, so with the hoist off the preprocessor
// skipped it and what came out instead was "avanza_relleno_por_muestra was not
// declared in this scope", which says nothing. It now sits at the very top of
// the file.
    // THE IMPLICIT-MAP TABLE.
    //
    // PER SAMPLE, paired against the Newton that ships (6 knob points x
    // variant, 15 pairs each, 87 of 90 with the same sign, and the REVERSED
    // direction reproduces the ratio to 0,25 %):
    //     drive 0,0   −4,58 % (808)  ·  −4,77 % (9RI)
    //     drive 0,5   −5,99 %         ·  −5,20 %
    //     drive 1,0   −7,32 %         ·  −6,70 %
    // The saving GROWS with drive because the table costs a constant and the
    // Newton iterates more the harder the diode conducts. The quotable figure
    // is the UNFAVOURABLE end, −4,6 %, not the 7,3, and the 100 % is the whole
    // PLUGIN, not stage 2.
    // Fidelity: worst of 6 cells **−96,50 dB** against the engine that ships —
    // the same place as before these two steps, 34 dB below the port's worst
    // cell.
    //
    // WHAT USED TO BLOCK IT WAS THE OTHER BUDGET: RE-TUNING.
    // `apply_knobs()` runs on the AUDIO THREAD and calls `prepare()` on every
    // control change, so building this table is paid there:
    //     moving a knob   0,0035 ms  ->  0,4817 ms
    //                       (0,4 %   ->  36,1 % of a 1,333 ms block)
    // A cache has TWO budgets, and the null, the ANMR and the % of CPU measure
    // only the first. The way out did not have to be invented: stage 4's cache
    // fills DEFERRED in pieces, using the expensive path meanwhile, and here
    // the expensive path is the Newton, which is still compiled.
    //
    // The coefficients are DERIVED from the `dF` the loop itself computes:
    //     dF = giF + G*kL_*(D_ + kRO*giF),  with giF = invRf_ + G4_ + gd
    //        = [invRf_+G4_ + G*kL_*(D_ + kRO*(invRf_+G4_))] + gd*[1 + G*kL_*kRO]
    // i.e. `dF = P + Q*gd`, which is exactly the measured form.
    // And the CONTROL is that the prototype measured them by another route — a
    // fit over 135.000 iteration pairs — and got `P = 2,591505e-05`,
    // `Q = 1,000315` at drive 1,0. If the derivative here does not reproduce
    // those, one of the two is wrong; they are independent routes, not the same
    // one twice.
    static constexpr int kTablaN = NLSC_E2_TABLA_N;
    static constexpr long kEspera = NLSC_E2_TABLA_ESPERA;
    // The fill solves from a seed by CONTINUATION, so it converges in a few
    // steps; the cap is generous and shared with the audio Newton rather than
    // carrying a hand-written 200 that nobody governs.
    static constexpr int kMaxItRelleno = 8 * NLSC_E2_ITER_CAP;
    static constexpr double kEspTolAbs = NLSC_E2_ESPERA_TOL_ABS;
    static constexpr double kEspTolRel = NLSC_E2_ESPERA_TOL_REL;
    double tabla_[kTablaN + 1] = {0.0};
    // THE OTHER THREE TABLES — ONE SINGLE INDEX.
    //
    // With `w` alone nothing is saved: the body still had to evaluate
    // `diode_par(w)` to get `id(w)` — which enters `iF` — and the two charges
    // `q(w)`, `q(-w)`, which are the NEXT sample's state. That was the only
    // exponential left on the path, and it is the one these three remove.
    //
    // They share `tabla_`'s index on purpose: `w` is a function of `R'`, so
    // everything that is a function of `w` is a function of `R'`, and the warp,
    // the division and the bin are computed ONCE. Four contiguous reads per
    // sample against one exponential and one division.
    //
    // And the price in memory has to be said: 4 x 4097 x 8 B = 131 kB, which is
    // L2 and NOT L1 — a cache that does not fit is paid in misses, not in
    // arithmetic, and no operation count sees that. That is what the paired
    // measurement has to answer, which is why no saving is published before
    // running it.
    double tID_[kTablaN + 1] = {0.0};   // ID(w): the part of `id` depending on w alone
    double tq1_[kTablaN + 1] = {0.0};   // q(+w)
    double tq2_[kTablaN + 1] = {0.0};   // q(-w)
    double tP_ = 0.0, tQ_ = 0.0, tS_ = 1.0;
    long fuera_caja_ = 0;      // samples the table REFUSED to answer
    long relleno_no_conv_ = 0; // bins that hit the cap without converging
#if NLSC_E2_TABLA
    // What the table hands a sample's loop. Members rather than locals because
    // the loop that consumes them sits behind several `#if`s, and a local
    // declared outside them would be unused in the other configurations.
    double id_tabla_ = 0.0, q1_tabla_ = 0.0, q2_tabla_ = 0.0;
#endif
#if NLSC_E2_MAP_PROBE || NLSC_E2_TABLA
    // THE DEFERRED FILL — the cursor's state.
    //
    // Building the whole table at once costs 0,4817 ms, i.e. 36,1 % of a block,
    // and `apply_knobs()` runs on the AUDIO THREAD => a peak there for every
    // knob the user moves. The pattern that fixes it is not invented here: it
    // is stage 4's cache, in `nls_cascada.h`, and it is measured.
    //
    // The detail that makes it work is WHEN it advances, not by how much: it
    // goes PER SAMPLE, not per block. Advancing per block, how many bins are
    // done at sample `n` depends on how the host SLICES, and block invariance
    // breaks; per sample, the schedule is a function of the SAMPLE INDEX and
    // stops seeing the slicing.
    //
    // While it is half built no new path and no new test are needed: `w` comes
    // from the NEWTON, which is still compiled — just as stage 4's cache falls
    // back to the solver when its point lands outside its box.
    bool   tb_lista_ = false;   // can it be read yet?
    bool   tb_pend_  = false;   // is there fill left to do?
    bool   tb_dif_   = false;   // deferred? (harnesses with no block boundary turn it off)
    int    tb_b_     = 0;       // the bin the cursor is on
    double tb_wprev_ = 0.0;     // the continuation seed, BETWEEN pieces
    long   tb_espera_ = 0;      // internal samples of a STILL knob still to go
    // IT ONLY WAITS IF THE RE-TUNE WAS ASKED FOR BY THE USER, and that fact is
    // not guessed here: the CALLER has it. `apply_knobs()` already tells the
    // two reasons for arriving apart (`reinit`), and the caller always decides.
    //
    // WHY, AND IT TOOK TWO ATTEMPTS. The wait exists so as not to rebuild WHILE
    // THE USER IS GESTURING; during set-up there is no gesture. The first
    // attempt armed it always and broke `make cache-rodajas`; the second
    // exempted "the first build" and failed too, because `prepare()`/`rest()`
    // call `hoist()` SEVERAL times and the second already looked like a
    // rebuild.
    // And `cache-rodajas` is NOT a block-size gate: it is stage 4's cache's
    // DEFERRED-against-ALL-AT-ONCE equivalence, so any deferral that survives
    // set-up breaks it by construction.
    bool   tb_gesto_ = false;
#endif

public:
    double tabla_P() const { return tP_; }
    double tabla_Q() const { return tQ_; }
private:

    // ARMS the cursor: computes the coefficients and leaves the table UNBUILT.
    // It fills nothing if the fill is deferred. If it is not — harnesses that
    // drive the stage directly and have no block boundary to hang the advance
    // on — it drains the whole thing right here, which is the SAME work in the
    // SAME order with the same seed, so the table comes out identical.
    // Without this, those harnesses would measure the NEWTON and label it as
    // the table.
    void construye_tabla()
    {
        const double G = Gleg_ + kInvRCM + kInvRIN;
        tP_ = (invRf_ + G4_) + G * kL_ * (D_ + kRO * (invRf_ + G4_));
        tQ_ = 1.0 + G * kL_ * kRO;
        // Warp scale: the `R'` at which `w` reaches one kT/q, which is where
        // the curve stops being linear. It comes off the curve itself rather
        // than being assumed.
        const double wkt = 0.02585;
        tS_ = std::fabs(tP_ * wkt + tQ_ * ID(wkt));
        if (!(tS_ > 0.0)) tS_ = 1e-9;
        // Re-arming DISCARDS whatever half table there was: its bins were
        // solved with the OLD coefficients, and mixing them would be a table
        // describing no circuit at all. That is what `build_cache()` does.
        tb_b_     = 0;
        tb_wprev_ = 0.0;
        tb_lista_ = false;
        // A harness with no block boundary cannot advance anything: it is
        // drained right here. Otherwise it would measure the NEWTON and label
        // it as the table.
        if (!tb_dif_) { tb_pend_ = true; tb_espera_ = 0;
                        while (!avanza_tabla(kTablaN + 1)) { } return; }
        // With the wait armed, this re-arm leaves NOTHING pending: it only
        // RESTARTS the count. While the knob moves, every block comes through
        // here and the count never reaches zero, so NOTHING is built.
        // THE `else` MUST NOT DRAIN. Written draining, it broke the deferred
        // fill that already existed: with the wait at 0 the `else` runs always,
        // so the 1-bin-per-sample arm went back to building the whole table at
        // once. The paired bench caught it because it carries the old arms
        // INSIDE: the deferred and all-at-once arms read 15,56 and 15,58 where
        // deferred was worth 4,89.
        // => With no wait armed, this leaves EXACTLY what came before: pending,
        // and the per-sample advance fills it at its own pace.
        if (kEspera > 0 && tb_gesto_) { tb_pend_ = false; tb_espera_ = kEspera; }
        else                          { tb_pend_ = true;  tb_espera_ = 0; }
    }

    // Solves at most `bins` bins and remembers where it was. Returns true if
    // this call left the table FINISHED.
    // The continuation seed survives the cut because `tb_wprev_` travels
    // between pieces: cutting it in half costs nothing, the previous `w` just
    // has to come along — like stage 4's cache cursor.
    bool avanza_tabla(int bins)
    {
        if (!tb_pend_) return false;
        while (bins-- > 0) {
            const int b = tb_b_;
            const double u  = -1.0 + 2.0 * double(b) / double(kTablaN);
            const double au = std::fabs(u);
            double w;
            if (au >= 1.0) {
                w = (u > 0.0) ? 1.0 : -1.0;
            } else {
                const double Rp = tS_ * u / (1.0 - au);
                // CONTINUATION SEED. The bins are walked in increasing `u` and
                // `w(R')` is monotone, so the previous bin's solution is one
                // step from this one. Starting each bin from ZERO — what the
                // prototype did — pays the whole path 4097 times.
                w = tb_wprev_;
                // Non-convergences are COUNTED: this loop does the 4097 bins
                // in ONE sample, and an unconverged bin would store a bad `w`
                // that is then read for the whole gesture with no signal at
                // all.
                int it = 0;
                for (; it < kMaxItRelleno; ++it) {
                    // ANALYTIC derivative, not numerical. `diode` already
                    // returns `g` (di/dv) and `c` (dq/dv), and it is the SAME
                    // expression the audio loop calls `gd`. A three-point one
                    // cost THREE evaluations of `ID` — six `diode` calls — per
                    // iteration, and it introduced an `h` that is one more
                    // tolerance nobody reviews.
                    double f_id, d_id;
                    ID_d(w, f_id, d_id);
                    const double f = tP_ * w + tQ_ * f_id - Rp;
                    const double d = tP_ + tQ_ * d_id;
                    if (!(std::fabs(d) > 0.0)) break;
                    double paso = -f / d;
                    if (paso >  0.05) paso =  0.05;
                    if (paso < -0.05) paso = -0.05;
                    w += paso;
                    if (std::fabs(paso) < NLSC_E2_TABLA_FILL_TOL) break;
                }
                if (it >= kMaxItRelleno) ++relleno_no_conv_;
                tb_wprev_ = w;
            }
            tabla_[b] = w;
            // The three companions are filled IN THE SAME step as their `w`,
            // not in a second pass: if the fill is cut in half, half a table
            // with `w` set and `ID`/`q` at zero is exactly a table that does not
            // fail and answers wrongly.
            tID_[b] = ID(w);
            tq1_[b] = mna::Engine::diode(d_, jd_,  w, kCharge).q;
            tq2_[b] = mna::Engine::diode(d_, jd_, -w, kCharge).q;
            if (++tb_b_ > kTablaN) {
                tb_pend_  = false;
                tb_lista_ = true;
                return true;
            }
        }
        return false;
    }

public:
    // The PER-SAMPLE advance. `process()` calls it, not the host: that way the
    // schedule is a function of the SAMPLE INDEX and not of the host's
    // slicing.
    void avanza_relleno_por_muestra()
    {
        if (!tb_dif_) return;
        // The still-knob countdown: one decrement per sample while the user
        // turns, and that is ALL that is paid in that regime.
        if (tb_espera_ > 0) {
            if (--tb_espera_ == 0) { tb_b_ = 0; tb_wprev_ = 0.0; tb_pend_ = true; }
            else return;
        }
        if (tb_pend_) avanza_tabla(NLSC_E2_TABLA_BINS_POR_MUESTRA);
    }
    void tabla_diferida(bool v) { tb_dif_ = v; }
    // Armed by whoever KNOWS why the re-tune happened, not by this stage.
    void tabla_gesto(bool v) { tb_gesto_ = v; }
    // During the WAIT there is nothing "pending" in the cursor and the table
    // does not exist either: returning false there would read as "already
    // ready".
    bool tabla_pendiente() const { return tb_pend_ || tb_espera_ > 0; }
    // Observability: "it never fires" has to be CHECKABLE rather than assumed
    // — it is half of what stage 4's cache box is for.
    long tabla_fuera_caja() const { return fuera_caja_; }
    long tabla_relleno_no_conv() const { return relleno_no_conv_; }
    void tabla_fuera_caja_reset() { fuera_caja_ = 0; relleno_no_conv_ = 0; }
    // Observability: how many bins are left, so that "the table comes back" is
    // CHECKABLE and not an assumption.
    long tabla_bins_restantes() const { return tb_pend_ ? long(kTablaN + 1 - tb_b_) : 0; }
private:

    // The four quantities that depend on `w`, with ONE warp, ONE division and
    // ONE bin. `id` does not leave here complete: it is missing its STATE term,
    // which the caller adds (the same `id0` as the index).
    struct Tab { double w, ID, q1, q2; };
    inline Tab tabla_en(double Rp, bool& en_caja) const
    {
        const double u = Rp / (tS_ + std::fabs(Rp));      // ONE division, no transcendentals
        double t = (u + 1.0) * 0.5 * double(kTablaN);
        int b = int(t); if (b < 0) b = 0; if (b >= kTablaN) b = kTablaN - 1;
        // THE TABLE'S BOX, AND WHY IT EXISTS.
        //
        // `avanza_tabla` **FABRICATES** `w = ±1` at bins 0 and N instead of
        // solving them (`if (au >= 1.0)`), so the first and last intervals carry
        // an invented value that moreover **no longer depends on `R'`**. Reading
        // them is reading a constant, and a constant cannot bring the engine
        // back.
        //
        // MEASURED, and that is why there is a box: after a square-wave abuse
        // the state pushes `R'` outside the warp and the table answers that
        // fixed value => the engine stays **MUTE FOREVER**. The count says it
        // without argument: at guitar level **0 of 192.000** samples read that
        // stretch, during the abuse 95.693, and over the 2 s of recovery
        // **192.000 of 192.000** — pinned, with no way back. A PINNED value is
        // not rounding, it is a stalled loop.
        // `make test` caught it when the table shipped, not a review: the defect
        // had been there since the fill landed and `build/abuso` had never been
        // run with it on.
        //
        // => Outside the box it SOLVES, which is what stage 4's cache already
        // does when its point falls outside. There is no new path: it is the
        // usual Newton. And it costs nothing where it is used: at guitar level
        // it NEVER fires.
        en_caja = (b > 0 && b < kTablaN - 1);
        const double f = t - double(b), g = 1.0 - f;
        return Tab{ tabla_[b] * g + tabla_[b + 1] * f,
                    tID_[b]   * g + tID_[b + 1]   * f,
                    tq1_[b]   * g + tq1_[b + 1]   * f,
                    tq2_[b]   * g + tq2_[b + 1]   * f };
    }
public:
#endif

    void hoist()
    {
        Y2_      = 1.0 / Rp_ + Gcp_;
        invY2_   = 1.0 / Y2_;
        kY2_     = 1.0 / (1.0 + gm_ / Y2_);
        invRf_   = 1.0 / Rf_;
#if NLSC_E2_CHARGE_E3
        invGsum_ = 1.0 / Gsum_;
        Yeq_     = G7_ * (1.0 - G7_ * invGsum_);
        D_       = 1.0 + kRO * Yeq_;
        invD_    = 1.0 / D_;
        kL_      = 1.0 / (D_ + gm_ / Y2_);
#endif
#if NLSC_E2_MAP_PROBE || NLSC_E2_TABLA
        construye_tabla();
#endif
    }
#endif

    // THE DC LOOP SOLVES WITH THE TABLE, AND THAT IS DELIBERATE — measured
    // after trying the opposite.
    //
    // An independent reviewer pointed out, correctly, that since the table
    // ships, the entry `hoist()` builds it with the DC coefficients and the 240
    // `process()` calls below READ it => the whole chain's rest point comes out
    // of an interpolation, and the exit `hoist()` throws that table away. It
    // looks like two defects: duplicated work on the audio thread, and a rest
    // point less exact than the Newton's.
    //
    // => **Forcing the Newton here was tried and it is WORSE, and `build/abuso`
    // says so**: with the fill suppressed during this loop, SIX abuses leave the
    // engine with output EXACTLY 0 and no recovery, in both variants. The reason
    // is that the table's bins are solved by CONTINUATION and to `1e-12`, while
    // the audio Newton starts from the previous state with the tolerance that
    // ships: for a DC solve from cold, the table is the better solver of the
    // two.
    // => What looked like wasted work is not: that table IS the DC solver. The
    // only real cost is ONE extra build per `reinit`, which with the wait in
    // place is a rare event (a variant change or `activate`).
    // Do not "fix" this without re-running `build/abuso`: the first attempt's
    // "0 failures" came from a STALE binary, because `build/abuso` did not
    // depend on this file.
    //
    // Rest: capacitors open and uncharged. Solved with the same Newton, which
    // then starts at the exact point instead of inside a transient.
    void rest(double v5)
    {
        v5_0_ = v5;
        cv4_ = 0.0; ci4_ = 0.0; cv3_ = v5 - vr_; ci3_ = 0.0;
        dq1_ = dq2_ = di1_ = di2_ = 0.0;
        v7_ = v5; va1_ = v5; va2_ = v5;
        v5_ = v5; v2_ = v5; cvcp_ = v5; icp_ = 0.0;

        const double G4_guardado = G4_, Gleg_guardado = Gleg_, g2h_guardado = g2h_;
        const double Gcp_guardado = Gcp_;
        G4_ = 0.0; Gleg_ = 0.0; g2h_ = 0.0; Gcp_ = 0.0;  // DC: no capacitors
#if NLSC_E2_HOIST
        hoist();   // `Gcp_` just changed: what was hoisted depends on it
#endif
        // With the REAL opamp `v5` is UNKNOWN, so C3's state — at rest
        // v5 − VR — depends on the solution itself. Iterate: solve, re-seed
        // C3, solve again. With the ideal opamp it converges first time,
        // because there v5 is given.
        for (int pasada = 0; pasada < 4; ++pasada) {
            for (int i = 0; i < 60; ++i) process(v5);
            cv3_ = v5_dc() - vr_;
        }
        G4_ = G4_guardado; Gleg_ = Gleg_guardado; g2h_ = g2h_guardado;
        Gcp_ = Gcp_guardado;
#if NLSC_E2_HOIST
        hoist();
#endif

        // Trapezoidal history consistent with d/dt = 0 at rest.
        cv4_ = v5_dc() - v7_;  ci4_ = 0.0;
        cv3_ = v5_dc() - vr_;  ci3_ = 0.0;
        cvcp_ = v2_;           icp_ = 0.0;
        // A bug lived here that gave NUMBERS, not errors: the charge
        // history was seeded with `true` even with the charge off, so the
        // next step subtracted a charge no longer computed. The null read
        // -7,6 dB instead of -38,0 and the junction charge looked
        // indispensable. The SCALAR path — which also turns it off and
        // gave -38,0 — betrayed it: two variants removing the same thing
        // cannot disagree by 30 dB.
        dq1_ = kCharge ? mna::Engine::diode(d_, jd_, va1_ - v7_, kCharge).q : 0.0;  di1_ = 0.0;
        dq2_ = kCharge ? mna::Engine::diode(d_, jd_, va2_ - v5_dc(), kCharge).q : 0.0;  di2_ = 0.0;
#if NLSC_E2_PREDICTOR >= 1
        // The DC loop above has called `process()` 240 times: without
        // this, the first REAL sample extrapolates from the settling.
        wp1_ = wp2_ = 0.0; n_pred_ = 0;
#endif
    }

    // DC `n5`: with the REAL opamp a solved unknown; with the ideal one
    // the input itself.
    double v5_dc() const
    {
#if defined(NLSC_E2_OPAMP_REAL) || defined(NLSC_E2_OPAMP_LIN)
        return v5_;
#else
        return v5_0_;
#endif
    }

    mna::DModel d_{};
    mna::Engine::Junc jd_{};
    double h_ = 0, g2h_ = 0, vr_ = 0, Rf_ = 0, G4_ = 0, Gleg_ = 0;
    double nvtD_ = 0, vcritD_ = 0, v5_0_ = 0;
    double v7_ = 0, va1_ = 0, va2_ = 0;
    double cv4_ = 0, ci4_ = 0, cv3_ = 0, ci3_ = 0;
    double dq1_ = 0, dq2_ = 0, di1_ = 0, di2_ = 0;
    // real-opamp path only
    double v5_ = 0, v2_ = 0, cvcp_ = 0, icp_ = 0;
    double n11_ = 0;
#if NLSC_E2_HOIST
    // Behind the flag: an unused member still changes the object, and
    // an "off" that moves the binary is no control.
    double Y2_ = 0.0, invY2_ = 0.0, kY2_ = 0.0, invRf_ = 0.0, invGsum_ = 0.0;
    double invD_ = 0.0;
#endif
    long   slew_ = 0;
    long   sat_  = 0;
#if NLSC_E2_MAP_PROBE
public:
    // Only under `NLSC_E2_MAP_PROBE`: it does not exist in what ships.
    struct MapaPunto { long muestra; double w, id, F; };
    static constexpr int kMapaTope = 4000000;
    const MapaPunto* mapa() const { return mapa_; }
    int mapa_n() const { return mapa_n_; }
    // RESETS ALL THREE. Resetting only `mapa_` leaves `wfin_` and `kfin_`
    // holding what `prepare()` put there while solving the rest point, and then
    // sample `s` of one is NOT sample `s` of the other: an INDEX OFFSET, which
    // raises no error and returns a cloud of points that looks like "there is
    // no function here".
    void mapa_reset() { mapa_n_ = 0; wfin_n_ = 0; kfin_n_ = 0; }
    // The last sample's converged `w`, which is what the table has to give.
    double w_convergida() const { return v5_ - v7_; }
private:
    MapaPunto* mapa_ = new MapaPunto[kMapaTope];
    int mapa_n_ = 0;
    double* wfin_ = new double[kMapaTope];
    int wfin_n_ = 0;
    double* kfin_ = new double[kMapaTope];
    int kfin_n_ = 0;
public:
    const double* wfin() const { return wfin_; }
    int wfin_n() const { return wfin_n_; }
    const double* kfin() const { return kfin_; }
    int kfin_n() const { return kfin_n_; }
private:
#endif
    static constexpr double kRO = 75.0;
    // Stage 3's load as seen from `n7` (see kR7/kC5/kR8).
    double G7_ = 0, G8_ = 0, G5_ = 0, Gsum_ = 0, cv5c_ = 0, ci5c_ = 0;
#if NLSC_E2_N9_ORACLE
    double n9_orac_ = 0.0;
    bool   use_n9_  = false;
#endif
    double Yeq_ = 0, Ieq_ = 0, D_ = 1.0, kL_ = 0;
    double gm_ = 0, inv_imax_ = 0, imax_ = 0, Rp_ = 0, Gcp_ = 0;
};

} // namespace nlsc
