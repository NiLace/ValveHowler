// The CASCADE of the four stages: the plugin's audio engine.
//
// The DK engine, which solves the ten nonlinear ports together, is a
// reference model and is not part of the plugin.
//
// THE DECOMPOSITION, AND WHY SPLITTING HERE IS LEGITIMATE
// -------------------------------------------------------
//     in --[1: Q1 buffer]--> n4 --[2: gain+diodes]--> n7
//        --[3: tone]--> n14 --[4: LEVEL + Q2 buffer]--> out
//
// The boundaries are not arbitrary: `n7` and `n14` are opamp outputs, and
// with an ideal opamp an opamp output is a voltage source. Whatever the next
// stage draws does not change its value, so the split approximates nothing
// beyond the opamp itself.
//
// WHAT THIS ARCHITECTURE DOES NOT REPRESENT
// -----------------------------------------
// 1. The backward coupling between stages (0,1 % from Q1 and 1,1 % from
//    opamp B).
// 2. The real opamp: finite gain, GBW pole, slew and clipping.
//
// INTER-STAGE INTERFACE: DEVIATION, NOT ABSOLUTE VOLTAGE
// ------------------------------------------------------
// Each stage delivers the deviation from its output node's rest, and the
// next adds its input node's rest. Carrying absolute voltages would add the
// DC difference between the rest points each stage was fitted at, which is
// bookkeeping error, not architecture. What does survive, as it must, is the
// rest displacement the signal itself produces.

#pragma once
#ifndef NLSC_WS_RECENTER
#define NLSC_WS_RECENTER 0
#endif
// STAGE 1 FROM THE CIRCUIT (`nls_stage1_phys.h`) instead of the fitted
// waveshaper. The fitted stage cannot follow Q1 into cut-off, which starts at
// ~1,8 V of input peak; see that header.
#ifndef NLSC_E1_PHYS
#define NLSC_E1_PHYS 1
#endif

#include <cmath>
#include <cstdio>
#include <cstdlib>

// RATE SELECTOR for the cascade's coefficient sets.
//
// The coefficients are discretised at one concrete rate; this picks the set.
// With nothing defined the default set is used. The filters the plugin runs
// are rebuilt at the real rate in `prepare()`.
//   -DNLSC_CASC_KHZ=192  ->  4x      -DNLSC_CASC_KHZ=96  ->  2x
#if defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 96
#  include "nls_e234_coef_96k.h"
#elif defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 192
#  include "nls_e234_coef_192k.h"
#else
#  include "nls_e234_coef.h"
#endif
// `rn3_`/`rn14_`/`rn19_` (what the rail injects into the stages, the return
// path of the same loop) are prepared from the bank's s-domain fits
// (`J::N3_B`, `tone::`, `level::`) and discretised at the real rate in
// `prepare()`. No symbol from this header's `rail::` namespace is read, so
// this selection only picks an unused initialiser.
#if defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 96
#  include "nls_rail_n3_96k.h"
#elif defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 192
#  include "nls_rail_n3_192k.h"
#else
#  include "nls_rail_n3.h"
#endif
// Which variant uses the subsystem. Compile-time switches, so every
// combination can be built; the defaults are the plugin's configuration.
#ifndef NLSC_E4_OPEN_BASE
#  define NLSC_E4_OPEN_BASE 1
#endif
// `NLSC_E4_SUB_808`: on the 808, stage 4 solves Q2's subsystem with a scalar
// Newton instead of the tabulated waveshaper, so Q2's base is driven by the
// current the subsystem computes rather than by a fitted impedance.
// `=0` returns to the tabulated waveshaper.
#ifndef NLSC_E4_SUB_808
#  define NLSC_E4_SUB_808 1
#endif
#ifndef NLSC_E4_SUB_V9RI
#  define NLSC_E4_SUB_V9RI 1
#endif
// THE `ib` LOOP, CLOSED INSTEAD OF DELAYED. See `SubQ2::process_loop`
// and `OpenBaseLevel::parte_sin_ib`. At 0 it falls back to a one-sample
// delay.
#ifndef NLSC_E4_CLOSED_LOOP
#  define NLSC_E4_CLOSED_LOOP 1
#endif
// STAGE 4's SOLUTION CACHE. A table replaces the Newton solve inside a box
// around the rest point. Outside the box, and while the table is still
// filling, the sample is solved instead of clamped, so the table accelerates
// the solve and never degrades it. When the plugin core requests it
// (`defer_fill(true)`), the table is filled incrementally on the audio thread
// (see `advance_cache()`); otherwise `prepare()` builds it at once. Memory: ~122 kB per instance at the default grid.
//
//   0 = Newton on every sample
//   1 = DESIGN B — table of the OPEN system `(n19, ieq) -> (n17, ib)`, with
//       the affine loop `n19 = a0 + kz·ib` closed outside by fixed point.
//       Knob-independent: one table serves the whole travel.
//   2 = DESIGN A — table of the CLOSED system `(a0, ieq) -> (n17, ib)`, one
//       access. Depends on `kz`, i.e. on LEVEL, so it rebuilds on every
//       `prepare()`.
// The rail is predicted within the sample; `=0` uses a one-sample delay
// instead.
#ifndef NLSC_CASC_RAIL_PREDICTED
#  define NLSC_CASC_RAIL_PREDICTED 1
#endif
#ifndef NLSC_E4_CACHE
#  define NLSC_E4_CACHE 1
#endif
// The cache grid, `n19` x `ieq` nodes. The `ieq` travel spans only ~1,5
// nodes of the box, so the resolution goes to `n19`: 1900 x 8 over ±2,70 V
// keeps every sample inside the box for inputs up to 2,0 V, in 121,6 kB of
// `float` with two outputs.
// The second axis is kept short because `kCN2` sets the byte distance between
// the two rows a bilinear read touches, `kCN2 * 2 * 4`: at 8 that is 64 B,
// the adjacent cache line, which more than halves the L1 load misses of a
// 32-node axis (four lines apart).
#ifndef NLSC_E4_CACHE_N1
#  define NLSC_E4_CACHE_N1 1900
#endif
#ifndef NLSC_E4_CACHE_N2
#  define NLSC_E4_CACHE_N2 8
#endif
// Iterations of design B's affine loop. The loop gain `kz·dib/dvbe` reaches
// ~0,09 at the worst `lvl` (kz = −24 080 Ω at mid-travel, 24x the `lvl` 0
// value), so two passes leave the term at ~1e-4 of its 2,4 mV. One pass
// leaves ~9 % of it: on the full knob grid against ngspice (36 cells, 4x
// rate, the 808 variant) 25 of 36 cells rise above −60 dB (worst −55,5). A
// per-axis sweep does not show it; the worst case is in the knob interaction.
#ifndef NLSC_E4_IEQ_CEILING
#  define NLSC_E4_IEQ_CEILING 0
#endif
#ifndef NLSC_E4_CACHE_IT
#  define NLSC_E4_CACHE_IT 2
#endif
// The BOX, in volts AROUND THE REST, so it follows the knobs and the rate
// on its own. Outside the box the sample falls through to the Newton and is
// counted (`out_of_box()`), so an edge never degrades silently.
// Store the DEVIATION from the follower's line instead of absolute `n17`:
// the stored values are millivolts, which keeps `float` quantisation (a floor
// near −82 dB with absolute `n17`) below the interpolation error.
#ifndef NLSC_E4_CACHE_LINE
#  define NLSC_E4_CACHE_LINE 1
#endif
// THE SLICE (per-block fill, `NLSC_E4_FILL_PER_SAMPLE=0`): how many grid
// nodes are solved per block while the table fills. It bounds the per-block
// cost of a rebuild; meanwhile the Newton handles the samples. 512 nodes per
// block fills the default 1900 x 8 grid (15 200 nodes) in ~30 blocks; blocks
// of 32 samples or fewer would need this retuned.
#ifndef NLSC_E4_CACHE_NODES_PER_BLOCK
#  define NLSC_E4_CACHE_NODES_PER_BLOCK 512
#endif

// The cache fill advances PER SAMPLE (`NLSC_E4_CACHE_NODES_PER_SAMPLE`
// nodes each), so the output does not depend on how the host slices blocks:
// block-size invariance is bit-exact. `=0` fills per block instead.
#ifndef NLSC_E4_FILL_PER_SAMPLE
#  define NLSC_E4_FILL_PER_SAMPLE 1
#endif
#ifndef NLSC_E4_CACHE_NODES_PER_SAMPLE
#  define NLSC_E4_CACHE_NODES_PER_SAMPLE 2
#endif

// Half-span of the box on the `n19` axis, in volts. It covers inputs up to
// 2,0 V. A box looser than the real travel does not fail, it silently spends
// resolution, so the span follows the travel that is actually reached.
#ifndef NLSC_E4_CACHE_DN19
#  define NLSC_E4_CACHE_DN19 2.70
#endif
// THE `vc` AXIS IS OFF-CENTRE ON PURPOSE. Under signal the rest point
// displaces, and `vc` is C9's DC, so the real travel sits above the static
// rest: about `q_n17 + [-0,002, +0,066]` V.
#ifndef NLSC_E4_CACHE_VC_LO
#  define NLSC_E4_CACHE_VC_LO (-0.030)
#endif
// The upper edge covers the 808 at 2,0 V of input with the 8-node axis.
#ifndef NLSC_E4_CACHE_VC_HI
#  define NLSC_E4_CACHE_VC_HI (+0.240)
#endif
// `E4GS` IS NOT APPLIED ON THE SUBSYSTEM PATH.
//
// `E4GS` is the `n17/n19` shape normalised at 8 kHz, the residual load step
// the TABULATED waveshaper needs because `poly()` is a memoryless curve and
// does not carry it. The subsystem instead solves `n17` with the real
// network (R13, R14, C9, R15) and its state in `C9`, so it already carries
// the step; applying `E4GS` again counts it twice and floors the output null
// near −64,5 dB. At 1 it is applied again.
#ifndef NLSC_E4_GS_IN_SUB
#  define NLSC_E4_GS_IN_SUB 0
#endif
// STAGE 4's NEWTON ITERATION CAP. Truncation is an implementation error and
// must stay below the model's error. Worst case over five knob positions
// with hard material (2 V segments), against the converged solution:
//
//   cap 1 -> −63,6 dB   cap 2 -> −83,3 dB   cap 3 -> −115,7   cap 4 -> −155,3
//
// Under normal signal the cap does not bite (the Newton exits on tolerance
// in 2-3 passes; `newton_escapes()` counts the exceptions), so 4 costs
// nothing over 2.
//
// SOFTWARE-PIPELINING PROBE (`NLSC_CASC_SEGMENTED`), off in the plugin. It
// runs stage 1 on sample `i`, stage 2 on `i-1`, stage 3 on `i-2` and stage 4
// on `i-3`, breaking the inter-sample dependency chain, to measure what that
// would buy in speed and cost in fidelity. Its output is incorrect by
// construction: the rail assembles from all four stages' outputs of the same
// sample (`s.n4`, `s.n7`, `s.n14`, `dev_n19_`, `dev_n3_`, `n11`) and feeds all
// four of the next, so pipelined the rail mixes samples. The chain is not
// just the signal, it is the loop.
#ifndef NLSC_CASC_SEGMENTED
#  define NLSC_CASC_SEGMENTED 0
#endif
#ifndef NLSC_E4_ITER
#  define NLSC_E4_ITER 4
#endif
#ifndef NLSC_E4_TOL
#  define NLSC_E4_TOL 1e-9
#endif
#include "nls_variantes.h"
#include "nls_mna.h"      // the DK engine's `bjt()`, for stage 4's subsystem
#include "nls_fijos_s.h"
#include "nls_stage1.h"
#include "nls_stage1_phys.h"
#include "nls_reposo_drive.h"
// THE SECOND BANK. Included ALWAYS, not under `#if`: the variant is
// chosen LIVE, so both banks of the current rate must be compiled at once.
// (`nls_stage1.h` already brings its own plus `fixed`/`reposo`.)
#if defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 96
#  include "nls_e234_coef_v9ri_96k.h"
#elif defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 192
#  include "nls_e234_coef_v9ri_192k.h"
#else
#  include "nls_e234_coef_v9ri_384k.h"
#endif
#include "nls_stage2.h"
#include "nls_tono.h"
#include "nls_nivel.h"
// `nls_juegos.h` goes LAST: it enumerates symbols from every namespace
// above, and each bank DECLARES which stage-4 subsystem it uses.
// `VRA` PARAMETRISED BY TONE: off by default.
// See `nls_tono.h::RailVraVariable` and the generated header.
#ifndef NLSC_VRA_PARAM
#define NLSC_VRA_PARAM 0
#endif

namespace nlsc {
inline constexpr bool kOpenBase = (NLSC_E4_OPEN_BASE != 0);
inline constexpr bool kSubIn808  = (NLSC_E4_SUB_808  != 0);
inline constexpr bool kSubInV9ri = (NLSC_E4_SUB_V9RI != 0);
inline constexpr bool kClosedLoop = (NLSC_E4_CLOSED_LOOP != 0);
inline constexpr bool kGsInSub    = (NLSC_E4_GS_IN_SUB != 0);
}
#include "nls_juegos.h"

namespace nlsc {

// The "a variant without a bank does not compile" guard lives in
// `nls_juegos.h`.



// Stage 3 — the tone control. Linear (its distortion sits at −156 dBc), so
// it is a filter and nothing more.
//
// The knob is a parameter: the coefficients come from `nls_tono.h`, derived
// by exact algebra over the netlist with the NJM4558 inside. Order 4 in
// direct form.
class Stage3 {
public:
    void prepare(double fs, double tone, bool reinit = true)
    { h_.prepare(fs, tone, reinit); }
    void reset() { h_.reset(); }
    double process(double dev_n7) { return h_.process(dev_n7); }
private:
    VariableTone h_;
};

// Stage 4 — the LEVEL pot and the Q2 output buffer. Wiener-Hammerstein,
// like stage 1 and for the same reason: Q2's distortion is flat in frequency
// (0,1 dB over two and a half decades), so it is memoryless.
class Stage4 {
public:
    Stage4()
    {
        for (int i = 0; i < e234::kWS2_N; ++i) ws2_[i] = e234::kWS2[i];
#if NLSC_WS_RECENTER
        ws2_[e234::kWS2_N - 1] = 0.0;   // see NLSC_WS_RECENTER in nls_stage1.h
#endif
        reset();
    }

    // `g1_` (the LEVEL pot's network) is parametric in `lvl`. The rest point
    // needs no parametrisation: `lvl` does not move it, because C8 blocks DC
    // between the pot and Q2's base. See `nls_nivel.h`.
    // THE VARIANT. The physical difference between the 808 and the 9/9RI lives
    // here: `R14` (series) and `R15` (the output load, after the coupling cap).
    // Both share Q2's emitter resistor (10 k) and therefore its quiescent point;
    // what moves is the load on `n17` and the output divider `E4G2`, 0,10 dB, a
    // very subtle difference.
    // `kWS2` is read per sample, so it is copied; the filters are read only in
    // `prepare()`, so choosing the table suffices.
    static_assert(fixed::kE4GS_NB == fixed_v9ri::kE4GS_NB &&
                  fixed::kE4GS_NA == fixed_v9ri::kE4GS_NA &&
                  fixed::kE4G2_NB == fixed_v9ri::kE4G2_NB &&
                  fixed::kE4G2_NA == fixed_v9ri::kE4G2_NA,
                  "both stage-4 banks must share the same filter orders");
    static_assert(e234::kWS2_N <= kWSMax && e234_v9ri::kWS2_N <= kWSMax,
                  "kWSMax is too small for Q2's waveshaper");

    // ═══════════════════════════════════════════════════════════════════════
    // STAGE 4 AS A SOLVED SUBSYSTEM
    // ═══════════════════════════════════════════════════════════════════════
    //
    // The TABULATED waveshaper is a static function of `n19`, and for the
    // 9/9RI variant that is not enough: against the reference `n17` under real
    // signal, the polynomial reaches −31,1 dB and this subsystem −70,0 dB. The
    // static curve extracted with a sine sweep and the one under the whole
    // chain's signal differ by up to 23 mV in the cutoff region: a model is
    // judged in its own regime, and this stage's regime is the whole chain.
    //
    // The circuit, and why ONE unknown suffices: Q2 is a follower with the
    // collector at 9 V, the emitter at `n17`, `R13` to ground and
    // `R14+C9+R15` towards the output. That branch, seen from `n17`, is
    // (R14+R15) in series with C9, so one unknown (`n17`) and one state (C9's
    // voltage).
    //
    //   Ie(vbe) = n17/R13 + (n17 − vc)·geq
    //
    // The transistor model is the SAME `bjt()` as the DK engine, not a copy, so
    // the two engines cannot drift apart.
    struct SubQ2 {
        // From the netlist: `R13` (Q2's emitter)
        // and C9 do not depend on the variant; `R14` (series) and `R15` (the
        // output load) DO, and they are the axis. `kR13` is the only fixed
        // resistor here; the two variant ones arrive through `prepare()`.
        static constexpr double kR13 = 10.0e3;
        static constexpr double kC9  = 10.0e-6;
        static constexpr double kVCC = 9.0;
        // The follower's gain, for the predictor: 0,94 on the 9/9RI and 0,98 on
        // the 808. An intermediate value serves both, since it only decides where
        // the Newton starts.
        static constexpr double kGainSeg = 0.96;
        // The Newton's iteration budget (see `NLSC_E4_ITER`).
        static constexpr int    kMaxIter   = NLSC_E4_ITER;
        // Observability: how often the budget ran out, of how many samples.
        long unconverged_ = 0;
        long samples_ = 0;
        static constexpr double kTolNewton = NLSC_E4_TOL;

        // Q2's JUNCTION CHARGES — `NLSC_E4_CHARGES`.
        //
        // `n19`'s error is `Z(s;lvl)·delta_ib`, where `delta_ib` is the displacement
        // current of Q2's junction capacitances: in quadrature at every harmonic and
        // growing with frequency. It cannot enter the static solve: the cache is a
        // table `(n19, ieq) -> (n17, ib)`, valid only because Q2 without charges is
        // memoryless, and giving Q2 its own state would add two history inputs to
        // the table. So `ib` is corrected OUTSIDE the solve: the static part comes
        // from the Newton (or the table) and the displacement current adds on top
        // through a companion model (see `advance_charges()`).
        // Its feedback on the emitter node is second order (~38 ppm of the ~37 uA
        // `R13` draws); in `ib` it is first order, because `ib` leaves the block
        // multiplied by `Z(s;lvl)`.
        //   0 = no charges     · 1 = depletion + diffusion
        //   2 = depletion only (default; the diffusion charge adds ~0,1 dB for ~15 %
        //       more CPU)
        // The defaults of `NLSC_E4_RB` and `NLSC_E4_CHARGES` sit here, before their
        // first use in `idisp_de()`: an undefined macro evaluates to 0 in `#if`
        // without any warning.
        //
        // `NLSC_E4_RB` — Q2's BASE RESISTANCE. `.model QBUF` carries RB = 50 ohm.
        // Evaluating the junction at the base terminal misses its drop (34 uV at
        // the rest `ib` = 6,803e-07 A, 4,70e-07 A of KCL imbalance at `n17` through
        // gm = 13,8 mS), and the solver would not land on the rest point from
        // ngspice's `.op`. The DK engine's MNA builds the same resistance with an
        // internal base node.
        // It is free: the level network imposes `n19 = a0 + kz*ib` and the internal
        // base is `u = n19 - RB*ib`, so `u = a0 + (kz - RB)*ib`, the SAME form. The
        // 2x2 Newton solves in `u` with `kz' = kz - RB`, the junction is evaluated
        // the same way, and `n19` is recovered at the end with the original `kz`.
        // Same unknowns, same cache table (indexed by the voltage the junction
        // sees) and one FMA per sample. At 0, `rb_` is 0 and every formula below
        // reduces to the terminal-base form.
#ifndef NLSC_E4_RB
#define NLSC_E4_RB 1
#endif
#ifndef NLSC_E4_CHARGES
#define NLSC_E4_CHARGES 2
#endif
        // `idisp_de()` gives the displacement current at given voltages without
        // touching the state, so it can be called inside the Newton as often as
        // needed; with `NLSC_E4_CHARGES == 0` it is zero and the compiler erases it.
        // Its derivatives must enter the Jacobian, or the Newton stops on step size
        // short of convergence. In mode 2 they are constant (the effective
        // depletion) and cost nothing. In mode 1 the diffusion charge's derivative
        // is neglected, so the Jacobian is approximate and Newton converges somewhat
        // slower.
        double didisp_dv() const
        {
#if NLSC_E4_CHARGES
            return gh_ * (jbe_.cj0 + jbc_.cj0);
#else
            return 0.0;
#endif
        }
        double didisp_dx() const
        {
#if NLSC_E4_CHARGES
            return -gh_ * jbe_.cj0;
#else
            return 0.0;
#endif
        }
        double idisp_de(double vbe, double vbc) const
        {
#if NLSC_E4_CHARGES == 2
            return gh_ * ((jbe_.cj0 * vbe - qbe_ant_) + (jbc_.cj0 * vbc - qbc_ant_));
#elif NLSC_E4_CHARGES
            const mna::Engine::QOut o =
                mna::Engine::bjt(qm_, jbe_, jbc_, vbe, vbc, true);
            return gh_ * ((o.qbe - qbe_ant_) + (o.qbc - qbc_ant_));
#else
            (void)vbe; (void)vbc; return 0.0;
#endif
        }
        static constexpr bool kCharges = (NLSC_E4_CHARGES != 0);
        // Only mode 1 asks `bjt()` for the charges; 2 computes them apart.
        static constexpr bool kExactCharges = (NLSC_E4_CHARGES == 1);
#if NLSC_E4_CHARGES
        double gh_ = 0.0;                       // 1/h (backward Euler)
        double qbe_ant_ = 0.0, qbc_ant_ = 0.0;
        double idisp_ = 0.0;

        // Advances the companion with charges already computed. One copy of the
        // formula: both paths (Newton and cache) call here.
        // BACKWARD EULER, NOT TRAPEZOIDAL. The trapezoidal companion
        // `i_n = (2/h)(q_n - q_{n-1}) - i_{n-1}` has the homogeneous solution
        // `(-1)^n`, an undamped Nyquist mode. Inside an MNA the companion
        // conductance `(2/h)·c` enters the Jacobian and damps it; here the term is
        // computed apart and feeds back through `kz` with one sample of delay, so
        // nothing damps it and it diverges.
        // `i_n = (q_n - q_{n-1})/h` is first order, but applied to a term worth
        // ~1,6 % of `ib`: the rule's error is ~(omega·h/2) of that 1,6 %, i.e.
        // 1,3e-4 of the total at 1 kHz and 384 kHz. It also has no current state of
        // its own, so there is no mode that could ring.
        void advance_charges(double qbe, double qbc)
        {
            idisp_ = gh_ * ((qbe - qbe_ant_) + (qbc - qbc_ant_));
            qbe_ant_ = qbe; qbc_ant_ = qbc;
        }
        // The CACHE path does not evaluate the transistor, so the charges are paid
        // here: one evaluation per sample, against the 2-3 iterations the table
        // saves.
        // Mode 2 (depletion only, `q = cj0_eff·v`: two multiplies, no exponential)
        // avoids a full transistor evaluation. The ruling term is `Cjc`, which sees
        // `vbc = n19 − 9 V`, i.e. `n19`'s whole travel; `Cbe` sees `vbe = n19 − n17`,
        // and the follower cancels ~96 % of that travel, so it contributes ~25x
        // less.
        void advance_charges_from(double vbe, double vbc)
        {
#if NLSC_E4_CHARGES == 2
            advance_charges(jbe_.cj0 * vbe, jbc_.cj0 * vbc);
#else
            const mna::Engine::QOut o =
                mna::Engine::bjt(qm_, jbe_, jbc_, vbe, vbc, true);
            advance_charges(o.qbe, o.qbc);
#endif
        }
        void seed_charges(double n19_q, double n17_q)
        {
#if NLSC_E4_CHARGES == 2
            qbe_ant_ = jbe_.cj0 * (n19_q - n17_q);
            qbc_ant_ = jbc_.cj0 * (n19_q - kVCC);
#else
            const mna::Engine::QOut o = mna::Engine::bjt(
                qm_, jbe_, jbc_, n19_q - n17_q, n19_q - kVCC, true);
            qbe_ant_ = o.qbe; qbc_ant_ = o.qbc;
#endif
            idisp_ = 0.0;                          // no displacement at rest
        }
#endif

        // `reinit`: `prepare()` is also called on a knob move. Pots are netlist
        // resistors, so a knob changes the network, not the state; dropping the
        // subsystem's state there would click.
        void prepare(double fs, double r15, double r14, double n17_rest,
                     double n19_rest, double kz = 0.0, bool reinit = true)
        {
            n19_rest_ = n19_rest;
            // Two paths load `n17`:
            //   · `g13_` — the DC path from `n17` to ground, i.e. R13 (10 k in both
            //              variants).
            //   · `rs`   — the LOAD the output branch puts on `n17`: R14 + R15 in
            //              series with C9. R15, the output load, is the variant axis.
            // The transfer `n17 -> out` is not here: `E4G2`, a generated coefficient,
            // carries `R15/(R14+R15)`.
            g13_ = 1.0 / kR13;
            const double rs = r14 + r15;
            const double gc = 2.0 * kC9 * fs;         // trapezoidal
            geq_ = 1.0 / (rs + 1.0 / gc);
            inv_gc_ = 1.0 / gc;
            k_vc_ = 1.0 / (2.0 * kC9 * fs);
#if NLSC_E4_CHARGES
            gh_ = fs;
#endif
            qm_.derive();
            // RB AND THE INTERNAL BASE AT REST go HERE, before `jbe_`/`jbc_`, because
            // the depletion capacitance is evaluated at the junction voltage, not at the
            // terminal's. `ib_rest()` solves the fixed point, and with `rb_ = 0` it
            // returns the terminal-base value on the first pass.
            // `bjt(..., false)` does not touch `jbe_`/`jbc_`, so calling it before they
            // are initialised is correct, and that is what makes this order possible.
            rb_ = (NLSC_E4_RB) ? qm_.RB : 0.0;
            u_rest_ = n19_rest - rb_ * ib_rest(n19_rest, n17_rest);
#if NLSC_E4_CHARGES
            // `jbe_`/`jbc_` must be initialised here: `qjunc` would otherwise run on an
            // uninitialised junction and produce NaN. They fill with the engine's same
            // `init`, from the same model table.
            // CONSTANT DEPLETION NEEDS THE RIGHT CONSTANT. `qjunc` uses `c = cj0`,
            // justified for a clipper's travel (`u^(-m)` moves only ±20 % around 1 for
            // VJ = 1 V, M = 0,4, v ∈ −0,7…0,7). Q2's base-collector junction does not
            // live there: it rests at −4,73 V, where `cj0` is 1,85x the real capacitance
            // (3,638 vs 1,970 pF), which overshoots the correction by ~1,6x.
            // So the exact capacitance is evaluated ONCE, in `prepare()`, at the rest
            // point, and used as the constant. Over the signal's travel that capacitance
            // moves ±0,7 % (1,955-1,984 pF). Hot-path cost: zero; one `pow` in
            // `prepare()`.
            // The formula lives in `nls_mna.h`, next to `qjunc`, the other writing of
            // the same law; they cannot be unified (one runs per sample with range
            // reduction, the other once with `pow`).
            // `NLSC_CJC_SCALE` — sensitivity probe, 1,0 by default. The service manual
            // specifies a 2SC1815BL and the model card is a 2N3904: at VCB = 10 V
            // Toshiba publishes Cob = 2,0-3,5 pF and the card gives 1,60. This scales
            // both junction capacitances to measure the effect on the audio.
#ifndef NLSC_CJC_SCALE
#define NLSC_CJC_SCALE 1.0
#endif
            jbe_.init(mna::Engine::depletion_cap(u_rest_ - n17_rest, qm_.CJE, qm_.VJE,
                                 qm_.MJE, qm_.FC) * double(NLSC_CJC_SCALE),
                      qm_.VJE, qm_.MJE, qm_.FC);
            jbc_.init(mna::Engine::depletion_cap(u_rest_ - kVCC, qm_.CJC, qm_.VJC,
                                 qm_.MJC, qm_.FC) * double(NLSC_CJC_SCALE),
                      qm_.VJC, qm_.MJC, qm_.FC);
            // On a knob move (no reinit) `cj0` changes here while `qbe_ant_`/`qbc_ant_`
            // keep the charge computed with the old one, so the next `advance_charges`
            // emits a one-sample displacement pulse, estimated below 1 µV. Accepted as a
            // negligible transient.
#endif
            if (reinit) reset(n17_rest);
#if NLSC_E4_CACHE
            // A REINIT STARTS THE CACHE OVER, like a fresh instance. Keeping a cache
            // built with the same parameters is right for a knob move, and wrong for
            // `activate()`: a reset instance must be indistinguishable from a new one,
            // which solves by Newton until its deferred fill completes, and the two
            // differ at the solver's tolerance.
            if (reinit) { built_ = false; cache_pending_ = false; }
            // The cache is indexed by the voltage the JUNCTION sees, not the
            // terminal's: `solve_node()` evaluates at `p1 - x`. With RB that is `u`, so
            // the table is the same and only the centre of its box moves: 34 uV on an
            // axis of hundreds of mV.
            build_cache(u_rest_, n17_rest, kz - rb_);
#else
            (void)kz;
#endif
        }

#if NLSC_E4_CACHE
        // ─────────────────────────────────────────────────────────── THE CACHE
        static constexpr int kCN1 = NLSC_E4_CACHE_N1;
        static constexpr int kCN2 = NLSC_E4_CACHE_N2;
        static constexpr int kCIt = NLSC_E4_CACHE_IT;

        long out_of_box() const { return out_of_box_; }
        // Per axis too: clipping the `ieq` axis (which enters linearly) is not
        // the same as clipping `n19`'s. `excess_*` is in NODES, i.e. grid steps.
        long outside_p()   const { return out_p_; }
        long outside_ieq() const { return out_i_; }
        double excess_p()   const { return excess_p_; }
        double excess_ieq() const { return excess_i_; }
        double w_min() const { return w_min_; }   // in NODES: 0 .. kCN2-1 is inside
        double w_max() const { return w_max_; }



        // Solves the OPEN system to true convergence. Called only when
        // BUILDING: the hot path solves nothing.
        void solve_node(double n19, double ieq, double& x, double& ib) const
        {
            const double vbc = n19 - kVCC;
            for (int it = 0; it < 200; ++it) {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, n19 - x, vbc, false);
                const double f  = (o.ib + o.ic) - x * g13_ - (x * geq_ - ieq);
                const double df = -(o.dic_be + o.dib_be) - g13_ - geq_;
                double step = f / df;
                if (step >  0.3) step =  0.3;
                if (step < -0.3) step = -0.3;
                x -= step;
                if (step < 1e-15 && step > -1e-15) break;
            }
            ib = mna::Engine::bjt(qm_, jbe_, jbc_, n19 - x, vbc, false).ib;
        }

        // The CLOSED system (design A): both unknowns at once, with
        // `process_loop`'s same 2x2 but converged.
        void solve_node_loop(double a0, double kz, double ieq,
                                double& x, double& v, double& ib) const
        {
            for (int it = 0; it < 200; ++it) {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, v - x, v - kVCC, false);
                const double gm = o.dib_be + o.dic_be;
                const double gcb = o.dib_bc + o.dic_bc;
                const double f1 = (o.ib + o.ic) - x * g13_ - (x * geq_ - ieq);
                const double f2 = a0 + kz * o.ib - v;
                const double j11 = -gm - g13_ - geq_;
                const double j12 = gm + gcb;
                const double j21 = -kz * o.dib_be;
                const double j22 = kz * (o.dib_be + o.dib_bc) - 1.0;
                const double det = j11 * j22 - j12 * j21;
                const double inv = (det > -1e-300 && det < 1e-300) ? 0.0 : 1.0 / det;
                double px = (f1 * j22 - f2 * j12) * inv;
                double pv = (j11 * f2 - j21 * f1) * inv;
                if (px >  0.3) px =  0.3;
                if (px < -0.3) px = -0.3;
                if (pv >  0.3) pv =  0.3;
                if (pv < -0.3) pv = -0.3;
                x -= px; v -= pv;
                if (px < 1e-15 && px > -1e-15 && pv < 1e-15 && pv > -1e-15) break;
            }
            ib = mna::Engine::bjt(qm_, jbe_, jbc_, v - x, v - kVCC, false).ib;
        }

        void build_cache(double n19_rep, double n17_rep, double kz)
        {
            // Building the whole table takes several block periods (a 64-sample block
            // at 48 kHz is 1,333 ms), and `prepare()` is also called on a knob move.
            // Design B depends on no knob: its table changes only with `fs`, the
            // variant or the rest point (which moves 1,9 mV over the whole drive
            // travel, nothing against the box), so it compares against what was built
            // and returns. Design A depends on `kz` (the LEVEL) and would rebuild on
            // every knob move, which rules it out.
            // When a rebuild is needed, the table is not built here: the geometry is
            // fixed (scalar arithmetic) and the table left PENDING. Meanwhile
            // `in_box()` says no and the Newton runs, the same path out-of-box samples
            // take. The fill advances via `advance_cache()`.
#if NLSC_E4_CACHE == 2
            const bool same_kz = (kz == c_kz_done_);   // table A carries it inside
#else
            const bool same_kz = true;                  // table B ignores it
#endif
            const bool same_params =
                same_kz && geq_ == c_geq_done_ && g13_ == c_g13_done_ &&
                n19_rep > c_p0_done_ - 1e-2 && n19_rep < c_p0_done_ + 1e-2;
            // `cache_pending_` belongs in the guard: without it, a `prepare()` with
            // the same parameters mid-fill would re-arm the cursor at zero. With a
            // knob the host interpolates, the fill would never end and the Newton
            // would run forever at full cost, with no error raised.
            if ((built_ || cache_pending_) && same_params) {
                return;
            }
            built_ = true;
            c_geq_done_ = geq_; c_g13_done_ = g13_;
            c_kz_done_ = kz;    c_p0_done_ = n19_rep;
            // The box is anchored to the rest point, not to absolute constants: it
            // follows the knobs (which move the rest) and the rate (which moves `geq`)
            // on its own.
            c_lo1_ = n19_rep - NLSC_E4_CACHE_DN19;
            const double hi1 = n19_rep + NLSC_E4_CACHE_DN19;
            c_lo2_ = geq_ * (n17_rep + (NLSC_E4_CACHE_VC_LO));
            const double hi2 = geq_ * (n17_rep + (NLSC_E4_CACHE_VC_HI));
            c_h1_ = (hi1 - c_lo1_) / double(kCN1 - 1);
            c_h2_ = (hi2 - c_lo2_) / double(kCN2 - 1);
            c_inv1_ = 1.0 / c_h1_;
            c_inv2_ = 1.0 / c_h2_;
            out_of_box_ = 0;
            // WHAT IS STORED IS THE DEVIATION FROM A LINE, NOT `n17`.
            //
            // Storing `n17` in `float` sets a floor no grid can lower: `n17` is ~3,7 V
            // and `float`'s relative eps is 6e-8, i.e. 2,2e-7 V of fixed quantisation,
            // around −70 dB against the 9,1e-4 V rms output at `lvl = 0`. The
            // follower's line is subtracted before storing and added back on read: the
            // deviation is millivolts, so the same `float` gives about four more
            // orders of room.
#if NLSC_E4_CACHE_LINE
            c_x0_ = n17_rep;
            c_p0_ = n19_rep;
            c_g_  = kGainSeg;
#else
            c_x0_ = 0.0; c_p0_ = 0.0; c_g_ = 0.0;   // ABSOLUTE `n17` is stored
#endif
#if NLSC_E4_CACHE == 2
            // Design A builds all at once: its predictor reads `ib_`, which is live
            // state, so slicing would build each column from a different `ib_`.
            for (int k = 0; k < kCN2; ++k) {
                const double ieq = c_lo2_ + c_h2_ * double(k);
                double x = n17_rep;
                double v = c_lo1_;
                for (int j = 0; j < kCN1; ++j) {
                    const double p1 = c_lo1_ + c_h1_ * double(j);
                    double ib = 0.0;
                    v = p1 + kz * ib_;                 // loop predictor
                    solve_node_loop(p1, kz, ieq, x, v, ib);
                    const size_t c = (size_t(j) * size_t(kCN2) + size_t(k)) * 2;
                    tab_[c]     = float(x - (c_x0_ + c_g_ * (p1 - c_p0_)));
                    tab_[c + 1] = float(ib);
                }
            }
#else
            (void)kz;
            // Here only the cursor is armed. `built_` stays false until
            // `advance_cache()` finishes, and meanwhile `in_box()` sends every sample
            // to the Newton.
            built_    = false;
            cache_pending_    = true;
            cur_k_ = 0; cur_j_ = 0;
            cur_x_ = n17_rep;      // `tab_` stores deviations: no good as a start
            c_n17_rep_ = n17_rep;  // each new column's warm start

            // Deferral is requested by whoever knows it runs on the audio thread
            // (the plugin core, via `defer_fill()`). By default the table is built at
            // once, right here: a caller driving `Cascade4` directly has no block
            // boundary from which to call `advance_cache()`, and would otherwise run
            // the Newton forever while believing it runs the cache.
            // Draining the cursor here does exactly the same work, in the same order
            // and with the same warm start as a sliced fill, so the table comes out
            // bit-identical either way.
            if (!deferred_fill_) {
                while (!advance_cache(kCN1)) { }
            }
#endif
        }

        // THE SLICED FILL. Called a few nodes per sample from
        // `advance_fill_per_sample()` (default), or once per block from
        // `Plugin::process` when `NLSC_E4_FILL_PER_SAMPLE=0`; never as a whole.
        // Each call solves at most `nodes` grid points and remembers where it was.
        // The per-call cost is bounded and deterministic, with no threads and no
        // allocation: the table is a fixed-size member.
        //
        // The warm start survives because `cur_x_` travels between slices: node
        // `j-1`'s solution is node `j`'s starting point, as in an all-at-once loop.
        // Slicing is safe because `solve_node()` is `const` and reads no per-sample
        // state: only `qm_`, `jbe_`, `jbc_`, `g13_` and `geq_`, which `prepare()`
        // fixes. If a knob changes them mid-fill, `build_cache()` re-arms the
        // cursor from zero and the half-built table is discarded whole.
        //
        // Returns true if THIS call left the table finished.
        bool advance_cache(int nodes)
        {
#if NLSC_E4_CACHE == 1
            if (!cache_pending_) return false;
            while (nodes-- > 0) {
                const double ieq = c_lo2_ + c_h2_ * double(cur_k_);
                const double p1  = c_lo1_ + c_h1_ * double(cur_j_);
                double ib = 0.0;
                solve_node(p1, ieq, cur_x_, ib);
                const size_t c = (size_t(cur_j_) * size_t(kCN2) + size_t(cur_k_)) * 2;
                tab_[c]     = float(cur_x_ - (c_x0_ + c_g_ * (p1 - c_p0_)));
                tab_[c + 1] = float(ib);
                if (++cur_j_ >= kCN1) {          // column finished
                    cur_j_ = 0;
                    cur_x_ = c_n17_rep_;         // next one starts from rest
                    if (++cur_k_ >= kCN2) {      // table finished
                        cache_pending_ = false;
                        built_ = true;
                        return true;
                    }
                }
            }
            return false;
#else
            (void)nodes;
            return false;
#endif
        }

        // Requests the deferred (incremental) fill; see `build_cache()`.
        void defer_fill(bool v) { deferred_fill_ = v; }

        // THE FILL, ADVANCED PER SAMPLE — `NLSC_E4_FILL_PER_SAMPLE`.
        //
        // A per-block fill makes the number of nodes done at sample `n` depend on
        // how the host slices blocks, so the output during the fill would depend
        // on the block size. Advancing inside `process`, between samples, makes the
        // schedule a function of the sample index: one 1024-sample block and
        // sixteen of 64 fill exactly alike, because both process the same samples
        // in the same order. Total work is unchanged: 2 nodes per internal sample
        // is the same rate as 512 per 64-sample block at 4x.
        void advance_fill_per_sample()
        {
#if NLSC_E4_CACHE == 1 && NLSC_E4_FILL_PER_SAMPLE
            if (deferred_fill_ && cache_pending_)
                advance_cache(NLSC_E4_CACHE_NODES_PER_SAMPLE);
#endif
        }
        bool cache_pending() const { return cache_pending_; }
        long cache_nodes_left() const
        {
            if (!cache_pending_) return 0;
            return long(kCN2 - cur_k_ - 1) * long(kCN1) + long(kCN1 - cur_j_);
        }

        // IS THIS POINT INSIDE? Asked before reading; outside it falls to the
        // Newton instead of clipping. A table is an accelerator, not a substitute:
        // clamping to the edge node would return wrong values wherever the
        // stimulus exceeds the box. The per-sample cost is therefore not constant,
        // deliberately.
        bool in_box(double p1, double ieq) const
        {
            // A half-built table is not "inside" either: same answer as outside the
            // box, so slice filling needs no new path. It sits here and not at the read
            // site because this is the only choke point: `process_loop` asks up to
            // `kCIt` times per sample and all must see the same thing.
            if (!built_) return false;
            const double u = (p1 - c_lo1_) * c_inv1_;
            if (!(u >= 0.0 && u <= double(kCN1 - 1))) return false;
            const double w = (ieq - c_lo2_) * c_inv2_;
            return (w >= 0.0 && w <= double(kCN2 - 1));
        }

        // Accounting goes on the out-of-box path, once per sample. It cannot live
        // inside `in_box()`: `process_loop` calls that up to `kCIt` times per
        // sample, so it would count the fixed point's trials, not samples.
        void note_outside(double p1, double ieq)
        {
            (void)p1; (void)ieq;
        }

        // The `ieq` axis's travel is logged inside and outside the box: it is the
        // magnitude the box is sized with, and logging it only inside would leave it
        // saturated at the edge.
        void note_range(double ieq)
        {
            (void)ieq;
        }

        // Bilinear read: four accesses, two outputs, all in `float`.
        // Called only with `in_box()` true: the index clamp below is a
        // SAFETY NET against a NaN, not an operating mode.
        void read_cache(double p1, double ieq, double& x, double& ib)
        {
            double u = (p1 - c_lo1_) * c_inv1_;
            int j = int(u);
            if (j < 0)             { j = 0; }
            else if (j > kCN1 - 2) { j = kCN1 - 2; }
            const float fu = float(u - double(j));
#if NLSC_E4_IEQ_CEILING
            // CEILING PROBE, not a candidate: its output is wrong. Pins the cache's
            // second axis so the compiler removes its whole path (the axis
            // computation, the clamps, half the loads and four of the six lerps). It
            // measures the most the second-axis cost could be worth; it loses the
            // `ieq` dependence entirely, and with it the fidelity.
            (void)ieq; const int k = 0; const float fw = 0.0f;
#else
            double w = (ieq - c_lo2_) * c_inv2_;
            int k = int(w);
            if (k < 0)             { k = 0; }
            else if (k > kCN2 - 2) { k = kCN2 - 2; }
            const float fw = float(w - double(k));
#endif
            const size_t c = (size_t(j) * size_t(kCN2) + size_t(k)) * 2;
            const size_t d = c + size_t(kCN2) * 2;
            const float a0x = tab_[c],     a0i = tab_[c + 1];
            const float a1x = tab_[c + 2], a1i = tab_[c + 3];
            const float b0x = tab_[d],     b0i = tab_[d + 1];
            const float b1x = tab_[d + 2], b1i = tab_[d + 3];
            const float mx = (a0x + (a1x - a0x) * fw);
            const float nx = (b0x + (b1x - b0x) * fw);
            const float mi = (a0i + (a1i - a0i) * fw);
            const float ni = (b0i + (b1i - b0i) * fw);
            x  = c_x0_ + c_g_ * (p1 - c_p0_) + double(mx + (nx - mx) * fu);
            ib = double(mi + (ni - mi) * fu);
        }
#endif  // NLSC_E4_CACHE


#if NLSC_E4_CHARGES
        // The PHYSICAL base current: static + displacement. It is what
        // leaves the block towards `Z(s;lvl)`, where the null measures it.
        double ib() const { return ib_ + idisp_; }
        double ib_static() const { return ib_; }
        double idisp() const { return idisp_; }
#else
        double ib() const { return ib_; }
#endif

        // The base current AT REST, with the same `bjt()` as the loop, so the two
        // cannot drift apart.
        double ib_rest(double n19_q, double n17_q)
        {
            qm_.derive();
            // With RB the junction is not at the terminal but at `n19 - RB*ib`, so `ib`
            // is a fixed point. Its loop gain is `RB*dib/dvbe ~ 50 * gm/BF ~ 1,4e-3`, so
            // three steps are plenty; four are taken. With `rb_ = 0` the first pass is
            // already exact.
            double ib = 0.0;
            for (int k = 0; k < 4; ++k) {
                const double u = n19_q - rb_ * ib;
                ib = mna::Engine::bjt(qm_, jbe_, jbc_, u - n17_q,
                                      u - kVCC, false).ib;
            }
            return ib;
        }


        void reset(double n17_rest)
        {
            x_ = vc_ = n17_rest;                    // at rest nothing flows in C9
            i_prev_ = 0.0;
            // `n19_prev_` is the Newton's variable, and with RB that variable
            // is the INTERNAL base: it is seeded with `u_rest_`, not with the
            // terminal.
            n19_prev_ = u_rest_;
            // `ib_` is `process_loop`'s PREDICTOR, so starting it at zero
            // injects a `kz·ib0` jump on the first sample. It is seeded with
            // the rest point, and with the loop's SAME `bjt()`.
            ib_ = ib_rest(n19_rest_, n17_rest);
#if NLSC_E4_CHARGES
            // The charges belong to the JUNCTION, so they are seeded at the internal
            // base: seeding them at the terminal would leave `idesp != 0` at rest and
            // produce a step at t = 0.
            seed_charges(u_rest_, n17_rest);
#endif
        }

        double process(double n19_abs)
        {
            // Oracle-injection path (exact `ib` supplied), not the one the plugin runs.
            // With RB the junction is not at the terminal here either:
            // `u = n19 - RB*ib`, a fixed point of gain ~1,4e-3 seeded with the previous
            // sample's `ib_`, so one step converges with room to spare. With `rb_ = 0`
            // it is a plain assignment.
            n19_abs -= rb_ * ib();
#if NLSC_E4_CACHE == 1
            // Guarded because `advance_fill_per_sample()` is defined only under
            // `#if NLSC_E4_CACHE == 1`; without the guard `-DNLSC_E4_CACHE=0` would not
            // compile.
            advance_fill_per_sample();
#endif
            const double ieq = geq_ * (vc_ + i_prev_ * inv_gc_);
#if NLSC_E4_CACHE == 1
            note_range(ieq);
            // DESIGN B: the table IS this system, so inside the box there is no
            // Newton. Outside, it falls to it without touching the state.
            if (in_box(n19_abs, ieq)) {
                read_cache(n19_abs, ieq, x_, ib_);
#if NLSC_E4_CHARGES
                // No Newton here, so the transistor evaluation is paid:
                // one per sample, against the 2-3 iterations saved.
                advance_charges_from(n19_abs - x_, n19_abs - kVCC);
#endif
                n19_prev_ = n19_abs;
                ++samples_;
                const double i_branch = x_ * geq_ - ieq;
                vc_ += (i_branch + i_prev_) * k_vc_;
                i_prev_ = i_branch;
                return x_;
            }
            ++out_of_box_;
            note_outside(n19_abs, ieq);
#endif
            const double vbc = n19_abs - kVCC;
            // THE FOLLOWER'S PREDICTOR. An emitter follower has near-constant gain
            // (0,94-0,97), so starting the Newton from `x + g·Δn19` starts where the
            // circuit is going to be, not from the previous sample. It costs one
            // multiply-add and saves whole iterations, which is where the exponentials
            // live. `g` is not tuned: if it is off, the Newton takes one more
            // iteration; the solution, the fixed point, does not change.
            x_ += kGainSeg * (n19_abs - n19_prev_);
            n19_prev_ = n19_abs;
            // Scalar Newton: 2-3 typical iterations. The budget is bounded
            // (`kMaxIter`, `kTolNewton`): iterating below the model's own error (~−70 dB)
            // buys nothing and costs the cascade its speed advantage.
            int it = 0;
            for (; it < kMaxIter; ++it) {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, n19_abs - x_, vbc, false);
                const double f  = (o.ib + o.ic) - x_ * g13_ - (x_ * geq_ - ieq);
                const double df = -(o.dic_be + o.dib_be) - g13_ - geq_;
                double step = f / df;
                // The step is clamped: without it a distant start throws
                // the exponential to infinity and the engine never returns.
                if (step >  0.3) step =  0.3;
                if (step < -0.3) step = -0.3;
                x_ -= step;
                if (step < kTolNewton && step > -kTolNewton) break;
            }
            // Non-convergence counter: only exhausting the budget without meeting
            // tolerance is counted, since leaving via the `break` is convergence. It
            // separates "the cap does not bite" from "it always bites". No new branch
            // on the normal path: `it == kMaxIter` is only true when the loop ran out.
            if (it == kMaxIter) ++unconverged_;
            ++samples_;
            // The last sample's BASE current, already computed.
            {
                // `kExactCharges`: this evaluation is already made, so on the Newton path
                // the charges are free.
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, n19_abs - x_, vbc, kExactCharges);
                ib_ = o.ib;
#if NLSC_E4_CHARGES == 2
                advance_charges_from(n19_abs - x_, vbc);
#elif NLSC_E4_CHARGES
                advance_charges(o.qbe, o.qbc);
#endif
            }
            const double i_branch = x_ * geq_ - ieq;
            vc_ += (i_branch + i_prev_) * k_vc_;
            i_prev_ = i_branch;
            return x_;
        }

        // THE SAME SUBSYSTEM, WITH `n19` ALSO UNKNOWN.
        //
        // `process()` receives `n19` ready-made, which forces its caller to use
        // the previous sample's base current. Here `n19` enters as the line the
        // level network imposes,
        //
        //     n19 = a0 + kz * ib
        //
        // and is solved together with `n17`. The cost is a 2x2 instead of a
        // division, with the same single `bjt()` call per iteration, which is where
        // the exponentials live; it runs ~5 % slower than the delayed version.
        //
        // The two equations, with `vbe = v - x` and `vbc = v - VCC`:
        //   f1 = (ib + ic) - x*g13 - (x*geq - ieq)          [KCL at n17]
        //   f2 = a0 + kz*ib - v                             [the level network]
        // and the Jacobian comes whole from the four derivatives `bjt()` already
        // returns: no finite differences, no fitted parameter.
        //
        // `kz = 0` reduces it exactly to the case `n19 = a0`: f2 pins `v` and the
        // 2x2 reduces to the scalar.
        double process_loop(double a0, double kz, double& n19_out)
        {
            // THE CHANGE OF VARIABLE. The level network imposes
            // `n19 = a0 + kz*ib_total`, and Q2's INTERNAL base is
            // `u = n19 - RB*ib_total`, so `u = a0 + (kz - RB)*ib_total`.
            // Same form, same unknown: the Newton solves in `u` with `kzp` and `n19` is
            // recovered at the end with the real `kz`. With `rb_ = 0`, `kzp == kz`.
            const double kzp = kz - rb_;
#if NLSC_E4_CACHE == 1
            // Guarded because `advance_fill_per_sample()` is defined only under
            // `#if NLSC_E4_CACHE == 1`; without the guard `-DNLSC_E4_CACHE=0` would not
            // compile.
            advance_fill_per_sample();
#endif
            const double ieq = geq_ * (vc_ + i_prev_ * inv_gc_);
#if NLSC_E4_CACHE
            note_range(ieq);
            // Work happens on COPIES and commits only if all the reads fell inside:
            // if any leaves, the state stays intact so the Newton below starts exactly
            // where it would have without the cache.
            {
                double xt = x_, ibt = ib_;
                bool inside = true;
#  if NLSC_E4_CACHE == 2
                // DESIGN A: the table already carries the loop (built with this `kz`),
                // so it is one read.
                double v = 0.0;
                if (in_box(a0, ieq)) { read_cache(a0, ieq, xt, ibt); v = a0 + kzp * ibt; }
                else inside = false;
#  else
                // DESIGN B: the table is the open system's and the affine loop closes
                // outside by fixed point. The loop gain is ~0,09 at the worst `lvl`, and
                // the step count is a constant (`kCIt`), not a `while`.
                // The fixed point carries the displacement current inside: taking it out
                // leaves `x` and `n19` describing different voltages.
                double v = a0 + kzp * (ibt + idisp_de(n19_prev_ - xt, n19_prev_ - kVCC));
                for (int it = 0; it < kCIt && inside; ++it) {
                    if (!in_box(v, ieq)) { inside = false; break; }
                    read_cache(v, ieq, xt, ibt);
                    v = a0 + kzp * (ibt + idisp_de(v - xt, v - kVCC));
                }
#  endif
                if (inside) {
                    x_ = xt; ib_ = ibt;
#if NLSC_E4_CHARGES
                    // `idisp_` is pinned with the fixed point's `v` and then `v` is
                    // recomputed with it, so `n19 = a0 + kz·ib()` holds by construction. What
                    // remains is the fixed point's truncation, which `kCIt` governs, the same
                    // as for `ib`.
                    advance_charges_from(v - x_, v - kVCC);
                    v = a0 + kzp * (ib_ + idisp_);
#endif
                    ++samples_;
                    n19_prev_ = v;
                    // `v` is the INTERNAL base; what leaves is the TERMINAL, which `dev_n19`
                    // refers to and the waveshaper consumes. It is recovered from the
                    // definition of the change of variable (`terminal = internal + RB*ib`), not
                    // by recomputing from `a0`, so with `rb_ = 0` this is exactly `n19_out = v`.
                    n19_out = v + rb_ * ib();
                    const double i_branch = x_ * geq_ - ieq;
                    vc_ += (i_branch + i_prev_) * k_vc_;
                    i_prev_ = i_branch;
                    return x_;
                }
                ++out_of_box_;
                note_outside(v, ieq);
            }
#endif
            // Predictor: `v` from the previous sample's current (the delayed
            // version's answer), and `x` with the follower's gain.
            double v = a0 + kzp * ib_;
            x_ += kGainSeg * (v - n19_prev_);
            int it = 0;
            for (; it < kMaxIter; ++it) {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, v - x_, v - kVCC, false);
                const double gm = o.dib_be + o.dic_be;       // d(ib+ic)/d(vbe)
                const double gc = o.dib_bc + o.dic_bc;       // d(ib+ic)/d(vbc)
                const double f1 = (o.ib + o.ic) - x_ * g13_ - (x_ * geq_ - ieq);
                // THE DISPLACEMENT CURRENT ENTERS INSIDE THE LOOP. With the previous
                // sample's value outside, the line `n19 = a0 + kz·ib` would no longer
                // close exactly. It is nearly free because in mode 2 the charge is
                // `cj0_eff·v`: the Euler companion only needs the previous sample's `q`, a
                // constant during this Newton.
                const double f2 = a0 + kzp * (o.ib + idisp_de(v - x_, v - kVCC)) - v;
                const double j11 = -gm - g13_ - geq_;        // df1/dx
                const double j12 = gm + gc;                  // df1/dv
                // Both displacement derivatives enter the Jacobian; without them the
                // Newton stops on step size short of closing the loop.
                const double j21 = -kzp * o.dib_be + kzp * didisp_dx();   // df2/dx
                const double j22 = kzp * (o.dib_be + o.dib_bc) - 1.0
                                 + kzp * didisp_dv();                    // df2/dv
                const double det = j11 * j22 - j12 * j21;
                // The determinant cannot vanish in this circuit (j11 and j22 are negative
                // and dominate), but a tiny `det` from an absurd excursion would launch the
                // step to infinity. Guarded like the step.
                const double inv = (det > -1e-300 && det < 1e-300) ? 0.0 : 1.0 / det;
                double px = (f1 * j22 - f2 * j12) * inv;
                double pv = (j11 * f2 - j21 * f1) * inv;
                // The step is clamped as in the scalar case: without this a
                // distant start throws the exponential to infinity and the
                // engine never returns.
                if (px >  0.3) px =  0.3;
                if (px < -0.3) px = -0.3;
                if (pv >  0.3) pv =  0.3;
                if (pv < -0.3) pv = -0.3;
                x_ -= px;
                v  -= pv;
                if (px < kTolNewton && px > -kTolNewton &&
                    pv < kTolNewton && pv > -kTolNewton) break;
            }
            if (it == kMaxIter) ++unconverged_;
            ++samples_;
            {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, v - x_, v - kVCC, kExactCharges);
                ib_ = o.ib;
#if NLSC_E4_CHARGES == 2
                advance_charges_from(v - x_, v - kVCC);
#elif NLSC_E4_CHARGES
                advance_charges(o.qbe, o.qbc);
#endif
            }
            n19_prev_ = v;
            // `v` is the INTERNAL base (the Newton's variable); the TERMINAL is what
            // leaves, because `dev_n19` refers to `q_n19_abs_` (ngspice's `n19`) and it
            // is what Q2's waveshaper consumes. Recovered from the definition of the
            // change of variable, not by recomputing from `a0`, so with `rb_ = 0` this
            // is exactly `n19_out = v`.
            n19_out = v + rb_ * ib();
            const double i_branch = x_ * geq_ - ieq;
            vc_ += (i_branch + i_prev_) * k_vc_;
            i_prev_ = i_branch;
            return x_;
        }

    private:
        mna::QModel qm_{};
        mna::Engine::Junc jbe_{}, jbc_{};   // no charges: `with_charges = false`
        double g13_ = 0.0, geq_ = 0.0, inv_gc_ = 0.0, k_vc_ = 0.0;
        double x_ = 0.0, vc_ = 0.0, i_prev_ = 0.0, n19_prev_ = 0.0;
        double ib_ = 0.0;
        double n19_rest_ = 0.0;
        // Q2's base resistance. At 0 every formula reduces to the terminal-base
        // form.
        double rb_ = 0.0;
        double u_rest_ = 0.0;      // INTERNAL base at rest = n19_rest - RB*ib0
#if NLSC_E4_CACHE
        // FIXED size: `prepare()` is also called on a knob move, and no
        // memory gets requested there (the audio-thread rule: no heap).
        float tab_[size_t(kCN1) * size_t(kCN2) * 2] = {};
        double c_lo1_ = 0.0, c_lo2_ = 0.0, c_h1_ = 1.0, c_h2_ = 1.0;
        double c_inv1_ = 1.0, c_inv2_ = 1.0;
        double c_x0_ = 0.0, c_p0_ = 0.0, c_g_ = 0.0;  // the line subtracted on store
        bool   built_ = false;
        // THE SLICE-FILL CURSOR: with these five scalars the table is filled
        // incrementally instead of all at once.
        bool   cache_pending_ = false;      // geometry fixed, content half-done
        bool   deferred_fill_ = false;  // set only by the plugin core
        int    cur_k_ = 0, cur_j_ = 0;   // where it was (column, node)
        double cur_x_ = 0.0;             // the warm start, BETWEEN slices
        double c_n17_rep_ = 0.0;         // rest each new column starts from
        double c_geq_done_ = 0.0, c_g13_done_ = 0.0, c_kz_done_ = 0.0, c_p0_done_ = 0.0;
        long out_of_box_ = 0, out_p_ = 0, out_i_ = 0;
        double excess_p_ = 0.0, excess_i_ = 0.0;
        double w_min_ = 1e300, w_max_ = -1e300;
#endif
    };

    // SEPARATE FROM `prepare()`: see the note at `Stage1::set_bank`.
    // Called BEFORE `set_rest()`, which subtracts `fit_n19_`/`fit_n17_`.
    // The bank is COPIED to members because it is read per sample.
    template <class J>
    void set_bank()
    {
        ws2_n_ = J::WS2_N;
        for (int i = 0; i < ws2_n_; ++i) ws2_[i] = J::WS2[i];
#if NLSC_WS_RECENTER
        ws2_[ws2_n_ - 1] = 0.0;   // see NLSC_WS_RECENTER in nls_stage1.h
#endif
        umin_ = J::WS2_UMIN;  umax_ = J::WS2_UMAX;
        fit_n19_ = J::Fit_n19;  fit_n17_ = J::Fit_n17;
        use_sub_ = J::use_sub;      // declared by the BANK, not an `idx == 1`
    }

    template <class J>
    void prepare_bank(double fs, double lvl, bool reinit)
    {
        set_bank<J>();
        g1o_.prepare_bank<J>(fs, lvl, reinit);
        // `kz` enters HERE because the cache's design A depends on it (and
        // therefore on LEVEL): always passed, and the Newton ignores it.
        sub_.prepare(fs, r15_, r14_, q_n17_, q_n19_abs_, g1o_.ib_gain(),
                     reinit);
        ib0_ = sub_.ib_rest(q_n19_abs_, q_n17_);
        if (reinit) ib_prev_ = ib0_;
        gs_.prepare(fs, J::E4GS_B, J::E4GS_A, J::ScaleF, reinit);
        g2_.prepare(fs, J::E4G2_B, J::E4G2_A, J::ScaleF, reinit);
    }


    void reset()
    {
        g1o_.reset(); gs_.reset(); g2_.reset();
        out_of_range_ = 0; u_min_ = 1e9; u_max_ = -1e9;
    }


    // THE LOOP, IN ONE PLACE — with the DISPLACEMENT current.
    //
    // The level network imposes `n19 = a0 + kz·ib`, and `ib` is the
    // PHYSICAL current: static + displacement. `process_loop` solves the
    // static part, so the displacement term enters through `a0`.
    //
    // `close_ib` receives the SAME current as `n19`: otherwise `Z`'s
    // state and the line using it would speak of two different currents.
    double process_loop_(double a0, double kz, double& n19_abs)
    {
        // `sub_.ib()` is already the PHYSICAL current (static +
        // displacement) and the loop closes with it INSIDE the Newton, so
        // there is nothing to correct out here: `n19` and `Z`'s state speak
        // of the SAME current.
        const double w = sub_.process_loop(a0, kz, n19_abs) - q_n17_;
        g1o_.close_ib(sub_.ib() - ib0_);
        last_w_ = w;
        return w;
    }

    double process(double dev_n14, double* dev_n19 = nullptr, double du = 0.0,
                   double dvr = 0.0)
    {
        {
            const double kz = g1o_.ib_gain();
            const double a0 = q_n19_abs_ + g1o_.part_without_ib(dev_n14, dvr)
                            - kz * ib0_;
            double n19_abs = 0.0;
            double w_loop;
            w_loop = process_loop_(a0, kz, n19_abs);
            // STRUCTURAL INVARIANT, in volts: how much this sample violates the
            // algebraic relation `n19 = a0 + kz·ib` the network imposes. It is
            // independent of the null: no ngspice, no reference.
            loop_resid_ = (a0 + kz * sub_.ib()) - n19_abs;
            ib_prev_ = sub_.ib();
            const double u_loop = (n19_abs - q_n19_abs_) + off_in_;
            if (u_loop < u_min_) u_min_ = u_loop;
            if (u_loop > u_max_) u_max_ = u_loop;
            if (dev_n19) *dev_n19 = u_loop;
            w_ = w_loop;
            (void)du;
            // Not `gs_`: the subsystem already solves `n17` with its own network.
            // See the `NLSC_E4_GS_IN_SUB` macro above.
            return g2_.process(kGsInSub ? gs_.process(w_loop) : w_loop);
        }
    }

    // The current rest against the one Q2's curve was fitted at. The gain pot
    // moves `n19` by 1,38 mV and `n17` by 1,37 mV between the knob's ends; the
    // waveshaper must evaluate in its fit frame and its output return to the
    // current one. See `nls_reposo_drive.h`. At zero offset nothing changes.
    // `kFit_*` belongs to the variant: it is the rest Q2's curve was fitted at,
    // and between the 808 and the 9/9RI it differs by 998 mV.
    void set_rest(double q_n19, double q_n17)
    {
        off_in_  = q_n19 - fit_n19_;
        off_out_ = q_n17 - fit_n17_;
        q_n19_abs_ = q_n19;
        q_n17_     = q_n17;
#if defined(NLSC_E4_Q_N17_TRIM)
        // DIAGNOSTIC: shifts the tabulated rest of `n17` by a fixed amount, to
        // test whether a start-up offset in `w = sub_.process_loop(...) - q_n17_`
        // comes from a mismatch between the tabulated and the solved rest.
        q_n17_ += (NLSC_E4_Q_N17_TRIM);
#endif
    }

    // The netlist values the VARIANT changes, for the subsystem.
    void set_resistances(double r15, double r14) { r15_ = r15; r14_ = r14; }

    // PROBE: the last sample's absolute `n17`. `w_` is stored by `process()`.
    // The frame depends on the path: on the tabulated path `w_` is the
    // waveshaper's output in its fit frame; on the subsystem path it is
    // `n17 − q_n17_`, a deviation against the current rest.
    double n17_abs() const { return (use_sub_ ? q_n17_ : fit_n17_) + w_; }

    double ws_umin() const { return umin_; }
    double ws_umax() const { return umax_; }

    // The current rest's steady state — see `ParamFilter::preset_dc`.
    void startup()
    {
        g1o_.reset(); ib_prev_ = ib0_;
        // This preset is the tabulated path's rest. The closed-loop path feeds
        // `g2_` from `w_loop` instead, so it starts with a tiny step through the
        // high-pass (about 1,5 µV, estimated, inaudible).
        g2_.preset_dc(gs_.preset_dc(poly(off_in_) - off_out_ * rest::kOffOutput));
    }

    // The guard's statistics zero AFTER the pre-run: otherwise they count
    // the startup transient, which is not what is being judged.
    void reset_stats() { out_of_range_ = 0; u_min_ = 1e9; u_max_ = -1e9; }

    long out_of_range() const { return out_of_range_; }
    double u_min() const { return u_min_; }
    double u_max() const { return u_max_; }
#if NLSC_E4_CACHE
    // The cache box's counters, published outward, so an edge is never hit
    // silently.
    long out_of_box() const { return sub_.out_of_box(); }
    long outside_p()   const { return sub_.outside_p(); }
    long outside_ieq() const { return sub_.outside_ieq(); }
    double excess_p()   const { return sub_.excess_p(); }
    double excess_ieq() const { return sub_.excess_ieq(); }
    double w_min() const { return sub_.w_min(); }
    double w_max() const { return sub_.w_max(); }
#endif


    // Subsystem Newton observability (see `SubQ2::process`).
    long newton_escapes() const { return sub_.unconverged_; }
    long newton_samples() const { return sub_.samples_; }
    // The cache's slice filling lives in the subsystem; this only forwards.
#if NLSC_E4_CACHE
    void defer_fill(bool v)        { sub_.defer_fill(v); }
    bool advance_cache(int nodes)        { return sub_.advance_cache(nodes); }
    bool cache_pending() const        { return sub_.cache_pending(); }
    long cache_nodes_left() const  { return sub_.cache_nodes_left(); }
#endif
    // PROBE: the `ib` loop's violation on the last sample, in volts. With the
    // loop closed it sits at the Newton's floor; with `ib` delayed it equals
    // `kz·Δib`.
    double loop_resid() const { return loop_resid_; }

private:
    double poly(double u) const
    {
        double w = 0.0;
        for (int i = 0; i < ws2_n_; ++i) w = w * u + ws2_[i];
        return w;
    }
    double tangent(double u0, double u) const
    {
        double d = 0.0;                     // derivative by Horner
        for (int i = 0; i < ws2_n_ - 1; ++i)
            d = d * u0 + ws2_[i] * (ws2_n_ - 1 - i);
        return poly(u0) + d * (u - u0);
    }

    VariableLevel     g1_;
    OpenBaseLevel  g1o_;   // the OPEN-base one, for the subsystem
    FixedFilter<fixed::kE4GS_NB, fixed::kE4GS_NA> gs_;
    FixedFilter<fixed::kE4G2_NB, fixed::kE4G2_NA> g2_;
    long out_of_range_ = 0;

    // COPY of the active variant's waveshaper: read PER SAMPLE. The
    // constructor leaves the 808's.
    double last_w_ = 0.0;   // the loop's own `w`, read back by `prepare()`
    double ws2_[kWSMax] = {0.0};
    int    ws2_n_ = e234::kWS2_N;
    double umin_  = e234::kWS2_UMIN;
    double umax_  = e234::kWS2_UMAX;
    double w_ = 0.0;          // last waveshaper output, for the probe
    double loop_resid_ = 0.0; // violation of `n19 = a0 + kz·ib`, in volts
    double ib_prev_ = 0.0, ib0_ = 0.0;   // base current and its rest
    double q_n19_abs_ = 0.0, q_n17_ = 0.0;
    double r15_ = 10.0e3, r14_ = 100.0;   // 808 by default: R15 10k, R14 100
    bool   use_sub_ = false;              // set by `set_bank`
    SubQ2  sub_{};
    double fit_n19_ = rest::kFit_n19;
    double fit_n17_ = rest::kFit_n17;
    double u_min_ = 1e9, u_max_ = -1e9;
    double off_in_ = 0.0, off_out_ = 0.0;
};

// The four chained stages.
struct Outputs { double n4, n7, n14, out; };

// The `n3`/`n6` probe (see the member of the same name): without the
// flag it is `nullptr` and stage 1 writes nothing.
#  define kProbeN6 nullptr

// GUARD: `NLSC_CASC_CORR` CANNOT BE TURNED ON WHILE E2C IS THE IDENTITY.
//
// The opamp post-filter correction is off, so the coefficient generator
// emits `E2C` as the identity (`kE2C_B` is bit-for-bit `kE2C_A` in both
// variant sets). Turning the switch on in that state would apply `H(z) = 1`
// and report no change, which reads as a refutation of the post-filter when
// it only means the post-filter is not there. With the switch undefined this
// block does not exist; defined, the build fails with the message below.
#ifdef NLSC_CASC_CORR
constexpr bool nlsc_coef_pair_is_identity(const double (&b)[3][1],
                                          const double (&a)[3][1])
{
    for (int i = 0; i < 3; ++i)
        if (b[i][0] != a[i][0]) return false;
    return true;
}
static_assert(!nlsc_coef_pair_is_identity(fixed::kE2C_B, fixed::kE2C_A),
              "NLSC_CASC_CORR is ON but E2C is the IDENTITY filter for the 808 set: "
              "the coefficients come from the generator's OFF branch, so this would "
              "measure nothing. The route back exists: measure stage 2's response, "
              "put the candidate where the generator reads it and re-run the "
              "generator to emit a real E2C -- by hand, deliberately, because that "
              "changes the generated coefficient headers.");
static_assert(!nlsc_coef_pair_is_identity(fixed_v9ri::kE2C_B, fixed_v9ri::kE2C_A),
              "NLSC_CASC_CORR is ON but E2C is the IDENTITY filter for the 9/9RI set: "
              "same cause and same fix as the 808 assertion above.");
#endif

class Cascade4 {
public:
    // The hoisted `lp_rail` coefficients start consistent with the default
    // `fs_`, so the filter never runs with zeroes if something processes before
    // the first `prepare_bank`. Seeded by the same method that keeps them in
    // step, so there is no second copy of the formula.
    Cascade4() { lp_set_rate(); }







    // Belt state (see `process`). `fs_`…`circuit_` are the anchor.
    double gain_ = 1.0, tone_ = 0.5, lvl_ = 0.0;   // `fs_` already exists below
    int    circuit_ = 0;        // CIRCUIT, not the selector row
    int    belt_streak_ = 0;
    int    belt_need_ = kRescue;   // doubled by each rescue (see belt_)
public:
    long   belt_broken_ = 0;   // samples with the output broken
    long newton_escapes() const { return e4_.newton_escapes(); }
    // Stage 2's probes, forwarded outwards.
    long   e2_unconverged()   const { return e2_.unconverged(); }
    double e2_v5() const { return e2_.probe_v5(); }
    double e2_v7() const { return e2_.probe_v7(); }
    double e2_v2() const { return e2_.probe_v2(); }
    double e2_w_extreme() const { return e2_.w_extreme(); }
    long newton_samples() const { return e4_.newton_samples(); }
    double loop_resid() const { return e4_.loop_resid(); }
    // Defers the cache fills. Only the plugin core turns it on: it is the
    // only caller that knows `prepare()` reaches this from the audio thread.
    // Other callers keep the table built at once in `prepare()`.
    void defer_fill(bool v)
    {
#if NLSC_E4_CACHE
        e4_.defer_fill(v);
#endif
        // Stage 2's table is deferred by the same switch: both are caches that
        // `prepare()` builds from the audio thread, and one decision gets one
        // switch.
#if NLSC_E2_TABLE
        e2_.table_deferred(v);
#endif
#if !NLSC_E4_CACHE && !NLSC_E2_TABLE
        (void)v;
#endif
    }
#ifndef NLSC_BELT_BACKOFF
#define NLSC_BELT_BACKOFF 1   // 0 = a rescue every kRescue broken samples, forever
#endif
#ifndef NLSC_REINIT_CLEARS_GESTURE
#define NLSC_REINIT_CLEARS_GESTURE 1   // 0 = a reinit leaves the gesture flag as it was
#endif
    // "The user asked for this re-tune": set by `apply_knobs()`, the only
    // caller that tells a gesture from a set-up. It exists with the table
    // switched off too, so the core does not need to know whether the table is
    // compiled in.
    void table_gesture(bool v)
    {
#if NLSC_E2_TABLE
        e2_.table_gesture(v);
#else
        (void)v;
#endif
    }
    // Stage 2 table observability, forwarded so that "it never fires" can be
    // checked from outside.
    long table_out_of_box() const
    {
#if NLSC_E2_TABLE
        return e2_.table_out_of_box();
#else
        return 0;
#endif
    }
    long table_fill_unconverged() const
    {
#if NLSC_E2_TABLE
        return e2_.table_fill_unconverged();
#else
        return 0;
#endif
    }
    bool advance_cache(int nodes = NLSC_E4_CACHE_NODES_PER_BLOCK)
    {
#if NLSC_E4_CACHE
        return e4_.advance_cache(nodes);
#else
        (void)nodes; return false;
#endif
    }
    bool cache_pending() const
    {
#if NLSC_E4_CACHE
        return e4_.cache_pending();
#else
        return false;
#endif
    }
    long cache_nodes_left() const
    {
#if NLSC_E4_CACHE
        return e4_.cache_nodes_left();
#else
        return 0;
#endif
    }
    long   rescues_ = 0;         // times it returned to rest

    // THE PARAMETER IS THE CIRCUIT, NOT THE SELECTOR ROW. A row is
    // (circuit × knob law), so several rows share a circuit; the caller
    // translates with `kVariants[row].circuit` and it arrives here translated.
    void prepare(double fs, double gain, double tone, double lvl,
                 int circuit = 0, bool reinit = true)
    {
        // Stored so the belt below can re-anchor: the absolute state to return
        // to.
        gain_ = gain; tone_ = tone; lvl_ = lvl; circuit_ = circuit;   // `fs_` set by `prepare_bank`
        belt_streak_ = 0;
        // A reinit is never a knob gesture, whoever calls it. The belt below
        // re-prepares with reinit=true without going through the core's gesture
        // flag; a flag left over from the last knob move would make stage 2 settle
        // its DC without its table and land on a dead state (output stuck at 0).
#if NLSC_REINIT_CLEARS_GESTURE
        if (reinit) table_gesture(false);
#endif
        // Every circuit reaches its own bank, because `Banks` is checked
        // against `kCircuits` (`nls_juegos.h`). Bank 0 is also where an index
        // outside the table lands, which can only be a corrupt value.
        prepare_circuit(Banks{}, circuit, fs, gain, tone, lvl, reinit);
    }

    template <class First, class... Rest>
    void prepare_circuit(BankList<First, Rest...>, int circuit, double fs,
                         double gain, double tone, double lvl, bool reinit)
    {
        bool done = false;
        ((done = done || (circuit == Rest::idx
                          && (prepare_bank<Rest>(fs, gain, tone, lvl, reinit), true))), ...);
        if (!done) prepare_bank<First>(fs, gain, tone, lvl, reinit);
    }

    template <class J>
    void prepare_bank(double fs, double gain, double tone, double lvl, bool reinit)
    {
        e4_.set_bank<J>();
        // The two resistors that ARE the variant, from the netlist. One source:
        // `nls_variantes.h`, the same table the GUI and `.ttl` read.
        e4_.set_resistances(kCircuits[J::idx].rout_shunt, kCircuits[J::idx].rout_ser);

        // THE REST FOLLOWS THE KNOB. See `nls_reposo_drive.h`: the gain pot moves
        // `n7` by 6,09 mV and `VR` by 1,90 mV between the ends. Evaluated once,
        // here. And the rest is the variant's: between the 808 and the 9/9RI `n17`
        // moves +998 mV and `n19` +947 mV, a volt of DC the rail loop would
        // otherwise integrate.
        q_vr_  = rest::ev(J::Rvr,  gain);
        q_n3_  = rest::ev(J::Rn3,  gain);
        q_n4_  = rest::ev(J::Rn4,  gain);
        q_n7_  = rest::ev(J::Rn7,  gain);
        q_n11_ = q_vr_;   // `n11` is opamp B's output, resting at VR
        q_n14_ = rest::ev(J::Rn14, gain);
        q_n19_ = rest::ev(J::Rn19, gain);
        const double q_n6  = rest::ev(J::Rn6,  gain);
        const double q_n17 = rest::ev(J::Rn17, gain);

        // The conductances each node injects into the rail with.
        g_n3_ = J::G_n3;  g_n4_ = J::G_n4;  g_n11_ = J::G_n11;  g_n19_ = J::G_n19;
        // The output's quiescent point is the variant's, like every other q_*.
        q_out_ = J::Q_out;

        // PROBE: upper bound of correcting stage 1's rest, sibling of
        // `NLSC_SHIFT_N19`. This stage's rest is evaluated once in `prepare()`, from
        // the knob alone, while under signal the rest displaces. The two axes are
        // separate because the residual means differ in sign (`n3` -0,4896 mV,
        // `n4` +0,1255), so one common offset cannot correct both. Undefined, the
        // path is unchanged.
        (void)q_n6;
        // PROBE: upper bound of correcting the bias displacement. Under signal the
        // real circuit shifts its working point (+10,0 mV at `n19`) while the
        // cascade evaluates the waveshaper centred on the static rest.
        e4_.set_rest(q_n19_, q_n17);

        e2_.prepare(fs, gain, q_vr_, q_n4_, 1e-3, reinit);
        e3_.prepare(fs, tone, reinit);
        e4_.prepare_bank<J>(fs, lvl, reinit);
        // The rail's injection into each stage depends on the SAME knob as
        // its signal network, for the same reason: same network, other
        // source.
        rn14_.prepare(fs, tone, reinit);
        rn19_.prepare_bank<J>(fs, lvl, reinit);
#if NLSC_E1_PHYS
        e1p_.prepare(fs);
#endif
#if NLSC_VRA_PARAM
        vra_.prepare(fs, tone, reinit);
#else
        vra_.prepare(fs, J::VRA_B, J::VRA_A, J::ScaleF, reinit);
#endif
        vrb_.prepare(fs, J::VRB_B, J::VRB_A, J::ScaleF, reinit);
        vrc_.prepare(fs, J::VRC_B, J::VRC_A, J::ScaleF, reinit);
        vrp_.prepare(fs, J::VRP_B, J::VRP_A, J::ScaleF, reinit);
        c2_.prepare(fs,  J::E2C_B, J::E2C_A, J::ScaleF, reinit);
        fs_ = fs;
        lp_set_rate();   // the ONLY writer of `fs_`: keep the hoist in step
        if (reinit) {
            c2_.reset(); e3_.reset(); e4_.reset();
            lp_x_ = lp_y_ = 0.0;
            dvr_ = dev_n3_ = dev_n19_ = 0.0;
            // Every piece of state must enter the reset: a reset instance has to be
            // indistinguishable from a new one (block bit-identity, in-place
            // processing, `activate()` twice).
            dvr_ant_ = 0.0;
#if NLSC_CASC_SEGMENTED
            seg_n4_ = seg_n7_ = seg_n14_ = 0.0;
#endif
            rn14_.reset(); rn19_.reset();
            vra_.reset(); vrb_.reset(); vrp_.reset(); vrc_.reset();
        }
        if (reinit) e4_.startup();
#if NLSC_E1_PHYS
        // The physical stage finds its own rest from the current VR: the rest
        // tables are not an input to it, the circuit is.
        if (reinit) e1p_.startup(q_vr_, e2_.probe_v5());
#endif
    }

    // STAGE 2's LINEAR POST-FILTER CORRECTION (`-DNLSC_CASC_CORR`), off.
    //
    // The term the cascade discards is the opamp's finite gain, which in small
    // signal is a linear error, so a filter could correct it:
    //
    //     Hcorr(s) = (real circuit's n7/n4) / (ideal model's n7/n4)
    //
    // It does not hold on real material: the ideal-opamp error depends on
    // level. Closed-loop gain with the diodes off is ~97; when they clip it
    // collapses, the loop gain rises and the ideal-opamp hypothesis gets
    // better, so a correction sized with the diodes off over-corrects exactly
    // when they conduct. A fixed post-filter cannot fix an error that moves
    // with level; the finite gain would have to go inside stage 2's loop.
    //
    // With the correction off the generator emits `E2C` as the identity; the
    // guard above `class Cascade4` stops the switch from silently doing
    // nothing.
    double correct(double dev_n7)
    {
#ifdef NLSC_CASC_CORR
        return c2_.process(dev_n7);
#else
        return dev_n7;
#endif
    }


    // THE CASCADE'S BELT. Extreme input (large squares or steps well above
    // the pedal's range) can drive the engine to NaN or to huge finite values
    // from which neither silence nor normal signal recovers it. Every fallback
    // needs an absolute anchor, not a relative one: here the anchor is
    // `prepare()` with `reinit`, which solves the rest from scratch, the only
    // state that is always valid.
    //
    // The trigger is not just `!isfinite`: a diverged engine can also sit at a
    // huge finite value. It also bounds by magnitude, with a physically
    // impossible threshold (the pedal runs at 9 V; 100 V is not a legitimate
    // transient, it is a diverged engine).
    //
    // `kRescue` consecutive samples are required: an isolated event must not
    // cause a click. The counter zeroes as soon as one sample comes out sane,
    // so on the normal path this costs one comparison.
    static constexpr int    kRescue = 128;
    static constexpr double kAbsurd = 100.0;   // volts
    // BACK-OFF. A rescue is a full re-prepare, and an input that stays absurd
    // (a finite +100 dBFS from upstream) breaks the engine again at once, which
    // would re-prepare every 128 internal samples and run slower than real
    // time. Each rescue doubles the streak the next one needs, up to
    // kRescueMax; one sane sample resets it.
    static constexpr int    kRescueMax = kRescue * 64;

    // The belt itself, shared by every entry point, the plugin's
    // `process_modelled()` included. A per-sample isfinite downstream would
    // mute without healing the state.
    inline Outputs belt_(Outputs s)
    {
        if (!(std::fabs(s.out) < kAbsurd)) {    // catches NaN, inf and absurd
            ++belt_broken_;
            // While the engine is broken, emit the quiescent point, not garbage.
            // Re-emitting the last good value would be a relative anchor, which
            // perpetuates itself.
            s = {q_n4_, q_n7_, q_n14_, q_out_};
            if (++belt_streak_ >= belt_need_) {
                prepare(fs_, gain_, tone_, lvl_, circuit_, true);   // ABSOLUTE anchor
                ++rescues_;
#if NLSC_BELT_BACKOFF
                belt_need_ = (belt_need_ < kRescueMax) ? 2 * belt_need_ : kRescueMax;
#endif
            }
        } else {
            belt_streak_ = 0;
            belt_need_ = kRescue;
        }
        return s;
    }


    // THE MODELLED RAIL.
    //
    // `VR` is not AC ground: it is a node, and its KCL is exact:
    //
    //     dVR · (Gtot + s·C11) = Σ dn_i / R_i
    //
    // (it matches ngspice's AC at −173,5 dB). At 300 Hz `n9` feeds 85,4 %,
    // `n18` 9,1 %, and `n4`/`n11`/`n19`/`n3` 6,5 % between the four. Since `n9`
    // and `n18` are linear functions of `n7` and `n14`, their contributions
    // enter through fitted transfers (`kVRA`, `kVRB`); `n19` enters through its
    // own filter (`vrc_`, coefficients `kVRC_*`), `n4`/`n11`/`n3` through scalar
    // conductances, and the node's pole (`kVRP`) applies once to the sum.
    Outputs process_modelled(double in)
    {
    // THE RAIL IS PREDICTED, NOT DELAYED. There is an algebraic loop
    // (`VR -> n7 -> VR`). Closing it exactly is not possible here: it passes
    // through stage 2, which is nonlinear, so the rail would have to be an
    // unknown of its Newton. A one-sample delay is a first-order error in `h`,
    // and a slow rail pole does not bound it: the loop gain times the signal's
    // slope does. A linear prediction cancels the first order in three
    // instructions, leaving the ~36 µV model floor intact. The loop gain goes
    // from 0,1496 to 0,4489 worst case (Nyquist), far below 1.
    // `NLSC_CASC_RAIL_PREDICTED=0` returns to the one-sample delay.
#if NLSC_CASC_RAIL_PREDICTED
        const double dvr_pred = dvr_ + (dvr_ - dvr_ant_);
        dvr_ant_ = dvr_;
        rail_ = q_vr_ + dvr_pred;
#else
        rail_ = q_vr_ + dvr_;      // one sample of delay
#endif
        const Outputs s = process_(in);
        // The next sample's rail, from what just left each stage. The CURRENTS
        // each node feeds the rail are summed and the node's impedance applies
        // ONCE: `dVR·(Gtot + s·C11) = Σ n_k/R_k`, the node's exact KCL. All three
        // rate banks carry `kVRA`/`kVRB` as currents (no pole); applying the rail
        // pole inside each contribution as well would count it twice and diverge.
        // `NLSC_CASC_RAIL_COMPOSED` belongs to an incompatible bank convention and
        // is rejected with `#error`, since with these banks it would silently give a
        // wrong model.
#if defined(NLSC_CASC_RAIL_COMPOSED)
#  error "NLSC_CASC_RAIL_COMPOSED assumes kVRA/kVRB are voltages, but the banks carry them as currents at all three rates: this branch would apply the rail pole once too few and give a wrong model without any error."
#endif
        dvr_ = kVrG * vrp_.process(kVrA * vra_.process(s.n7 - q_n7_)
                          + vrb_.process(s.n14 - q_n14_)
                          + g_n4_  * (s.n4 - q_n4_)
                          + g_n11_ * (e2_.n11() - q_n11_)
                          + vrc_.process(dev_n19_)
                          + g_n3_  * dev_n3_);
        // The belt wraps the plugin's path too. A broken sample also poisons the
        // rail update above (dvr_ goes NaN with it); the belt's absolute anchor,
        // prepare(reinit), re-seeds dvr_/dvr_ant_ and every filter along with the
        // stages.
        return belt_(s);
    }

    Outputs process_(double in)
    {
        // Stage 1: `nls_stage1.h` returns ABSOLUTE `n4` over its own rest
        // (`kVr`), so it converts to deviation by subtraction.
        // HOW THE RAIL ENTERS `n4`, and it is not 1:1. `n4` sees VR through `R4`
        // (10 k) and `n6` through `C2` (1 µF), so
        //     n4 = VR·1/(1+sR4C2) + n6·sR4C2/(1+sR4C2)
        // The second term already sits inside stage 1's `H2` fit. What must be
        // added is the first: a one-pole low-pass at 15,9 Hz on the rail's motion
        // (adding the rail's motion raw would be right only at DC).
        // The physical stage 1 carries the rail itself, so the correction is
        // computed only where a fitted stage 1 still reads it.
#if !NLSC_E1_PHYS
        const double drail = lp_rail(rail_ - q_vr_);
#endif
        double n3_abs = 0.0;
// The injections are independent `#if` blocks, not an `#elif` chain, so
// several can be compiled at once and their bounds compared in one run.
#if NLSC_E1_PHYS
        const double dev_n4 = e1p_.process(in, rail_, e2_.probe_v5(), &n3_abs, kProbeN6)
                            - q_n4_;
#else
        const double dev_n4 = e1_.process(in, rn3_.process(rail_ - q_vr_), &n3_abs,
                                          kProbeN6) - e1_.rest_n4() + drail;
#endif
        dev_n3_ = n3_abs - q_n3_;

#if NLSC_CASC_SEGMENTED
        // Stage `k` consumes what `k-1` produced on the PREVIOUS sample.
        // Same work, same instructions, same register pressure — only the
        // intra-sample dependency disappears.
        const double seg_n4 = seg_n4_;   seg_n4_ = dev_n4;
#endif
        // Stage 2: works in absolutes (its equations are the circuit's).
        e2_.set_vr(rail_);
#if NLSC_CASC_SEGMENTED
        const double dev_n7 = correct(e2_.process(q_n4_ + seg_n4) - q_n7_);
#else
        const double dev_n7 = correct(e2_.process(q_n4_ + dev_n4) - q_n7_);
#endif

        // The rail enters stage 3 through `R8` (10 k) towards `n9`, and
        // stage 4 through `R12` (510 k) towards `n19`. The fits were made
        // with VR pinned, so these are the missing summands.
        const double drv = rail_ - q_vr_;
#if NLSC_CASC_SEGMENTED
        const double seg_n7 = seg_n7_;   seg_n7_ = dev_n7;
        const double dev_n14 = e3_.process(seg_n7) + rn14_.process(drv);
#else
        const double dev_n14 = e3_.process(dev_n7) + rn14_.process(drv);
#endif
        // ATTRIBUTION PROBE (not a plugin mode): switch off the rail's injection
        // into `n19` to see how much of that node's error is the rail's. The rail
        // enters after the level pot, so its error does not attenuate with the
        // knob.
#  if NLSC_CASC_SEGMENTED
        const double seg_n14 = seg_n14_;  seg_n14_ = dev_n14;
        const double dev_out = e4_.process(seg_n14, &dev_n19_, rail_to_n19(drv), drv);
#  else
        const double dev_out = e4_.process(dev_n14, &dev_n19_, rail_to_n19(drv), drv);
#  endif

        return {q_n4_ + dev_n4, q_n7_ + dev_n7,
                q_n14_ + dev_n14, q_out_ + dev_out};
    }



private:
    // The current rests (`prepare` sets them from the knob; see above).
    // The conductances into the rail, from the active variant's bank.
    // Read PER SAMPLE, so they go to members like the waveshaper.
    double g_n3_  = e234::kG_n3,  g_n4_  = e234::kG_n4;
    double g_n11_ = e234::kG_n11, g_n19_ = e234::kG_n19;
    double q_vr_  = e234::kQ_vr,  q_n3_ = e234::kQ_n3, q_n4_  = e234::kQ_n4;
    double q_n7_  = e234::kQ_n7,  q_n11_ = e234::kQ_n11, q_n14_ = e234::kQ_n14;
    double q_out_ = e234::kQ_out;
    double q_n19_ = e234::kQ_n19;
    // MEASUREMENT SCALARS on the rail loop, both 1,0 by default.
    // `NLSC_VR_GAIN` scales the whole loop's gain, to test whether a rail DC
    // shortfall is a gain error (constant over level) rather than a missing
    // nonlinear term.
    // `NLSC_VR_GA` scales `n7`'s term alone (the `n9` branch via `R8`): `kVRA`
    // comes from `(n9/n7)/R8` measured with VR pinned (0,9091 = 10/11), while
    // with VR free the DC quotient is 0,9643, because VR moves with `n7`.
    static constexpr double kVrA =
#ifdef NLSC_VR_GA
        NLSC_VR_GA;
#else
        1.0;
#endif
    static constexpr double kVrG =
#ifdef NLSC_VR_GAIN
        NLSC_VR_GAIN;
#else
        1.0;
#endif
    double rail_ = e234::kQ_vr;


    // One-pole low-pass (R4·C2 = 10 ms), trapezoidal at the internal rate.
    // The coefficients depend only on `fs_`, so they are hoisted out of the
    // per-sample path. `lp_set_rate()` is the one place that writes them:
    // anything that changes `fs_` has to call it.
    void lp_set_rate()
    {
        // The real rate, not `e234::kFs`: that constant belongs to the tabulated
        // coefficient bank.
        const double T = 1.0 / fs_, tau = 10e3 * 1e-6;
        lp_b0_ = T / (T + 2.0 * tau);
        lp_a1_ = (T - 2.0 * tau) / (T + 2.0 * tau);
    }
    // The rail's path into `n19` only feeds stage 4's TABULATED path. When every
    // bank takes the closed loop, stage 4 never reads it, so it is not computed.
    inline double rail_to_n19(double drv)
    {
        if constexpr (kSubIn808 && kSubInV9ri && kOpenBase && kClosedLoop) {
            (void)drv;
            return 0.0;
        } else {
            return rn19_.process(drv);
        }
    }
    double lp_rail(double x)
    {
        const double b0 = lp_b0_, a1 = lp_a1_;
        const double y = b0 * (x + lp_x_) - a1 * lp_y_;
        lp_x_ = x; lp_y_ = y;
        return y;
    }
    double lp_x_ = 0.0, lp_y_ = 0.0;
    double lp_b0_ = 0.0, lp_a1_ = 0.0;   // set by `lp_set_rate()`
    double fs_ = e234::kFs;
    double dvr_ = 0.0, dev_n3_ = 0.0, dev_n19_ = 0.0;
    double dvr_ant_ = 0.0;      // the previous sample, to predict the rail
#if NLSC_CASC_SEGMENTED
    // The pipelining probe's registers. They start at zero because they are
    // deviations over rest, which is a stage's value at rest.
    double seg_n4_ = 0.0, seg_n7_ = 0.0, seg_n14_ = 0.0;
#endif
    FixedFilter<fixed::kN3_NB, fixed::kN3_NA> rn3_;
    // The rail's path into `n14` depends on the tone knob: the rail enters
    // through `R8` before the tone network, so what it injects is filtered by
    // the knob. `VR -> n3`, instead, does not depend on drive (−101 to −113 dB
    // across the travel), so `rn3_` is a fixed filter.
    RailN14Variable rn14_;
    // The rail's path into `n19` is parametric in the level knob: `VR -> n19`
    // changes 58x along the level pot (0,0128 -> 0,742 at 1 kHz, ngspice), and
    // since the rail enters through `R12`, after the pot, its error does not
    // attenuate with the knob. It comes from the same algebra as `E4G1`. The
    // fixed `rail::kN19` in `nls_rail_n3.h` is not used.
    RailN19Variable rn19_;
#if NLSC_VRA_PARAM
    RailVraVariable vra_;
#else
    FixedFilter<fixed::kVRA_NB, fixed::kVRA_NA> vra_;
#endif
    FixedFilter<fixed::kVRB_NB, fixed::kVRB_NA> vrb_;
    // `n19`'s BRANCH INTO THE RAIL IS A FILTER, NOT A CONSTANT. `n19` reaches
    // the rail through `R12` and also through `C8` into the level pot's `n18`,
    // so its admittance moves 15,7 dB between DC and the high band, while its
    // three passive siblings (`n3`, `n4`, `n11`) are flat. A scalar 1/R12
    // leaves the rail's DC 1,19 % low.
    FixedFilter<fixed::kVRC_NB, fixed::kVRC_NA> vrc_;
    FixedFilter<fixed::kVRP_NB, fixed::kVRP_NA> vrp_;

    Stage1 e1_;
#if NLSC_E1_PHYS
    Stage1Phys e1p_;
#endif
    Stage2 e2_;
    FixedFilter<fixed::kE2C_NB, fixed::kE2C_NA> c2_;
    Stage3 e3_;
    Stage4 e4_;
};

} // namespace nlsc
