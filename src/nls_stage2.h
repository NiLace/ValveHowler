// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// Stage 2 — gain and clipping (IC1A + D1/D2).
//
// DECOUPLING BY THE VIRTUAL GROUND
// --------------------------------
// With an ideal opamp (infinite gain, virtual ground) the input network, the
// feedback network and the clipper stop seeing each other, and `n7` is a
// voltage source: whatever stage 3 draws through `R7` does not change it.
// That is what lets stages 2 and 3 be solved separately without approximating
// anything beyond the opamp itself.
//
// The real opamp is an NJM4558: GBW 3 MHz, Avol 1e5. At max drive the
// closed-loop gain at 1 kHz is ≈69 (Rf = 551 kΩ against |Zin| ≈ 8 kΩ) and the
// open-loop gain there is 3000, so the loop gain is ≈43: a 2.3 % gain error
// (−33 dB) if the opamp is taken as ideal. The shipped path
// (`NLSC_E2_OPAMP_LIN`) therefore models the finite gain, the rail clipping
// and stage 3's load, and still solves a scalar equation.
//
// THE EQUATIONS (reference path, ideal opamp)
// -------------------------------------------
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
//   known, it falls out explicitly: it does not enter the Newton.
// - `RCM` (the opamp model's 500 MΩ) is required: without it the rest point
//   comes out with `v7 = v5`, while the circuit holds 4.65 mV between the
//   two, which would be 4.65 mV of DC fed into the chain.
//
// The diode model is the same object the DK engine uses (`mna::DModel` +
// `mna::Engine::diode`), junction charge included, with the same trapezoidal
// discretisation, so the two engines cannot differ by their device model.

#pragma once

// Cost switches, both on by default:
//
//   NLSC_E2_HOIST        hoist the per-sample constants into `hoist()`
//   NLSC_E2_CHARGE_CACHE reuse the last Newton diode evaluation for the charge
//
// Set either to 0 to opt out. `NLSC_E2_PAIR_EXP` evaluates the antiparallel
// pair with a single `diode_par` call; off by default.
// The defaults of these switches, and of the other `NLSC_E2_*` switches in
// this header block, are defined before their first use: an undefined macro
// inside `#if` silently reads as 0. `NLSC_E2_MAP_PROBE` has no default and so
// reads as 0 (off); the per-element alphas (`NLSC_E2_A_*`), `NLSC_E2_ITER_CAP`
// and `NLSC_E2_TOL_*` are defined inside the class, next to their use.
#ifndef NLSC_E2_PAIR_EXP
#define NLSC_E2_PAIR_EXP 0
#endif
#ifndef NLSC_E2_HOIST
#define NLSC_E2_HOIST 1
#endif

// STAGE 2 SOLVES BY TABLE (on by default).
//
// The Newton of this stage reduces algebraically to `P·w + Q·ID(w) = R'` with
// `P` and `Q` constant between knob changes, so `w` is a function of ONE
// scalar and can be tabulated. Four tables share one index — `w`, `ID(w)`,
// `q(w)`, `q(−w)` — with one warp and one division per sample, 131 kB, and the
// path evaluates no diode at all.
//
// Its difference against the Newton is −96.5 dB, and most of it is the
// Newton's own stopping error, not the table's: tightening the Newton
// tolerance reduces the difference, refining the table does not.
//
// The table needs `NLSC_E2_HOIST`: without the hoist, the coefficients `P` and
// `Q` do not exist (the `#error` below enforces it).
//
// Building it runs on the audio thread, so after a user gesture
// (`table_gesture(true)`) it waits until the knob has been still for
// `NLSC_E2_TABLE_WAIT` samples and is then built in ONE sample (at the default
// `NLSC_E2_TABLE_BINS_PER_SAMPLE`); until then `w` comes from the Newton.
// The all-at-once fill keeps the output independent of the host's block
// slicing and keeps a knob change continuous; its peak cost is paid once per
// gesture, not once per block.
#ifndef NLSC_E2_TABLE
#define NLSC_E2_TABLE 1      // on by default
#endif

// The table needs the hoist. This guard sits outside every conditional so it
// can fire whatever else is switched off.
#if (NLSC_E2_TABLE) && !NLSC_E2_HOIST
#error "NLSC_E2_TABLE needs NLSC_E2_HOIST: without the hoist, the table's P and Q coefficients do not exist. To build without the hoist, also pass -DNLSC_E2_TABLE=0."
#endif

// THE TABLE'S BOX. At 1 (the default) the table refuses to answer over its
// first and last intervals, whose end values are fixed rather than solved, and
// there falls back to the Newton. Without the box a sustained overload (e.g. a
// large square wave) pushes the index into those intervals, the table returns
// a constant, and the stage stays silent after the overload ends.
// At guitar level the box never fires, so it does not change normal output.
#ifndef NLSC_E2_TABLE_BOX
#define NLSC_E2_TABLE_BOX 1
#endif

// The table's resolution. Memory is 4 x (N+1) x 8 B = 131 kB at 4096; a finer
// grid does not reduce its difference against the Newton.
#ifndef NLSC_E2_TABLE_N
#define NLSC_E2_TABLE_N 4096
#endif
#ifndef NLSC_E2_CHARGE_E3
#define NLSC_E2_CHARGE_E3 0
#endif
// THE SLEW LIMIT in the scalar path: the macromodel's current-limited
// transconductance, `imax·tanh(gm·(x − v5)/imax)`, instead of `gm·(x − v5)`.
// With it `v5` is no longer an explicit function of `w`: it is solved by an
// inner Newton on a monotone scalar equation (derivative >= 1). The table
// assumes the linear law, so this path always takes the Newton.
//   1 = every sample (no table; needs a flat start, NLSC_E2_PREDICTOR=0).
//   2 = only the samples that need it: the sample is solved as usual (table,
//       predictor) and, if that LINEAR solution asks the opamp for more than
//       `NLSC_E2_SLEW_ON`·imax, it is solved again with the limit, from a flat
//       start and without the table. At guitar level it never fires, and the
//       cost is one comparison.
#ifndef NLSC_E2_SLEW
#define NLSC_E2_SLEW 2
#endif
#ifndef NLSC_E2_SLEW_ON
#define NLSC_E2_SLEW_ON 0.1
#endif
#ifndef NLSC_E2_CHARGE_CACHE
#define NLSC_E2_CHARGE_CACHE 1
#endif

// The tolerance each bin of the table is solved to. It is the same Newton and
// the same `ID_d` as the audio loop, which stops at `kTolAbs + kTolRel*|x|`
// = 1e-6 + 1e-5|x|; here it stops six orders tighter because the fill is paid
// once per knob gesture, not once per sample. The margin is large: the
// continuation step between bins only grows visibly at a loose tolerance
// (1e-2) on a coarse grid.
#ifndef NLSC_E2_TABLE_FILL_TOL
#define NLSC_E2_TABLE_FILL_TOL 1e-12
#endif

// OPTIONAL TIGHTER NEWTON TOLERANCE DURING THE WAIT ONLY.
//
// While the knob moves, the Newton solves and the table waits; this sets a
// separate tolerance (`NLSC_E2_WAIT_TOL_ABS/REL`) for that stretch only, so
// the Newton and the table agree more closely when the table takes over. It
// does not touch the tolerance of the steady-state path.
// Default 0 = the usual tolerance. The switch is an INTEGER separate from the
// values because `#if` only does integer arithmetic.
#ifndef NLSC_E2_WAIT_TOL
#define NLSC_E2_WAIT_TOL 0
#endif
#ifndef NLSC_E2_WAIT_TOL_ABS
#define NLSC_E2_WAIT_TOL_ABS 1e-9
#endif
#ifndef NLSC_E2_WAIT_TOL_REL
#define NLSC_E2_WAIT_TOL_REL 1e-8
#endif

// THE WAIT: NOTHING IS BUILT WHILE THE KNOB IS MOVING.
//
// `apply_knobs()` invalidates the table whenever a value changes, i.e. on
// every block while a host automation lane moves a knob; rebuilding it every
// block would cost more than having no table. While the knob moves the Newton
// solves; once it has been still for this many INTERNAL samples, the table is
// built in one go.
//
// The wait is counted in samples, not blocks, so when the table appears is a
// function of the sample index and does not depend on how the host slices.
// 2048 internal samples are 512 host samples at 4x (10.7 ms at 48 kHz):
// longer than any block below 512, so under automation the count never
// reaches zero. At exactly 512 it reaches zero on the block's last sample,
// and at 1024 once per block; there a whole fill (~0.5 ms) fits in the
// 21.3 ms period. During the wait the Newton runs, so no fidelity is lost;
// only the saving is delayed.
#ifndef NLSC_E2_TABLE_WAIT
#define NLSC_E2_TABLE_WAIT 2048   // on by default
#endif

// How many bins of the table are solved per sample once filling starts. The
// default is the whole table in one sample (see the table's description);
// smaller values spread the fill and lower the per-block peak.
#ifndef NLSC_E2_TABLE_BINS_PER_SAMPLE
#define NLSC_E2_TABLE_BINS_PER_SAMPLE (NLSC_E2_TABLE_N + 1)   // the whole table at once
#endif



// NEWTON SEED PREDICTOR.
//
//   0  the previous sample's `w`
//   1  linear    w = 2*w[n-1] - w[n-2]
//   2  quadratic w = 3*w[n-1] - 3*w[n-2] + w[n-3]   (default)
//
// Extrapolating costs two subtractions and two doubles of state per sample,
// against whole Newton passes of two diode evaluations each (an `exp` and a
// `pow` apiece). The converged root is unchanged to within the stopping
// tolerance, so this is not a model change, although it can move the last
// bits. The history is cleared at the end of `rest()`: otherwise the DC
// settling loop would be extrapolated into sample 0, and a history guard keeps
// the first samples from extrapolating from the DC solve.
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
        // In this companion form `h` only enters through the conductance, so it
        // costs nothing at run time: it is computed once here. With every alpha
        // at 1.0 the arithmetic is a multiply by one and the rule is the plain
        // trapezoidal, bit-identical.
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
    // `R8` and `R5`, so it follows the signal (about 10 % of `n5` at 98 Hz).
    // The branch that sets this stage's gain goes precisely to VR, so the rail
    // is an input, not a constant.
    void set_vr(double vr) { vr_ = vr; }

    // `n11` (the C3-R5 junction) contributes 2.0 % of the current that moves
    // the rail, and only this stage knows it.
    double n11() const { return n11_; }
    // Probes: the three nodes the stage solves (diagnostics).
    double probe_v5() const { return v5_; }
    double probe_v7() const { return v7_; }
    double probe_v2() const { return v2_; }

    // Input and output in ABSOLUTE VOLTAGE (volts to ground): `v5` is what
    // stage 1 puts on `n4`, and the return is `n7`.
    //
    // Build variants of the diode solve, separate so that the cost and the
    // fidelity of each can be attributed:
    //
    //   -DNLSC_E2_NO_RS      `RS` = 0 => the diodes' two internal nodes
    //                        vanish and the Newton goes from 3x3 to SCALAR.
    //   -DNLSC_E2_CHARGE_TT  removes ONLY the depletion (`CJO`), where the
    //                        single `pow` lives. Diffusion (`TT`) stays and
    //                        costs two multiplies, reusing the exponential
    //                        already computed.
    //   -DNLSC_E2_NO_CHARGE  removes the junction charge ENTIRELY (`CJO`
    //                        and `TT`) => one exponential per iteration
    //                        instead of two `diode()` calls, one state less.
    //
    // The pair is not solved in closed form (Lambert W / omega): that requires
    // dropping the subdominant exponential, which costs about −30.5 dB of
    // residual at max drive. The full `sinh` stays; only how it is solved
    // changes.
#if defined(NLSC_E2_OPAMP_LIN)
    // ---- FINITE-gain but LINEAR opamp: the SCALAR path (the shipped one) ----
    //
    // The full macromodel (the `NLSC_E2_OPAMP_REAL` path) adds two unknowns
    // and a 3x3 Newton. Its two nonlinearities are handled apart here: the rail
    // clipping inside `residual()`, and the slew limit by `NLSC_E2_SLEW` on the
    // samples that need it. Neither acts at guitar level; at booster level
    // (3 V of input peak) `n7` reaches the rail and slews at 0.8-0.9 V/µs
    // against the model's 1 V/µs.
    //
    // With their nonlinearity removed, the pole node solves by hand:
    //
    //     v2 = [gm·(x − v5) + s2] / Y2        (linear in v5)
    //     v7 = v2 + Ro·iF(w)                  (no clipping, v3 = v2)
    //  =>  v5 = [w + (gm·x + s2)/Y2 + Ro·iF(w)] / (1 + gm/Y2)
    //
    // i.e. `v5` is an EXPLICIT function of `w`, and `n5`'s KCL is again ONE
    // scalar equation. `slew_exercised()` counts the samples where the slew
    // limit would have acted.
    //
    // THE RESIDUAL'S BODY, IN ONE PLACE. The table path evaluates `F` at two
    // points — `w = 0`, which gives the index, and the `w` the table returns —
    // and the Newton at every iteration, all through this one function.
    //
    // `id` and `gd` arrive already evaluated because they are the only costly
    // part: the caller decides whether they come from `diode_par` or, at
    // `w = 0`, from state arithmetic — there the antiparallel pair sees the
    // same thing on both sides, so currents and charges cancel to EXACTLY ZERO
    // and no transcendental is needed.
    struct Body { double F, dF, v5; };

#if NLSC_E2_HOIST
    template <bool kSlew = false>
    inline Body residual(double w, double id, double gd,
                          double x, double c, double s2) const
#else
    template <bool kSlew = false>
    inline Body residual(double w, double id, double gd,
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

        // RAIL CLIPPING. Without it the linear model has no bound on the
        // output (a 10 V input peak would drive it to hundreds of kV). As in
        // the full model, `v3` (what `Ro` sees) is clipped, NOT `v2`, which
        // keeps integrating.
        //
        // With `v2` inside the rails:  v5 = k*(w + c + Ro*iF)
        // With `v2` outside:           v5 = w + Vsat + Ro*iF
        // Both are EXPLICIT in `w`, so the Newton stays scalar.
#if NLSC_E2_CHARGE_E3
        // WITH STAGE 3's LOAD. The current `n7` delivers is
        //     i_carga = Yeq*v7 - Ieq        (linear in v7)
        // so  v7*(1 - Ro*Yeq) = v2 + Ro*iF - Ro*Ieq, and `v5` stays
        // EXPLICIT in `w`: the scalar path is preserved.
        // Algebraic check: with Yeq = 0 it gives D = 1 and the formula below.
        v5 = kL_ * (w * D_ + c + kRO * iF + kRO * Ieq_);
        double dv5 = kL_ * (D_ + kRO * giF);
#else
        // Without stage 3's load: `Yeq_ = 0`, so `D_ = 1` and `Ieq_ = 0`, and
        // the branch above reduces literally to this one.
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

#if NLSC_E2_SLEW
        if (NLSC_E2_SLEW == 1 || kSlew) {
            // v7·D = v3 + RO·(iF + Ieq) and v7 = v5 − w  =>  h(v5) = 0 with
            //   h(v5) = v5 − w − [v3(v5) + RO·(iF + Ieq)] / D
            // v3 is the pole node clipped to the rails, and v2 falls with v5,
            // so h' = 1 − v3'/D >= 1. Started from the linear solution above.
#if NLSC_E2_HOIST
            const double iY2 = invY2_;
#else
            const double iY2 = 1.0 / Y2;
#endif
#if NLSC_E2_CHARGE_E3 && NLSC_E2_HOIST
            const double invDd = invD_, Ie = Ieq_;
#elif NLSC_E2_CHARGE_E3
            const double invDd = 1.0 / D_, Ie = Ieq_;
#else
            const double invDd = 1.0, Ie = 0.0;
#endif
            const double base = w + kRO * (iF + Ie) * invDd;
            double u = v5, hp = 1.0;
            for (int k = 0; k < kSlewIt; ++k) {
                const double t  = std::tanh(gm_ * (x - u) * inv_imax_);
                const double v2 = (imax_ * t + s2) * iY2;
                double v3 = v2, dv3 = -gm_ * (1.0 - t * t) * iY2;
                if (v2 <= kVeeSat)      { v3 = kVeeSat; dv3 = 0.0; }
                else if (v2 >= kVccSat) { v3 = kVccSat; dv3 = 0.0; }
                hp = 1.0 - dv3 * invDd;
                const double du = -(u - base - v3 * invDd) / hp;
                u += du;
                if (std::fabs(du) < 1e-14 + 1e-13 * std::fabs(u)) break;
            }
            v5  = u;
            dv5 = (1.0 + kRO * giF * invDd) / hp;
        }
#endif
        const double i_leg = Gleg_ * (v5 - vr_ - cv3_)
                           - Gleg_ * invG3_ * ci3_;
        const double F  = iF + i_leg + v5 * kInvRCM + (v5 - x) * kInvRIN;
        const double dF = giF + (Gleg_ + kInvRCM + kInvRIN) * dv5;
        return Body{F, dF, v5};
    }
#if NLSC_E2_SLEW == 2
    // The hot sample: the same Newton with the slew law, from `w_flat`, no
    // table. Out of line on purpose. The diode evaluation mirrors the main
    // loop's switches, so every control build of those switches keeps
    // compiling AND keeps the slew.
#if NLSC_E2_CHARGE_CACHE && !defined(NLSC_E2_NO_CHARGE)
#  define NLSC_E2_HOT_CACHE 1
#else
#  define NLSC_E2_HOT_CACHE 0
#endif
    __attribute__((noinline))
    bool solve_hot(double x, double c, double s2, double w_flat,
                   double& w, double& v5
#if NLSC_E2_HOT_CACHE
                   , mna::Engine::DOut& o1u, mna::Engine::DOut& o2u
#endif
                   ) const
    {
#if !NLSC_E2_HOIST
        const double Y2 = 1.0 / Rp_ + Gcp_;
#endif
        w = w_flat;
        for (int it = 0; it < kMaxIt; ++it) {
            double id, gd;
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
#if NLSC_E2_HOT_CACHE
            o1u = o1; o2u = o2;
#endif
            id = (o1.i + g2h_ * (o1.q - dq1_) - di1_)
               - (o2.i + g2h_ * (o2.q - dq2_) - di2_);
            gd = (o1.g + g2h_ * o1.c) + (o2.g + g2h_ * o2.c);
#endif
#if NLSC_E2_HOIST
            const Body cb = residual<true>(w, id, gd, x, c, s2);
#else
            const Body cb = residual<true>(w, id, gd, x, c, s2, Y2);
#endif
            v5 = cb.v5;
            const double dw = -cb.F / cb.dF;
            const double tol = kTolAbs + kTolRel * std::fabs(x);
            const double wn = w + dw;
            w = (wn >= 0.0) ?  mna::Engine::pnjlim( wn,  w, nvtD_, vcritD_)
                            : -mna::Engine::pnjlim(-wn, -w, nvtD_, vcritD_);
            if (std::fabs(dw) < tol) return true;
        }
        return false;
    }
#endif
    double process(double x)
    {
#if NLSC_E2_HOIST
        // `Y2` and the other per-knob constants are hoisted (see `hoist()`).
        const double s2 = Gcp_ * cvcp_ + icp_;
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
        const double s5 = G5_ * cv5c_ + ci5c_;
#if NLSC_E2_HOIST
        Ieq_ = G7_ * (s5 + vr_ * G8_) * invGsum_;   // `Yeq_`, `D_` and `kL_`: in prepare()
#else
        Yeq_ = G7_ * (1.0 - G7_ / Gsum_);
        Ieq_ = G7_ * (s5 + vr_ * G8_) / Gsum_;
#endif
        // THE SIGN: `iF` is defined as current ENTERING `n7`
        // (`v7 = v2 + Ro*iF`), and the load DRAWS current => subtract.
#if !NLSC_E2_HOIST
        D_   = 1.0 + kRO * Yeq_;
        kL_  = 1.0 / (D_ + gm_ / Y2);
#endif
#endif
        double w = v5_ - v7_;
#if NLSC_E2_SLEW == 2
        // The flat start for a hot sample is the current state, not the
        // predictor's guess: with the opamp slewing, `v5` follows `w` and a
        // quadratic guess lands where the diodes carry amperes and `pnjlim`
        // runs out of iterations.
        const double w_flat = w;
#endif
#if NLSC_E2_PREDICTOR >= 1
        // High order ONLY with enough history: a quadratic started from two
        // samples (one of them the rest point) throws the seed far away and
        // costs iterations instead of saving them. The DK engine carries the
        // same guard.
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
        bool converged = false;

#if NLSC_E2_CHARGE_CACHE && !defined(NLSC_E2_NO_CHARGE)
        // THE CHARGE IS CACHED FROM THE LAST NEWTON STEP.
        //
        // The state update needs `q(w)` and `q(−w)`, and each `diode()` call
        // is one `exp` and one `pow` (the depletion). The Newton has already
        // evaluated them; the only difference is that its `w` is the one
        // before the last `dw`, and the loop exits when `|dw| < tol`. At
        // 1e-6 abs and 1e-5 rel, over a charge of a few pF, that is ~1e-17 C,
        // far below what the solver accepts as converged. This margin is set by
        // the tolerance: if the tolerance is relaxed, re-examine it.
        mna::Engine::DOut o1u{}, o2u{};
#endif
#if NLSC_E2_TABLE
        // THE INDEX, WITHOUT A SINGLE TRANSCENDENTAL.
        //
        // The residual at `w = 0` IS `-R'`, the table's index. At `w = 0` the
        // two diodes of the ANTIPARALLEL pair see exactly the same thing, so in
        // the subtraction that forms `id` their currents and their charges
        // cancel to EXACTLY ZERO in floating point (`e = 1` => `IS*(1-1) = 0`;
        // and with the constant depletion, `q = cj0*0 = 0`). All that survives
        // is the STATE term, which is arithmetic already done.
        //
        // It is written with the SAME order of operations as the body: the
        // leading `0.0 +` and `0.0 -` stand for the `o1.i` and `o1.q` that
        // cancel, so the result is bit-identical to evaluating the body at 0.
        // The rest of `F` comes from `residual()`, the same text the Newton
        // runs.
        //
        // `gd = 0` because it only feeds `dF`, and here `dF` is discarded: the
        // table does not take a Newton step, it returns the solved `w`.
        // The fill advances BEFORE it is used, so the schedule is a function of
        // the sample index and not of the host's slicing.
        advance_fill_per_sample();
#if NLSC_E2_SLEW == 1
        bool uses_table = false;
#else
        bool uses_table = tb_ready_;
#endif
        if (uses_table) {
            const double id0 = (0.0 + g2h_ * (0.0 - dq1_) - di1_)
                             - (0.0 + g2h_ * (0.0 - dq2_) - di2_);
            const Body c0 = residual(0.0, id0, 0.0, x, c, s2);
            // `id` is recomposed as `ID(w) + id0`: the first term is a function
            // of `w` alone and comes from the table, the second is the state
            // term computed above. The decomposition is algebraically exact —
            // the body's own subtraction regrouped — but not bit-identical,
            // because the order of the additions changes.
            bool in_box_ = false;
            const Tab tb = table_at(-c0.F, in_box_);
            if (in_box_ || !NLSC_E2_TABLE_BOX) {
                w         = tb.w;
                id_table_ = tb.ID + id0;
                q1_table_ = tb.q1;
                q2_table_ = tb.q2;
            } else {
                // Outside the box it does NOT answer: it falls back to the
                // Newton, which is still compiled and is the same fallback
                // stage 4's cache uses.
                uses_table = false;
                ++out_of_box_;
            }
        }
#endif
#if NLSC_E2_TABLE && NLSC_E2_TABLE_WAIT > 0 && NLSC_E2_WAIT_TOL
        const bool waiting_ = (tb_wait_ > 0);
        const double tol_abs_ = waiting_ ? kEspTolAbs : kTolAbs;
        const double tol_rel_ = waiting_ ? kEspTolRel : kTolRel;
#endif
        for (int it = 0; it < kMaxIt; ++it) {
            double id, gd;
#if NLSC_E2_TABLE
            // THE TABLE PATH DOES NOT EVALUATE THE DIODE: `id` arrives
            // recomposed from above and both charges are already set. `gd` only
            // feeds `dF`, which this path discards.
            // The choice is made at RUN TIME: while the table is not ready, or
            // outside its box, the Newton below solves — the same fallback
            // stage 4's cache uses when its point falls outside its box.
            if (uses_table) {
            id = id_table_;
            gd = 0.0;
#if NLSC_E2_CHARGE_CACHE
            o1u.q = q1_table_;
            o2u.q = q2_table_;
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
#if NLSC_E2_TABLE
            }
#endif
#if NLSC_E2_HOIST
            const Body cb = residual(w, id, gd, x, c, s2);
#else
            const Body cb = residual(w, id, gd, x, c, s2, Y2);
#endif
            const double F = cb.F, dF = cb.dF;
            v5 = cb.v5;
#if NLSC_E2_TABLE
            (void)dF;
            // The iteration counter is incremented on this path too, so the
            // invariant "bin 0 stays at zero, because the loop runs at least
            // once" holds for every path. The table takes ONE pass of the body.
            if (uses_table) {
                (void)it;
                converged = true;
                break;                                    // everything below is already set
            }
#endif
            const double dw = -F / dF;
#if NLSC_E2_TABLE && NLSC_E2_TABLE_WAIT > 0 && NLSC_E2_WAIT_TOL
            // While the table waits for the knob to stop, the Newton is the
            // only thing solving: there it can be asked for more, because the
            // price is paid per GESTURE and not per sample. `tb_wait_` does
            // not change inside the loop, so the selection is hoisted out of
            // it.
            const double tol = tol_abs_ + tol_rel_ * std::fabs(x);
#else
            const double tol = kTolAbs + kTolRel * std::fabs(x);
#endif
            // THE LIMITER IS SYMMETRIC.
            //
            // `pnjlim` is SPICE's junction limiting and bounds the FORWARD
            // excursion. Here the diodes are an ANTIPARALLEL pair and the return
            // one sees `-w`, so the limit is applied on whichever side `w`
            // goes. Limiting only `w` would leave the negative half
            // unprotected: above ~3 V of input peak `w` would run away towards
            // −4.65 V, impossible across two silicon diodes, and the Newton
            // would stop converging.
            const double wn = w + dw;
            w = (wn >= 0.0) ?  mna::Engine::pnjlim( wn,  w, nvtD_, vcritD_)
                            : -mna::Engine::pnjlim(-wn, -w, nvtD_, vcritD_);
            if (std::fabs(dw) < tol) { converged = true; break; }
        }
#if NLSC_E2_SLEW == 2
        // The LINEAR solution's pole-node current is what the real opamp
        // would have to deliver; past a fraction of `imax` the tanh is no
        // longer the identity and the sample is solved again with it, out of
        // line, so the path every guitar-level sample takes stays as it was.
        slew_on_ = false;
        if (__builtin_expect(std::fabs(gm_ * (x - v5)) > NLSC_E2_SLEW_ON * imax_, 0)) {
            slew_on_ = true;
            if (slew_hot_++ == 0) slew_first_ = samples_;
#if NLSC_E2_HOT_CACHE
            converged = solve_hot(x, c, s2, w_flat, w, v5, o1u, o2u);
#else
            converged = solve_hot(x, c, s2, w_flat, w, v5);
#endif
        }
#endif
        if (!converged) ++unconverged_count_;
        if (std::fabs(w) > std::fabs(w_ext_)) w_ext_ = w;
        ++samples_;
        v5_ = v5;
        v7_ = v5 - w;
#if NLSC_E2_HOIST
#if NLSC_E2_SLEW
        if (NLSC_E2_SLEW == 1 || slew_on_)
            v2_ = (imax_ * std::tanh(gm_ * (x - v5) * inv_imax_) + s2) * invY2_;
        else
            v2_ = (gm_ * (x - v5) + s2) * invY2_;
#else
        v2_ = (gm_ * (x - v5) + s2) * invY2_;
#endif
#elif NLSC_E2_SLEW
        if (NLSC_E2_SLEW == 1 || slew_on_)
            v2_ = (imax_ * std::tanh(gm_ * (x - v5) * inv_imax_) + s2) / Y2;
        else
            v2_ = (gm_ * (x - v5) + s2) / Y2;
#else
        v2_ = (gm_ * (x - v5) + s2) / Y2;
#endif

        // THESE TWO PROBES COUNT, THEY DO NOT BOUND: they flag samples where
        // this APPROXIMATION is outside its range.
        //
        // This path's core is a LINEAR opamp. The reference 3x3 path carries
        // the two nonlinearities that hold it in — the current-limited
        // transconductance, `imax*tanh(gm*(x-v5)/imax)`, and the clip against
        // the rails, `v3 = clamp(v2, kVeeSat, kVccSat)`. Here the slew law is
        // applied only on hot samples (`NLSC_E2_SLEW`), and the rail clip acts
        // on `v3`, not on `v2`. With the table OFF and no slew law engaged,
        // nothing holds `v2` under a sustained overload: it runs to hundreds of
        // volts and then to NaN, and since the internal state is then
        // non-finite, the output stays at zero for good.
        //
        // The TABLE also bounds this regime: it returns `w` from a grid over
        // the warp `u = R'/(S+|R'|)`, bounded by construction, so on the same
        // overload `max|v7|` stays at a few volts and there is no NaN. Do not
        // retire the table assuming this regime is covered elsewhere.
        // GUARD: would the slew limit have acted?
        if (std::fabs(gm_ * (x - v5)) > imax_) ++slew_;
        // GUARD: did the output HARD CLAMP against the rails fire? The clamp is
        // a discontinuity, and a discontinuity has unbounded harmonic order, so
        // whether it fires at all matters for aliasing.
        {
#if NLSC_E2_HOIST
#if NLSC_E2_SLEW
            const double v2p = v2_;
#else
            const double v2p = (gm_ * (x - v5) + s2) * invY2_;
#endif
#elif NLSC_E2_SLEW
            const double v2p = v2_;
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

    long slew_exercised() const { return slew_; }
    long sat_exercised() const { return sat_; }
#elif defined(NLSC_E2_OPAMP_REAL)
    // ---- path with the REAL OPAMP inside the loop (3x3) ----
    //
    // The ideal opamp's error scales with the closed-loop gain the drive pot
    // sets and depends on level, so a post-filter cannot fix it: the finite
    // gain must live INSIDE the solve.
    //
    // The whole NJM4558 macromodel is used, the SAME one the DK engine has:
    // `Rin`/`Rcm` at the input, current-LIMITED transconductance (the slew
    // mechanism), dominant pole `Rp`‖`Cp`, rail clipping and `Ro` at the
    // output. Unknowns: `v5`, `v7` and the pole's internal node.
    //
    // OMITTED: the load stage 3 hangs off `n7`. The feedback taps AFTER `Ro`,
    // so the loop corrects that drop up to the loop gain: 75 Ω × (v/1.7 kΩ)
    // ÷ T≈31 => ~−57 dB; the loop gain falls with frequency, so the error
    // grows towards the top of the band. The load is LINEAR and known; the
    // scalar path includes it (`NLSC_E2_CHARGE_E3`).
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
            const double iF  = w / Rf_ + iC4 + id;          // current n5 -> n7
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
            const bool   inside = (v2_ > kVeeSat && v2_ < kVccSat);
            const double v3 = inside ? v2_ : (v2_ <= kVeeSat ? kVeeSat : kVccSat);

            const double F1 = iF + i_leg + v5_ * kInvRCM + (v5_ - x) * kInvRIN;
            const double F2 = v2_ / Rp_ + iCp - ign;
            const double F3 = (v7_ - v3) * kInvRO - iF;

            const double J11 = giF + Gleg_ + kInvRCM + kInvRIN, J12 = -giF, J13 = 0.0;
            const double J21 = dign, J22 = 0.0, J23 = 1.0 / Rp_ + Gcp_;
            const double J31 = -giF, J32 = kInvRO + giF, J33 = inside ? -kInvRO : 0.0;

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
    // Netlist values: the circuit's components.
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
    // It does NOT include the TONE network (RTONEA/RTONEB, >= 20 k): at high
    // frequency `C5` dominates the load.
    // Off by default in this header; enabled with -DNLSC_E2_CHARGE_E3=1, which
    // the product build sets.
    static constexpr double kR7  = 1.0e3;
    static constexpr double kC5  = 0.22e-6;
    static constexpr double kR8  = 10.0e3;
    // ONE definition, shared with the DK engine (`NLSC_DIODE_RS`), so the two
    // cannot drift onto different diode parameters.
    static constexpr double kRS  = NLSC_DIODE_RS;  // the MA150's RS
    static constexpr double kInvRS  = 1.0 / kRS;
    static constexpr double kInvRCM = 1.0 / 500e6; // macromodel Rcm2
    static constexpr double kInvRIN = 1.0 / 5e6;   // Rin between the inputs
    // `NLSC_E2_RO0` makes `Ro` ≈ 0: a DISCRIMINATOR, not a model. With a rigid
    // output the load stage 3 hangs off `n7` cannot matter, which isolates
    // that load's contribution. (The loop gain is 2140 at 1 kHz but 8.2 at
    // 5 kHz, where the drop across `Ro` is about −42 dB.)
#if defined(NLSC_E2_RO0)
    static constexpr double kInvRO  = 1.0 / 1e-3;
#else
    static constexpr double kInvRO  = 1.0 / 75.0;  // macromodel Ro
#endif
    static constexpr double kVccSat = 9.0 - 1.5;   // clipping against the rails
    static constexpr double kVeeSat = 0.0 + 1.5;
    // THREE JUNCTION-CHARGE MODES, plus none. The charge's two terms do NOT
    // cost the same: DEPLETION carries a `pow` (the most expensive
    // transcendental here) and DIFFUSION is two multiplies reusing the
    // exponential already computed. They are separate switches because one is
    // mainly a COST lever and the other a FIDELITY lever.
    //   (nothing)               -> 1, depletion + diffusion (default)
    //   -DNLSC_E2_CHARGE_TT     -> 2, diffusion ONLY: saves the `pow`
    //   -DNLSC_E2_NO_DIFFUSION  -> 3, depletion only
    //   -DNLSC_E2_NO_CHARGE     -> 0, none
    //
    // DIFFUSION IS IN BY DEFAULT. The diffusion capacitance is proportional to
    // the diode current, so it exists only while the diodes conduct and it
    // acts near Nyquist: the residual barely sees it, but without it this stage
    // adds about 4.8 dB of alias at 4x (pinched-harmonic ANMR −10.9 dB instead
    // of −15.3). The other modes are for measurement only; modes 0 and 2 cost
    // 15.2 dB.
#if defined(NLSC_E2_NO_CHARGE)
    static constexpr int    kCharge = 0;   // none — costs 15.2 dB
#elif defined(NLSC_E2_CHARGE_TT)
    static constexpr int    kCharge = 2;   // diffusion only — costs 15.2 dB
#elif defined(NLSC_E2_NO_DIFFUSION)
    static constexpr int    kCharge = 3;   // depletion only
#else
    static constexpr int    kCharge = 1;   // DEFAULT: depletion + diffusion
#endif
#if NLSC_E2_PREDICTOR >= 1
    // Predictor history. `n_pred_` counts how many VALID samples sit behind:
    // without that guard the first extrapolation leaves from rest.
    double wp1_ = 0.0, wp2_ = 0.0;
    int    n_pred_ = 0;
#endif
    // Convergence-probe counters (diagnostics).
    long   unconverged_count_ = 0, samples_ = 0;
    double w_ext_ = 0.0;
public:
    long   unconverged() const   { return unconverged_count_; }
    long   samples() const  { return samples_; }
    double w_extreme() const { return w_ext_; }
    void   reset_probe()
    {
        unconverged_count_ = samples_ = 0; w_ext_ = 0.0;
    }
private:
// THE ITERATION CAP. The Newton must converge: a cap too low to converge
// (e.g. 1) leaves the stage silent, and silence is finite, so a NaN check
// does not catch it.
#ifndef NLSC_E2_ITER_CAP
#  define NLSC_E2_ITER_CAP 40
#endif
    static constexpr int    kMaxIt = NLSC_E2_ITER_CAP;
    // THE STOPPING CRITERION: 1e-6 V absolute + 1e-5 relative.
    //
    // The tolerance and the seed predictor are a PAIR: the quadratic seed lands
    // the first step nearer the root, which is what lets the tolerance be this
    // loose without loss, and changing one re-opens the other.
    // A badly resolved solver is paid in ALIAS before it is paid in residual,
    // so the tolerance is judged on the ANMR, not on the residual; a looser
    // setting (1e-4/1e-3) exceeds the −60 dB porting budget.
    //
    // Overridable, so the sweep stays reproducible without editing this file.
#ifndef NLSC_E2_TOL_ABS
#define NLSC_E2_TOL_ABS 1e-6
#endif
#ifndef NLSC_E2_TOL_REL
#define NLSC_E2_TOL_REL 1e-5
#endif
    static constexpr double kTolAbs = NLSC_E2_TOL_ABS;
    static constexpr double kTolRel = NLSC_E2_TOL_REL;


#if NLSC_E2_HOIST
    // HOISTED OUT OF THE PER-SAMPLE LOOP.
    //
    // `Y2`, `k`, `Yeq_`, `D_` and `kL_` depend only on `Rp_`, `Gcp_`, `gm_`,
    // `Rf_` and `G5_`/`G7_`/`G8_`. The divisions by `Y2`, `Gsum_` and `Rf_`
    // are by constants too, so they become precomputed reciprocals.
    // Multiplying by the reciprocal is NOT bit-identical to dividing (1 ULP).
    //
    // WHY THIS IS A METHOD AND NOT A BLOCK INSIDE `prepare()`.
    // `rest()` deliberately zeroes `Gcp_` (and `G4_`, `Gleg_`, `g2h_`) to
    // solve the DC operating point, then restores them. A value hoisted once in
    // `prepare()` would not see that temporary change, so the quiescent point
    // would be solved with the wrong `Y2` and the plugin would start from a
    // wrong state (the steady state is unaffected; the first 0.1 s is not).
    // => every writer of an input must call `hoist()` afterwards.
#if NLSC_E2_TABLE
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

#if NLSC_E2_TABLE
    // THE IMPLICIT-MAP TABLE.
    //
    // The saving grows with drive: the table costs a constant while the Newton
    // iterates more the harder the diode conducts.
    //
    // Building it runs on the AUDIO THREAD (`apply_knobs()` calls `prepare()`
    // on every control change), which is why the fill waits for the knob to
    // stop and can be deferred; meanwhile the Newton solves, as stage 4's cache
    // falls back to its solver while it fills.
    //
    // The coefficients are DERIVED from the `dF` the loop itself computes:
    //     dF = giF + G*kL_*(D_ + kRO*giF),  with giF = invRf_ + G4_ + gd
    //        = [invRf_+G4_ + G*kL_*(D_ + kRO*(invRf_+G4_))] + gd*[1 + G*kL_*kRO]
    // i.e. `dF = P + Q*gd`. An independent fit of the iteration pairs at drive
    // 1.0 gives `P = 2.591505e-05`, `Q = 1.000315`, which this derivation must
    // reproduce.
    static constexpr int kTableN = NLSC_E2_TABLE_N;
    static constexpr long kWait = NLSC_E2_TABLE_WAIT;
    // The fill solves from a seed by CONTINUATION, so it converges in a few
    // steps; the cap is generous and derived from the audio Newton's cap.
    static constexpr int kMaxItFill = 8 * NLSC_E2_ITER_CAP;
    static constexpr double kEspTolAbs = NLSC_E2_WAIT_TOL_ABS;
    static constexpr double kEspTolRel = NLSC_E2_WAIT_TOL_REL;
    double table_[kTableN + 1] = {0.0};
    // THE OTHER THREE TABLES — ONE SINGLE INDEX.
    //
    // With `w` alone the body would still evaluate `diode_par(w)` to get
    // `id(w)` — which enters `iF` — and the two charges `q(w)`, `q(-w)`, which
    // are the NEXT sample's state. These three remove that last exponential.
    //
    // They share `table_`'s index: `w` is a function of `R'`, so everything
    // that is a function of `w` is a function of `R'`, and the warp, the
    // division and the bin are computed ONCE. Four contiguous reads per sample
    // against one exponential and one division.
    //
    // Memory: 4 x 4097 x 8 B = 131 kB, which is L2 and NOT L1.
    double tID_[kTableN + 1] = {0.0};   // ID(w): the part of `id` depending on w alone
    double tq1_[kTableN + 1] = {0.0};   // q(+w)
    double tq2_[kTableN + 1] = {0.0};   // q(-w)
    double tP_ = 0.0, tQ_ = 0.0, tS_ = 1.0;
    long out_of_box_ = 0;      // samples the table REFUSED to answer
    long fill_unconverged_ = 0; // bins that hit the cap without converging
#if NLSC_E2_TABLE
    // What the table hands a sample's loop. Members rather than locals because
    // the loop that consumes them sits behind several `#if`s, and a local
    // declared outside them would be unused in the other configurations.
    double id_table_ = 0.0, q1_table_ = 0.0, q2_table_ = 0.0;
#endif
#if NLSC_E2_TABLE
    // THE DEFERRED FILL — the cursor's state.
    //
    // The fill advances PER SAMPLE, not per block: advancing per block, how
    // many bins are done at sample `n` would depend on how the host SLICES,
    // and block invariance would break; per sample, the schedule is a function
    // of the SAMPLE INDEX.
    //
    // While the table is incomplete `w` comes from the NEWTON, which is still
    // compiled — just as stage 4's cache falls back to the solver when its
    // point lands outside its box.
    bool   tb_ready_ = false;   // can it be read yet?
    bool   tb_pending_  = false;   // is there fill left to do?
    bool   tb_deferred_   = false;   // deferred? (callers with no block boundary turn it off)
    int    tb_b_     = 0;       // the bin the cursor is on
    double tb_wprev_ = 0.0;     // the continuation seed, BETWEEN pieces
    long   tb_wait_ = 0;      // internal samples of a STILL knob still to go
    // IT ONLY WAITS IF THE RE-TUNE WAS ASKED FOR BY THE USER, and that fact is
    // not guessed here: the CALLER has it. `apply_knobs()` already tells the
    // two reasons for arriving apart (`reinit`). During set-up there is no
    // gesture, so there is no wait: the table is left pending and filled by
    // the per-sample advance (the whole table on the first sample at the
    // default `NLSC_E2_TABLE_BINS_PER_SAMPLE`), or drained immediately if the
    // fill is not deferred. It cannot be inferred here because
    // `prepare()`/`rest()` call `hoist()` several times.
    // Stage 4's cache compares a DEFERRED fill against an ALL-AT-ONCE one, so
    // any deferral that survives set-up would break that equivalence.
    bool   tb_gesture_ = false;
#endif

public:
    double table_P() const { return tP_; }
    double table_Q() const { return tQ_; }
private:

    // ARMS the cursor: computes the coefficients and leaves the table UNBUILT.
    // It fills nothing if the fill is deferred. If it is not — callers that
    // drive the stage directly and have no block boundary to hang the advance
    // on — it drains the whole thing right here, which is the SAME work in the
    // SAME order with the same seed, so the table comes out identical.
    void build_table()
    {
        const double G = Gleg_ + kInvRCM + kInvRIN;
        tP_ = (invRf_ + G4_) + G * kL_ * (D_ + kRO * (invRf_ + G4_));
        tQ_ = 1.0 + G * kL_ * kRO;
        // Warp scale: the `R'` at which `w` reaches about one kT/q, which is
        // where the curve stops being linear. It only SPACES the bins; the
        // diode equations use `mna::VT`.
        // The table is built from `residual()`'s NON-saturating branch. It
        // stays valid because the slew re-solve (`NLSC_E2_SLEW`, threshold
        // `NLSC_E2_SLEW_ON`·imax) takes the sample out of the table before
        // the op-amp reaches its rails: raising that threshold, or switching
        // the slew off, lets the table answer for an equation it was not
        // built from, with no error.
        const double wkt = 0.02585;
        tS_ = std::fabs(tP_ * wkt + tQ_ * ID(wkt));
        if (!(tS_ > 0.0)) tS_ = 1e-9;
        // Re-arming DISCARDS whatever half table there was: its bins were
        // solved with the OLD coefficients. `build_cache()` does the same.
        tb_b_     = 0;
        tb_wprev_ = 0.0;
        tb_ready_ = false;
        // Not deferred: drained right here.
        if (!tb_deferred_) { tb_pending_ = true; tb_wait_ = 0;
                        while (!advance_table(kTableN + 1)) { } return; }
        // With the wait armed, this re-arm leaves NOTHING pending: it only
        // RESTARTS the count. While the knob moves, every block comes through
        // here and the count never reaches zero, so NOTHING is built.
        // With no wait armed, the table is left pending and the per-sample
        // advance fills it at its own pace; the `else` must not drain, or a
        // spread-out fill would become an all-at-once one.
        if (kWait > 0 && tb_gesture_) { tb_pending_ = false; tb_wait_ = kWait; }
        else                          { tb_pending_ = true;  tb_wait_ = 0; }
    }

    // Solves at most `bins` bins and remembers where it was. Returns true if
    // this call left the table FINISHED.
    // The continuation seed survives the cut because `tb_wprev_` travels
    // between pieces: cutting it in half costs nothing, the previous `w` just
    // has to come along — like stage 4's cache cursor.
    bool advance_table(int bins)
    {
        if (!tb_pending_) return false;
        while (bins-- > 0) {
            const int b = tb_b_;
            const double u  = -1.0 + 2.0 * double(b) / double(kTableN);
            const double au = std::fabs(u);
            double w;
            if (au >= 1.0) {
                w = (u > 0.0) ? 1.0 : -1.0;
            } else {
                const double Rp = tS_ * u / (1.0 - au);
                // CONTINUATION SEED. The bins are walked in increasing `u` and
                // `w(R')` is monotone, so the previous bin's solution is one
                // step from this one.
                w = tb_wprev_;
                // Non-convergences are COUNTED: an unconverged bin would store
                // a bad `w` that is then read for the whole gesture with no
                // signal at all.
                int it = 0;
                for (; it < kMaxItFill; ++it) {
                    // ANALYTIC derivative, not numerical. `diode` already
                    // returns `g` (di/dv) and `c` (dq/dv), and it is the SAME
                    // expression the audio loop calls `gd`: one evaluation of
                    // the pair per iteration and no finite-difference step.
                    double f_id, d_id;
                    ID_d(w, f_id, d_id);
                    const double f = tP_ * w + tQ_ * f_id - Rp;
                    const double d = tP_ + tQ_ * d_id;
                    if (!(std::fabs(d) > 0.0)) break;
                    double step = -f / d;
                    if (step >  0.05) step =  0.05;
                    if (step < -0.05) step = -0.05;
                    w += step;
                    if (std::fabs(step) < NLSC_E2_TABLE_FILL_TOL) break;
                }
                if (it >= kMaxItFill) ++fill_unconverged_;
                tb_wprev_ = w;
            }
            table_[b] = w;
            // The three companions are filled IN THE SAME step as their `w`,
            // not in a second pass: if the fill is cut in half, half a table
            // with `w` set and `ID`/`q` at zero is exactly a table that does not
            // fail and answers wrongly.
            tID_[b] = ID(w);
            tq1_[b] = mna::Engine::diode(d_, jd_,  w, kCharge).q;
            tq2_[b] = mna::Engine::diode(d_, jd_, -w, kCharge).q;
            if (++tb_b_ > kTableN) {
                tb_pending_  = false;
                tb_ready_ = true;
                return true;
            }
        }
        return false;
    }

public:
    // The PER-SAMPLE advance. `process()` calls it, not the host: that way the
    // schedule is a function of the SAMPLE INDEX and not of the host's
    // slicing.
    void advance_fill_per_sample()
    {
        if (!tb_deferred_) return;
        // The still-knob countdown: one decrement per sample while the user
        // turns, and that is ALL that is paid in that regime.
        if (tb_wait_ > 0) {
            if (--tb_wait_ == 0) { tb_b_ = 0; tb_wprev_ = 0.0; tb_pending_ = true; }
            else return;
        }
        if (tb_pending_) advance_table(NLSC_E2_TABLE_BINS_PER_SAMPLE);
    }
    void table_deferred(bool v) { tb_deferred_ = v; }
    // Armed by whoever KNOWS why the re-tune happened, not by this stage.
    void table_gesture(bool v) { tb_gesture_ = v; }
    // During the WAIT there is nothing "pending" in the cursor and the table
    // does not exist either: returning false there would read as "already
    // ready".
    bool table_pending() const { return tb_pending_ || tb_wait_ > 0; }
    // Observability: "the box never fires in normal use" is CHECKABLE.
    long table_out_of_box() const { return out_of_box_; }
    long table_fill_unconverged() const { return fill_unconverged_; }
    void table_out_of_box_reset() { out_of_box_ = 0; fill_unconverged_ = 0; }
    // Observability: how many bins are left, so that "the table comes back" is
    // CHECKABLE and not an assumption.
    long table_bins_left() const { return tb_pending_ ? long(kTableN + 1 - tb_b_) : 0; }
private:

    // The four quantities that depend on `w`, with ONE warp, ONE division and
    // ONE bin. `id` does not leave here complete: it is missing its STATE term,
    // which the caller adds (the same `id0` as the index).
    struct Tab { double w, ID, q1, q2; };
    inline Tab table_at(double Rp, bool& in_box_) const
    {
        const double u = Rp / (tS_ + std::fabs(Rp));      // ONE division, no transcendentals
        double t = (u + 1.0) * 0.5 * double(kTableN);
        int b = int(t); if (b < 0) b = 0; if (b >= kTableN) b = kTableN - 1;
        // THE TABLE'S BOX.
        //
        // `advance_table` sets `w = ±1` at bins 0 and N instead of solving
        // them (`if (au >= 1.0)`), so the first and last intervals carry a
        // fixed value that does not depend on `R'`. After a sustained overload
        // the state can push `R'` into that stretch; reading a constant there
        // cannot bring the engine back, and the stage would stay silent.
        //
        // => Outside the box it SOLVES with the usual Newton, which is what
        // stage 4's cache does when its point falls outside. At guitar level
        // it never fires.
        in_box_ = (b > 0 && b < kTableN - 1);
        const double f = t - double(b), g = 1.0 - f;
        return Tab{ table_[b] * g + table_[b + 1] * f,
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
#if NLSC_E2_TABLE
        build_table();
#endif
    }
#endif

    // THE DC LOOP SOLVES WITH THE TABLE, DELIBERATELY.
    //
    // The entry `hoist()` builds the table with the DC coefficients, the 240
    // `process()` calls below read it, and the exit `hoist()` discards it. For
    // a DC solve from cold the table is the better solver of the two: its bins
    // are solved by CONTINUATION and to `1e-12`, while the audio Newton starts
    // from the previous state with the audio tolerance. Forcing the Newton here
    // leaves the stage silent after some overloads. The cost is ONE extra
    // build per `reinit` (a variant change or `activate`).
    //
    // Rest: capacitors open and uncharged. Solved with `process()` itself,
    // which then starts at the exact point instead of inside a transient.
    void rest(double v5)
    {
        v5_0_ = v5;
        cv4_ = 0.0; ci4_ = 0.0; cv3_ = v5 - vr_; ci3_ = 0.0;
        dq1_ = dq2_ = di1_ = di2_ = 0.0;
        v7_ = v5; va1_ = v5; va2_ = v5;
        v5_ = v5; v2_ = v5; cvcp_ = v5; icp_ = 0.0;

        const double G4_saved = G4_, Gleg_saved = Gleg_, g2h_saved = g2h_;
        const double Gcp_saved = Gcp_;
        G4_ = 0.0; Gleg_ = 0.0; g2h_ = 0.0; Gcp_ = 0.0;  // DC: no capacitors
#if NLSC_E2_HOIST
        hoist();   // `Gcp_` just changed: what was hoisted depends on it
#endif
        // With the REAL opamp `v5` is UNKNOWN, so C3's state — at rest
        // v5 − VR — depends on the solution itself. Iterate: solve, re-seed
        // C3, solve again. With the ideal opamp it converges first time,
        // because there v5 is given.
        for (int pass_ = 0; pass_ < 4; ++pass_) {
            for (int i = 0; i < 60; ++i) process(v5);
            cv3_ = v5_dc() - vr_;
        }
        G4_ = G4_saved; Gleg_ = Gleg_saved; g2h_ = g2h_saved;
        Gcp_ = Gcp_saved;
#if NLSC_E2_HOIST
        hoist();
#endif

        // Trapezoidal history consistent with d/dt = 0 at rest.
        cv4_ = v5_dc() - v7_;  ci4_ = 0.0;
        cv3_ = v5_dc() - vr_;  ci3_ = 0.0;
        cvcp_ = v2_;           icp_ = 0.0;
        // The charge history is seeded only when the charge is on: otherwise
        // the next step would subtract a charge that is no longer computed.
        dq1_ = kCharge ? mna::Engine::diode(d_, jd_, va1_ - v7_, kCharge).q : 0.0;  di1_ = 0.0;
        dq2_ = kCharge ? mna::Engine::diode(d_, jd_, va2_ - v5_dc(), kCharge).q : 0.0;  di2_ = 0.0;
#if NLSC_E2_PREDICTOR >= 1
        // The DC loop above has called `process()` 240 times: without
        // this, the first REAL sample extrapolates from the settling.
        wp1_ = wp2_ = 0.0; n_pred_ = 0;
#endif
    }

#if NLSC_E2_SLEW
    long slew_hot() const { return slew_hot_; }   // samples solved again with the limit
    long slew_first() const { return slew_first_; }  // index of the first one, -1 if none
#endif

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
#if NLSC_E2_SLEW
    bool slew_on_ = (NLSC_E2_SLEW == 1);
    long slew_hot_ = 0;
    long slew_first_ = -1;
#endif
    double n11_ = 0;
#if NLSC_E2_HOIST
    // Behind the flag: an unused member still changes the object's layout.
    double Y2_ = 0.0, invY2_ = 0.0, kY2_ = 0.0, invRf_ = 0.0, invGsum_ = 0.0;
    double invD_ = 0.0;
#endif
    long   slew_ = 0;
    long   sat_  = 0;
    static constexpr double kRO = 75.0;
    static constexpr int kSlewIt = 40;
    // Stage 3's load as seen from `n7` (see kR7/kC5/kR8).
    double G7_ = 0, G8_ = 0, G5_ = 0, Gsum_ = 0, cv5c_ = 0, ci5c_ = 0;
    double Yeq_ = 0, Ieq_ = 0, D_ = 1.0, kL_ = 0;
    double gm_ = 0, inv_imax_ = 0, imax_ = 0, Rp_ = 0, Gcp_ = 0;
};

} // namespace nlsc
