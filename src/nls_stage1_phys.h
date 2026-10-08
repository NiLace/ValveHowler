// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// Stage 1 solved from the circuit: the Q1 input buffer as a netlist, not a fit.
//
//     in --C1--n2--R1--n3--R2--VR          n6--R3--GND
//                      |                    |
//                     Q1 base   emitter ----+--C2--n4--R4--VR
//                      |                            |
//                    CJC to VCC                   RCM to GND, RIN to n5
//
// Why it is solved rather than shaped: the follower sources current but cannot
// sink it, and its AC load (R3 in parallel with C2 → R4) takes half of the
// emitter swing. The emitter current reaches zero at a deviation of about
// −1,79 V, Q1 cuts off, and from there `n6` is held by C2's charge, not by
// `n3`. That is memory (C2's voltage pumps with a ~20 ms time constant), which
// a static waveshaper such as `Stage1` (nls_stage1.h) cannot carry.
//
// Every element but the base-emitter junction is linear, so with trapezoidal
// companions for C1, C2, CJC and the BE charge the two nodes `n3` and `n6` are
// affine functions of the base and emitter currents. What is left is one
// scalar equation in the internal `vbe`:
//
//     vbe = n3(ib) − RB·ib − n6(ie)
//
// solved, without the table (`NLSC_E1_PHYS_TAB=0`), by Newton with SPICE's
// junction limiting, warm-started from the previous sample. The default build
// replaces that per-sample Newton with the table described below.
//
// The device is `mna::QModel` evaluated by `mna::Engine::bjt()`, the same
// object and law the DK engine solves, and the component values are read
// from `mna::Netlist`. Nothing here is a number of its own.
//
// Simplifications, each below −100 dB against ngspice at 0,3-3 V, 1 kHz:
//   · CJC at its rest value instead of the depletion law (no `pow` per sample);
//   · CJE constant (`cj0`), as every junction of the cascade;
//   · `RC`, the BC junction's diffusion charge and the substrate are left out:
//     the BC junction is reverse-biased by ~4,8 V and never conducts.

#pragma once
#include <cmath>
#include "nls_mna.h"

// The general `mna::Engine::bjt()` evaluates the whole Gummel-Poon card: two
// exponentials (one of them for `ISE`, which is 0 here), the BC junction, a
// square root and three divisions per Newton pass. The fast path is the same
// card specialised to Q1's operating region:
//   · the BC junction is reverse-biased by ~4,8 V: its exponential is exactly 0
//     in double, so its terms are constants;
//   · `ISE = 0`: the second exponential goes;
//   · the Early factor `q1` is taken once per sample from the previous `n3`;
//   · the start is a linear prediction, the loop stops at |dv| < 1e-8 V and the
//     last step is applied through the derivatives instead of re-evaluating.
// With `NLSC_E1_PHYS_TAB=0`: one exponential per pass; `NLSC_E1_PHYS_FAST=0`
// selects the exact card, and `NLSC_E1_EARLY_REST=1` freezes the Early factor
// at the rest point. Neither has any effect with the table (the default).
#ifndef NLSC_E1_EARLY_REST
#define NLSC_E1_EARLY_REST 0
#endif
#ifndef NLSC_E1_PHYS_FAST
#define NLSC_E1_PHYS_FAST 1
#endif
// The fast path's BC guard: a sample whose n3 comes within kBcMargin of the
// collector (or out not finite) is solved again with the whole card. 0 disables
// the guard and is never a product setting.
#ifndef NLSC_E1_FAST_BC_GUARD
#define NLSC_E1_FAST_BC_GUARD 1
#endif
// The table. With the Early factor at its rest value `k0`, the sample's whole
// nonlinear problem depends on ONE scalar,
//     Phi0(v) = v + (Ra + Rth6)·P(v) + Rth6·k0·G(v) = d
//     d = A3 − A6 − (Ra + Rth6)·Hs − Rth6·ISoBR
// where P is the base current plus the BE charge current's `v` part, G the
// transport current without the Early factor (ic = k·G + ISoBR, k = 1/q1) and
// Hs the charge's history term. So P, G and the charge are functions of `d`
// alone: they are tabulated in `startup()` (Newton per node, once) and read
// with a cubic Hermite, on an axis warped as u = x/(S + |x|), x = d − d_knee,
// so the nodes crowd where the junction turns on.
// The Early factor does not stay at rest (`n3` moves by volts), so its
// deviation enters as a first-order shift of `d`, with `k` taken from the
// previous sample's `n3` (the same one-sample lag the fast path has on `q1`)
// and G extrapolated linearly from the last two samples:
//     d_eff = d − Rth6·(k − k0)·(2·G_prev − G_prev2)
// No exponential and no Newton left in the sample: one division (the warp).
// The exception is the BC guard below, which re-solves a rare sample with
// the whole card.
#ifndef NLSC_E1_PHYS_TAB
#define NLSC_E1_PHYS_TAB 1
#endif
// The sample as affine maps. With the table in place the only nonlinearity is
// the lookup, so everything else is written as precomputed affine
// combinations of the states and the inputs, and each capacitor keeps one
// history source (its Norton current J = G·v + i, updated as J' = 2G·v' − J)
// instead of the voltage/current pair. This shortens the dependency chain from
// one sample to the next (the stage is latency-bound, not arithmetic-bound),
// and the products that do not depend on the lookup can run beside it. Same
// equations as `NLSC_E1_PHYS_NORTON=0`, which differs only by rounding.
#ifndef NLSC_E1_PHYS_NORTON
#define NLSC_E1_PHYS_NORTON NLSC_E1_PHYS_TAB
#endif

namespace nlsc {

class Stage1Phys {
public:
    Stage1Phys() { read_netlist(); }

    // The step is the oversampled one: the stage runs at the engine's rate.
    void prepare(double fs)
    {
        h_ = 1.0 / fs;
        g2h_ = 2.0 / h_;
        G1_ = g2h_ * C1_;
        G2_ = g2h_ * C2_;
        Ga_ = 1.0 / (1.0 / G1_ + R1_);
        derive();
    }

    // Everything the sample needs that only changes with the rate or the rest.
    void derive()
    {
        invG1_ = 1.0 / G1_;  invG2_ = 1.0 / G2_;
        invR2_ = 1.0 / R2_;  invR4_ = 1.0 / R4_;
        Rth3_ = 1.0 / (Ga_ + invR2_ + Gj_);
        Rt_   = 1.0 / (invR4_ + kInvRCM + kInvRIN);
        Gb_   = 1.0 / (invG2_ + Rt_);
        Rth6_ = 1.0 / (1.0 / R3_ + Gb_);
        // The affine maps of the Norton form (see NLSC_E1_PHYS_NORTON).
        a_in_  = Ga_ * Rth3_;          a_J1_ = -Ga_ * Rth3_ * invG1_;
        a_vr3_ = Rth3_ * invR2_;       a_Jj_ = Rth3_;
        a_c3_  = Gj_ * mna::VCC * Rth3_;
        b_vr_  = Gb_ * Rth6_ * Rt_ * invR4_;
        b_v5_  = Gb_ * Rth6_ * Rt_ * kInvRIN;
        b_J2_  = Gb_ * Rth6_ * invG2_;
        kH_    = Rth3_ + RB_ + Rth6_;
        c0_    = Rth6_ * q_.ISoBR;
        // n4 = Vt + Rt·Gb·(n6 − Eb), Eb = Vt + J2/G2
        e_vr_  = Rt_ * invR4_ * (1.0 - Rt_ * Gb_);
        e_v5_  = Rt_ * kInvRIN * (1.0 - Rt_ * Gb_);
        e_n6_  = Rt_ * Gb_;
        e_J2_  = -Rt_ * Gb_ * invG2_;
        // vc1 = in − n2 = (1 − R1·Ga)·(in − n3) + R1·Ga·J1/G1, and J1' = 2·G1·vc1 − J1
        f_in_  = 2.0 * G1_ * (1.0 - R1_ * Ga_);
        f_J1_  = 2.0 * R1_ * Ga_ - 1.0;
    }

    // The DC operating point for a given VR, capacitors open. Sets every
    // state so that the first sample starts at rest instead of settling.
    void startup(double vr, double v5)
    {
        const double Gt = 1.0 / R4_ + kInvRCM + kInvRIN;
        const double n4 = (vr / R4_ + v5 * kInvRIN) / Gt;
        double vbe = 0.6, ib = 0.0, n3 = vr, n6 = 0.0;
        for (int it = 0; it < 60; ++it) {
            const auto o = mna::Engine::bjt(q_, jbe_, jbc_, vbe, n3 - mna::VCC, false);
            ib = o.ib;
            const double ie = o.ic + o.ib;
            n3 = vr - R2_ * ib;
            n6 = R3_ * ie;
            const double f  = vbe - (n3 - RB_ * ib - n6);
            const double df = 1.0 + (R2_ + RB_) * o.dib_be + R3_ * (o.dic_be + o.dib_be);
            const double dv = f / df;
            vbe -= dv;
            if (std::fabs(dv) < 1e-15) break;
        }
        vbe_ = vbe;
        vc1_ = 0.0 - n3;  ic1_ = 0.0;           // input at 0 V, no current in R1
        vc2_ = n6 - n4;   ic2_ = 0.0;
        cjc_ = mna::Engine::depletion_cap(n3 - mna::VCC, q_.CJC, q_.VJC, q_.MJC, q_.FC);
        Gj_ = g2h_ * cjc_;
        derive();
        vcj_ = n3 - mna::VCC;  icj_ = 0.0;
        qbe_ = qbe(vbe);  iq_ = 0.0;
        vbe_prev_ = vbe;  have_prev_ = false;
#if NLSC_E1_PHYS_TAB
        k0_ = 1.0 - (n3 - mna::VCC) * q_.inv_VAF;
        // A mode change reinits from `run()` (under the fade): the table is
        // rebuilt only when what it depends on moved. ~0,24 ms when it does.
        if (g2h_ != tab_g2h_ || Gj_ != tab_gj_ || k0_ != tab_k0_) {
            tab_build();
            tab_g2h_ = g2h_;  tab_gj_ = Gj_;  tab_k0_ = k0_;
        }
        g_prev_ = g_prev2_ = dev_at(vbe).G;
#endif
        J1_ = G1_ * vc1_ + ic1_;
        J2_ = G2_ * vc2_ + ic2_;
        Jj_ = Gj_ * vcj_ + icj_;
        Hs_ = -g2h_ * qbe_ - iq_;
        n3_ = n3;  n6_ = n6;  n4_ = n4;
#if NLSC_E1_EARLY_REST
        q1_rest_ = 1.0 / (1.0 - (n3 - mna::VCC) * q_.inv_VAF);
#endif
    }

    // One sample. `vr` is the rail's absolute voltage, `v5` the opamp's
    // inverting input (for the current through its 5 MΩ `RIN`). Returns `n4`
    // absolute; `n3` and `n6` absolute through the pointers.
    double process(double in, double vr, double v5,
                   double* n3_out = nullptr, double* n6_out = nullptr)
    {
#if NLSC_E1_PHYS_NORTON
#if !NLSC_E1_PHYS_TAB
#error "NLSC_E1_PHYS_NORTON needs the table"
#endif
        const double A3 = a_in_ * in + a_J1_ * J1_ + a_vr3_ * vr + a_Jj_ * Jj_ + a_c3_;
        const double A6 = b_vr_ * vr + b_v5_ * v5 + b_J2_ * J2_;
        const double k  = 1.0 - (n3_ - mna::VCC) * q_.inv_VAF;
        const double d  = A3 - A6 - kH_ * Hs_ - c0_
                        - Rth6_ * (k - k0_) * (2.0 * g_prev_ - g_prev2_);
        double P, G, Q;
        tab_eval(d, P, G, Q);
        double ib = P + Hs_;
        double ie = k * G + q_.ISoBR + ib;
        double n3 = A3 - Rth3_ * ib;
        // The table assumes the BC junction reverse-biased, and past ~4,5 V of
        // input peak it is not: the base climbs over the 9 V collector, the BC
        // junction conducts, clips the base and pumps charge into C1. Such a
        // sample is solved with the whole card, from the table's point. It
        // does not happen at guitar level (up to 3 V of peak).
        if (__builtin_expect(n3 > mna::VCC - kBcMargin, 0)) {
            const double v0 = d - ka_ * P - kb_ * G;
            solve_full(A3, A6, v0, ib, ie, Q, G);
            n3 = A3 - Rth3_ * ib;
            ++bc_hot_;
        }
        const double n6 = A6 + Rth6_ * ie;
        const double n4 = e_vr_ * vr + e_v5_ * v5 + e_n6_ * n6 + e_J2_ * J2_;
        J1_ = f_in_ * (in - n3) + f_J1_ * J1_;
        J2_ = 2.0 * G2_ * (n6 - n4) - J2_;
        Jj_ = 2.0 * Gj_ * (n3 - mna::VCC) - Jj_;
        Hs_ = -2.0 * g2h_ * Q - Hs_;
        g_prev2_ = g_prev_;
        g_prev_ = G;
        ++iters_;
        n3_ = n3;  n6_ = n6;  n4_ = n4;
        if (n3_out) *n3_out = n3;
        if (n6_out) *n6_out = n6;
        return n4;
#else
        // Input branch C1 + R1 as a Norton source into n3.
        const double J1 = G1_ * vc1_ + ic1_;
        const double Ea = in - J1 * invG1_;
        // CJC from n3 to the collector (VCC, an AC ground).
        const double Jj = Gj_ * vcj_ + icj_;
        const double Rth3 = Rth3_;
        const double A3 = (Ga_ * Ea + vr * invR2_ + Gj_ * mna::VCC + Jj) * Rth3;
        // Output branch C2 in series with n4's Thevenin (R4 to VR, RCM to
        // ground, RIN to n5). The Thevenin resistances are constants of the
        // rate and the rest point: `derive()`.
        const double Rt = Rt_;
        const double Vt = (vr * invR4_ + v5 * kInvRIN) * Rt;
        const double J2 = G2_ * vc2_ + ic2_;
        const double Gb = Gb_;
        const double Eb = Vt + J2 * invG2_;
        const double Rth6 = Rth6_;
        const double A6 = Gb * Eb * Rth6;

#if NLSC_E1_PHYS_TAB
        double ib, ie;
        {
            const double Ra = Rth3 + RB_;
            const double Hs = -g2h_ * qbe_ - iq_;
            const double k  = 1.0 - (n3_ - mna::VCC) * q_.inv_VAF;
            // G predicted linearly from the last two samples: a plain
            // one-sample lag costs accuracy at n4 (G moves ~2e-6 A a sample at
            // 1 kHz, times Rth6·(k − k0)).
            const double d  = A3 - A6 - (Ra + Rth6) * Hs - Rth6 * q_.ISoBR
                            - Rth6 * (k - k0_) * (2.0 * g_prev_ - g_prev2_);
            double P, G, Q;
            tab_eval(d, P, G, Q);
            ib = P + Hs;
            ie = k * G + q_.ISoBR + ib;
            iq_ = g2h_ * (Q - qbe_) - iq_;
            qbe_ = Q;
            g_prev2_ = g_prev_;
            g_prev_ = G;
            ++iters_;
        }
#elif NLSC_E1_PHYS_FAST
        double vbe = vbe_, ib = 0.0, ie = 0.0, iq = 0.0;
        const double qbe_in = qbe_;
        {
#if NLSC_E1_EARLY_REST
            const double q1 = q1_rest_;
#else
            const double q1 = 1.0 / (1.0 - (n3_ - mna::VCC) * q_.inv_VAF);
#endif
            double v = have_prev_ ? 2.0 * vbe_ - vbe_prev_ : vbe_;
            const double Ra = Rth3 + RB_;
            for (int it = 0; it < kMaxIter; ++it) {
                const double x  = v * mna::inv_VT;
                const double ex = (x < -50.0) ? 0.0 : NLSC_EXP(x);
                const double e  = ex - 1.0;
                const double r  = std::sqrt(1.0 + 4.0 * q_.ISoIKF * e);
                const double iqb = 2.0 / (q1 * (1.0 + r));
                const double ict = q_.IS * ex;
                const double exv = ex * mna::inv_VT;
                const double ibdc = q_.ISoBF * e - q_.ISoBR;
                const double icdc = ict * iqb + q_.ISoBR;
                const double gb = q_.ISoBF * exv;
                const double dqb = q1 * q_.ISoIKF * exv / r;
                const double gc = (q_.IS * exv - ict * dqb * iqb) * iqb;
                const double qb = cje_ * v + q_.TFIS * e;
                const double gq = g2h_ * (cje_ + q_.TFIS * exv);
                const double iqv = g2h_ * (qb - qbe_) - iq_;
                const double ibv = ibdc + iqv;
                const double iev = icdc + ibdc + iqv;
                const double f  = v - (A3 - Ra * ibv - (A6 + Rth6 * iev));
                const double gib = gb + gq, gie = gc + gb + gq;
                const double df = 1.0 + Ra * gib + Rth6 * gie;
                double vn = v - f / df;
                vn = mna::Engine::pnjlim(vn, v, mna::VT, vcrit_);
                const double dv = vn - v;
                ++iters_;
                if (std::fabs(dv) < kTolFast || it == kMaxIter - 1) {
                    // The last step through the derivatives: its error is
                    // O(dv^2), below 1e-15 V at this tolerance.
                    vbe = vn;
                    ib = ibv + gib * dv;
                    ie = iev + gie * dv;
                    iq = iqv + gq * dv;
                    qbe_ = qb + (gq / g2h_) * dv;
                    break;
                }
                v = vn;
            }
        }
        // The fast card assumes the BC junction reverse-biased (its terms are
        // constants), and past ~4,5 V of input peak the base climbs over the
        // collector and this Newton can diverge. Same guard as the table's
        // (`kBcMargin`): such a sample, or one that did not come out finite, is
        // solved again with the whole card, from the previous sample's state.
        if (NLSC_E1_FAST_BC_GUARD
            && __builtin_expect(!(A3 - Rth3 * ib <= mna::VCC - kBcMargin), 0)) {
            qbe_ = qbe_in;
            newton_card(A3, A6, Rth3, Rth6, vbe, ib, ie, iq);
            ++bc_hot_;
        }
        vbe_prev_ = vbe_;  have_prev_ = true;
        iq_ = iq;
        vbe_ = vbe;
#else
        double vbe, ib, ie, iq;
        newton_card(A3, A6, Rth3, Rth6, vbe, ib, ie, iq);
        iq_ = iq;
        vbe_ = vbe;
#endif

        const double n3 = A3 - Rth3 * ib;
        const double n6 = A6 + Rth6 * ie;
        const double ia = Ga_ * (Ea - n3);
        const double i2 = Gb * (n6 - Eb);
        const double n4 = Vt + Rt * i2;
        const double n2 = n3 + R1_ * ia;

        const double vc1 = in - n2;   ic1_ = G1_ * vc1 - J1;  vc1_ = vc1;
        const double vc2 = n6 - n4;   ic2_ = G2_ * vc2 - J2;  vc2_ = vc2;
        const double vcj = n3 - mna::VCC;  icj_ = Gj_ * vcj - Jj;  vcj_ = vcj;

        n3_ = n3;  n6_ = n6;  n4_ = n4;
        if (n3_out) *n3_out = n3;
        if (n6_out) *n6_out = n6;
        return n4;
#endif
    }

#if NLSC_E1_PHYS_TAB
    // The device's pieces at `v`, with the Early factor at rest.
    struct Dev { double P, G, Q, dP, dG, dQ; };
    Dev dev_at(double v) const
    {
        const double x  = v * mna::inv_VT;
        const double ex = (x < -50.0) ? 0.0 : std::exp(x);
        const double exv = ex * mna::inv_VT;
        const double r  = std::sqrt(1.0 + 4.0 * q_.ISoIKF * (ex - 1.0));
        Dev o;
        o.Q  = cje_ * v + q_.TFIS * (ex - 1.0);
        o.dQ = cje_ + q_.TFIS * exv;
        o.P  = q_.ISoBF * (ex - 1.0) - q_.ISoBR + g2h_ * o.Q;
        o.dP = q_.ISoBF * exv + g2h_ * o.dQ;
        o.G  = 2.0 * q_.IS * ex / (1.0 + r);
        const double dr = 2.0 * q_.ISoIKF * exv / r;
        o.dG = 2.0 * q_.IS * exv / (1.0 + r) - 2.0 * q_.IS * ex * dr / ((1.0 + r) * (1.0 + r));
        return o;
    }

    // Built once per startup: Rth3 depends on the rest CJC, Rth6 on the rate.
    void tab_build()
    {
        const double Rth6 = Rth6_;
        const double Ra = Rth3_ + RB_;
        ka_ = Ra + Rth6;  kb_ = Rth6 * k0_;
        auto phi = [&](double v, const Dev& o) { return v + ka_ * o.P + kb_ * o.G; };
        d_knee_ = phi(0.55, dev_at(0.55));
        double v = -20.0;
        for (int i = 0; i < kTabN; ++i) {
            const double u = -kUmax + 2.0 * kUmax * i / (kTabN - 1);
            const double xw = kS * u / (1.0 - std::fabs(u));
            const double d = d_knee_ + xw;
            // Newton on the monotone Phi0, from the previous node, with the
            // junction limited as in SPICE.
            for (int it = 0; it < 200; ++it) {
                const Dev o = dev_at(v);
                const double f  = phi(v, o) - d;
                const double df = 1.0 + ka_ * o.dP + kb_ * o.dG;
                double vn = v - f / df;
                vn = mna::Engine::pnjlim(vn, v, mna::VT, vcrit_);
                const double dv = vn - v;
                v = vn;
                if (std::fabs(dv) < 1e-15 + 1e-15 * std::fabs(v)) break;
            }
            const Dev o = dev_at(v);
            const double dphi = 1.0 + ka_ * o.dP + kb_ * o.dG;
            const double ddu = kS / ((1.0 - std::fabs(u)) * (1.0 - std::fabs(u)));
            const double s = ddu * kHu / dphi;      // d(.)/du · h, per unit d(.)/dv
            tab_[i] = Node{o.P, o.G, o.Q, o.dP * s, o.dG * s, o.dQ * s};
        }
    }

    void tab_eval(double d, double& P, double& G, double& Q) const
    {
        const double x = d - d_knee_;
        double u = x / (kS + std::fabs(x));
        // Written so a NaN `d` lands on the edge instead of indexing outside the
        // table: `u < -kUmax` is false for a NaN, `!(u > -kUmax)` is true. The
        // cascade's recovery handles a non-finite state from there.
        if (!(u > -kUmax)) u = -kUmax;
        if (u >  kUmax) u =  kUmax;
        const double pos = (u + kUmax) * kInvHu;
        int i = static_cast<int>(pos);
        if (i > kTabN - 2) i = kTabN - 2;
        const double t = pos - i, t2 = t * t, t3 = t2 * t;
        const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0, h10 = t3 - 2.0 * t2 + t;
        const double h01 = -2.0 * t3 + 3.0 * t2,      h11 = t3 - t2;
        const Node& a = tab_[i];
        const Node& b = tab_[i + 1];
        P = h00 * a.P + h10 * a.mP + h01 * b.P + h11 * b.mP;
        G = h00 * a.G + h10 * a.mG + h01 * b.G + h11 * b.mG;
        Q = h00 * a.Q + h10 * a.mQ + h01 * b.Q + h11 * b.mQ;
    }
#endif

#if !NLSC_E1_PHYS_NORTON
    // Newton on vbe with the whole Gummel-Poon card. `vbc` follows the previous
    // iterate's n3 (the Early term is weak: the collector sits 4,8 V above the
    // base). The exact solve (`NLSC_E1_PHYS_FAST=0`), and the fast path's
    // fallback when the BC junction conducts. Leaves `qbe_` at the solution.
    __attribute__((noinline))
    void newton_card(double A3, double A6, double Rth3, double Rth6,
                     double& vbe, double& ib, double& ie, double& iq)
    {
        vbe = vbe_;
        double n3i = n3_;
        ib = ie = iq = 0.0;
        for (int it = 0; it < kMaxIter; ++it) {
            const auto o = mna::Engine::bjt(q_, jbe_, jbc_, vbe, n3i - mna::VCC, true);
            const double qb = o.qbe, cb = o.cbe;
            iq = g2h_ * (qb - qbe_) - iq_;
            ib = o.ib + iq;
            ie = o.ic + o.ib + iq;
            n3i = A3 - Rth3 * ib;
            const double f  = vbe - (n3i - RB_ * ib - (A6 + Rth6 * ie));
            const double gq = g2h_ * cb;
            const double df = 1.0 + (Rth3 + RB_) * (o.dib_be + gq)
                                  + Rth6 * (o.dic_be + o.dib_be + gq);
            double vn = vbe - f / df;
            vn = mna::Engine::pnjlim(vn, vbe, mna::VT, vcrit_);
            const double dv = vn - vbe;
            vbe = vn;
            ++iters_;
            if (std::fabs(dv) < kTol) break;
        }
        // Final evaluation at the converged point, so the states are updated
        // with the currents OF that point.
        {
            const auto o = mna::Engine::bjt(q_, jbe_, jbc_, vbe, n3i - mna::VCC, true);
            iq = g2h_ * (o.qbe - qbe_) - iq_;
            ib = o.ib + iq;
            ie = o.ic + o.ib + iq;
            qbe_ = o.qbe;
        }
    }
#endif

#if NLSC_E1_PHYS_NORTON
    // The sample with the whole Gummel-Poon card (BC junction and Early from the
    // iterate), in the Norton form: the charge history is `Hs_`, so the BE
    // charge current is g2h·qbe(v) + Hs_. Returns the total currents and the
    // charge; `G` is refreshed for the Early extrapolation of the next sample.
    __attribute__((noinline))
    void solve_full(double A3, double A6, double v0,
                    double& ib, double& ie, double& Q, double& G)
    {
        // Two unknowns, vbe and vbc: with the BC junction conducting, the base
        // current depends on vbc as strongly as on vbe, and a Newton that only
        // follows vbe diverges. Both junctions limited as in SPICE.
        //   F1 = n3 − A3 + Rth3·ib                     (KCL at n3, n3 = VCC + vbc)
        //   F2 = vbe − n3 + RB·ib + A6 + Rth6·ie       (KVL base-emitter)
        double v = v0, c = n3_ - mna::VCC;
        for (int it = 0; it < kMaxIter; ++it) {
            const auto o = mna::Engine::bjt(q_, jbe_, jbc_, v, c, true);
            const double gq = g2h_ * o.cbe;
            const double ibv = o.ib + g2h_ * o.qbe + Hs_;
            const double iev = o.ic + ibv;
            const double n3 = mna::VCC + c;
            const double F1 = n3 - A3 + Rth3_ * ibv;
            const double F2 = v - n3 + RB_ * ibv + A6 + Rth6_ * iev;
            const double dib_v = o.dib_be + gq, dib_c = o.dib_bc;
            const double die_v = o.dic_be + dib_v, die_c = o.dic_bc + dib_c;
            const double J11 = Rth3_ * dib_v,                       J12 = 1.0 + Rth3_ * dib_c;
            const double J21 = 1.0 + RB_ * dib_v + Rth6_ * die_v,    J22 = -1.0 + RB_ * dib_c + Rth6_ * die_c;
            const double det = J11 * J22 - J12 * J21;
            double dv = -( F1 * J22 - J12 * F2) / det;
            double dc = -(J11 * F2 - F1 * J21) / det;
            const double vn = mna::Engine::pnjlim(v + dv, v, mna::VT, vcrit_);
            const double cn = mna::Engine::pnjlim(c + dc, c, mna::VT, vcrit_);
            dv = vn - v;  dc = cn - c;
            v = vn;  c = cn;
            if (std::fabs(dv) < kTol && std::fabs(dc) < kTol) break;
        }
        const auto o = mna::Engine::bjt(q_, jbe_, jbc_, v, c, true);
        ib = o.ib + g2h_ * o.qbe + Hs_;
        ie = o.ic + ib;
        Q  = o.qbe;
        // G is the transport current without the Early factor, as the table
        // stores it: ic = ict/qb − ISoBR·e_bc and G = (ict/qb)·q1 = (ic + ISoBR·e_bc)/k.
        const double xbc = c * mna::inv_VT;
        const double e_bc = (xbc < -50.0 ? 0.0 : std::exp(std::fmin(xbc, 50.0))) - 1.0;
        const double k = 1.0 - c * q_.inv_VAF;
        G  = (o.ic + q_.ISoBR * e_bc) / k;
    }
    long bc_hot() const { return bc_hot_; }
#endif

    // Observability: how many Newton iterations have run in total.
    long iterations() const { return iters_; }
    double n4() const { return n4_; }

private:
    // Tolerance on the vbe update. 1e-12 V is ~4e-11 VT: the exponential is
    // then exact to double precision.
    static constexpr double kTol = 1e-12;
    static constexpr int kMaxIter = 30;
    static constexpr double kTolFast = 1e-8;
    static constexpr double kInvRCM = 1.0 / mna::OModel{}.RCM;
    static constexpr double kInvRIN = 1.0 / mna::OModel{}.RIN;

    double qbe(double vbe) const
    {
        return mna::Engine::bjt(q_, jbe_, jbc_, vbe, n3_ - mna::VCC, true).qbe;
    }

    // Reads the six passive components from the DK's netlist, nominal
    // specimen. Looked up by node pair, so a change of value there is a
    // change here, and a change of topology leaves a zero that `ok()` names.
    void read_netlist()
    {
        mna::Netlist nl;
        nl.build();
        auto res = [&](int a, int b) {
            for (int i = 0; i < nl.nres; ++i)
                if ((nl.res[i].a == a && nl.res[i].b == b) ||
                    (nl.res[i].a == b && nl.res[i].b == a)) return nl.res[i].r;
            return 0.0;
        };
        auto cap = [&](int a, int b) {
            for (int i = 0; i < nl.ncap; ++i)
                if ((nl.cap[i].a == a && nl.cap[i].b == b) ||
                    (nl.cap[i].a == b && nl.cap[i].b == a)) return nl.cap[i].c;
            return 0.0;
        };
        R1_ = res(mna::N_n2, mna::N_n3);
        R2_ = res(mna::N_n3, mna::N_vr);
        R3_ = res(mna::N_n6, mna::N_GND);
        R4_ = res(mna::N_n4, mna::N_vr);
        C1_ = cap(mna::N_IN, mna::N_n2);
        C2_ = cap(mna::N_n6, mna::N_n4);
        q_.derive();
        RB_ = NLSC_MNA_RB ? q_.RB : 0.0;
        jbe_.init(q_.CJE, q_.VJE, q_.MJE, q_.FC);
        cje_ = q_.CJE;
        jbc_.init(q_.CJC, q_.VJC, q_.MJC, q_.FC);
        vcrit_ = mna::VT * std::log(mna::VT / (std::sqrt(2.0) * q_.IS));
        prepare(192000.0);
    }

public:
    // Every component found: a missing one reads as 0 and would divide by it.
    bool ok() const { return R1_ > 0 && R2_ > 0 && R3_ > 0 && R4_ > 0 && C1_ > 0 && C2_ > 0; }

private:
    mna::QModel q_{};
    mna::Engine::Junc jbe_{}, jbc_{};
    double R1_ = 0, R2_ = 0, R3_ = 0, R4_ = 0, C1_ = 0, C2_ = 0, RB_ = 0;
    double vcrit_ = 0.0;
    double h_ = 0, g2h_ = 0, G1_ = 0, G2_ = 0, Ga_ = 0, Gj_ = 0, cjc_ = 0;
    // States: capacitor voltages and trapezoidal currents, BE charge, the
    // last solution (Newton's warm start).
    double vc1_ = 0, ic1_ = 0, vc2_ = 0, ic2_ = 0, vcj_ = 0, icj_ = 0;
    double qbe_ = 0, iq_ = 0, vbe_ = 0.6, vbe_prev_ = 0.6, cje_ = 0;
    bool have_prev_ = false;
    double q1_rest_ = 1.0;
    double invG1_ = 0, invG2_ = 0, invR2_ = 0, invR4_ = 0;
    double Rth3_ = 0, Rt_ = 0, Gb_ = 0, Rth6_ = 0;
    double a_in_ = 0, a_J1_ = 0, a_vr3_ = 0, a_Jj_ = 0, a_c3_ = 0;
    double b_vr_ = 0, b_v5_ = 0, b_J2_ = 0, kH_ = 0, c0_ = 0;
    double e_vr_ = 0, e_v5_ = 0, e_n6_ = 0, e_J2_ = 0, f_in_ = 0, f_J1_ = 0;
    double J1_ = 0, J2_ = 0, Jj_ = 0, Hs_ = 0;     // Norton histories
    long bc_hot_ = 0;
    // n3 this close to the collector sends the sample to the full card. At
    // 0,5 V of reverse bias the BC junction carries IS·e^(−19): nothing.
    static constexpr double kBcMargin = 0.5;
#if NLSC_E1_PHYS_TAB
    // 2049 nodes: odd, so the warp's kink at x = 0 falls ON a node. S = 1 V
    // puts ~1 mV between nodes at the knee and ~0,12 V at 10 V from it.
    static constexpr int    kTabN  = 2049;
    static constexpr double kS     = 1.0;
    static constexpr double kUmax  = 0.999;
    static constexpr double kHu    = 2.0 * kUmax / (kTabN - 1);
    static constexpr double kInvHu = 1.0 / kHu;
    struct Node { double P, G, Q, mP, mG, mQ; };
    Node tab_[kTabN] = {};
    double k0_ = 1.0, ka_ = 0.0, kb_ = 0.0, d_knee_ = 0.0, g_prev_ = 0.0, g_prev2_ = 0.0;
    double tab_g2h_ = -1.0, tab_gj_ = -1.0, tab_k0_ = -1.0;   // the built table's key
#endif
    double n3_ = 0, n6_ = 0, n4_ = 0;
    long iters_ = 0;
};

} // namespace nlsc
