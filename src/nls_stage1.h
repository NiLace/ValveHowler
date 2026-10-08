// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// Stage 1 — the Q1 input buffer.
//
// Structure:
//
//     in --H1(s)--> n3 --waveshaper--> --Hs(s)--> n6 --H2(s)--> n4
//
// The buffer is not linear above ~0.2 V of input (at hot-humbucker level it
// distorts at -46 dBc), but its nonlinearity is memoryless (THD flat from
// 82 Hz to 8 kHz), so it is modelled as a static waveshaper between linear
// filters: one evaluation per sample, no iteration.
//
// 1. `n3` does not rest at VR but 1.15 V below it: the base current drops
//    across R2 (510 k). The waveshaper works in deviation from that rest.
// 2. Hs is normalised at 8 kHz. The static curve already contains the
//    follower's gain (slope 0.9800) because it was extracted at that
//    frequency; applying the raw Hs would count it twice.
// 3. The filters are not first order. The follower's base impedance
//    depends on C2, the output capacitor, so the input network inherits a
//    pole from the output one.

#pragma once
#ifndef NLSC_WS_RECENTER
#define NLSC_WS_RECENTER 0
#endif

// Selector for the coefficient sets, which are tabulated at one concrete
// rate each (default: 8x). It matters only to offline measurement, which
// reads the tabulated sets:
//   -DNLSC_CASC_KHZ=192  ->  4x      -DNLSC_CASC_KHZ=96  ->  2x
// It does not change the plugin: the plugin build passes
// `-DNLSC_CASC_KHZ=192`, but the filters are rebuilt from their s-domain fits
// at the real rate in `prepare()`, so the plugin binary is identical whichever
// set is selected.
#if defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 96
#  include "nls_stage1_coef_96k.h"
#elif defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 192
#  include "nls_stage1_coef_192k.h"
#else
#  include "nls_stage1_coef.h"
#endif
#include "nls_filtro_param.h"
#include "nls_fijos_s.h"
#include "nls_reposo_drive.h"

// The second bank, the 9/9RI variant's. It is always included: the user
// picks the variant at run time, so both banks are compiled in. Which
// tabulated set is included follows the rate selector above (see there).
#if defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 96
#  include "nls_stage1_coef_v9ri_96k.h"
#elif defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 192
#  include "nls_stage1_coef_v9ri_192k.h"
#else
#  include "nls_stage1_coef_v9ri_384k.h"
#endif
#include "nls_fijos_s_v9ri.h"
#include "nls_reposo_drive_v9ri.h"

namespace nlsc {

// Capacity of the waveshaper coefficients copied into members; enforced by
// a `static_assert` below.
inline constexpr int kWSMax = 24;

class Stage1 {
public:
    // Starts with the 808's bank, so the class is usable without calling
    // `prepare()`.
    Stage1()
    {
        for (int i = 0; i < stage1::kWS_N; ++i) ws_[i] = stage1::kWS[i];
        reset();
    }


    void reset()
    {
        h1_.reset();
        hs_.reset();
        h2_.reset();
        out_of_range_ = 0;
    }

    // Waveshaper guard. Extrapolating the fitted polynomial outside its
    // travel does not degrade gracefully, it blows up; so outside the fitted
    // travel the curve continues along the endpoint's tangent, which is
    // bounded and monotonic, and every such sample is counted.
    //
    // A passive-pickup DI stays well inside the fit (-0.339/+0.413 V against
    // -1.373/+1.417 V). With an active pickup or a booster in front the input
    // reaches 2-3 V peak and the guard fires from ~1.5 V; there Q1 cuts off
    // and `n6` hangs on C2's charge, which is memory a static curve cannot
    // represent. `NLSC_E1_PHYS` (nls_stage1_phys.h) solves the buffer from
    // the circuit instead.
    double tangent(double u0, double u) const
    {
        double d = 0.0;                       // derivative by Horner
        for (int i = 0; i < ws_n_ - 1; ++i)
            d = d * u0 + ws_[i] * (ws_n_ - 1 - i);
        double w0 = 0.0;
        for (int i = 0; i < ws_n_; ++i) w0 = w0 * u0 + ws_[i];
        return w0 + d * (u - u0);
    }

    // Evaluates the waveshaper with the guard on. The three paths of this
    // class (normal, `n3`-given and `n6`-given) share it.
    double ws(double u)
    {
        if (u < umin_)      { ++out_of_range_; return tangent(umin_, u); }
        if (u > umax_)      { ++out_of_range_; return tangent(umax_, u); }
        double w = 0.0;                        // Horner, descending order
        for (int i = 0; i < ws_n_; ++i) w = w * u + ws_[i];
        return w;
    }

    // Observability: how many samples left the fitted travel.
    long out_of_range() const { return out_of_range_; }
    double ws_umin() const { return umin_; }
    double ws_umax() const { return umax_; }


    // The rest point as a function of the drive knob.
    //
    // This stage was fitted with the circuit at drive 0.5, and its rest
    // points (`kQn3`, `kQn6`, `kVr`) are constants of that point. The gain
    // pot moves the whole pedal's rest through the shared rail: from drive
    // 1.0 to 0.0, `n3` drops 1.38 mV and `VR` 1.90 mV, which dominates with
    // the drive closed and a weak signal (see `nls_reposo_drive.h`).
    //
    // The current rest is passed in and its difference from the fit's rest
    // is stored, to be applied:
    //   - at the waveshaper's input, to evaluate it in its own frame;
    //   - at the output, because `v` is a deviation from `kQn6` and the
    //     output is referred to the current rest.
    // If this is never called the offsets stay zero.
    void set_rest(double q_n3, double q_n6, double q_n4)
    {
        off_n3_ = q_n3 - qn3_;
        off_n6_ = q_n6 - qn6_;
        q_n4_   = q_n4;
    }

    double rest_n4() const { return q_n4_; }

    // Both banks must share the filters' orders, because `FixedFilter<NB,NA>`
    // is a template and there is one member per filter. If they diverge,
    // this does not compile.
    static_assert(fixed::kH1_NB == fixed_v9ri::kH1_NB &&
                  fixed::kH1_NA == fixed_v9ri::kH1_NA &&
                  fixed::kHS_NB == fixed_v9ri::kHS_NB &&
                  fixed::kHS_NA == fixed_v9ri::kHS_NA &&
                  fixed::kH2_NB == fixed_v9ri::kH2_NB &&
                  fixed::kH2_NA == fixed_v9ri::kH2_NA,
                  "both stage-1 banks must share the same filter orders");
    static_assert(stage1::kWS_N <= kWSMax && stage1_v9ri::kWS_N <= kWSMax,
                  "kWSMax is too small for stage 1's waveshaper");

    // Loads the variant's bank, copied into members because it is read per
    // sample. Separate from `prepare()` because `set_rest()` runs before
    // `prepare()` and subtracts `qn3_`/`qn6_`, which must already be the
    // selected variant's.
    template <class J>
    void set_bank()
    {
        ws_n_ = J::WS_N;
        for (int i = 0; i < ws_n_; ++i) ws_[i] = J::WS[i];
// `NLSC_WS_RECENTER` (off by default) zeroes the waveshaper's independent
// term. The fit does not pass exactly through the origin (Q1: +9.14 uV on
// the 808), so at start-up, with every filter state at zero, it injects a
// constant step that the stage high-passes take hundreds of ms to wash out.
#if NLSC_WS_RECENTER
        ws_[ws_n_ - 1] = 0.0;
#endif
        qn3_ = J::Qn3;  qn6_ = J::Qn6;
        umin_ = J::WS_UMIN;  umax_ = J::WS_UMAX;
    }

    template <class J>
    void prepare_bank(double fs, bool reinit)
    {
        set_bank<J>();
        h1_.prepare(fs, J::H1_B, J::H1_A, J::ScaleF, reinit);
        hs_.prepare(fs, J::HS_B, J::HS_A, J::ScaleF, reinit);
        h2_.prepare(fs, J::H2_B, J::H2_A, J::ScaleF, reinit);
    }

    void startup()
    {
        h1_.preset_dc(0.0);
        const double w = ws(off_n3_);
        h2_.preset_dc(hs_.preset_dc(w) - off_n6_ * rest::kOffOutput);
    }

    // The rail enters `n3` through `R2` (510 k), and this stage's fits
    // were made with VR pinned, so that summand must be added here. `du` is
    // the rail's contribution to `n3`, already filtered by the caller.
    double process(double in, double du, double* n3_out = nullptr, double* n6_out = nullptr)
    {
        du_ = du;
        const double r = process(in, n3_out, n6_out);
        du_ = 0.0;
        return r;
    }

    double process(double in, double* n3_out = nullptr, double* n6_out = nullptr)
    {
        // `u` is `H1(in)` plus the rail term and the rest offset.
        // `off_n3_` brings `u` into the frame the waveshaper was fitted
        // in (see `set_rest`); `kQn3 + u` is still the absolute `n3`.
        const double u = h1_.process(in) + du_ + off_n3_;  // n3 deviation

        const double w = ws(u);

        const double v = hs_.process(w) - off_n6_ * rest::kOffOutput;      // n6 deviation
        if (n6_out) *n6_out = qn6_ + v;
        if (n3_out) *n3_out = qn3_ + u;

        // No base-current correction is applied: the signal-dependent
        // displacement is ~0.1 % of the base current, below what a separate
        // `ib` model can resolve (see `NLSC_E1_PHYS` for a nodal solve).
        return q_n4_ + h2_.process(v);                  // n4 rests at VR
    }

    // A hand-written default for VR's rest, used only when `set_rest()` is
    // not called. It is not the circuit's rest: the generated per-variant,
    // per-drive value lives in `nls_reposo_drive*.h` (`kvr[]`), and whoever
    // prepares this stage passes that one through `set_rest()`. The default
    // plugin build (`NLSC_E1_PHYS=1`) runs the physical stage 1 of
    // `nls_stage1_phys.h` and does not run this class; see
    // `NLSC_CASC_FITTED_E1` in `nls_cascada.h`.
    static constexpr double kVr = 4.478416;

private:
    // Filters defined in `s` and discretised in `prepare()`, so the stage
    // runs at any rate. Their coefficients come from inverting the bilinear
    // transform over the tabulated sets (exact to 1e-16).
    FixedFilter<fixed::kH1_NB, fixed::kH1_NA> h1_;
    FixedFilter<fixed::kHS_NB, fixed::kHS_NA> hs_;
    FixedFilter<fixed::kH2_NB, fixed::kH2_NA> h2_;
    double du_ = 0.0;     // the rail's contribution to `n3`, set by the caller
    // Rest displacement against the point the stage was fitted at
    // (zero unless `set_rest` is called).
    double off_n3_ = 0.0, off_n6_ = 0.0, q_n4_ = kVr;

    // Copy of the active variant's bank, read per sample. The constructor
    // loads the 808's, so the class is usable without calling `prepare()`.
    double ws_[kWSMax] = {0.0};
    int    ws_n_ = stage1::kWS_N;
    // The fit's travel comes from the bank: each variant fits its own curve
    // and may have another travel.
    double umin_ = stage1::kWS_UMIN;
    double umax_ = stage1::kWS_UMAX;
    long   out_of_range_ = 0;
    double qn3_  = stage1::kQn3;
    double qn6_  = stage1::kQn6;
};

} // namespace nlsc
