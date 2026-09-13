// Stage 1 — the Q1 input buffer.
//
// Structure, and why THIS one:
//
//     in --H1(s)--> n3 --waveshaper--> --Hs(s)--> n6 --H2(s)--> n4
//
// The brief took it for LINEAR ("1st-order high-pass + unity gain").
// Measured, that is false above ~0,2 V of input: at hot-humbucker level it
// distorts at −46 dBc, 14 dB above the brief's own threshold
// (docs/LINEALIDAD_ETAPAS.md). But the nonlinearity is MEMORYLESS — THD
// flat from 82 Hz to 8 kHz, and the curve's auto-null at −93,6 dB — so it
// is modelled as a static waveshaper between linear filters, one evaluation
// per sample, no iteration.
//
// Three things that are NOT obvious and cost real measurement:
//
// 1. **`n3` does not rest at VR** but 1,15 V below it: the base current
//    drops across R2 (510 k). The waveshaper works in DEVIATION from that
//    rest, not from VR.
// 2. **Hs is normalised at 8 kHz.** The static curve already contains the
//    follower's gain (slope 0,9800) because it was extracted at that
//    frequency; applying the raw Hs would count it twice.
// 3. **The filters are not first order.** The follower's base impedance
//    depends on C2, the OUTPUT capacitor, so the input network inherits a
//    pole from the output one. First order stops at −46,8 dB.

#pragma once
#ifndef NLSC_WS_RECENTER
#define NLSC_WS_RECENTER 0
#endif

// RATE SELECTOR — only for the CASCADE bench, never for the plugin.
//
// The coefficients are discretised at one concrete rate. Measuring the
// cascade at 8x, 4x and 2x takes three sets, chosen here. With nothing
// defined the usual one is taken and the binary is BIT-IDENTICAL: it is a
// selector, not a change.
//   -DNLSC_CASC_KHZ=192  ->  4x      -DNLSC_CASC_KHZ=96  ->  2x
// RETIRED: `-DNLSC_CASC_DIG` (sets fitted in the DIGITAL domain, the
// control for the bilinear's warping). Product decision.
// That control ALREADY ANSWERED: the 96 kHz set redone without the
// bilinear gave −39,1 dB against −38,9 => 0,2 dB, no contamination
// (`CASCADA_CUATRO_ETAPAS.md` §5).
// And it was broken in THREE places: it did not compile (the `e234*dig`
// sets missed five constants), it could not be regenerated (the digital
// `E4G1` fit comes out UNSTABLE), and the three stage-1 `dig` sets carried
// poles OUTSIDE the circle (|z| = 1,0095 · 1,00004 · 1,0024).
// => If the question ever reopens, the generators keep their `--digital`
// flag: build the instrument anew — do not resurrect the corpse.
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

// THE SECOND BANK, the 9/9RI variant's. It is ALWAYS included, never under
// `#if`: the user picks the variant LIVE, so both banks of the current rate
// must be compiled at once. The rate keeps being chosen at COMPILE time,
// which is what it already did.
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

// Waveshaper cap, copied into members. It is a COMPILE-TIME cap with a
// `static_assert` behind it, not an assumption.
inline constexpr int kWSMax = 24;

// (`SosCascade` — a direct-form-II-transposed cascade of second-order
// sections — was removed once nothing instantiated it: every consumer moved
// to runtime-discretised filters. Its `preset_dc` technique survives in
// `ParamFilter::preset_dc`.)

class Stage1 {
public:
    // Starts with the 808's bank, which is the one that ships: the class
    // works without calling `prepare()`, which is what several harnesses do.
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
        corr_ = 0.0;
        out_of_range_ = 0;
    }

    // THE WAVESHAPER GUARD — it arrived late, and the cost was known.
    //
    // Stage 4 has had one forever; this one ran with its input UNBOUNDED,
    // and extrapolating an order-9 polynomial does not degrade: it BLOWS UP
    // (+41 dB at 2,8 V, measured in `docs/WAVESHAPER_ETAPAS_1_4.md` §4).
    //
    // And it is not only robustness: it was what BLOCKED raising the
    // order. `gen_coef_etapa1.py` says it itself — order 11 buys 4 dB over
    // the excursion that occurs and multiplies the extrapolation disaster by
    // 5, "and if the input is ever BOUNDED, that deal changes". This is that
    // sentence, exercised.
    //
    // Outside the fitted travel the curve continues along the endpoint's
    // TANGENT, which is bounded and monotonic, and it is COUNTED: a guard
    // that is not counted is a guard nobody knows fired.
    // The REAL travel in the engine is −0,339/+0,413 V against a fit of
    // −1,373/+1,417 => with normal signal this never fires, and the output is
    // BIT-IDENTICAL. It is a net, not a model change.
    double tangent(double u0, double u) const
    {
        double d = 0.0;                       // derivative by Horner
        for (int i = 0; i < ws_n_ - 1; ++i)
            d = d * u0 + ws_[i] * (ws_n_ - 1 - i);
        double w0 = 0.0;
        for (int i = 0; i < ws_n_; ++i) w0 = w0 * u0 + ws_[i];
        return w0 + d * (u - u0);
    }

    // Evaluates the waveshaper with the guard on. ONE place: the three
    // routes of this class (normal, `n3`-oracle and `n6`-oracle) call it,
    // because three copies of a Horner diverge like three copies of a body.
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

    // ATTRIBUTION PROBE: the stage with `n3` GIVEN by the oracle.
    //
    // Not a product mode: it splits stage 1 in two, to tell whether what
    // fails is the INPUT network (`H1` plus the rail coupling) or the
    // WAVESHAPER and the output. Without it one only knows stage 1 is the
    // floor, not which part of it.
    double process_con_n3(double n3_abs, double* n6_out = nullptr)
    {
        const double u = n3_abs - qn3_;
        const double w = ws(u);
        const double v = hs_.process(w) - off_n6_ * rest::kOffOutput;
        if (n6_out) *n6_out = qn6_ + v;
        return q_n4_ + h2_.process(v);
    }

    // ATTRIBUTION PROBE: the stage with `n6` GIVEN by the oracle.
    //
    // Splits what sits BELOW `n3`, where `n3`-oracle and `n4`-oracle left
    // 3,54 dB unattributed:
    //     n6-oracle -> ONLY `H2` remains, the output coupling (`C2` vs `R4`)
    //     by difference with `n3`-oracle: the WAVESHAPER plus `Hs`
    //
    // `H1`, the waveshaper and `Hs` KEEP RUNNING, and not for style:
    //   1. `n3` is needed by the RAIL model upstream — the caller feeds it
    //      into `dev_n3_`, this node's contribution through `R2` (510 k) —
    //      so skipping it would measure the rail with a frozen `n3`.
    //   2. A filter whose state does not advance is not "the same filter,
    //      unused": the next sample finds it stalled.
    // The ONLY thing that changes is what ENTERS `H2`.
    //
    // And `H2` advances EXACTLY ONCE. Same trap as written in
    // `process_con_n3`: calling the normal path AND this one would advance
    // the state twice — a silent error exactly the size of a result.
    double process_con_n6(double in, double du, double n6_abs_oraculo,
                          double* n3_out = nullptr, double* n6_out = nullptr)
    {
        const double u = h1_.process(in) + corr_ + du + off_n3_;

        const double w = ws(u);

        // The stage's own `n6` is computed and EXPOSED — it is what the
        // probe compares against the oracle's — but it is NOT what feeds
        // `H2`.
        const double v_propia = hs_.process(w) - off_n6_ * rest::kOffOutput;
        if (n6_out) *n6_out = qn6_ + v_propia;
        if (n3_out) *n3_out = qn3_ + u;

        corr_ = 0.0;
        return q_n4_ + h2_.process(n6_abs_oraculo - qn6_);
    }

    // THE REST POINT AS A FUNCTION OF THE KNOB — measured.
    //
    // This stage was fitted WHOLE with the circuit at drive 0,5, and its
    // rest points (`kQn3`, `kQn6`, `kVr`) are constants of that point. But
    // the gain pot moves the WHOLE pedal's rest through the shared rail:
    // from drive 1,0 to 0,0, `n3` drops 1,38 mV and `VR` 1,90 mV. With the
    // drive closed and the signal weak that offset DOMINATES (see
    // `nls_reposo_drive.h`).
    //
    // Here the CURRENT rest arrives and the DIFFERENCE against the fit's
    // rest is stored — which is what must be shifted:
    //   - at the waveshaper's INPUT, to evaluate it in its own frame;
    //   - at the OUTPUT, because `v` is deviation against `kQn6` and the
    //     true one is measured against today's rest.
    // Never called, everything stays zero and the path is bit-identical.
    void set_rest(double q_n3, double q_n6, double q_n4)
    {
        off_n3_ = q_n3 - qn3_;
        off_n6_ = q_n6 - qn6_;
        q_n4_   = q_n4;
    }

    double rest_n4() const { return q_n4_; }

    // Both banks must share the filters' ORDERS, because
    // `FixedFilter<NB,NA>` is a template and the member is ONE. It is
    // guaranteed by `NLSC_ORDERS` in `harness/gen_variantes.sh`, and
    // checked by the `static_assert` below: if they ever diverge, this does
    // not compile — infinitely better than running with the other circuit's
    // bank.
    static_assert(fijos::kH1_NB == fijos_v9ri::kH1_NB &&
                  fijos::kH1_NA == fijos_v9ri::kH1_NA &&
                  fijos::kHS_NB == fijos_v9ri::kHS_NB &&
                  fijos::kHS_NA == fijos_v9ri::kHS_NA &&
                  fijos::kH2_NB == fijos_v9ri::kH2_NB &&
                  fijos::kH2_NA == fijos_v9ri::kH2_NA,
                  "both stage-1 banks must share the same filter orders");
    static_assert(stage1::kWS_N <= kWSMax && stage1_v9ri::kWS_N <= kWSMax,
                  "kWSMax is too small for stage 1's waveshaper");

    // SEPARATE FROM `prepare()` ON PURPOSE: `set_rest()` is called
    // BEFORE `prepare()` and subtracts `qn3_`/`qn6_`, so if the bank were
    // chosen inside `prepare()` the rest would subtract against the OTHER
    // variant's. Measured: that left the waveshaper's input shifted by a
    // full VOLT and the null at −11,9 dB, with the correct bank loaded.
    // The variant's bank, COPIED into members because it is read per
    // sample.
    template <class J>
    void set_bank()
    {
        ws_n_ = J::WS_N;
        for (int i = 0; i < ws_n_; ++i) ws_[i] = J::WS[i];
// EXPERIMENT `NLSC_WS_RECENTER` — NEVER SHIPPED AS IS.
// Zeroes each waveshaper's INDEPENDENT TERM. The fits do not pass exactly
// through the origin (Q1: +9,14 uV, Q2: +1,48 uV on the 808), so at t=0, with
// every filter state at zero, each shaper injects a constant step that the
// stage high-passes then take hundreds of ms to wash out. This flag prices
// that mechanism: if the start-up bump collapses, it is attributed.
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
    // were made with VR PINNED, so that summand must be added here. `du` is
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
        // `corr_` is the previous sample's correction. The loop is implicit
        // — the base current depends on n3, which depends on the correction
        // — but its gain is minuscule (the nonlinear part is nanoamps) and
        // its bandwidth sits miles below fs/2, so one sample of delay is
        // invisible. Iterating here would be paying for nothing.
        // `off_n3_` brings `u` into the frame the waveshaper was fitted
        // in (see `set_rest`). With it inside, `kQn3 + u` is still ABSOLUTE
        // `n3`, so the `n3_out` probe below does not change.
        const double u = h1_.process(in) + corr_ + du_ + off_n3_;  // n3 deviation

        const double w = ws(u);

        const double v = hs_.process(w) - off_n6_ * rest::kOffOutput;      // n6 deviation
        if (n6_out) *n6_out = qn6_ + v;
        if (n3_out) *n3_out = qn3_ + u;

        // HERE went the rest-point displacement path, RETIRED after
        // measurement: it worsened the model from −50 dB to −25 dB.
        //
        // The idea was to compute the base current with the Gummel-Poon law,
        // remove its linear part (H1 already carries it) and inject the
        // residue into the node's impedance. It does not work, and the
        // reason is structural, not a fitting failure: the displacement is
        // ~2,4 nA over 2266 nA, i.e. 0,1 %, so rebuilding it by differences
        // demands modelling `ib` beyond 60 dB of precision. Measured, it is
        // not even close:
        //   - the law fed with ngspice's REAL `vbe` reproduces `ib` at
        //     −33,1 dB;
        //   - the empirical curve `ib(n3)` has −34 dB of hysteresis and its
        //     fit stalls there no matter the order.
        // It is catastrophic cancellation: a tiny difference between large
        // quantities modelled separately. Details, and the route that can
        // work (solving the node equations, where the DC balance holds by
        // construction instead of being reconstructed), in
        // docs/DESPLAZAMIENTO_DE_REPOSO.md §6.
        corr_ = 0.0;

        return q_n4_ + h2_.process(v);                  // n4 rests at VR
    }

    // VR's rest measured on the full circuit's .op. It is not 4,5 V: the two
    // followers' base currents pull it down.
    //
    // NOT THE SOURCE OF TRUTH, AND IT MUST NOT BE READ AS ONE. It is
    // the 808's rest, hand-written, in a file that serves both variants —
    // and the 9/9RI rests 9,17 mV higher (4,479338 vs 4,488509 V on the
    // full circuit's `.op`). The good value exists GENERATED, per variant,
    // in `nls_reposo_drive*.h` (`kvr[]`), as a function of drive besides.
    // => In the product this is DEAD: `prepare()` calls `set_rest()` with
    // the generated one. Verified with a canary — poisoning it to 1,234567
    // leaves the output BIT-IDENTICAL in both variants and both engines,
    // with the .so's md5 changing. It survives only as the default for
    // harnesses that instantiate this stage bare, which is why it stays:
    // changing it would move THEM.
    // Declared in `puerta_constantes.py`, which also scans `src/`.
    static constexpr double kVr = 4.478416;

private:
    // FILTERS IN `s`, DISCRETISED IN `prepare()`.
    // They used to be SOS cascades with tables pre-discretised at a fixed
    // rate, which tied the stage — and the whole cascade with it — to the
    // three tabulated rates. The coefficients are the SAME: they come from
    // inverting the bilinear over those tables (`harness/gen_fijos_s.py`,
    // exact control at 1e-16).
    FixedFilter<fijos::kH1_NB, fijos::kH1_NA> h1_;
    FixedFilter<fijos::kHS_NB, fijos::kHS_NA> hs_;
    FixedFilter<fijos::kH2_NB, fijos::kH2_NA> h2_;
    double corr_ = 0.0;
    double du_ = 0.0;     // the rail's contribution to `n3`, set by the caller
    // Rest displacement against the point the stage was fitted at.
    // At zero = the old behaviour, bit for bit (see `set_rest`).
    double off_n3_ = 0.0, off_n6_ = 0.0, q_n4_ = kVr;

    // COPY of the active variant's bank: read PER SAMPLE.
    // The constructor leaves the 808's, so the class is usable without
    // calling `prepare()` — which is what several harnesses do.
    double ws_[kWSMax] = {0.0};
    int    ws_n_ = stage1::kWS_N;
    // The fit's travel comes from the BANK, not from a loose constant:
    // the 9/9RI fits its own curve and may have another travel.
    double umin_ = stage1::kWS_UMIN;
    double umax_ = stage1::kWS_UMAX;
    long   out_of_range_ = 0;
    double qn3_  = stage1::kQn3;
    double qn6_  = stage1::kQn6;
};

} // namespace nlsc
