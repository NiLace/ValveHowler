// nls_juegos.h — EACH VARIANT'S COEFFICIENT BANK, in ONE place.
//
// WHY IT EXISTS
// -----------------
// The whole plugin was already N-variant except the cascade:
// `gen_variantes.sh` generates one bank per row, the DK engine builds its
// matrices from `kVariants[idx]`, the harnesses take the index, and the GUI
// and `.ttl` offer whatever the table says. The cascade did NOT: it chose
// with `if (variant == 1) … else …` in EIGHT places across three files, and
// every `else` fell through to the 808.
//
// => Adding a third row to `nls_variantes.h` — which that file explicitly
// invites — would have made the manifest announce a new circuit while the
// DEFAULT engine delivered the 808, with not a single error. A silent
// failure the rest of the system already permits is not hypothetical.
//
// HERE each variant enumerates its symbols ONCE, and the code that uses
// them is written ONCE (templates). Adding a variant is TWO things and both
// fail at compile time if missing: a `struct` here and a `case` in the one
// dispatch, in `Cascade4::prepare`.
//
// WHY `static constexpr auto&` AND NOT POINTERS: a reference keeps the
// ARRAY TYPE (`const double (&)[7][1]`), so `ParamFilter::prepare` still
// checks at compile time that the table has the rows its template expects.
// With pointers that check disappears exactly where it hurts most — a
// wrong-size table raises no error; it becomes a different filter.
//
// What does NOT belong here, and it is not an oversight:
//   · `tone::`  — stage 3 does NOT depend on the variant. Measured: in the
//     full circuit, stages 1-3 move by less than 0,0004 dB when `R13`/`R14`
//     change.
//   · `rest::kOffOutput` — that is `NLSC_OUT_OFF`, a compile constant.
//   · `kScale` — 2*pi*1000, the same convention in every bank.
#pragma once

namespace nlsc {

// ─────────────────────────────────────────────────────────────────────────────
//  The **OD-8** bank (index 0) — the one that ships, and carries NO suffix.
// ─────────────────────────────────────────────────────────────────────────────
struct Bank808 {
    // stage 1 — fixed filters and its waveshaper (Q1)
    static constexpr auto& H1_B = fijos::kH1_B;   static constexpr auto& H1_A = fijos::kH1_A;
    static constexpr auto& HS_B = fijos::kHS_B;   static constexpr auto& HS_A = fijos::kHS_A;
    static constexpr auto& H2_B = fijos::kH2_B;   static constexpr auto& H2_A = fijos::kH2_A;
    static constexpr const double* WS = stage1::kWS;
    static constexpr int    WS_N = stage1::kWS_N;
    static constexpr double Qn3 = stage1::kQn3, Qn6 = stage1::kQn6;
    // The travel of Q1's waveshaper fit, for its tangent GUARD. Each
    // variant fits its own curve, so each brings its own range.
    static constexpr double WS_UMIN = stage1::kWS_UMIN, WS_UMAX = stage1::kWS_UMAX;

    // stage 4 — fixed filters and Q2's waveshaper
    static constexpr auto& E4GS_B = fijos::kE4GS_B; static constexpr auto& E4GS_A = fijos::kE4GS_A;
    static constexpr auto& E4G2_B = fijos::kE4G2_B; static constexpr auto& E4G2_A = fijos::kE4G2_A;
    static constexpr const double* WS2 = e234::kWS2;
    static constexpr int    WS2_N = e234::kWS2_N;
    static constexpr double WS2_UMIN = e234::kWS2_UMIN, WS2_UMAX = e234::kWS2_UMAX;

    // rail and opamp correction
    static constexpr auto& N3_B  = fijos::kN3_B;  static constexpr auto& N3_A  = fijos::kN3_A;
    static constexpr auto& VRA_B = fijos::kVRA_B; static constexpr auto& VRA_A = fijos::kVRA_A;
    static constexpr auto& VRB_B = fijos::kVRB_B; static constexpr auto& VRB_A = fijos::kVRB_A;
    static constexpr auto& VRP_B = fijos::kVRP_B; static constexpr auto& VRP_A = fijos::kVRP_A;
    // `VRC` is the `n19` branch, which used to enter through the CONSTANT
    // `G_n19`. Measured: its admittance moves 15,7 dB from DC to the top of
    // the band, because `n19` reaches the rail through `C8` towards the
    // pot's `n18` as well, not just through `R12`. Its three passive sisters
    // come out FLAT at 0,0 dB — the control that the census does not flag
    // everything.
    static constexpr auto& VRC_B = fijos::kVRC_B; static constexpr auto& VRC_A = fijos::kVRC_A;
    static constexpr auto& E2C_B = fijos::kE2C_B; static constexpr auto& E2C_A = fijos::kE2C_A;
    static constexpr double ScaleF = fijos::kScale;

    // level — the FITTED bank (tabulated path) and the OPEN-BASE one (subsystem)
    static constexpr auto& nB  = level::kB;   static constexpr auto& nA  = level::kA;
    static constexpr auto& nBr = level::kBr;  static constexpr auto& nAr = level::kAr;
    static constexpr auto& nBo = level::kBo;  static constexpr auto& nAo = level::kAo;
    static constexpr auto& nBz = level::kBz;  static constexpr auto& nBro = level::kBro;
    static constexpr double ScaleN = level::kScale;

    // rest points as functions of the knob, and each waveshaper's fit frame
    static constexpr auto& Rvr  = rest::kvr;   static constexpr auto& Rn3  = rest::kn3;
    static constexpr auto& Rn4  = rest::kn4;   static constexpr auto& Rn6  = rest::kn6;
    static constexpr auto& Rn7  = rest::kn7;   static constexpr auto& Rn14 = rest::kn14;
    static constexpr auto& Rn17 = rest::kn17;  static constexpr auto& Rn19 = rest::kn19;
    static constexpr double Fit_n19 = rest::kFit_n19, Fit_n17 = rest::kFit_n17;

    // rest points and rail conductances, from the measurement's working point
    static constexpr double Q_vr = e234::kQ_vr,  Q_n3  = e234::kQ_n3;
    static constexpr double Q_n4 = e234::kQ_n4,  Q_n7  = e234::kQ_n7;
    static constexpr double Q_n11 = e234::kQ_n11, Q_n14 = e234::kQ_n14;
    static constexpr double Q_n19 = e234::kQ_n19, Q_out = e234::kQ_out;
    static constexpr double G_n3 = e234::kG_n3,  G_n4  = e234::kG_n4;
    static constexpr double G_n11 = e234::kG_n11, G_n19 = e234::kG_n19;

    // What stage 4 needs to SOLVE Q2 instead of tabulating it. Comes from
    // `nls_variantes.h`, the ONE table the GUI and the `.ttl` also read.
    static constexpr int    idx = 0;
    // HOW Q2 is solved in stage 4: `false` = the tabulated polynomial,
    // `true` = the scalar-Newton subsystem. It lives HERE, in the bank,
    // and not in an `(idx == 1) ? … : …` in the engine: that was another
    // two-way hard-wire, and a third variant would have inherited the 808's
    // choice in silence.
    static constexpr bool   use_sub = kSubEn808;
};

// ─────────────────────────────────────────────────────────────────────────────
//  The **OD-9** bank (index 1)
// ─────────────────────────────────────────────────────────────────────────────
struct Bank9ri {
    static constexpr auto& H1_B = fijos_v9ri::kH1_B;   static constexpr auto& H1_A = fijos_v9ri::kH1_A;
    static constexpr auto& HS_B = fijos_v9ri::kHS_B;   static constexpr auto& HS_A = fijos_v9ri::kHS_A;
    static constexpr auto& H2_B = fijos_v9ri::kH2_B;   static constexpr auto& H2_A = fijos_v9ri::kH2_A;
    static constexpr const double* WS = stage1_v9ri::kWS;
    static constexpr int    WS_N = stage1_v9ri::kWS_N;
    static constexpr double Qn3 = stage1_v9ri::kQn3, Qn6 = stage1_v9ri::kQn6;
    // The travel of Q1's waveshaper fit, for its tangent GUARD. Each
    // variant fits its own curve, so each brings its own range.
    static constexpr double WS_UMIN = stage1_v9ri::kWS_UMIN, WS_UMAX = stage1_v9ri::kWS_UMAX;

    static constexpr auto& E4GS_B = fijos_v9ri::kE4GS_B; static constexpr auto& E4GS_A = fijos_v9ri::kE4GS_A;
    static constexpr auto& E4G2_B = fijos_v9ri::kE4G2_B; static constexpr auto& E4G2_A = fijos_v9ri::kE4G2_A;
    static constexpr const double* WS2 = e234_v9ri::kWS2;
    static constexpr int    WS2_N = e234_v9ri::kWS2_N;
    static constexpr double WS2_UMIN = e234_v9ri::kWS2_UMIN, WS2_UMAX = e234_v9ri::kWS2_UMAX;

    static constexpr auto& N3_B  = fijos_v9ri::kN3_B;  static constexpr auto& N3_A  = fijos_v9ri::kN3_A;
    static constexpr auto& VRA_B = fijos_v9ri::kVRA_B; static constexpr auto& VRA_A = fijos_v9ri::kVRA_A;
    static constexpr auto& VRB_B = fijos_v9ri::kVRB_B; static constexpr auto& VRB_A = fijos_v9ri::kVRB_A;
    static constexpr auto& VRP_B = fijos_v9ri::kVRP_B; static constexpr auto& VRP_A = fijos_v9ri::kVRP_A;
    // `VRC` is the `n19` branch — see the 808 bank's note; same mechanism,
    // this variant's own numbers.
    static constexpr auto& VRC_B = fijos_v9ri::kVRC_B; static constexpr auto& VRC_A = fijos_v9ri::kVRC_A;
    static constexpr auto& E2C_B = fijos_v9ri::kE2C_B; static constexpr auto& E2C_A = fijos_v9ri::kE2C_A;
    static constexpr double ScaleF = fijos_v9ri::kScale;

    static constexpr auto& nB  = level_v9ri::kB;   static constexpr auto& nA  = level_v9ri::kA;
    static constexpr auto& nBr = level_v9ri::kBr;  static constexpr auto& nAr = level_v9ri::kAr;
    static constexpr auto& nBo = level_v9ri::kBo;  static constexpr auto& nAo = level_v9ri::kAo;
    static constexpr auto& nBz = level_v9ri::kBz;  static constexpr auto& nBro = level_v9ri::kBro;
    static constexpr double ScaleN = level_v9ri::kScale;

    static constexpr auto& Rvr  = rest_v9ri::kvr;   static constexpr auto& Rn3  = rest_v9ri::kn3;
    static constexpr auto& Rn4  = rest_v9ri::kn4;   static constexpr auto& Rn6  = rest_v9ri::kn6;
    static constexpr auto& Rn7  = rest_v9ri::kn7;   static constexpr auto& Rn14 = rest_v9ri::kn14;
    static constexpr auto& Rn17 = rest_v9ri::kn17;  static constexpr auto& Rn19 = rest_v9ri::kn19;
    static constexpr double Fit_n19 = rest_v9ri::kFit_n19, Fit_n17 = rest_v9ri::kFit_n17;

    static constexpr double Q_vr = e234_v9ri::kQ_vr,  Q_n3  = e234_v9ri::kQ_n3;
    static constexpr double Q_n4 = e234_v9ri::kQ_n4,  Q_n7  = e234_v9ri::kQ_n7;
    static constexpr double Q_n11 = e234_v9ri::kQ_n11, Q_n14 = e234_v9ri::kQ_n14;
    static constexpr double Q_n19 = e234_v9ri::kQ_n19, Q_out = e234_v9ri::kQ_out;
    static constexpr double G_n3 = e234_v9ri::kG_n3,  G_n4  = e234_v9ri::kG_n4;
    static constexpr double G_n11 = e234_v9ri::kG_n11, G_n19 = e234_v9ri::kG_n19;

    static constexpr int    idx = 1;
    static constexpr bool   use_sub = kSubEnV9ri;
};

// THE GATE. Each bank declares ITS index, and here the list is checked
// against the whole table. Adding a row to `nls_variantes.h` makes this NOT
// compile — which is exactly the intent: a circuit announced but not
// delivered is a silent failure, and a silent failure gets turned into a
// compile error.
static_assert(Bank808::idx == 0 && Bank9ri::idx == 1,
              "bank indices must follow the order of kVariants");
// It counts CIRCUITS, not selector rows. A row that only changes a knob's
// law (`ToneLaw`) reuses a circuit and needs NO bank, so it must not have to
// touch this number — bumping it for such a row would disarm the guard for
// the next row that IS a new circuit. See the two tables in
// `nls_variantes.h`.
static_assert(kNumCircuits == 2,
              "A CIRCUIT'S BANK IS MISSING. To add one: (1) generate its bank with "
              "`harness/gen_variantes.sh` (one more entry in VARIANTES=), (2) copy a "
              "`struct Bank…` from this file changing the namespace suffix, "
              "and (3) add its `case` to the ONE dispatch, in `Cascade4::prepare`. "
              "⛔ Do not bump this number without doing all three.");

} // namespace nlsc
