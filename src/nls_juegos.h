// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// nls_juegos.h — each circuit's coefficient bank, in one place.
//
// Each variant enumerates its symbols once, and the code that uses them is
// written once (templates), so no `if (variant == …)` can fall through to the
// wrong circuit. Adding a circuit takes two steps: generate its bank, and
// add a `struct` here and its name to `Banks`. `Cascade4::prepare` dispatches
// over `Banks`, so a bank in the list is reachable and a circuit without one
// does not compile.
//
// Why `static constexpr auto&` and not pointers: a reference keeps the
// array type (`const double (&)[7][1]`), so `ParamFilter::prepare` still
// checks at compile time that the table has the rows its template expects.
// With a pointer a wrong-size table raises no error; it becomes a different
// filter.
//
// What does not belong here:
//   · `tone::`  — stage 3 does not depend on the variant: in the full
//     circuit, stages 1-3 move by less than 0,0004 dB when `R14`/`R15`
//     change.
//   · `rest::kOffOutput` — that is `NLSC_OUT_OFF`, a compile constant.
//   · `kScale` — 2*pi*1000, the same convention in every bank.
#pragma once

namespace nlsc {

// ─────────────────────────────────────────────────────────────────────────────
//  The OD-8 bank (index 0), whose namespaces carry no suffix.
// ─────────────────────────────────────────────────────────────────────────────
struct Bank808 {
    // stage 1 — fixed filters and its waveshaper (Q1)
    static constexpr auto& H1_B = fixed::kH1_B;   static constexpr auto& H1_A = fixed::kH1_A;
    static constexpr auto& HS_B = fixed::kHS_B;   static constexpr auto& HS_A = fixed::kHS_A;
    static constexpr auto& H2_B = fixed::kH2_B;   static constexpr auto& H2_A = fixed::kH2_A;
    static constexpr const double* WS = stage1::kWS;
    static constexpr int    WS_N = stage1::kWS_N;
    static constexpr double Qn3 = stage1::kQn3, Qn6 = stage1::kQn6;
    // The travel of Q1's waveshaper fit, for its tangent guard. Each
    // variant fits its own curve, so each brings its own range.
    static constexpr double WS_UMIN = stage1::kWS_UMIN, WS_UMAX = stage1::kWS_UMAX;

    // stage 4 — fixed filters and Q2's waveshaper
    static constexpr auto& E4GS_B = fixed::kE4GS_B; static constexpr auto& E4GS_A = fixed::kE4GS_A;
    static constexpr auto& E4G2_B = fixed::kE4G2_B; static constexpr auto& E4G2_A = fixed::kE4G2_A;
    static constexpr const double* WS2 = e234::kWS2;
    static constexpr int    WS2_N = e234::kWS2_N;
    static constexpr double WS2_UMIN = e234::kWS2_UMIN, WS2_UMAX = e234::kWS2_UMAX;

    // rail and opamp correction
    static constexpr auto& N3_B  = fixed::kN3_B;  static constexpr auto& N3_A  = fixed::kN3_A;
    static constexpr auto& VRA_B = fixed::kVRA_B; static constexpr auto& VRA_A = fixed::kVRA_A;
    static constexpr auto& VRB_B = fixed::kVRB_B; static constexpr auto& VRB_A = fixed::kVRB_A;
    static constexpr auto& VRP_B = fixed::kVRP_B; static constexpr auto& VRP_A = fixed::kVRP_A;
    // `VRC` is the `n19` branch as a filter rather than a constant
    // conductance: its admittance moves 15,7 dB from DC to the top of the
    // band, because `n19` reaches the rail through `C8` towards the pot's
    // `n18` as well, not just through `R12`. The other three rail branches
    // are flat and stay constants.
    static constexpr auto& VRC_B = fixed::kVRC_B; static constexpr auto& VRC_A = fixed::kVRC_A;
    static constexpr auto& E2C_B = fixed::kE2C_B; static constexpr auto& E2C_A = fixed::kE2C_A;
    static constexpr double ScaleF = fixed::kScale;

    // level — the fitted bank (tabulated path) and the open-base one (subsystem)
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

    // What stage 4 needs to solve Q2 instead of tabulating it. Comes from
    // `nls_variantes.h`, the one table the GUI and the `.ttl` also read.
    static constexpr int    idx = 0;
    // How Q2 is solved in stage 4: `false` = the tabulated polynomial,
    // `true` = the scalar-Newton subsystem. It lives in the bank, not in an
    // index comparison in the engine, so each circuit states its own choice.
    static constexpr bool   use_sub = kSubIn808;
};

// ─────────────────────────────────────────────────────────────────────────────
//  The **OD-9** bank (index 1)
// ─────────────────────────────────────────────────────────────────────────────
struct Bank9ri {
    static constexpr auto& H1_B = fixed_v9ri::kH1_B;   static constexpr auto& H1_A = fixed_v9ri::kH1_A;
    static constexpr auto& HS_B = fixed_v9ri::kHS_B;   static constexpr auto& HS_A = fixed_v9ri::kHS_A;
    static constexpr auto& H2_B = fixed_v9ri::kH2_B;   static constexpr auto& H2_A = fixed_v9ri::kH2_A;
    static constexpr const double* WS = stage1_v9ri::kWS;
    static constexpr int    WS_N = stage1_v9ri::kWS_N;
    static constexpr double Qn3 = stage1_v9ri::kQn3, Qn6 = stage1_v9ri::kQn6;
    // The travel of Q1's waveshaper fit, for its tangent guard. Each
    // variant fits its own curve, so each brings its own range.
    static constexpr double WS_UMIN = stage1_v9ri::kWS_UMIN, WS_UMAX = stage1_v9ri::kWS_UMAX;

    static constexpr auto& E4GS_B = fixed_v9ri::kE4GS_B; static constexpr auto& E4GS_A = fixed_v9ri::kE4GS_A;
    static constexpr auto& E4G2_B = fixed_v9ri::kE4G2_B; static constexpr auto& E4G2_A = fixed_v9ri::kE4G2_A;
    static constexpr const double* WS2 = e234_v9ri::kWS2;
    static constexpr int    WS2_N = e234_v9ri::kWS2_N;
    static constexpr double WS2_UMIN = e234_v9ri::kWS2_UMIN, WS2_UMAX = e234_v9ri::kWS2_UMAX;

    static constexpr auto& N3_B  = fixed_v9ri::kN3_B;  static constexpr auto& N3_A  = fixed_v9ri::kN3_A;
    static constexpr auto& VRA_B = fixed_v9ri::kVRA_B; static constexpr auto& VRA_A = fixed_v9ri::kVRA_A;
    static constexpr auto& VRB_B = fixed_v9ri::kVRB_B; static constexpr auto& VRB_A = fixed_v9ri::kVRB_A;
    static constexpr auto& VRP_B = fixed_v9ri::kVRP_B; static constexpr auto& VRP_A = fixed_v9ri::kVRP_A;
    // `VRC` is the `n19` branch — see the 808 bank's note; same mechanism,
    // this variant's own numbers.
    static constexpr auto& VRC_B = fixed_v9ri::kVRC_B; static constexpr auto& VRC_A = fixed_v9ri::kVRC_A;
    static constexpr auto& E2C_B = fixed_v9ri::kE2C_B; static constexpr auto& E2C_A = fixed_v9ri::kE2C_A;
    static constexpr double ScaleF = fixed_v9ri::kScale;

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
    static constexpr bool   use_sub = kSubInV9ri;
};

// The banks, in circuit order. This list is the dispatch: `Cascade4::prepare`
// walks it, so nothing else names a bank by index.
template <class... B>
struct BankList {
    static constexpr int size = int(sizeof...(B));
};
using Banks = BankList<Bank808, Bank9ri>;

// A list covers the circuit table when it has one bank per circuit and each
// bank's `idx` is its position. It counts circuits, not selector rows: a row
// that only changes a knob's law (`ToneLaw`) reuses a circuit and needs no
// bank. See the two tables in `nls_variantes.h`.
template <class... B>
constexpr bool banks_cover_circuits(BankList<B...>)
{
    int pos = 0;
    bool in_order = true;
    ((in_order = in_order && B::idx == pos++), ...);
    return in_order && int(sizeof...(B)) == kNumCircuits;
}
static_assert(banks_cover_circuits(Banks{}),
              "Banks must hold one bank per circuit of kCircuits, in order. To add a "
              "circuit: generate its coefficient bank, copy a `struct Bank…` from this "
              "file changing the namespace suffix and `idx`, and append it to `Banks`.");

} // namespace nlsc
