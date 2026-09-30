// GENERATED FILE — DO NOT EDIT BY HAND.
//
// Stage 3 (TONE) as an explicit function of the knob, by exact
// algebra, valid over the knob's whole travel.
//
// b(s) and a(s) with `s` in units of kScale, monic denominator.
// Each coefficient is a degree-2 polynomial in `tone` (Horner).
//
// Properties:
//   · residual against ngspice: -120.2 to -132.8 dB at 5 knob positions
//   · at tone=0.5 it matches the fitted stage-3 banks of
//     `nls_e234_coef*.h`
//   · left-half-plane poles over the whole travel
#pragma once

namespace nlsc {
namespace tone {

static constexpr double kScale = 6283.1853071795867;
static constexpr int kDegreeNum = 3;
static constexpr int kDegreeDen = 4;

static constexpr int kKnobDeg = 2;

// [power of s, highest to lowest][t^2 ... t^0]
static constexpr double kB[4][3] = {
    {-7.97455187033592662e-05, +7.97455187033592662e-05, +2.20177381214971160e-04},
    {-1.59491037431898822e+04, +1.67465589302234730e+04, +1.75441726986499390e+02},
    {-2.51324623623180265e+01, +2.63890854802454342e+01, +5.77180788589165445e+02},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +9.09080947051778221e-01},
};
static constexpr double kA[5][3] = {
    {-7.35585757937967077e+00, +7.75112947689782050e+00, +8.52628198938115978e-02},
    {-2.20525452708957200e+04, +2.20526062830849878e+04, +2.43146389571143771e+02},
    {-1.75789399432083665e+04, +1.67814863072403168e+04, +1.78853041092934905e+03},
    {-2.76459851611921152e+01, +2.63893643366018971e+01, +6.37419468183096228e+02},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +1.00000000000000000e+00},
};

// ═══════════════════════════════════════════════════════════════
//  The rail path: v(n14)/v(VR), also a function of the knob.
// ═══════════════════════════════════════════════════════════════
// The rail enters via `R8`, before the tone network, so what it
// injects is filtered by the knob. Computed from the same matrix as
// `kB`/`kA` with the other right-hand side, opamp included: the
// superposition the cascade sums.
static constexpr int kDegreeNumR = 4;
static constexpr int kDegreeDenR = 4;
static constexpr int kKnobDegR = 2;

static constexpr double kBr[5][3] = {
    {-5.45813555311992771e-03, +5.73098776033448189e-03, +6.30411579170300741e-05},
    {-4.51595444477832005e-03, +4.54387900077281651e-03, +4.90853277348840154e-04},
    {-1.59491050464606292e+03, +1.67465602394159191e+03, +1.75443588386582867e+01},
    {-2.51324623623180265e+00, +2.63890854802454333e+00, +5.77180840228669183e+01},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +9.09080947051778276e-02},
};
static constexpr double kAr[5][3] = {
    {-7.35585757937967077e+00, +7.75112947689782050e+00, +8.52628198938115978e-02},
    {-2.20525452708957200e+04, +2.20526062830849878e+04, +2.43146389571143771e+02},
    {-1.75789399432083665e+04, +1.67814863072403168e+04, +1.78853041092934905e+03},
    {-2.76459851611921152e+01, +2.63893643366018971e+01, +6.37419468183096228e+02},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +1.00000000000000000e+00},
};

// ═══════════════════════════════════════════════════════════════
//  `VRA`: the current `n7` feeds the rail through `R8`.
// ═══════════════════════════════════════════════════════════════
// `VRA` is physically `(n9/n7)/R8`, and `n9` is loaded by the tone
// network, so it depends on the knob. The counterpart of the rail
// paths in the opposite direction: not what the rail does to a node,
// but what the node feeds the rail.
// From the same matrix, solving for `v9` instead of `v14`.
static constexpr int kDegreeNumV = 3;
static constexpr int kDegreeDenV = 4;
static constexpr int kKnobDegV = 2;

static constexpr double kBv[4][3] = {
    {-5.32145952017390195e-04, +5.60741168542538667e-04, +6.16818147638823058e-06},
    {-1.59492717637460713e+00, +1.59492807926009794e+00, +1.75645732675157491e-02},
    {-2.51327137372065390e-03, +2.51327272459050564e-03, +5.77187172816626543e-02},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +9.09090494219479318e-05},
};
static constexpr double kAv[5][3] = {
    {-7.35585757937967077e+00, +7.75112947689782050e+00, +8.52628198938115978e-02},
    {-2.20525452708957200e+04, +2.20526062830849878e+04, +2.43146389571143771e+02},
    {-1.75789399432083665e+04, +1.67814863072403168e+04, +1.78853041092934905e+03},
    {-2.76459851611921152e+01, +2.63893643366018971e+01, +6.37419468183096228e+02},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +1.00000000000000000e+00},
};

} // namespace tone
} // namespace nlsc
