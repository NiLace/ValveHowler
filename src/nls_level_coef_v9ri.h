// GENERATED FILE — DO NOT EDIT BY HAND.
//
// The LEVEL network (E4G1 = n19/n14) as an explicit function of the
// knob, valid over its whole travel.
//
// Q2's base enters as [rpi + (beta+1)*Ze(s)] || Cb: the follower
// describes the lows and the junction the highs; both are needed.
// Fitted at four knob positions at once: two numbers for the whole
// travel, not a table. Valid because the pot sits before the base, so
// the load Q2 presents does not depend on the knob.
//
// Fit: rpi = 192.2089 kOhm, Cb = 2.6939 pF (beta = 416.4 from the netlist)
//
// Circuit:  R15 = 100000 Ohm (output load, after C9)  ·  R14 = 470 Ohm  ·  R13 = 10000 Ohm (Q2 emitter, both variants)
//   worst signal residual n19/n14 :  -105.7 dB
//   worst rail -> n19 residual    :   -95.1 dB
#pragma once

namespace nlsc {
namespace level_v9ri {

static constexpr double kScale = 6283.1853071795867;
static constexpr int kDegreeNum = 3;
static constexpr int kDegreeDen = 4;
static constexpr int kKnobDeg = 2;

// [power of s, highest to lowest][lvl^2 ... lvl^0]
static constexpr double kB[4][3] = {
    {+0.00000000000000000e+00, -1.15869099502823997e+09, +1.15869100661514974e+09},
    {+0.00000000000000000e+00, -1.82855331304656778e+05, +1.82855333133210079e+05},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +0.00000000000000000e+00},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +0.00000000000000000e+00},
};
static constexpr double kA[5][3] = {
    {-1.96124944278866309e+06, +1.94163694836077630e+06, +1.96125142365061984e+04},
    {-2.52351924506418169e+08, +2.49825283835919440e+08, +1.17283609776914001e+09},
    {-3.94784176043574334e+04, -3.62546928411178640e+05, +6.48734166638722178e+06},
    {+0.00000000000000000e+00, -6.28318530717958623e+01, +7.38060580212308469e+03},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +1.00000000000000000e+00},
};

// ═══════════════════════════════════════════════════════════════════
//  The rail path: v(n19)/v(VR), also a function of the knob.
// ═══════════════════════════════════════════════════════════════════
// The rail enters `n19` via `R12`, after the pot, so the knob does not
// attenuate it the way it attenuates the signal: at 1 kHz this bank's
// v(n19)/v(VR) rises from about 0.013 at lvl = 0 (LEVEL at maximum) to
// about 0.76 at lvl = 0.75, roughly 59x, and as the volume drops the rail
// term carries a growing share of n19. A filter frozen at one knob
// position cannot follow that, so this path is a function of the knob
// as well. Computed from the same matrix as
// `kB`/`kA` with the other right-hand side and the same fitted Q2-base
// model: the superposition the cascade sums.
static constexpr int kDegreeNumR = 3;
static constexpr int kDegreeDenR = 4;
static constexpr int kKnobDegR = 2;

static constexpr double kBr[4][3] = {
    {-2.27194312750635266e+08, +1.38361336465136886e+09, +1.38588669593612831e+07},
    {-3.58539865303248589e+04, -1.43240201196617913e+05, +5.85996103379713371e+06},
    {+0.00000000000000000e+00, -5.70633918585143434e+01, +6.67932613126670458e+03},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +9.08192088387237417e-01},
};
static constexpr double kAr[5][3] = {
    {-1.96124944278866309e+06, +1.94163694836077630e+06, +1.96125142365061984e+04},
    {-2.52351924506418169e+08, +2.49825283835919440e+08, +1.17283609776914001e+09},
    {-3.94784176043574334e+04, -3.62546928411178640e+05, +6.48734166638722178e+06},
    {+0.00000000000000000e+00, -6.28318530717958623e+01, +7.38060580212308469e+03},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +1.00000000000000000e+00},
};

// ═══════════════════════════════════════════════════════════════
//  Open base + explicit base current (for the stage-4 subsystem)
// ═══════════════════════════════════════════════════════════════
// n19 = kBo/kAo · n14  +  kBro/kAo · vr  +  kBz/kAo · ib
// Without `rpi`, `beta` or `Cb`: the network is passive and the
// solution is exact superposition. The base is represented by the
// current the subsystem computes per sample.
static constexpr int kDegreeNumO = 2;
static constexpr int kDegreeDenO = 2;
static constexpr int kKnobDegO = 2;
static constexpr int kDegreeNumZ = 2;
static constexpr int kDegreeNumRo = 2;
static constexpr double kBo[3][3] = {
    {+0.00000000000000000e+00, -2.01339929782222927e+05, +2.01339931795622222e+05},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +0.00000000000000000e+00},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +0.00000000000000000e+00},
};
static constexpr double kAo[3][3] = {
    {-3.94784176043574334e+04, +3.90836334283138567e+04, +2.03748117681619333e+05},
    {+0.00000000000000000e+00, -6.28318530717958623e+01, +1.01787603295778217e+03},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +1.00000000000000000e+00},
};
static constexpr double kBro[3][3] = {
    {-3.94784176043574334e+04, +2.40423563210536784e+05, +2.40818588599712302e+03},
    {+0.00000000000000000e+00, -6.28318530717958623e+01, +1.01787603295778217e+03},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, +1.00000000000000000e+00},
};
static constexpr double kBz[3][3] = {
    {+2.01339929782222900e+10, -1.99326530484400673e+10, -2.01340133135554016e+08},
    {+0.00000000000000000e+00, +3.20442450666158907e+07, -3.55691126968727827e+08},
    {+0.00000000000000000e+00, +0.00000000000000000e+00, -5.10000000000000000e+05},
};

} // namespace level_v9ri
} // namespace nlsc
