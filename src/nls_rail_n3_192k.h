// GENERATED FILE — DO NOT EDIT BY HAND.
//
// Internal rate: 192000 Hz.
//
// The stage fits are made against the AC response with VR pinned, so
// they describe only the signal path. This file holds the other addend
// of the superposition: what each internal node does when the rail moves.
// The rail enters each stage through its bias resistor — `R2` (510 k) to
// `n3`, `R8` (10 k) to `n9`, `R12` (510 k) to `n19` — and without this
// term the cascade loses 20 dB of accuracy in the audible band.
//
// Measurement: the pedal input is muted (`alter @Vin[acmag] = 0`) and the
// rail is driven by splitting `C11` with a series source, so `n_i/vr` is
// the transfer. `n7`/`n14` are opamp outputs, which fight a capacitive
// pin, so each stage input is cut and fed from an ideal source at rest,
// the topology the cascade has.
//
// Double counting is excluded: `out/vr` divided by `n19/vr` is 0.97 over
// the whole range, so all of stage 4's coupling enters via `n19` and is
// injected there only. In stage 1 the total coupling to `n4` (0.398 at
// 82 Hz) is the direct path through `R4`·`C2` (0.190) plus `n3/vr`
// (0.221) through the stage's path.

#pragma once

namespace nlsc {
namespace rail {

// VR -> n3  (stage 1, via R2 510k) — order 5, residual -176.8 dB
static constexpr int kN3_N = 3;
static constexpr double kN3[3][6] = {
    {+2.51583300736126034e-03, -4.10624584449952815e-03, +1.67007526930663757e-03, +1.00000000000000000e+00, -8.62571520376584244e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, +9.21628564949968809e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, +2.14663333969948855e-02, -9.70402758031708945e-01},
    {+1.00000000000000000e+00, -1.97394368215036797e+00, +9.73955749753257827e-01, +1.00000000000000000e+00, -1.99887389081004452e+00, +9.98874180681049051e-01},
};

// VR -> n14 (stage 3, via R8 10k) — order 5, residual -150.1 dB
static constexpr int kN14_N = 3;
static constexpr double kN14[3][6] = {
    {+1.26991326801058325e-03, +2.45932034066673232e-03, +1.26559345409561221e-03, +1.00000000000000000e+00, -6.74515542870182672e-02, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -6.74542192812255387e-02, +0.00000000000000000e+00, +1.00000000000000000e+00, -1.86388198243393477e-02, -9.26036775067065498e-01},
    {+1.00000000000000000e+00, -1.99429141820351052e+00, +9.94297937069166804e-01, +1.00000000000000000e+00, -1.99431985294839542e+00, +9.94326326935723981e-01},
};

// VR -> n19 (stage 4, via R12 510k) — order 3, residual -118.0 dB
static constexpr int kN19_N = 2;
static constexpr double kN19[2][6] = {
    {+1.18950915278748376e-02, -1.17320660565777401e-02, +0.00000000000000000e+00, +1.00000000000000000e+00, -9.99853798740382937e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -1.99991656809173213e+00, +9.99916569741864714e-01, +1.00000000000000000e+00, -1.99990969614301584e+00, +9.99909698169167194e-01},
};

} // namespace rail
} // namespace nlsc
