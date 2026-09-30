// GENERATED FILE — DO NOT EDIT BY HAND.
//
// The circuit's rest point as a function of the drive knob: the gain pot
// moves it. From drive 1.0 to drive 0.0 the rests shift by `n7` 6.09 mV,
// `n14` 5.71 mV, `vr` and `n4` 1.90 mV, `n3`/`n19` 1.38 mV. With the drive
// closed and a weak signal, a fixed rest would leave that offset dominating
// the error.
//
// Each waveshaper receives the displacement between the current rest and
// the rest its curve was fitted at.
//
// Cubic in the drive: a quadratic already fits within 1.05 uV over the
// whole travel, the cubic reaches 0.5 uV for three multiplies, evaluated
// once in `prepare()`.

#pragma once

namespace nlsc {
namespace rest {

// Coefficients in descending order, as np.polyval.
// vr: from 4.493294 to 4.495215 V (+1.921 mV), fit ±0.49 uV
static constexpr double kvr[4] = {+6.68777785761912707e-06, -1.04263293295633807e-04, +2.01825121654599900e-03, +4.49329407547524884e+00};
// n3: from 4.146424 to 4.148169 V (+1.745 mV), fit ±0.49 uV
static constexpr double kn3[4] = {+9.35364321755430606e-06, -9.87105444828962784e-05, +1.83397481957653916e-03, +4.14642428336156676e+00};
// n6: from 3.576309 to 3.578040 V (+1.731 mV), fit ±0.52 uV
static constexpr double kn6[4] = {+3.66749110013902685e-06, -8.84992455079379345e-05, +1.81600246775248404e-03, +3.57630860662525896e+00};
// n4: from 4.493204 to 4.495125 V (+1.921 mV), fit ±0.49 uV
static constexpr double kn4[4] = {+6.68777785328272834e-06, -1.04263293289782950e-04, +2.01825121654483847e-03, +4.49320407547524781e+00};
// n7: from 4.493615 to 4.499768 V (+6.153 mV), fit ±0.45 uV
static constexpr double kn7[4] = {+1.26975321686687857e-05, -3.17385030472292818e-04, +6.45756534509319899e-03, +4.49361513363447873e+00};
// n14: from 4.493537 to 4.499306 V (+5.769 mV), fit ±0.51 uV
static constexpr double kn14[4] = {+1.24817973821046769e-05, -2.99271895169931333e-04, +6.05551962912185496e-03, +4.49353711970638070e+00};
// n19: from 4.146424 to 4.148169 V (+1.745 mV), fit ±0.49 uV
static constexpr double kn19[4] = {+9.35364321755430606e-06, -9.87105444828962784e-05, +1.83397481957653916e-03, +4.14642428336156676e+00};
// n17: from 3.576309 to 3.578040 V (+1.731 mV), fit ±0.52 uV
static constexpr double kn17[4] = {+3.66749110013902685e-06, -8.84992455079379345e-05, +1.81600246775248404e-03, +3.57630860662525896e+00};

// Rests each waveshaper was fitted at. The difference against the
// current rest is the displacement its input receives.
static constexpr double kFit_n3 = 4.147318;
static constexpr double kFit_n6 = 3.577195;
static constexpr double kFit_n19 = 4.148169;
static constexpr double kFit_n17 = 3.578040;

inline double ev(const double (&c)[4], double g)
{
    double r = 0.0;
    for (int i = 0; i <= 3; ++i) r = r * g + c[i];
    return r;
}

// Output-offset switch: whether the waveshaper's output is corrected
// as well as its input. The output offset is a constant and the networks
// behind it block DC, so the steady state is the same either way. From
// a zeroed filter state the choice changes a transient below 35 Hz;
// `startup()` presets the filters to their DC rest, this offset term
// included, so the engine does not start from zero and does not go
// through that transient. The default corrects both;
//   -DNLSC_OUT_OFF=0.0  ->  only the input is corrected.
#ifndef NLSC_OUT_OFF
#  define NLSC_OUT_OFF 1.0
#endif
static constexpr double kOffOutput = NLSC_OUT_OFF;

} // namespace rest
} // namespace nlsc
