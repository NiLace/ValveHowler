// GENERATED FILE — DO NOT EDIT BY HAND.
//
// Stage 1 linear filters, fitted against ngspice's AC sweep and
// discretised by bilinear at 192000 Hz.
//
// H1 needs more than first order (which stops at -46.8 dB): the
// follower's base impedance depends on C2, the output capacitor, and
// the input network inherits that pole.
//
// Fit residuals against ngspice (20 Hz - 20 kHz band):
//   H1: order 4, -160.5 dB   (n3/in — input network (C1 against R1+R2‖Zbase))
//   HS: order 5, -185.4 dB   (n6/n3 normalised by the waveshaper's slope — only the residual charge step)
//   H2: order 4, -182.7 dB   (n4/n6 — output coupling (C2 against R4))

#pragma once

namespace nlsc {
namespace stage1 {

static constexpr double kFs = 192000.0;

// n3/in — input network (C1 against R1+R2‖Zbase)
static constexpr int kH1_N = 2;
static constexpr double kH1[2][6] = {
    {+9.96108483217948870e-01, +3.78516230160035549e-01, -6.16478619352722301e-01, +1.00000000000000000e+00, +3.77782673376677813e-01, -6.17516874384691494e-01},
    {+1.00000000000000000e+00, -1.99948301797532668e+00, +9.99483017975459909e-01, +1.00000000000000000e+00, -1.99887388572430202e+00, +9.98874175598759906e-01},
};

// n6/n3 normalised by the waveshaper's slope — only the residual charge step
static constexpr int kHS_N = 3;
static constexpr double kHS[3][6] = {
    {+9.99965147415947886e-01, -8.64052517358365080e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, -8.64083041194313117e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, +1.93196485687989150e+00, +9.33692829349302422e-01, +1.00000000000000000e+00, +1.93187845135800740e+00, +9.33612105598186193e-01},
    {+1.00000000000000000e+00, -1.97258011813071610e+00, +9.72594124747499733e-01, +1.00000000000000000e+00, -1.97258383591506625e+00, +9.72597742134990173e-01},
};

// n4/n6 — output coupling (C2 against R4)
static constexpr int kH2_N = 2;
static constexpr double kH2[2][6] = {
    {+9.99739550109634334e-01, -1.83664017042087657e+00, +8.40529141916025457e-01, +1.00000000000000000e+00, -1.83711848384121956e+00, +8.40747950883008466e-01},
    {+1.00000000000000000e+00, -1.99596752530861332e+00, +9.95967525312809299e-01, +1.00000000000000000e+00, -1.99544681839266258e+00, +9.95448918133227334e-01},
};

// Not read by the engine: impedance seen by node n3, in ohms, which
// turns the non-linear base current into a voltage.
// Fit residual: -80.8 dB
static constexpr int kZN3_N = 2;
static constexpr double kZN3[2][6] = {
    {+1.13123463915095135e+03, -8.71838215182595150e+02, +0.00000000000000000e+00, +1.00000000000000000e+00, -9.99266460990731686e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -1.99944211136125638e+00, +9.99442123427919382e-01, +1.00000000000000000e+00, -1.99958345182121722e+00, +9.99583460952938041e-01},
};

// Base current by the Gummel-Poon law, not by a fit:
//   ib(vbe) = (IS/BF)*(exp(vbe/VT)-1) + ISE*(exp(vbe/(NE*VT))-1)
// The parameters come from the 2N3904 model card. At these currents
// the recombination term ISE/NE (1685 nA) dominates the ideal /BF
// term (606 nA). The law reproduces the measured current to 1.1% and
// its slope to 0.3% with no curve extraction, which a static
// extraction could not do: ib is not memoryless at 8 kHz.
static constexpr double kIS  = 6.734e-15;
static constexpr double kBF  = 416.4;
static constexpr double kISE = 6.734e-15;
static constexpr double kNE  = 1.259;
static constexpr double kVT  = 0.025852;
// Slope measured in ngspice rather than the law's: it is the one H1
// already carries, and subtracting the law's would leave a spurious
// -55 dB linear term.
static constexpr double kGib = 1.4565e-6;   // A/V, d(ib)/d(n3)

// Emitter follower waveshaper, fitted on the 1.4 V run, which covers
// the whole real guitar range: an extrapolated polynomial diverges
// (+41 dB at 2.8 V).
// Input and output are deviations from rest, not absolute.
// Fit residual: -92.1 dB
static constexpr double kQn3 = 4.147318;   // n3 rest (not VR)
static constexpr double kQn6 = 3.577195;   // n6 rest
static constexpr int kWS_N = 14;

// The fit's travel.
// Outside it the polynomial diverges (+41 dB at 2.8 V), so the
// engine continues the curve along its tangent at both edges.
static constexpr double kWS_UMIN = -1.400217112;
static constexpr double kWS_UMAX = +1.392811438;
// descending order, as np.polyval
static constexpr double kWS[] = {
    -6.36148047073371316e-06,
    -7.87116181385396886e-06,
    +2.32601392845176634e-05,
    +7.65938197974738461e-05,
    -5.82960123316181067e-05,
    -1.58909448721491190e-04,
    -4.74070300066420180e-06,
    +3.10867743021148125e-04,
    -2.59298960914636310e-04,
    +4.55826227688464921e-04,
    -1.38822147454969449e-03,
    +3.88891865524565146e-03,
    +9.85326236102061936e-01,
    +9.10180119519233157e-06,
};

} // namespace stage1
} // namespace nlsc
