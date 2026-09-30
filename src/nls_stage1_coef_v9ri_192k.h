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
//   HS: order 5, -185.2 dB   (n6/n3 normalised by the waveshaper's slope — only the residual charge step)
//   H2: order 4, -182.6 dB   (n4/n6 — output coupling (C2 against R4))

#pragma once

namespace nlsc {
namespace stage1_v9ri {

static constexpr double kFs = 192000.0;

// n3/in — input network (C1 against R1+R2‖Zbase)
static constexpr int kH1_N = 2;
static constexpr double kH1[2][6] = {
    {+9.96108483395263256e-01, +3.78722338093406719e-01, -6.16271938298700528e-01, +1.00000000000000000e+00, +3.77989587226022827e-01, -6.17309844955822640e-01},
    {+1.00000000000000000e+00, -1.99948301794892913e+00, +9.99483017949062913e-01, +1.00000000000000000e+00, -1.99887388569784541e+00, +9.98874175572319167e-01},
};

// n6/n3 normalised by the waveshaper's slope — only the residual charge step
static constexpr int kHS_N = 3;
static constexpr double kHS[3][6] = {
    {+9.99965143414733526e-01, -8.64053544168493559e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, -8.64084071497057349e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, +1.94084221221473907e+00, +9.42590024799846216e-01, +1.00000000000000000e+00, +1.94075580837878792e+00, +9.42508531999562083e-01},
    {+1.00000000000000000e+00, -1.97257819911503085e+00, +9.72592206730759523e-01, +1.00000000000000000e+00, -1.97258191689815776e+00, +9.72595824109864560e-01},
};

// n4/n6 — output coupling (C2 against R4)
static constexpr int kH2_N = 2;
static constexpr double kH2[2][6] = {
    {+9.99739550109765229e-01, -1.83663973154376303e+00, +8.40528564962800040e-01, +1.00000000000000000e+00, -1.83711804484927188e+00, +8.40747373778908624e-01},
    {+1.00000000000000000e+00, -1.99595017460028701e+00, +9.95950174604503524e-01, +1.00000000000000000e+00, -1.99542946768368057e+00, +9.95431576458897238e-01},
};

// Not read by the engine: impedance seen by node n3, in ohms, which
// turns the non-linear base current into a voltage.
// Fit residual: -80.8 dB
static constexpr int kZN3_N = 2;
static constexpr double kZN3[2][6] = {
    {+1.13113717072736540e+03, -8.71739197439542863e+02, +0.00000000000000000e+00, +1.00000000000000000e+00, -9.99266284985696251e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -1.99944186083430120e+00, +9.99441872906046025e-01, +1.00000000000000000e+00, -1.99958336083177279e+00, +9.99583369965204693e-01},
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
static constexpr double kWS_UMIN = -1.400217132;
static constexpr double kWS_UMAX = +1.392811418;
// descending order, as np.polyval
static constexpr double kWS[] = {
    -6.33967193899028751e-06,
    -7.86185303095492391e-06,
    +2.31261020493938463e-05,
    +7.65451696033735612e-05,
    -5.79791115077058281e-05,
    -1.58814527060535367e-04,
    -5.10089135107843802e-06,
    +3.10780994907205695e-04,
    -2.59098623806738502e-04,
    +4.55864878461109625e-04,
    -1.38827078975457428e-03,
    +3.88891057296029182e-03,
    +9.85326240333731174e-01,
    +9.10229013680464560e-06,
};

} // namespace stage1_v9ri
} // namespace nlsc
