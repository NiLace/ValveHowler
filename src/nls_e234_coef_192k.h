// GENERATED FILE — DO NOT EDIT BY HAND.
//
// Coefficients of cascade stages 3 and 4, plus the rest points all
// four stages need. Fitted against the complete circuit's AC sweep
// at the working point:
// drive=1.0, tone=0.5, lvl=0.0 (LEVEL at maximum).
//
// The coefficients depend on the knobs; this bank holds for that
// point only and carries no knob law.
//
// Fit band: 20 Hz - 86400 Hz (internal-rate Nyquist).
// Fit residuals (whole band / 20 Hz-20 kHz only):
//   E3: order 3, -151.2 / -151.2 dB   (n14/n7 — stage 3 (tone), linear (distortion at -156 dBc))
//   E4G1: order 3, -173.8 / -173.7 dB   (n19/n14 — stage 4 input network (LEVEL pot + C8))
//   E4GS: order 5, -107.2 / -106.7 dB   (n17/n19 normalised at 8 kHz — only the residual charge step)
//   E4G2: order 1, -174.9 / -174.8 dB   (out/n17 — stage 4 output (R14, C9, R15))
//   VRA: order 2, -121.6 / -121.6 dB   (current n7 feeds the rail: (n9/n7)/R8, VR pinned, without the pole)
//   VRB: order 1, -38.8 / -38.8 dB   (current n14 feeds the rail: (n18/n14)/RLVL, VR pinned, without the pole)
//   VRP: order 2, -310.5 / -310.5 dB   (node VR impedance, 1/(Gtot + s*C11), applied once to the sum)

#pragma once

namespace nlsc {
namespace e234 {

static constexpr double kFs = 192000.0;

// Complete-circuit rest point (ngspice .op). The cascade works in
// deviation from each node's rest; no node rests at VR.
//
// Which rate's bank is compiled is chosen by NLSC_CASC_KHZ in
//   nls_cascada.h: default build -> nls_e234_coef.h and
//   nls_e234_coef_v9ri_384k.h; 192 -> the *_192k.h pair; 96 -> the
//   *_96k.h pair. The two lists below hold for the bank being compiled.
// Read by the engine: kFs · kQ_vr · kQ_n3 · kQ_n4 · kQ_n7 · kQ_n11 ·
//   kQ_n14 · kQ_n19 · kQ_out · kG_* · kWS2*  (nls_juegos.h and
//   nls_cascada.h hold the mapping).
// Not read by the engine: kE3 (stage 3 comes from `tone::`), kE4G1
//   (from `level::`), kVRA/kVRB/kVRP (from `fixed::`, fitted in `s` in
//   nls_rail_vr.h), kE4G2 · kE4GS · kE2C (from `fixed::`, the s-domain
//   form of the default-rate bank, nls_fijos_s*.h), and the rests
//   kQ_n5 · kQ_n6 · kQ_n9 · kQ_n17 (n6/n17 come from
//   nls_reposo_drive*.h; n5/n9 are not used).
static constexpr double kQ_vr = 4.495215;
static constexpr double kQ_n3 = 4.148169;
static constexpr double kQ_n6 = 3.578040;
static constexpr double kQ_n4 = 4.495125;
static constexpr double kQ_n5 = 4.495080;
static constexpr double kQ_n7 = 4.499768;
static constexpr double kQ_n9 = 4.499344;
static constexpr double kQ_n11 = 4.495215;
static constexpr double kQ_n14 = 4.499306;
static constexpr double kQ_n19 = 4.148169;
static constexpr double kQ_n17 = 3.578040;
static constexpr double kQ_out = 0.000000;

// Conductances feeding the VR rail from nodes the cascade already
// has. n7's and n14's live inside kVRA/kVRB, which are currents
// (no pole): the `kVRP` pole applies once to their sum, as the
// VR node does in the circuit.
static constexpr double kG_n3 = 1.960784313725e-06;
static constexpr double kG_n4 = 1.000000000000e-04;
static constexpr double kG_n11 = 2.127659574468e-04;
static constexpr double kG_n19 = 1.960784313725e-06;


// n14/n7 — stage 3 (tone), linear (distortion at -156 dBc)
static constexpr int kE3_N = 2;
static constexpr double kE3[2][6] = {
    {+1.25054565488118830e-02, +2.50120338224857491e-02, +1.25065355941280099e-02, +1.00000000000000000e+00, +9.56001685442990468e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -9.95869735090606278e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, -1.96757075154797922e+00, +9.67686945810779853e-01},
};

// n19/n14 — stage 4 input network (LEVEL pot + C8)
static constexpr int kE4G1_N = 2;
static constexpr double kE4G1[2][6] = {
    {+9.86602016456784803e-01, +9.86230252304083899e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, +9.97433598208212979e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -2.00000106535949262e+00, +1.00000106538042077e+00, +1.00000000000000000e+00, -1.99981576742571598e+00, +9.99815772989250728e-01},
};

// n17/n19 normalised at 8 kHz — only the residual charge step
static constexpr int kE4GS_N = 3;
static constexpr double kE4GS[3][6] = {
    {+9.99973748205858715e-01, +1.08628183864643035e+00, +3.83155338490715036e-01, +1.00000000000000000e+00, +1.08625925685965630e+00, +3.83145551942351836e-01},
    {+1.00000000000000000e+00, -9.15792007249580875e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, -9.15792124832584320e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -1.94466354901796623e+00, +9.44666840432807464e-01, +1.00000000000000000e+00, -1.94466355517955636e+00, +9.44666826847736507e-01},
};

// out/n17 — stage 4 output (R14, C9, R15)
static constexpr int kE4G2_N = 1;
static constexpr double kE4G2[1][6] = {
    {+9.90073482039104391e-01, -9.90073482039569353e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, -9.99948433673232495e-01, +0.00000000000000000e+00},
};

// current n7 feeds the rail: (n9/n7)/R8, VR pinned, without the pole
static constexpr int kVRA_N = 1;
static constexpr double kVRA[1][6] = {
    {+1.16719262794669046e-06, +5.28241772237199523e-09, -1.16191185469832325e-06, +1.00000000000000000e+00, -1.96757071260256478e+00, +9.67686907980996103e-01},
};

// current n14 feeds the rail: (n18/n14)/RLVL, VR pinned, without the pole
static constexpr int kVRB_N = 1;
static constexpr double kVRB[1][6] = {
    {+9.85591650843347854e-06, -9.85587038417358170e-06, +0.00000000000000000e+00, +1.00000000000000000e+00, -9.99939541058896597e-01, +0.00000000000000000e+00},
};

// node VR impedance, 1/(Gtot + s*C11), applied once to the sum
static constexpr int kVRP_N = 1;
static constexpr double kVRP[1][6] = {
    {+5.54058778979174063e-02, +1.42431562720223785e-03, -5.39815622707151693e-02, +1.00000000000000000e+00, -1.97422364051096544e+00, +9.74225399815991100e-01},
};

// E2C = identity: the ideal-opamp correction is off. It gains
// 24.8 dB in small signal and loses 9.2 dB on real material,
// because the error it corrects depends on level.
static constexpr int kE2C_N = 1;
static constexpr double kE2C[1][6] = {
    {+1.0e+00, +0.0e+00, +0.0e+00, +1.0e+00, +0.0e+00, +0.0e+00},
};

// Q2 follower waveshaper, from its own extraction sweep injected
// near Q2. Driven from the pedal input, the diode clipping and the
// 8 kHz tone roll-off keep `n19` under ±0.29 V, while real material
// covers ±0.61 V, and an extrapolated polynomial diverges (+41 dB).
// Input and output are deviations from rest, not absolute.
// Chosen run: 0.80 V injection at 8 kHz.
// Travel covered by the fit: [-0.7949, +0.7938] V
// Fit residual: -103.1 dB
static constexpr double kWS2_UMIN = -7.949431813e-01;
static constexpr double kWS2_UMAX = +7.937527387e-01;
static constexpr int kWS2_N = 18;
// descending order, as np.polyval
static constexpr double kWS2[] = {
    -7.70106380239356150e-03,
    -2.06725592613586154e-02,
    +2.16420837634208087e-02,
    +5.53431452111169814e-02,
    -2.53275134439891539e-02,
    -6.09970016287392572e-02,
    +1.59573278136742464e-02,
    +3.56796881474045410e-02,
    -5.85197893531832353e-03,
    -1.18662565613852945e-02,
    +1.20071481800467753e-03,
    +2.35269249629290562e-03,
    -3.80844106209668584e-04,
    +3.22186144738221105e-04,
    -1.36128599118675729e-03,
    +3.83690922045147554e-03,
    +9.85404163516363729e-01,
    +1.47913696724298930e-06,
};

} // namespace e234
} // namespace nlsc
