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
//   E4G1: order 3, -175.1 / -174.9 dB   (n19/n14 — stage 4 input network (LEVEL pot + C8))
//   E4GS: order 5, -108.0 / -107.7 dB   (n17/n19 normalised at 8 kHz — only the residual charge step)
//   E4G2: order 1, -174.6 / -174.5 dB   (out/n17 — stage 4 output (R14, C9, R15))
//   VRA: order 2, -121.6 / -121.6 dB   (current n7 feeds the rail: (n9/n7)/R8, VR pinned, without the pole)
//   VRB: order 1, -39.3 / -39.3 dB   (current n14 feeds the rail: (n18/n14)/RLVL, VR pinned, without the pole)
//   VRP: order 2, -310.5 / -310.5 dB   (node VR impedance, 1/(Gtot + s*C11), applied once to the sum)

#pragma once

namespace nlsc {
namespace e234_v9ri {

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
    {+1.25054613863675879e-02, +2.50119045449430644e-02, +1.25064013572644007e-02, +1.00000000000000000e+00, +9.55991571096521553e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -9.95869735205537787e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, -1.96757075166991058e+00, +9.67686945929449371e-01},
};

// n19/n14 — stage 4 input network (LEVEL pot + C8)
static constexpr int kE4G1_N = 2;
static constexpr double kE4G1[2][6] = {
    {+9.86844340243291152e-01, +9.86583782803261755e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, +9.97694734606481171e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -1.99999995278488396e+00, +9.99999952784856094e-01, +1.00000000000000000e+00, -1.99982411696188356e+00, +9.99824122800572312e-01},
};

// n17/n19 normalised at 8 kHz — only the residual charge step
static constexpr int kE4GS_N = 3;
static constexpr double kE4GS[3][6] = {
    {+9.99965125497729757e-01, -1.41789014020731230e+00, +5.02620164014276893e-01, +1.00000000000000000e+00, -7.08378094813491122e-01, +0.00000000000000000e+00},
    {+1.00000000000000000e+00, -1.97077335624840932e+00, +9.70986902372537597e-01, +1.00000000000000000e+00, -1.69489721800123494e+00, +6.99156241925380839e-01},
    {+1.00000000000000000e+00, +9.99813114964328453e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, +1.43018669018004640e-02, -9.85180577075862485e-01},
};

// out/n17 — stage 4 output (R14, C9, R15)
static constexpr int kE4G2_N = 1;
static constexpr double kE4G2[1][6] = {
    {+9.95319406845414312e-01, -9.95319406814737850e-01, +0.00000000000000000e+00, +1.00000000000000000e+00, -9.99994816013984433e-01, +0.00000000000000000e+00},
};

// current n7 feeds the rail: (n9/n7)/R8, VR pinned, without the pole
static constexpr int kVRA_N = 1;
static constexpr double kVRA[1][6] = {
    {+1.16719262849252082e-06, +5.28241661000788224e-09, -1.16191185417120257e-06, +1.00000000000000000e+00, -1.96757071262315475e+00, +9.67686908001124890e-01},
};

// current n14 feeds the rail: (n18/n14)/RLVL, VR pinned, without the pole
static constexpr int kVRB_N = 1;
static constexpr double kVRB[1][6] = {
    {+9.86006327782055505e-06, -9.86001813129449753e-06, +0.00000000000000000e+00, +1.00000000000000000e+00, -9.99939588933897339e-01, +0.00000000000000000e+00},
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
// Travel covered by the fit: [-0.7946, +0.7943] V
// Fit residual: -104.6 dB
static constexpr double kWS2_UMIN = -7.946190413e-01;
static constexpr double kWS2_UMAX = +7.943405087e-01;
static constexpr int kWS2_N = 18;
// descending order, as np.polyval
static constexpr double kWS2[] = {
    -6.58243885258752610e-03,
    -1.71636483557835694e-02,
    +1.84611131010205681e-02,
    +4.60742003573590270e-02,
    -2.15442117011113987e-02,
    -5.09254996120820716e-02,
    +1.35322851866671104e-02,
    +2.98649814530244119e-02,
    -4.93858007251095962e-03,
    -9.97378675516555173e-03,
    +1.05171872609346323e-03,
    +1.89271874932676952e-03,
    -1.37347213948100702e-04,
    -1.35949418731982284e-04,
    -2.32139815279181725e-04,
    +1.19750418013569765e-03,
    +9.91738155424296997e-01,
    -4.89521794003472982e-08,
};

} // namespace e234_v9ri
} // namespace nlsc
