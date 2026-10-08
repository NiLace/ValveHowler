// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// The OD-8 circuit's MNA engine: modified nodal analysis + Newton per sample.
//
// A port of a 33-unknown prototype that was validated against ngspice: rest
// point within 1.82 µV, output null −82.5 dB with 8x oversampling and
// −94.6 dB with 16x. Those figures are the prototype's; this engine has 29
// unknowns and different device cards, and they were not re-measured on it.
//
// ## Why this formulation and not a cascade
//
// A `filter -> waveshaper -> filter` cascade cannot reproduce the rest-point
// displacement: it is 2.4 nA over 2266 nA, and rebuilding it by subtracting
// models demands more than 60 dB of precision. Here `n3 = vr − R2·ib` is
// imposed by construction and the displacement follows: DC error −102 dB
// against the cascade's −50 dB.
//
// ## Real-time contract
//
// No allocation, no exceptions, no locks: all state is fixed-size arrays.
// The only variability is the Newton iteration count, which is bounded.

#pragma once

// Here `nlsc::` names carry no leading `::`. This header also compiles inside
// an ISA namespace (`nls_isa_tu.cpp`): `::nlsc::fast::exp_` would point
// outside that namespace, where it does not exist; without the `::` the name
// resolves in both contexts.

// The potentiometer's residual resistance (ohms). See the note next to `eps`.
#ifndef NLSC_POT_EPS
#define NLSC_POT_EPS 1e-3
#endif

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "nls_fastmath.h"

// Per-function switches for the fast transcendentals (`nls_fastmath.h`);
// `-DNLSC_FAST_EXP=0 -DNLSC_FAST_TANH=0` returns to libm. The own exp and
// tanh are faster than glibc's (−8.8 % of clock together); the own `pow` is
// not (+2.2 %), so `pow` stays on libm.
#ifndef NLSC_FAST_EXP
#define NLSC_FAST_EXP 1
#endif
#ifndef NLSC_FAST_POW
#define NLSC_FAST_POW 0
#endif
#ifndef NLSC_FAST_TANH
#define NLSC_FAST_TANH 1
#endif
#if NLSC_FAST_EXP
#  define NLSC_EXP(x)      nlsc::fast::exp_(x)
#else
#  define NLSC_EXP(x)      std::exp(x)
#endif
#if NLSC_FAST_POW
#  define NLSC_POWNEG(u,m) nlsc::fast::pow_neg((u), (m))
#else
#  define NLSC_POWNEG(u,m) std::pow((u), -(m))
#endif
// u^(−m) by range reduction + exponent table (`Junc::powneg`). It needs the
// `Junc` (its fitted coefficients live inside), so it has its own macro. It
// is faster than `std::pow` (−13.5 % of clock on the engine) with no
// measurable change in output quality. `-DNLSC_POW_RANGE=0` returns to
// `std::pow`.
#ifndef NLSC_POW_RANGE
#define NLSC_POW_RANGE 1
#endif

// `NLSC_BJT_DIV` selects how `bjt()` divides by `qb`: 1 (the default) keeps
// the three divisions, 0 uses one reciprocal. The reciprocal is not faster in
// practice, so the divisions stay.
#ifndef NLSC_BJT_DIV
#define NLSC_BJT_DIV 1      // 1 = three divisions (default) · 0 = one reciprocal
#endif

// `NLSC_JUNC_LIN` is the default of `qjunc`'s `LIN` template parameter: the
// linearised depletion capacitance (see `qjunc`). `nls_dk.h` requests the
// exact law explicitly. The switch is defined above its first use: a
// `#define` placed below it would silently select the other branch.
#ifndef NLSC_JUNC_LIN
#define NLSC_JUNC_LIN 1
#endif
#if NLSC_POW_RANGE
#  define NLSC_JPOWNEG(j,u) (j).powneg(u)
#else
#  define NLSC_JPOWNEG(j,u) NLSC_POWNEG((u), (j).m)
#endif
#if NLSC_FAST_TANH
#  define NLSC_TANH(x)     nlsc::fast::tanh_(x)
#else
#  define NLSC_TANH(x)     std::tanh(x)
#endif


namespace nlsc {


namespace mna {

// ---------------------------------------------------------------------------
// Physical constants and model cards — circuit data, not fits
// ---------------------------------------------------------------------------
// Thermal voltage derived, not copied: k·T/q at 300.15 K (ngspice's TNOM).
inline constexpr double kBoltz = 1.380649e-23;
inline constexpr double kQelec = 1.602176634e-19;
inline constexpr double kTnom  = 300.15;
inline constexpr double VT     = kBoltz * kTnom / kQelec;

// Precomputed reciprocal of VT. Even with `VT` constexpr, GCC at -O3 emits
// the division: turning it into a multiply requires `-freciprocal-math`,
// which this project does not use because it loosens floating point.
inline constexpr double inv_VT = 1.0 / VT;

inline constexpr double VCC = 9.0;

// 2SC1815 (bin BL) — the transistor the factory service manual specifies for
// Q101/Q103, the two emitter followers. The 2N3904 is a common substitute but
// not interchangeable here: the difference in the Vbe law alone leaves a
// −38.9 dB residual through the whole circuit. The model card is derived from
// the datasheet. The same card exists as `.model QBUF` in the SPICE netlist
// and an automated check compares the two, so change both together.
//
// This models the original 1979 unit; the datasheet marks the part "Not
// Recommended for New Design", and reissues fit a 2N3904.
//
// `NLSC_MNA_RB` — at 0 removes the base resistance of both transistors. Both
// copies of the stamp read it (`buildConstant` here and `nls_dk.h`), and it
// is defined here because `nls_dk.h` includes this file.
#ifndef NLSC_MNA_RB
#define NLSC_MNA_RB 1
#endif

struct QModel {
    double IS = 90.164e-15, BF = 495.0, BR = 0.7371, VAF = 74.03;
    double ISE = 0.0, NE = 2.0, IKF = 66.78e-3;
    double RB = 50.0, RC = 1.0;
    double CJE = 4.493e-12, MJE = 0.2593, VJE = 0.75;
    double CJC = 4.547e-12, MJC = 0.3085, VJC = 0.75;
    double TF = 1.823e-9, TR = 239.5e-9, FC = 0.5;
    // Derived values with constant model denominators: computed once in
    // `prepare()` via `derive()`; the hot path only multiplies.
    double inv_NEVT = 0, ISoBF = 0, ISoBR = 0, ISoIKF = 0, inv_VAF = 0,
           TFIS = 0, TRIS = 0;
    void derive() {
        inv_NEVT = 1.0 / (NE * VT);  ISoBF = IS / BF;   ISoBR  = IS / BR;
        ISoIKF   = IS / IKF;         inv_VAF = 1.0 / VAF;
        TFIS     = TF * IS;          TRIS    = TR * IS;
    }
};
// MA150 — the clipping diode the factory service manual specifies for
// D101-D106. It is not interchangeable with a 1N914 at clipping currents:
// the saturation currents IS agree to 4 %, but the ideality N differs (a
// softer knee), so the forward-voltage gap grows with current, from 17 mV at
// 1 uA to 36 mV at 1 mA.
//
// The datasheet publishes neither IS nor N; both are fitted to its VF-Ta
// curve, which the fit holds over three decades to under 0.13 mV.
//
// Derived rather than published:
//   TT   the sheet gives trr <= 10 ns at IF = IR = 10 mA; trr = TT*ln(1+IF/IR)
//        puts TT <= 14.4 ns. The plausible range, 5 to 40 ns, is worth 0.19 dB.
//   CJO  the device's Ct-VR curve reads 1.44 pF at low bias (the family table
//        gives 0.9 typ / 2 max pF; the device curve is used). Known shape
//        error: the real part's capacitance falls 15.4 % from 0.4 to 6.4 V
//        where this M/VJ pair falls 48.6 %, because most of its Ct is package
//        capacitance rather than junction.
//   M, VJ  not given by the datasheet; taken from a standard 1N914 card.
//
// `NLSC_DIODE_RS` is the single definition of the diode's series resistance.
// This engine and the DK model it with an internal node per diode
// (`kD[i].a` -> `kD[i].ai`, see `buildConstant()`); the cascade's stage 2
// evaluates the pair directly on `w = v5 − v7`, without it.
#ifndef NLSC_DIODE_RS
#define NLSC_DIODE_RS 1.1
#endif
struct DModel {
    double IS = 2.407e-9, N = 1.851, RS = NLSC_DIODE_RS;
    double CJO = 1.43e-12, M = 0.4, TT = 14.4e-9, VJ = 1.0, FC = 0.5;
    double inv_nvt = 0;
    void derive() { inv_nvt = 1.0 / (N * VT); }
};
// NJM4558. The netlist specifies an RC4558; the two datasheets differ only in
// slew rate (1.7 V/us for the RC4558, 1.0 for the NJM4558; both declare
// GBW = 3 MHz). `NLSC_OPAMP_SR` sets the slew rate in V/s.
#ifndef NLSC_OPAMP_SR
#  define NLSC_OPAMP_SR 1e6
#endif
struct OModel {
    double AVOL = 1e5, GBW = 3e6, SR = NLSC_OPAMP_SR, CP = 1e-9, VSAT = 1.5;
    double RO = 75.0, RIN = 5e6, RCM = 500e6;
    double gm()   const { return 2.0 * M_PI * GBW * CP; }
    double rp()   const { return AVOL / gm(); }
    double imax() const { return SR * CP; }
    double inv_RO = 0;
    void derive() { inv_RO = 1.0 / RO; }
};

// ---------------------------------------------------------------------------
// Nodes. The three fixed ones (ground, supply, input) are not unknowns:
// their KCL is never posed. The rest are, plus one branch current per opamp.
// ---------------------------------------------------------------------------
enum Node : int {
    N_n1, N_n2, N_n3, N_n4, N_n5, N_n6, N_n7, N_n8, N_n9, N_n10,
    N_n11, N_n12, N_n13, N_n14, N_n16, N_n17, N_n18, N_n19, N_n20,
    N_out, N_vr,
    N_Q1bi, N_Q1ci, N_Q2bi, N_Q2ci,
    N_D1ai, N_D2ai,
    N_A2, N_B2,
    NUM_NODES,
    // fixed nodes, encoded with negative indices so `nv()` resolves them
    N_GND = -1, N_VCC = -2, N_IN = -3,
};
// The opamps' output sources add no unknowns. `Bout` is an ideal source to
// ground whose voltage is a known function of `V(2)`, so it is substituted:
// `Ro` sits between a known voltage and the output node, a controlled
// source. Besides saving 4 unknowns, this matters because a branch current's
// row has no diagonal, and minimum-degree reordering would pick it first
// (degree 2) and pivot on a zero.
inline constexpr int N = NUM_NODES;

struct Res  { int a, b; double r; };
struct Cap  { int a, b; double c; };

// The netlist as data. R11 = 1 kΩ, confirmed by five sources.
struct Netlist {
    // `eps` is not physics: it is the regularisation that keeps a pot off
    // 0 ohms. Its value matters, because at full knob the branch collapses to
    // `eps`, and with the capacitor hanging off it that is a parasitic pole:
    //
    //   eps = 1e-3 ohm  ->  tau = 0.22 ns  ->  pole at 723 MHz (1880x Nyquist at 8x)
    //   eps = 10 ohm    ->  tau = 2.2 us   ->  pole at 72.3 kHz (below Nyquist)
    //
    // A fixed-step trapezoidal solver and an adaptive-step one discretise a
    // 723 MHz pole differently: at tone = 1.0 the null against ngspice is
    // −75.0 dB, 7 dB worse than over the rest of the travel. A real
    // potentiometer never reaches zero: the Bourns PDB24 specifies "Residual
    // Resistance: 10 ohms max." for R < 500 kohm, so 10 ohm is the physical
    // upper bound.
    double gain = 0.5, tone = 0.5, lvl = 0.0, eps = NLSC_POT_EPS;

    // The unit seed. `0` is the ideal unit, with the schematic's nominal values
    // (the default). Any other value draws each component's tolerance and
    // produces one concrete unit. The tolerance bands are assumptions: the maker
    // publishes none.
    unsigned seed = 0;

    // The variant axis — the two output resistors (the netlist's `R14` series
    // and `R15` shunt). They are data, not a code branch: changing them touches
    // no topology, so the LU's sparsity pattern and the generated code
    // (`nls_dk_elim_gen.h`) remain valid.
    //
    // Three independent sources agree on it: a published variant table, a
    // pedal-modification book that marks them on a photo of the board (`808-1` =
    // 100 Ω and `808-2` = 10 kΩ, p. 255), and the netlist itself, which carries
    // the factory 808. The table's third circuit (an extra R at position `RA`)
    // needs a new element and its value is disputed between sources (1 kΩ or
    // 220 Ω), so it is not exposed. The defaults are the 808's.
    double rout_ser = 100.0, rout_shunt = 10e3;

    // The parasitics are added in `buildConstant()` and in
    // `dk::Engine::prepare()`.
    // `kMaxRes`/`kMaxCap` bound the stamping lambdas in build(): `res` has
    // headroom (39 of 48 used) but `cap` is exactly full. On overflow the stamp
    // is skipped and `overflowed` latches: no stdio here, because build() is
    // reachable from the audio thread through a knob re-tune. `talpha` is sized
    // with `cap`.
    static constexpr int kMaxRes = 48, kMaxCap = 12;
    bool overflowed = false;
    Res res[kMaxRes];   // netlist + the devices' parasitics
    Cap cap[kMaxCap];
    int nres = 0, ncap = 0;

    // Differentiated discretisation (Germain & Werner, WASPAA 2017 / AES 142).
    //
    // Each reactive element may carry its own bilinear-transform parameter:
    // `s -> (2/T'_l)*(1-z^-1)/(1+z^-1)` with `T'_l = talpha[l]*T`, instead of one
    // shared `T` for all twelve. In the DK formulation that is exactly
    // `gc[l] = 2*C_l/(h*talpha[l])`, i.e. zero run-time cost: it only changes a
    // coefficient computed once in `prepare()`.
    //
    // It is not a component value and does not belong in `build()`: `build()`
    // describes the circuit, this describes how it is discretised. Mixing them
    // would let a numerical parameter later read as a capacitor tolerance.
    //
    // With all of them at 1.0 the engine is bit-identical to the plain
    // trapezoidal one by construction: `h*1.0 == h`. That is the default.
    double talpha[kMaxCap] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                              1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    void build()
    {
        nres = ncap = 0;

        // ── Tolerance draw ──────────────────────────────────────────────────
        //
        // The order of the `R()`/`C()` calls is each component's identity: a seed
        // gives the same unit only while that order holds, so reordering two lines
        // below changes every seeded unit without touching a value. New components
        // go at the end.
        //
        // Own PRNG (SplitMix64): the standard library's random distributions do
        // not guarantee the same sequence across implementations, and this must
        // give the same unit on every machine.
        unsigned long long est = 0x9E3779B97F4A7C15ull * (seed + 1ull);
        auto draw = [&est](double tol) -> double {
            if (tol <= 0.0) return 1.0;
            est += 0x9E3779B97F4A7C15ull;
            unsigned long long z = est;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
            z ^= z >> 31;
            // uniform in [-tol, +tol]
            const double u = double(z >> 11) * (1.0 / 9007199254740992.0);
            return 1.0 + tol * (2.0 * u - 1.0);
        };
        const bool ideal = (seed == 0);
        // Bands per type. An electrolytic is much wider than a film
        // part; one shared band would understate the dispersion.
        const double tR  = ideal ? 0.0 : 0.05;   // film resistors
        const double tCf = ideal ? 0.0 : 0.10;   // film capacitors
        const double tCe = ideal ? 0.0 : 0.20;   // electrolytics (>= 1 uF)
        const double tP  = ideal ? 0.0 : 0.20;   // potentiometers

        // One pot = one draw, shared by both halves. A real pot has one total
        // resistance with its tolerance, split by the wiper; drawing each half
        // separately would build a component that does not exist and would break
        // the knob's law.
        const double kPgain = draw(tP), kPtone = draw(tP), kPlvl = draw(tP);

        // Bounds checks: an overflow latches `overflowed` instead of writing past
        // the arrays.
        auto R = [&](int a, int b, double r) {
            if (nres >= kMaxRes) { overflowed = true; return; }
            res[nres++] = {a, b, r * draw(tR)};
        };
        auto C = [&](int a, int b, double c) {
            if (ncap >= kMaxCap) { overflowed = true; return; }
            cap[ncap++] = {a, b, c * draw(c >= 1e-6 ? tCe : tCf)};
        };
        // Pots stamp with their shared factor, not `R()`'s.
        auto RP = [&](int a, int b, double r, double k) {
            if (nres >= kMaxRes) { overflowed = true; return; }
            res[nres++] = {a, b, r * k};
        };
        R(N_VCC, N_vr, 10e3);   R(N_vr, N_GND, 10e3);   R(N_IN, N_GND, 1e6);
        R(N_n2, N_n3, 1e3);     R(N_n3, N_vr, 510e3);   R(N_n6, N_GND, 10e3);
        R(N_n4, N_vr, 10e3);    R(N_n5, N_n1, 51e3);
        RP(N_n1, N_n7, 500e3 * gain + eps, kPgain);
        R(N_n11, N_vr, 4.7e3);  R(N_n7, N_n9, 1e3);     R(N_n9, N_vr, 10e3);
        RP(N_n9, N_n8, 20e3 * tone + eps, kPtone);
        RP(N_n8, N_n10, 20e3 * (1.0 - tone) + eps, kPtone);
        R(N_n12, N_GND, 220.0); R(N_n10, N_n14, 1e3);   R(N_n13, N_n16, 1e3);
        RP(N_vr, N_n18, 100e3 * (1.0 - lvl) + eps, kPlvl);
        RP(N_n18, N_n16, 100e3 * lvl + eps, kPlvl);
        // `R13` is 10k and the variant axis is `R15`, the output load after the
        // coupling cap. This copy lives in C++ and is not generated from the SPICE
        // netlist; an automated check compares its resistors and capacitors
        // with the netlist's, so change both together.
        R(N_n19, N_vr, 510e3);  R(N_n17, N_GND, 10e3);
        R(N_n17, N_n20, rout_ser);
        R(N_out, N_GND, rout_shunt);
        C(N_vr, N_GND, 47e-6);  C(N_IN, N_n2, 0.02e-6); C(N_n6, N_n4, 1e-6);
        C(N_n5, N_n11, 0.047e-6); C(N_n5, N_n7, 51e-12); C(N_n9, N_GND, 0.22e-6);
        C(N_n8, N_n12, 0.22e-6); C(N_n14, N_n13, 1e-6); C(N_n18, N_n19, 0.1e-6);
        C(N_n20, N_out, 10e-6);
        // The opamps' compensation: it provides the dominant pole and, with the
        // gm's current limit, the slew.
        C(N_A2, N_GND, OModel{}.CP);  C(N_B2, N_GND, OModel{}.CP);
    }
};


// Fixed-pattern sparse LU.
//
// The matrix has 119 nonzeros of 1089 (10.9 % density), so a dense LU does
// 89 % of its work on zeros. The pattern — fill-in included — is computed
// once in `prepare()` by symbolic elimination, and every solve touches only
// the nonzeros.
//
// No pivoting: validated against the dense LU (`lu_solve`), which it
// matches on this matrix.
struct SparseLU;
class Engine;

struct SparseLU {
    // for each row k: columns < k to eliminate against, and columns >= k
    // that get touched
    int8_t nz[N][N];          // structural pattern (fill-in included)
    int    above[N][N], nabove[N];
    int    below[N][N], nbelow[N];
    int    perm[N];           // elimination order
    int    pat_i[512], pat_j[512], npat = 0;   // the nonzeros, for copying

    // The elimination order decides the fill-in, and here it decides a
    // lot: natural order takes 117 nonzeros to 360, minimum degree
    // leaves them at 133 — practically fill-free. A 2.7x factor of work per
    // solve, for free, computed once in `prepare()`.
    void minDegree(const int8_t P[N][N])
    {
        // On the stack, not `static`: a static array would be shared by every
        // instance. N is small and this runs once in `prepare()`.
        bool G[N][N];
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) G[i][j] = P[i][j] || P[j][i];
        bool outside[N] = {false};
        for (int s = 0; s < N; ++s) {
            int best = -1, gmin = 1 << 30;
            for (int i = 0; i < N; ++i) {
                if (outside[i]) continue;
                int g = 0;
                for (int j = 0; j < N; ++j) if (!outside[j] && j != i && G[i][j]) ++g;
                if (g < gmin) { gmin = g; best = i; }
            }
            perm[s] = best;
            outside[best] = true;
            // eliminating a node connects its neighbours to each other
            for (int a = 0; a < N; ++a) {
                if (outside[a] || !G[best][a]) continue;
                for (int b = 0; b < N; ++b)
                    if (!outside[b] && b != a && G[best][b]) G[a][b] = G[b][a] = true;
            }
        }
    }

    void symbolic(const int8_t P[N][N])
    {
        minDegree(P);
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) nz[i][j] = P[perm[i]][perm[j]];
        // symbolic elimination: row i inherits row k's pattern
        for (int k = 0; k < N; ++k)
            for (int i = k + 1; i < N; ++i)
                if (nz[i][k])
                    for (int j = k + 1; j < N; ++j)
                        if (nz[k][j]) nz[i][j] = 1;
        npat = 0;
        for (int i = 0; i < N; ++i) {
            nabove[i] = nbelow[i] = 0;
            for (int j = i; j < N; ++j) if (nz[i][j]) above[i][nabove[i]++] = j;
            for (int j = 0; j < N; ++j)
                if (nz[i][j]) { pat_i[npat] = i; pat_j[npat] = j; ++npat; }
        }
        // by columns: which rows to eliminate at step k. Scanning the N-k
        // rows for `nz[i][k]` costs N²/2 branches per solve, which at this
        // matrix size outweighs the arithmetic.
        for (int k = 0; k < N; ++k) {
            nbelow[k] = 0;
            for (int i = k + 1; i < N; ++i) if (nz[i][k]) below[k][nbelow[k]++] = i;
        }
    }

    int nnz() const
    {
        int c = 0;
        for (int i = 0; i < N; ++i) for (int j = 0; j < N; ++j) c += nz[i][j];
        return c;
    }

    // In-place Doolittle, walking only the pattern, on the permuted matrix.
    // Only the nonzeros are copied (133 of 1089), so permuting costs
    // nothing: a dense copy would have eaten the gain.
    bool solve(const double Asrc[N][N], const double bsrc[N], double out[N]) const
    {
        // On the stack, NOT `static thread_local`: every thread-local
        // access goes through a TLS resolution, and in a loop running
        // 384 000 times per second that outweighs the arithmetic itself.
        double A[N][N], b[N], xp[N];
        for (int t = 0; t < npat; ++t)
            A[pat_i[t]][pat_j[t]] = Asrc[perm[pat_i[t]]][perm[pat_j[t]]];
        for (int i = 0; i < N; ++i) b[i] = bsrc[perm[i]];
        for (int k = 0; k < N; ++k) {
            const double d = A[k][k];
            if (d == 0.0) return false;
            const double inv = 1.0 / d;
            for (int t2 = 0; t2 < nbelow[k]; ++t2) {
                const int i = below[k][t2];
                const double f = A[i][k] * inv;
                if (f == 0.0) continue;
                A[i][k] = 0.0;
                const int* col = above[k];
                for (int t = 1; t < nabove[k]; ++t) {   // t=0 is the diagonal
                    const int j = col[t];
                    A[i][j] -= f * A[k][j];
                }
                b[i] -= f * b[k];
            }
        }
        for (int i = N - 1; i >= 0; --i) {
            double s = b[i];
            const int* col = above[i];
            for (int t = 1; t < nabove[i]; ++t) s -= A[i][col[t]] * xp[col[t]];
            xp[i] = s / A[i][i];
        }
        for (int i = 0; i < N; ++i) out[perm[i]] = xp[i];
        return true;
    }
};


// ---------------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------------
class Engine {
public:
    // One junction's constants. `f1`, `f2` and `f3` depend only on FC and M, so
    // they are computed once.
    struct Junc {
        double cj0, vj, m, fc, fcvj, kq, f1, f2, f3, inv_vj, inv_f2;

        // --- u^(−m) by range reduction -------------------------------------
        // Generic `pow` is the engine's most expensive transcendental. Here the
        // exponent is a device-model constant, so:
        //
        //     u = mant·2^e  (bitwise frexp, mant ∈ [0.5, 1))
        //     u^(−m) = mant^(−m) · 2^(−m·e)
        //
        // The second factor is a table indexed by `e` (`e` spans {0..4} over the
        // whole knob and amplitude travel). The first is a polynomial over one
        // octave, evaluated as a tree of depth ~4: the cost of these routines is
        // ruled by dependency-chain depth, not operation count, which is why a
        // serial log2 -> exp2 does not pay.
        //
        // The coefficients are fitted here from the model's `m` (Chebyshev
        // interpolation, 9 nodes), not stored as constants.
        static constexpr int kDeg  = 8;          // degree 8 => −148.6 dB worst case
        static constexpr int kEMin = -4, kEMax = 12;
        double pc[kDeg + 1] = {0};
        double pw2[kEMax - kEMin + 1] = {0};

        void tune_powneg()
        {
            // Chebyshev nodes in [−1, 1] and their image in mant ∈ [0.5, 1).
            const int n = kDeg + 1;
            double V[kDeg + 1][kDeg + 2];
            for (int i = 0; i < n; ++i) {
                const double t = std::cos((2.0 * i + 1.0) * M_PI / (2.0 * n));
                const double x = 0.75 + 0.25 * t;
                double p = 1.0;
                for (int k = 0; k < n; ++k) { V[i][k] = p; p *= t; }
                V[i][n] = std::pow(x, -m);
            }
            // Gaussian elimination with partial pivoting. cond(V) ≈ 627: benign.
            for (int col = 0; col < n; ++col) {
                int piv = col;
                for (int r = col + 1; r < n; ++r)
                    if (std::fabs(V[r][col]) > std::fabs(V[piv][col])) piv = r;
                if (piv != col)
                    for (int k = col; k <= n; ++k) std::swap(V[col][k], V[piv][k]);
                const double d = V[col][col];
                for (int r = col + 1; r < n; ++r) {
                    const double f = V[r][col] / d;
                    if (f == 0.0) continue;
                    for (int k = col; k <= n; ++k) V[r][k] -= f * V[col][k];
                }
            }
            for (int r = n - 1; r >= 0; --r) {
                double s = V[r][n];
                for (int k = r + 1; k < n; ++k) s -= V[r][k] * pc[k];
                pc[r] = s / V[r][r];
            }
            for (int e = kEMin; e <= kEMax; ++e)
                pw2[e - kEMin] = std::pow(2.0, -m * double(e));
        }

        // u^(−m). Outside the tabulated range it falls back to libm; that does not
        // happen over the knob and amplitude travel.
        inline double powneg(double u) const
        {
            std::uint64_t b;
            std::memcpy(&b, &u, sizeof b);
            const int e = int((b >> 52) & 0x7FF) - 1022;
            if (e < kEMin || e > kEMax || u <= 0.0) return std::pow(u, -m);
            b = (b & 0x000FFFFFFFFFFFFFull) | 0x3FE0000000000000ull;
            double mant;
            std::memcpy(&mant, &b, sizeof mant);          // mant ∈ [0.5, 1)
            const double z = (mant - 0.75) * 4.0;
            const double z2 = z * z, z4 = z2 * z2;
            const double a = pc[0] + pc[1] * z, c1 = pc[2] + pc[3] * z;
            const double d = pc[4] + pc[5] * z, c3 = pc[6] + pc[7] * z;
            const double p = (a + c1 * z2) + (d + c3 * z2) * z4 + pc[8] * (z4 * z4);
            return p * pw2[e - kEMin];
        }

        void init(double cj0_, double vj_, double m_, double fc_)
        {
            cj0 = cj0_; vj = vj_; m = m_; fc = fc_;
            fcvj = fc * vj;  inv_vj = 1.0 / vj;
            kq = cj0 * vj / (1 - m);
            f1 = vj / (1 - m) * (1 - std::pow(1 - fc, 1 - m));
            f2 = std::pow(1 - fc, 1 + m);  inv_f2 = 1.0 / f2;
            f3 = 1 - fc * (1 + m);
#if NLSC_POW_RANGE
            tune_powneg();
#endif
        }
    };

    void prepare(double fs_oversampled, const Netlist& nl)
    {
        h_ = 1.0 / fs_oversampled;
        nl_ = nl;
        nl_.build();
        vcritQ_ = VT * std::log(VT / (std::sqrt(2.0) * q_.IS));
        nvtD_   = d_.N * VT;
        vcritD_ = nvtD_ * std::log(nvtD_ / (std::sqrt(2.0) * d_.IS));
        q_.derive(); d_.derive(); o_.derive();
        jbe_.init(q_.CJE, q_.VJE, q_.MJE, q_.FC);
        jbc_.init(q_.CJC, q_.VJC, q_.MJC, q_.FC);
        jd_ .init(d_.CJO, d_.VJ,  d_.M,   d_.FC);
        g2h_ = 2.0 / h_;
        gm_ = o_.gm();
        inv_imax_ = 1.0 / o_.imax();
        // The pattern is cleared before the constant part is built: `assemble`
        // starts from a copy of the constant matrix, so entries cleared afterwards
        // would never be re-marked and the solve would hit a null pivot.
        std::memset(pat_, 0, sizeof pat_);
        buildConstant();
        // The pattern is collected from real assemblies, marking every entry
        // touched. Several run under different conditions — including the opamps
        // clipping against the rails, which changes the structure — so the pattern
        // is the union of everything possible.
        for (int i = 0; i < N; ++i) x_[i] = 4.4;
        assemble(true);
        assemble(false);
        for (int i = 0; i < N; ++i) x_[i] = 0.5;    // opamps against the low rail
        assemble(true);
        for (int i = 0; i < N; ++i) x_[i] = 8.5;    // and against the high one
        assemble(true);
        slu_.symbolic(pat_);
        solveDc();
        capInit();
    }

    // Returns the output; `iters` receives the iterations consumed.
    double process(double vin, int* iters = nullptr)
    {
        const int k = step(vin);
        if (iters) *iters = k;
        return x_[N_out];
    }

    double node(int n) const { return x_[n]; }

public:
    // The device physics is public and static on purpose: the DK engine
    // (`nls_dk.h`) reuses it, so the equations exist once.
private:
    // -- node-voltage access, resolving the fixed ones ----------------------
    double nv(int n) const
    {
        if (n >= 0) return x_[n];
        if (n == N_GND) return 0.0;
        if (n == N_VCC) return VCC;
        return vin_;                       // N_IN
    }
    void add(int n, double val)          { if (n >= 0) F_[n] += val; }
    // `pat_` marks every entry touched, whatever its value. The sparsity
    // pattern cannot come from the nonzeros of one concrete matrix: some
    // entries are exactly zero at a working point through underflow — the
    // base-collector junction sits at −5.7 V and `exp(−220)` is 0 in
    // `double` — yet exist structurally.
    void jac(int r, int c, double val)
    {
        if (r >= 0 && c >= 0) { J_[r][c] += val; pat_[r][c] = 1; }
    }

    void stampR(int a, int b, double r)
    {
        const double g = 1.0 / r, i = (nv(a) - nv(b)) * g;
        add(a, i);  jac(a, a, g);  jac(a, b, -g);
        add(b, -i); jac(b, a, -g); jac(b, b, g);
    }

    // -- devices ------------------------------------------------------------
    // `exp(x) − 1` instead of `expm1(x)`: the exponential is needed anyway, and
    // the cancellation of exp(x)−1 at small x gives a relative error ~eps/x on a
    // current of IS·x ≈ 1e-22 A, which is irrelevant. Below −50 the exponential
    // underflows to zero, so the call is skipped (the base-collector junction
    // sits at −220 in VT units).
    static double fexp(double x) { return (x < -50.0) ? 0.0 : NLSC_EXP(x); }

public:
    struct QOut {
        double ib, ic, dib_be, dib_bc, dic_be, dic_bc;
        double qbe, cbe, qbc, cbc;
    };
    template <bool LIN = (NLSC_JUNC_LIN != 0)>
    static QOut bjt(const QModel& q_, const Junc& jbe_, const Junc& jbc_,
                    double vbe, double vbc, bool with_charges)
    {
        const double ex_be = fexp(vbe * inv_VT), ex_bc = fexp(vbc * inv_VT);
        const double e_be = ex_be - 1.0, e_bc = ex_bc - 1.0;
        const double ex_bee = fexp(vbe * q_.inv_NEVT);
        const double q1 = 1.0 / (1.0 - vbc * q_.inv_VAF);
        const double q2 = q_.ISoIKF * e_be;
        const double r  = std::sqrt(1.0 + 4.0 * q2);
        const double qb = 0.5 * q1 * (1.0 + r);
        const double ict = q_.IS * (e_be - e_bc);
        QOut o;
        o.ib = q_.ISoBF * e_be + q_.ISE * (ex_bee - 1.0)
             + q_.ISoBR * e_bc;
        // `NLSC_BJT_DIV=0` replaces the three divisions by `qb` with one
        // reciprocal. It is not bit-identical: `a/b` and `a*(1/b)` differ in the
        // last bit.
#if NLSC_BJT_DIV
        o.ic = ict / qb - q_.ISoBR * e_bc;
#else
        const double inv_qb  = 1.0 / qb;
        const double inv_qb2 = inv_qb * inv_qb;
        o.ic = ict * inv_qb - q_.ISoBR * e_bc;
#endif
        o.dib_be = q_.ISoBF * ex_be * inv_VT + q_.ISE * ex_bee * q_.inv_NEVT;
        o.dib_bc = q_.ISoBR * ex_bc * inv_VT;
        const double dict_be = q_.IS * ex_be * inv_VT, dict_bc = -q_.IS * ex_bc * inv_VT;
        const double dqb_be = q1 * (q_.ISoIKF * ex_be * inv_VT) / r;
        const double dqb_bc = 0.5 * (q1 * q1 * q_.inv_VAF) * (1.0 + r);
#if NLSC_BJT_DIV
        o.dic_be = (dict_be * qb - ict * dqb_be) / (qb * qb);
        o.dic_bc = (dict_bc * qb - ict * dqb_bc) / (qb * qb)
                 - q_.ISoBR * ex_bc * inv_VT;
#else
        o.dic_be = (dict_be * qb - ict * dqb_be) * inv_qb2;
        o.dic_bc = (dict_bc * qb - ict * dqb_bc) * inv_qb2
                 - q_.ISoBR * ex_bc * inv_VT;
#endif
        if (with_charges) {
            // The charges reuse the exponentials above.
            qjunc<LIN>(vbe, jbe_, o.qbe, o.cbe);
            qjunc<LIN>(vbc, jbc_, o.qbc, o.cbc);
            o.qbe += q_.TFIS * e_be;  o.cbe += q_.TFIS * ex_be * inv_VT;
            o.qbc += q_.TRIS * e_bc;  o.cbc += q_.TRIS * ex_bc * inv_VT;
        }
        return o;
    }

    Junc jbe_{}, jbc_{}, jd_{};

    // A junction's charge and capacitance.
    //
    // One `pow`, not two: u^(1−m) = u · u^(−m), so the charge follows from the
    // capacitance without a second call.
    //
    // `LIN` is a template parameter, not the macro, so that the cascade and the
    // DK engine can be compiled in the same translation unit with different
    // laws: the cascade uses the default (the macro's value), and `nls_dk.h`
    // requests the exact law.
    template <bool LIN = (NLSC_JUNC_LIN != 0)>
    static void qjunc(double v, const Junc& j, double& q, double& c)
    {
        // Constant depletion capacitance — the default. It removes the assembly's
        // only `pow` (the diodes' two junctions and the two BJTs' four) for −7.46 %
        // of clock, at −96.5 dB against the exact law (−78.0 dB at the worst knob
        // point, high level, where Q2's junction sits after the attenuator).
        //
        // It is nearly free because over a clipper's travel `u^(−m)` moves only
        // ±20 % around 1 (VJ = 1 V, M = 0.4, v ∈ −0.7…0.7). The depletion
        // capacitance itself matters (removing it costs 15.2 dB); only its
        // nonlinearity does not. `-DNLSC_JUNC_LIN=0` selects the exact law, which
        // uses `powneg`.
        // The same law, evaluated once, lives in `depletion_cap()` below: a change
        // to one must be made to the other.
        if constexpr (LIN) {
            c = j.cj0;
            q = j.cj0 * v;
        } else {
            if (v < j.fcvj) {
                const double u = 1.0 - v * j.inv_vj;
                const double um = NLSC_JPOWNEG(j, u);     // the only pow
                c = j.cj0 * um;
                q = j.kq * (1.0 - u * um);
            } else {
                q = j.cj0 * (j.f1 + (j.f3 * (v - j.fcvj)
                             + j.m * 0.5 * j.inv_vj * (v * v - j.fcvj * j.fcvj)) * j.inv_f2);
                c = j.cj0 * j.inv_f2 * (j.f3 + j.m * v * j.inv_vj);
            }
        }
    }

    // The same depletion law, evaluated once. It cannot be unified with
    // `qjunc`: that one runs per sample, so its exact branch uses the
    // `NLSC_JPOWNEG` range reduction and also returns the charge; this one runs
    // once in a `prepare()`, uses `std::pow` and returns only the capacitance.
    // A change to the law must be made to both.
    //
    // `SubQ2` needs the depletion capacitance at the rest point as its
    // constant. Under `NLSC_JUNC_LIN` the hot path returns a bare `cj0`, valid
    // for a clipper (v ∈ −0.7…0.7) but not for Q2's base-collector junction,
    // which rests at −4.73 V where `cj0` is 1.85x the real capacitance.
    static double depletion_cap(double v, double cj0, double vj, double m, double fc)
    {
        if (v < fc * vj) return cj0 * std::pow(1.0 - v / vj, -m);
        const double f2 = std::pow(1.0 - fc, 1.0 + m);
        const double f3 = 1.0 - fc * (1.0 + m);
        return cj0 * (f3 + m * v / vj) / f2;
    }

    // The diode's current and charge in one pass, with one exponential.
    //
    // `charge` selects the junction charge's terms:
    //
    //   0 = none
    //   1 = depletion + diffusion
    //   2 = diffusion only
    //   3 = depletion only
    //
    // In a clipper the diodes are cut off or barely conducting most of the
    // time, and there the depletion capacitance sits in parallel with `C4`
    // (51 pF); diffusion only exists under strong conduction, a small fraction
    // of the wave, and it costs two extra `diode()` calls per sample in the
    // state update.
    struct DOut { double i, g, q, c; };
    template <bool LIN = (NLSC_JUNC_LIN != 0)>
    static DOut diode(const DModel& d_, const Junc& jd_, double vd, int charge)
    {
        const double ex = fexp(vd * d_.inv_nvt);
        DOut o;
        o.i = d_.IS * (ex - 1.0);
        o.g = d_.IS * ex * d_.inv_nvt;
        o.q = o.c = 0.0;
        if (charge) {
            if (charge != 2) qjunc<LIN>(vd, jd_, o.q, o.c);   // depletion
            if (charge != 3) {                           // diffusion
                o.q += d_.TT * o.i;
                o.c += d_.TT * o.g;
            }
        }
        return o;
    }

    // The antiparallel pair, with a single exponential. The clipper evaluates
    // the same diode at `w` and at `-w`, so the second exponential is the
    // reciprocal of the first: one `exp` and one division per Newton iteration
    // instead of two `exp`.
    //
    // `fexp` clamps only the negative side (`x < -50 -> 0`), so `1.0 / e` would
    // be `1/0 = inf` where `fexp(-x)` returns a finite huge number. The three
    // branches below reproduce `fexp(x)` and `fexp(-x)` term by term; only the
    // middle one takes the reciprocal. `1.0 / e` is not bit-identical to
    // `exp(-x)`.
    template <bool LIN = (NLSC_JUNC_LIN != 0)>
    static void diode_par(const DModel& d_, const Junc& jd_, double w, int charge,
                          DOut& o1, DOut& o2)
    {
        const double x = w * d_.inv_nvt;
        double e, ei;
        if (x < -50.0)     { e = 0.0;            ei = NLSC_EXP(-x); }
        else if (x > 50.0) { e = NLSC_EXP(x);    ei = 0.0;          }
        else               { e = NLSC_EXP(x);    ei = 1.0 / e;      }

        o1.i = d_.IS * (e  - 1.0);  o1.g = d_.IS * e  * d_.inv_nvt;
        o2.i = d_.IS * (ei - 1.0);  o2.g = d_.IS * ei * d_.inv_nvt;
        o1.q = o1.c = o2.q = o2.c = 0.0;
        if (charge) {
            if (charge != 2) {                       // depletion
                qjunc<LIN>( w, jd_, o1.q, o1.c);
                qjunc<LIN>(-w, jd_, o2.q, o2.c);
            }
            if (charge != 3) {                       // diffusion
                o1.q += d_.TT * o1.i;  o1.c += d_.TT * o1.g;
                o2.q += d_.TT * o2.i;  o2.c += d_.TT * o2.g;
            }
        }
    }

    // SPICE's junction voltage limiting. Without it Newton runs away from a
    // cold start; with it on only one junction, the other overflows to NaN.
    static double pnjlim(double vnew, double vold, double vt, double vcrit)
    {
        if (vnew > vcrit && std::fabs(vnew - vold) > 2.0 * vt) {
            if (vold > 0.0) {
                const double a = 1.0 + (vnew - vold) / vt;
                vnew = (a > 0.0) ? vold + vt * std::log(a) : vcrit;
            } else {
                vnew = vt * std::log(vnew / vt);
            }
        }
        return vnew;
    }

private:
    void buildConstant();
    void assemble(bool transient);
    int  step(double vin);
    void solveDc();
    void capInit();
    void capUpdate();

    // state
    Netlist nl_{};
    QModel  q_{};
    DModel  d_{};
    OModel  o_{};
    double  h_ = 1.0 / 384000.0, vin_ = 0.0;
    double  vcritQ_ = 0, vcritD_ = 0, nvtD_ = 0;
    double  x_[N] = {0};
    double  F_[N] = {0};
    double  J_[N][N] = {{0}};
    double  Jc_[N][N] = {{0}};                 // linear part, constant
    int8_t  pat_[N][N] = {{0}};                // structural pattern
    double  gcap_[12] = {0};
    double  g2h_ = 0.0;          // 2/h, appearing in every companion model
    double  gm_ = 0.0, inv_imax_ = 0.0;
    SparseLU slu_{};
    double  cv_[12] = {0}, ci_[12] = {0};      // capacitors
    double  dq_[2] = {0}, di_[2] = {0};        // diodes
    double  qbe_[2] = {0}, ibe_[2] = {0}, qbc_[2] = {0}, ibc_[2] = {0};  // BJT
};


// ---------------------------------------------------------------------------
// Instance tables of the nonlinear devices
// ---------------------------------------------------------------------------
struct QInst { int b, bi, c, ci, e; };
struct DInst { int a, ai, k; };
struct OInst { int p, n, o, i2; };

inline constexpr QInst kQ[2] = {
    {N_n3,  N_Q1bi, N_VCC, N_Q1ci, N_n6},
    {N_n19, N_Q2bi, N_VCC, N_Q2ci, N_n17},
};
inline constexpr DInst kD[2] = {
    {N_n5, N_D1ai, N_n7},
    {N_n7, N_D2ai, N_n5},
};
inline constexpr OInst kO[2] = {
    {N_n4, N_n5,  N_n7,  N_A2},
    {N_n9, N_n10, N_n14, N_B2},
};

// The Jacobian's linear part never changes: not between iterations nor
// between samples (while no knob or step moves). Built once, each iteration
// starts from a copy. Saves walking 25 resistors with their divisions and
// branches — nearly half the assembly cost.
inline void Engine::buildConstant()
{
    // The devices' parasitic resistors (RB, RC, RS, RIN, RCM, Rp, Ro) go on
    // the same list as the netlist's, so each contributes to both the Jacobian
    // and the residual.
    for (int i = 0; i < 2; ++i) {
        auto R = [&](int a, int b, double r) {
            if (nl_.nres >= Netlist::kMaxRes) { nl_.overflowed = true; return; }
            nl_.res[nl_.nres++] = {a, b, r};
        };
        // `NLSC_MNA_RB=0` removes the base resistance of both transistors; the
        // default (1) includes it.
        R(kQ[i].b, kQ[i].bi, (NLSC_MNA_RB) ? q_.RB : 1e-9);
        R(kQ[i].c, kQ[i].ci, q_.RC);
        R(kD[i].a, kD[i].ai, d_.RS);
        R(kO[i].p, kO[i].n, o_.RIN);
        R(kO[i].p, N_GND, o_.RCM);
        R(kO[i].n, N_GND, o_.RCM);
        R(kO[i].i2, N_GND, o_.rp());
    }
    std::memset(F_, 0, sizeof F_);
    std::memset(J_, 0, sizeof J_);
    for (int i = 0; i < nl_.nres; ++i)
        stampR(nl_.res[i].a, nl_.res[i].b, nl_.res[i].r);
    for (int i = 0; i < nl_.ncap; ++i) {
        const Cap& c = nl_.cap[i];
        const double g = 2.0 * c.c / h_;
        jac(c.a, c.a, g); jac(c.a, c.b, -g);
        jac(c.b, c.a, -g); jac(c.b, c.b, g);
        gcap_[i] = g;
    }
    // Ro: between the opamp's output (known voltage) and the output node.
    // Only its conductance is constant; the V(2)-dependent part goes in
    // `assemble` because clipping nulls it outside the linear region.
    for (int i = 0; i < 2; ++i) jac(kO[i].o, kO[i].o, 1.0 / o_.RO);
    std::memcpy(Jc_, J_, sizeof J_);
}

inline void Engine::assemble(bool transient)
{
    std::memset(F_, 0, sizeof F_);
    std::memcpy(J_, Jc_, sizeof J_);

    for (int i = 0; i < nl_.nres; ++i) {
        const Res& r = nl_.res[i];
        const double cur = (nv(r.a) - nv(r.b)) / r.r;
        add(r.a, cur); add(r.b, -cur);
    }

    if (transient) {
        for (int i = 0; i < nl_.ncap; ++i) {
            const Cap& c = nl_.cap[i];
            const double cur = gcap_[i] * ((nv(c.a) - nv(c.b)) - cv_[i]) - ci_[i];
            add(c.a, cur); add(c.b, -cur);
        }
    } else {
        // at DC the capacitors do not conduct: their conductance is removed
        for (int i = 0; i < nl_.ncap; ++i) {
            const Cap& c = nl_.cap[i];
            jac(c.a, c.a, -gcap_[i]); jac(c.a, c.b, gcap_[i]);
            jac(c.b, c.a, gcap_[i]);  jac(c.b, c.b, -gcap_[i]);
        }
    }

    for (int i = 0; i < 2; ++i) {
        const QInst& q = kQ[i];
        const double vbe = nv(q.bi) - nv(q.e), vbc = nv(q.bi) - nv(q.ci);
        QOut o = bjt(q_, jbe_, jbc_, vbe, vbc, transient);
        if (transient) {
            const double ibe = g2h_ * (o.qbe - qbe_[i]) - ibe_[i];
            const double ibc = g2h_ * (o.qbc - qbc_[i]) - ibc_[i];
            o.ib += ibe + ibc;
            o.dib_be += g2h_ * o.cbe;
            o.dib_bc += g2h_ * o.cbc;
            o.ic -= ibc;
            o.dic_bc -= g2h_ * o.cbc;
        }
        add(q.bi, o.ib);
        jac(q.bi, q.bi, o.dib_be + o.dib_bc); jac(q.bi, q.e, -o.dib_be); jac(q.bi, q.ci, -o.dib_bc);
        add(q.ci, o.ic);
        jac(q.ci, q.bi, o.dic_be + o.dic_bc); jac(q.ci, q.e, -o.dic_be); jac(q.ci, q.ci, -o.dic_bc);
        add(q.e, -(o.ib + o.ic));
        jac(q.e, q.bi, -(o.dib_be + o.dib_bc + o.dic_be + o.dic_bc));
        jac(q.e, q.e, o.dib_be + o.dic_be);
        jac(q.e, q.ci, o.dib_bc + o.dic_bc);
    }

    for (int i = 0; i < 2; ++i) {
        const DInst& dd = kD[i];
        const double vd = nv(dd.ai) - nv(dd.k);
        const DOut dv = diode(d_, jd_, vd, transient);
        double cur = dv.i, g = dv.g;
        if (transient) {
            cur += g2h_ * (dv.q - dq_[i]) - di_[i];
            g   += g2h_ * dv.c;
        }
        add(dd.ai, cur);  jac(dd.ai, dd.ai, g);  jac(dd.ai, dd.k, -g);
        add(dd.k, -cur);  jac(dd.k, dd.ai, -g);  jac(dd.k, dd.k, g);
    }

    for (int i = 0; i < 2; ++i) {
        const OInst& oa = kO[i];
        // current-limited transconductance = the slew mechanism.
        // `Bgm 0 2` injects current INTO internal node 2.
        const double vd = nv(oa.p) - nv(oa.n);
        // sech²(x) = 1 − tanh²(x): the derivative falls out of the tanh
        // itself and the `cosh` is redundant. Two fewer calls per assembly,
        // exact.
        const double arg = gm_ * vd * inv_imax_;
        const double t = NLSC_TANH(arg);
        const double ign = o_.imax() * t;
        const double dign = gm_ * (1.0 - t * t);
        add(oa.i2, -ign);  jac(oa.i2, oa.p, -dign); jac(oa.i2, oa.n, dign);
        // Output: V = clip(V(2)), then Ro to the output node. The clip is a hard
        // switch.
        const double v2 = nv(oa.i2);
        const double lo = o_.VSAT, hi = VCC - o_.VSAT;
        double vt_, dvo;
        if (v2 < lo)      { vt_ = lo; dvo = 0.0; }
        else if (v2 > hi) { vt_ = hi; dvo = 0.0; }
        else              { vt_ = v2; dvo = 1.0; }
        const double gro = 1.0 / o_.RO;
        add(oa.o, (nv(oa.o) - vt_) * gro);
        jac(oa.o, oa.i2, -dvo * gro);
    }
}

// Dense LU with partial pivoting. Kept as the reference to validate the
// sparse one against, not for production.
inline bool lu_solve(double A[N][N], double b[N], double out[N])
{
    for (int k = 0; k < N; ++k) {
        int p = k; double mx = std::fabs(A[k][k]);
        for (int i = k + 1; i < N; ++i) {
            const double v = std::fabs(A[i][k]);
            if (v > mx) { mx = v; p = i; }
        }
        if (mx == 0.0) return false;
        if (p != k) {
            for (int j = 0; j < N; ++j) { const double t = A[k][j]; A[k][j] = A[p][j]; A[p][j] = t; }
            const double t = b[k]; b[k] = b[p]; b[p] = t;
        }
        const double inv = 1.0 / A[k][k];
        for (int i = k + 1; i < N; ++i) {
            const double f = A[i][k] * inv;
            if (f == 0.0) continue;
            A[i][k] = 0.0;
            for (int j = k + 1; j < N; ++j) A[i][j] -= f * A[k][j];
            b[i] -= f * b[k];
        }
    }
    for (int i = N - 1; i >= 0; --i) {
        double s = b[i];
        for (int j = i + 1; j < N; ++j) s -= A[i][j] * out[j];
        out[i] = s / A[i][i];
    }
    return true;
}

inline void Engine::capInit()
{
    for (int i = 0; i < nl_.ncap; ++i) {
        cv_[i] = nv(nl_.cap[i].a) - nv(nl_.cap[i].b);
        ci_[i] = 0.0;
    }
    for (int i = 0; i < 2; ++i) {
        dq_[i] = diode(d_, jd_, nv(kD[i].ai) - nv(kD[i].k), true).q;
        di_[i] = 0.0;
        const QOut o = bjt(q_, jbe_, jbc_, nv(kQ[i].bi) - nv(kQ[i].e),
                           nv(kQ[i].bi) - nv(kQ[i].ci), true);
        qbe_[i] = o.qbe; qbc_[i] = o.qbc;
        ibe_[i] = ibc_[i] = 0.0;
    }
}

inline void Engine::capUpdate()
{
    for (int i = 0; i < nl_.ncap; ++i) {
        const double vc = nv(nl_.cap[i].a) - nv(nl_.cap[i].b);
        ci_[i] = (2.0 * nl_.cap[i].c / h_) * (vc - cv_[i]) - ci_[i];
        cv_[i] = vc;
    }
    for (int i = 0; i < 2; ++i) {
        const double qq = diode(d_, jd_, nv(kD[i].ai) - nv(kD[i].k), true).q;
        di_[i] = g2h_ * (qq - dq_[i]) - di_[i];
        dq_[i] = qq;
        const QOut o = bjt(q_, jbe_, jbc_, nv(kQ[i].bi) - nv(kQ[i].e),
                           nv(kQ[i].bi) - nv(kQ[i].ci), true);
        ibe_[i] = g2h_ * (o.qbe - qbe_[i]) - ibe_[i];
        ibc_[i] = g2h_ * (o.qbc - qbc_[i]) - ibc_[i];
        qbe_[i] = o.qbe; qbc_[i] = o.qbc;
    }
}

// Limits the junctions and rebuilds the nodes from them. Limiting the nodes
// separately does not bound the junctions, which are their differences.
inline void limit_junctions(double* xn, const double* xo,
                            double vt, double vcritQ, double nvtD, double vcritD)
{
    for (int i = 0; i < 2; ++i) {
        const int b = kQ[i].bi, e = kQ[i].e;
        const double v = Engine::pnjlim(xn[b] - xn[e], xo[b] - xo[e], vt, vcritQ);
        xn[e] = xn[b] - v;
    }
    for (int i = 0; i < 2; ++i) {
        const int a = kD[i].ai, k = kD[i].k;
        const double v = Engine::pnjlim(xn[a] - xn[k], xo[a] - xo[k], nvtD, vcritD);
        xn[k] = xn[a] - v;
    }
}

inline int Engine::step(double vin)
{
    vin_ = vin;
    double xo[N];
    std::memcpy(xo, x_, sizeof x_);
    int k = 0;
    for (; k < 30; ++k) {
        assemble(true);
        double rhs[N], dx[N];
        for (int i = 0; i < N; ++i) rhs[i] = -F_[i];
        if (!slu_.solve(J_, rhs, dx)) break;
        double mx = 0.0;
        for (int i = 0; i < N; ++i) mx = std::fmax(mx, std::fabs(dx[i]));
        // Criterion on the step, not the current residual. With discretised
        // capacitors the companion conductance is 2C/h — for C9 = 10 µF at
        // 384 kHz that is 7.68 S, i.e. currents in amperes — and an absolute
        // current threshold would be 1e-16 relative: Newton would never converge.
        for (int i = 0; i < N; ++i) x_[i] += dx[i];
        if (mx < 1e-9) { ++k; break; }
        double xn[N];
        std::memcpy(xn, x_, sizeof x_);
        limit_junctions(xn, xo, VT, vcritQ_, nvtD_, vcritD_);
        std::memcpy(x_, xn, sizeof x_);
        std::memcpy(xo, x_, sizeof x_);
    }
    capUpdate();
    return k;
}

inline void Engine::solveDc()
{
    vin_ = 0.0;
    for (int i = 0; i < N; ++i) x_[i] = 4.4;
    x_[N_n3] = x_[N_n19] = x_[N_n2] = x_[N_Q1bi] = x_[N_Q2bi] = 3.3;
    x_[N_Q1ci] = x_[N_Q2ci] = VCC;
    x_[N_n6] = x_[N_n17] = x_[N_n20] = 2.7;
    x_[N_n12] = x_[N_out] = 0.0;
    double xo[N];
    for (int k = 0; k < 300; ++k) {
        std::memcpy(xo, x_, sizeof x_);
        assemble(false);
        double rhs[N], dx[N];
        for (int i = 0; i < N; ++i) rhs[i] = -F_[i];
        if (!slu_.solve(J_, rhs, dx)) break;
        double mx = 0.0;
        for (int i = 0; i < N; ++i) { x_[i] += dx[i]; mx = std::fmax(mx, std::fabs(dx[i])); }
        if (mx < 1e-12) break;
        double xn[N];
        std::memcpy(xn, x_, sizeof x_);
        limit_junctions(xn, xo, VT, vcritQ_, nvtD_, vcritD_);
        std::memcpy(x_, xn, sizeof x_);
    }
}

} // namespace mna
} // namespace nlsc
