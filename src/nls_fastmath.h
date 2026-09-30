// Fast transcendentals for the solver's hot path.
//
// In the DK engine every Newton iteration makes 16 transcendental calls:
// 8 `exp` (three per transistor, one per diode), 6 `pow` (the junction
// charge) and 2 `tanh` (the opamp's slew limiter). glibc's `pow` is by far
// the most expensive of them because it is generic: here the exponent is a
// device model constant (MJE, MJC, M), never a variable. `u^-m` with fixed m
// is one log2 plus one exp2, without the extended-precision machinery glibc
// needs for the general case.
//
// Precision: a relative error of 1e-13 in a junction capacitance (itself a
// correction term) sits far below the model's accuracy against ngspice.
//
// The product compiles them with `-fassociative-math -fno-signed-zeros
// -fno-trapping-math` (`PRODFP` in the Makefile): reassociation only moves
// rounding here (there is no magic-constant trick for it to fold).
// The full `-ffast-math` must not be used: its `-ffinite-math-only` removes
// the NaN guards the engine relies on.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace nlsc {
namespace fast {

// The polynomials are latency-bound, not instruction-bound:
//   1. Tree evaluation (Estrin) instead of Horner: log depth instead of
//      linear, so the processor keeps several operations in flight (a
//      degree-9 Horner is nine chained multiply-adds).
//   2. Degrees sized to the margin that is needed: a relative error of 5e-11
//      is −206 dB, far below the model's accuracy.

// 2^f for f ∈ [−0,5, 0,5]. Degree 7, relative error 5,6e-11 = −205 dB.
inline constexpr double kP2[8] = {
    9.99999999959548691e-01,
    6.93147180545930941e-01,
    2.40226512135955361e-01,
    5.55041094121648398e-02,
    9.61802560292859821e-03,
    1.33334505594005347e-03,
    1.54697319841336223e-04,
    1.53100814061525208e-05
};

// 2^n built by hand in the exponent field. `std::ldexp` is a call; this is
// two instructions. The range is clamped before reaching here.
inline double pow2i(int n)
{
    if (n < -1022) return 0.0;
    if (n >  1023) return HUGE_VAL;
    const std::uint64_t b = static_cast<std::uint64_t>(n + 1023) << 52;
    double d; std::memcpy(&d, &b, sizeof d); return d;
}

inline double exp2_(double x)
{
    if (!(x > -1000.0)) return 0.0;          // also traps NaN
    if (x > 1000.0) return HUGE_VAL;
    const double n = std::floor(x + 0.5);
    const double f = x - n;                  // ∈ [−0,5, 0,5]
    const double f2 = f * f, f4 = f2 * f2;
    const double* c = kP2;
    const double a = c[0] + c[1] * f, b = c[2] + c[3] * f;
    const double d = c[4] + c[5] * f, e = c[6] + c[7] * f;
    const double p = (a + b * f2) + (d + e * f2) * f4;
    return p * pow2i(static_cast<int>(n));
}

inline constexpr double kLog2E = 1.4426950408889634074;
inline constexpr double kLn2   = 0.69314718055994530942;

inline double exp_(double x) { return exp2_(x * kLog2E); }

// log2 via the atanh series over the mantissa reduced to [1/√2, √2]. There
// s = (m−1)/(m+1) never exceeds 0,1716, so the remainder past s⁹ is ~2e-9
// relative — and it is further multiplied by the model exponent (0,26…0,5).
inline double log2_(double x)
{
    std::uint64_t b; std::memcpy(&b, &x, sizeof b);
    int e = static_cast<int>((b >> 52) & 0x7FF) - 1023;
    b = (b & 0x000FFFFFFFFFFFFFull) | 0x3FF0000000000000ull;
    double m; std::memcpy(&m, &b, sizeof m);   // m ∈ [1, 2)
    if (m > 1.4142135623730951) { m *= 0.5; ++e; }
    const double s = (m - 1.0) / (m + 1.0), s2 = s * s, s4 = s2 * s2;
    // (2/ln2)·(s + s³/3 + s⁵/5 + s⁷/7 + s⁹/9), as a tree
    const double a = 1.0 + s2 / 3.0, c = 1.0 / 5.0 + s2 / 7.0;
    // No division by s: s == 0 for every exact power of two (m == 1).
    const double t = a + (c + s4 / 9.0) * s4;             // = a + c·s⁴ + s⁸/9
    return double(e) + (2.0 * kLog2E) * s * t;
}

// u^(−m) with m a model constant. It is the only shape of `pow` this project
// needs.
inline double pow_neg(double u, double m) { return exp2_(-m * log2_(u)); }

// tanh(x) = x·P(x²) for |x| ≤ 0,6. Degree 6 in x², relative error 8,7e-11.
//
// This branch is the normal case: the argument is gm·v/imax with v the
// opamp's differential input, which in closed loop is microvolts, and that is
// where the exponential form (e−1)/(e+1) loses precision by cancellation
// (5,6e-10 relative error at x=1,2e-5).
inline constexpr double kTH[7] = {
    9.99999999917060789e-01,
    -3.33333310510560588e-01,
    1.33332304196908474e-01,
    -5.39507475657221847e-02,
    2.17261849876820325e-02,
    -8.24830328184371006e-03,
    2.21469946389090538e-03
};

// tanh. Away from zero, from one exponential: glibc's `std::tanh` goes
// through `expm1` and costs more. Above 20 the result is forced to ±1, which
// is what the double returns anyway.
inline double tanh_(double x)
{
    const double a = std::fabs(x);
    if (a <= 0.6) {
        const double y = x * x, y2 = y * y;
        const double* c = kTH;
        const double p0 = c[0] + c[1] * y, p1 = c[2] + c[3] * y;
        const double p2 = c[4] + c[5] * y;
        return x * ((p0 + p1 * y2) + (p2 + c[6] * y2) * y2 * y2);
    }
    if (x > 20.0)  return  1.0;
    if (x < -20.0) return -1.0;
    const double e = exp2_(2.0 * x * kLog2E);
    return (e - 1.0) / (e + 1.0);
}

} // namespace fast
} // namespace nlsc
