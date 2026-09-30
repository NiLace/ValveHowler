// Polyphase oversampler for the nonlinear engine.
//
// The circuit distorts, so it generates harmonics above Nyquist; without
// oversampling those harmonics fold back and sound like intermodulation the
// real pedal does not have. The factor is set by the measured ASR
// (aliasing-to-signal ratio).
//
// Standard structure: interpolation by zero insertion + low-pass, and
// decimation by low-pass + discard, both from the same Kaiser prototype. The
// implementation is polyphase, i.e. it never multiplies by the zeros it just
// inserted: L/N multiplications per output sample instead of L.
//
// The filter introduces delay and it must be declared to the host through
// the latency port, or a user mixing in parallel with the dry signal gets a
// comb.

#pragma once

#include <cstdio>   // only for `warn_if_clamped` (see its comment)
#include <cmath>
#include <cstring>

// The length of each polyphase branch, the Kaiser beta and the prototype's
// cutoff. The filter is all of the plugin's latency (`taps - 1` base
// samples), so these three trade latency and CPU against the top octave.
//
// 32 taps, beta 5, cutoff at the host's Nyquist: 31 samples of latency
// (0.65 ms at 48 kHz), and harmonics 17 / 19 of a 1057 Hz tone (0.3 V input,
// drive 0.9, tone 1) within 0.6 dB of the same build run at 192 kHz, measured
// at 44.1 and 48 kHz. Near 20 kHz this filter also rejects more alias than a
// short one, which leaks past its nominal stopband. Changing any of the three
// moves the declared latency and the level of the top octave, so a retune has
// to be checked on both.
#ifndef NLSC_OS_TAPS
#define NLSC_OS_TAPS 32
#endif
#ifndef NLSC_OS_BETA
#define NLSC_OS_BETA 5.0
#endif
// The prototype's cutoff as a fraction of the base rate's Nyquist. At 0.9 the
// transition band ends at the host's Nyquist, so nothing folds back, but it
// starts below 20 kHz at 44.1 and 48 kHz and attenuates the top octave the
// clipper makes. At 1.0 the transition straddles Nyquist, which needs more
// taps to keep 20 kHz in the passband.
#ifndef NLSC_OS_FC
#define NLSC_OS_FC 1.0
#endif

// `NLSC_OS_F32=1` runs the oversampler in float32: coefficient and delay-line
// storage and the inner loops move to float, while the filter design stays in
// double (it runs once and the Kaiser window uses a Bessel series). Off by
// default: on x86 the working set already fits in L1 and the FIR is a small
// share of the DSP, so it buys no measurable speed. With 0 the code is the
// double-precision path.
#ifndef NLSC_OS_F32
#define NLSC_OS_F32 0
#endif

// How many partial sums the decimation accumulator is split into. 4 is one
// 256-bit vector register of doubles; 1 is a single serial accumulator. See
// the note on `downsample()`.
#ifndef NLSC_OS_ACC_LANES
#define NLSC_OS_ACC_LANES 4
#endif

// The same, for the interpolation accumulator, as a separate switch because
// the two legs differ in consequence. `downsample()` is terminal: its result
// leaves through a `float` port and nothing reads it back. Splitting it
// reassociates a float sum, so the returned value is not bit-identical; in the
// default double build (`NLSC_OS_F32=0`) the difference did not survive the
// cast to the float port, but with `NLSC_OS_F32=1` it reaches the port
// directly. A change here is judged with a sample-by-sample audio diff, not a
// null.
// `upsample()` feeds the nonlinear core, where a last-bit change enters an
// iterative solver. It stays at 1: `upsample()`'s independent polyphase
// branches already provide the instruction-level parallelism, so splitting
// its accumulator buys no speed and only perturbs the output.
#ifndef NLSC_OS_UP_ACC_LANES
#define NLSC_OS_UP_ACC_LANES 1
#endif

namespace nlsc {

#if NLSC_OS_F32
using osreal = float;
#else
using osreal = double;
#endif

// Modified Bessel of the first kind, order 0, by series. Only used when
// designing the filter (once), so clarity rules.
inline double besselI0(double x)
{
    double s = 1.0, t = 1.0;
    for (int k = 1; k < 60; ++k) {
        t *= (x * 0.5) / k;
        const double c = t * t;
        s += c;
        if (c < 1e-18 * s) break;
    }
    return s;
}

class Oversampler {
public:
    // The factor cap. `upsample()` writes N values into the caller's buffer,
    // so it is public: callers size that buffer with it instead of repeating
    // a literal, and `prepare()` clamps N to it.
    static constexpr int kMaxOS = 32;
    // The same for the taps per phase: the buffers below are fixed arrays
    // sized by these two caps. Fixed arrays keep allocation out of the
    // engine and emit no template code that could be merged at link time
    // across the ISA-specific translation units.
    static constexpr int kMaxTaps = 32;
    static constexpr int kMaxLen  = kMaxOS * kMaxTaps;

    // `taps_per_phase` is the length of each polyphase branch; the
    // prototype's is `taps_per_phase * N`. The filter is all of the plugin's
    // latency (`tf_ - 1` base samples), so both it and beta are
    // compile-time adjustable. Note that the filter feeds the nonlinear
    // engine: a weaker anti-alias leaves more high-frequency content for the
    // solver, so a shorter filter does not necessarily make the plugin
    // cheaper.
    void prepare(int N, int taps_per_phase = NLSC_OS_TAPS, double beta = NLSC_OS_BETA)
    {
        // Clamped from above, and the clamp is recorded in a member
        // (`clamped()`) rather than printed: no I/O belongs in the plugin's
        // code path (`fprintf` takes the FILE lock and may block). A caller
        // that may print reports it with `warn_if_clamped()`.
        clamped_ = (N > kMaxOS) || (taps_per_phase > kMaxTaps);
        if (N > kMaxOS) N = kMaxOS;
        if (taps_per_phase > kMaxTaps) taps_per_phase = kMaxTaps;
        n_ = (N < 1) ? 1 : N;
        tf_ = (taps_per_phase < 1) ? 1 : taps_per_phase;
        const int L = tf_ * n_;
        len_ = L;
        for (int i = 0; i < L; ++i) h_[i] = 0.0;
        if (n_ == 1) { h_[0] = 1.0; hd_[0] = osreal(1); reset(); return; }

        // Cutoff at `NLSC_OS_FC` of the base rate's Nyquist, referred to the
        // oversampled rate: fc = 0.5/N · NLSC_OS_FC.
        const double fc = 0.5 * double(NLSC_OS_FC) / double(n_);
        const double m = 0.5 * double(L - 1);
        const double i0b = besselI0(beta);
        double sum = 0.0;
        for (int i = 0; i < L; ++i) {
            const double x = double(i) - m;
            const double sinc = (std::fabs(x) < 1e-12)
                              ? 2.0 * fc
                              : std::sin(2.0 * M_PI * fc * x) / (M_PI * x);
            const double r = 2.0 * double(i) / double(L - 1) - 1.0;
            const double w = besselI0(beta * std::sqrt(1.0 - r * r)) / i0b;
            h_[i] = sinc * w;
            sum += h_[i];
        }
        // Normalise to unity DC gain. Interpolation additionally needs the
        // factor N, because inserting zeros splits the energy across the N
        // phases.
        for (int i = 0; i < L; ++i) h_[i] /= sum;
        // Transposed polyphase branches: `hp_[f*tf_ + k]` = `h_[k*n_ + f]`,
        // so `upsample`'s inner loop reads contiguous memory.
        for (int f = 0; f < n_; ++f)
            for (int k = 0; k < tf_; ++k)
                hp_[size_t(f) * tf_ + k] = osreal(h_[k * n_ + f]);
        // The downsampling prototype, in working precision. It is `h_` as
        // is; copied rather than converted per tap inside the loop, which
        // would block vectorisation.
        for (int k = 0; k < L; ++k) hd_[k] = osreal(h_[k]);
        // The state lines are zeroed in reset(); the doubled length keeps the
        // window contiguous (see `upsample`).
        reset();
    }

    int factor()  const { return n_; }
    // Total round-trip delay, in base-rate samples.
    int latency() const { return (n_ == 1) ? 0 : (tf_ - 1); }

    // Whether the last `prepare()` had to clamp the factor or the taps. If
    // true, the oversampler is not running at the requested factor; a caller
    // that takes the factor from user input must check it.
    bool clamped() const { return clamped_; }

    void reset()
    {
        // Reached from run() (a mode switch re-seeds the oversampler): only
        // the used prefix of each state line is cleared.
        for (int i = 0; i < 2 * tf_; ++i) zu_[i] = osreal(0);
        for (int i = 0; i < 2 * len_; ++i) zd_[i] = osreal(0);
        iu_ = id_ = 0;
    }

    // One base sample -> N oversampled samples.
    //
    // The delay line is stored doubled (each sample at `pos` and at
    // `pos+tf_`), so the window of the last `tf_` values, newest to oldest,
    // is always contiguous with no wraparound branch; and the polyphase
    // branches are transposed into `hp_` so each one is contiguous. Both
    // give the inner loop contiguous, branch-free accesses; the reduction
    // itself vectorises only where reassociation is allowed (a build with
    // `-fassociative-math`, or `NLSC_OS_UP_ACC_LANES` > 1).
    void upsample(double x, osreal* outside)
    {
        if (n_ == 1) { outside[0] = osreal(x); return; }
        iu_ = (iu_ == 0) ? tf_ - 1 : iu_ - 1;
        zu_[iu_] = osreal(x);  zu_[iu_ + tf_] = osreal(x);
        const osreal* z = &zu_[iu_];
        for (int f = 0; f < n_; ++f) {
            const osreal* hf = &hp_[size_t(f) * tf_];
#if NLSC_OS_UP_ACC_LANES > 1
            // The same accumulator split as `downsample()`, applied inside
            // each branch (see `NLSC_OS_UP_ACC_LANES`).
            osreal acc[NLSC_OS_UP_ACC_LANES];
            for (int j = 0; j < NLSC_OS_UP_ACC_LANES; ++j) acc[j] = 0;
            const int Kv = tf_ - (tf_ % NLSC_OS_UP_ACC_LANES);
            for (int k = 0; k < Kv; k += NLSC_OS_UP_ACC_LANES)
                for (int j = 0; j < NLSC_OS_UP_ACC_LANES; ++j)
                    acc[j] += hf[k + j] * z[k + j];
            osreal a = 0;
            for (int k = Kv; k < tf_; ++k) a += hf[k] * z[k];
            for (int j = 0; j < NLSC_OS_UP_ACC_LANES; ++j) a += acc[j];
#else
            osreal a = 0;
            for (int k = 0; k < tf_; ++k) a += hf[k] * z[k];
#endif
            outside[f] = a * osreal(n_);
        }
    }

    // N oversampled samples -> one base sample.
    //
    // The accumulator is split into `NLSC_OS_ACC_LANES` partial sums. With a
    // single accumulator the dot product is one serial chain of dependent
    // adds (a floating-point reduction does not vectorise without
    // reassociation), so the loop is latency-bound. With one partial sum per
    // lane the adds are independent, the compiler keeps them in a vector
    // register, and the horizontal sum is paid once at the end. That
    // reasoning holds for a build without `-fassociative-math`. The plugin
    // itself is built with `-fassociative-math`, where the compiler may
    // reassociate either way; there the explicit split fixes the summation
    // order.
    //
    // The tail loop handles an `L = tf_ * n_` that is not a multiple of the
    // lane count, which the plugin's factors never produce but other
    // prototypes may.
    double downsample(const osreal* inside)
    {
        if (n_ == 1) return double(inside[0]);
        const int L = len_;
        for (int f = 0; f < n_; ++f) {
            id_ = (id_ == 0) ? L - 1 : id_ - 1;
            zd_[id_] = inside[f];  zd_[id_ + L] = inside[f];
        }
        const osreal* z = &zd_[id_];
#if NLSC_OS_ACC_LANES > 1
        osreal acc[NLSC_OS_ACC_LANES];
        for (int j = 0; j < NLSC_OS_ACC_LANES; ++j) acc[j] = 0;
        const int Lv = L - (L % NLSC_OS_ACC_LANES);
        for (int k = 0; k < Lv; k += NLSC_OS_ACC_LANES)
            for (int j = 0; j < NLSC_OS_ACC_LANES; ++j)
                acc[j] += hd_[k + j] * z[k + j];
        osreal a = 0;
        for (int k = Lv; k < L; ++k) a += hd_[k] * z[k];
        for (int j = 0; j < NLSC_OS_ACC_LANES; ++j) a += acc[j];
#else
        osreal a = 0;
        for (int k = 0; k < L; ++k) a += hd_[k] * z[k];
#endif
        return double(a);
    }

private:
    bool clamped_ = false;   // the last prepare() had to clamp the factor
    int n_ = 1, tf_ = NLSC_OS_TAPS, len_ = 1, iu_ = 0, id_ = 0;
    // `h_` stays double: it is the design prototype (and the source of
    // `hp_`), computed once. What runs per sample are `hp_` (up) and `hd_`
    // (down), which use `osreal`.
    double h_[kMaxLen] = {};
    osreal hp_[kMaxLen] = {}, hd_[kMaxLen] = {}, zu_[2 * kMaxTaps] = {}, zd_[2 * kMaxLen] = {};
};


// Reports a clamped factor on stderr. It lives outside `prepare()` so that
// the plugin's code path contains no I/O: being `inline` and never called by
// the plugin, it is not emitted into the plugin binary, which therefore
// references no `fprintf`. A caller that takes the factor from user input
// should call it, so a clamp is never silent.
inline bool warn_if_clamped(const Oversampler& ovs, int requested, const char* who)
{
    if (!ovs.clamped()) return false;
    std::fprintf(stderr,
        "%s: requested oversampling factor %d exceeds the cap %d and has been "
        "clamped; the output is not at the requested factor.\n",
        who, requested, Oversampler::kMaxOS);
    return true;
}

} // namespace nlsc
