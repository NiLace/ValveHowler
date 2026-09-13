// Polyphase oversampler for the nonlinear engine.
//
// The circuit distorts, so it generates harmonics above Nyquist; without
// oversampling those harmonics fold back and sound like intermodulation the
// real pedal does not have. The factor is NOT chosen by taste: the ASR
// (aliasing-to-signal) is measured and the figure decides — see
// `docs/SOBREMUESTREO.md`.
//
// Standard structure: interpolation by zero insertion + low-pass, and
// decimation by low-pass + discard, both from the SAME Kaiser prototype. The
// implementation is polyphase, i.e. it never multiplies by the zeros it just
// inserted: L/N multiplications per output sample instead of L.
//
// The filter introduces DELAY and it must be declared to the host through
// the latency port, or a user mixing in parallel with the dry signal gets a
// comb.

#pragma once

#include <cstdio>   // only for `warn_if_clamped` (see its comment)
#include <cmath>
#include <cstring>
#include <vector>

// Lever D5 of `docs/OPTIMIZACIONES.md`. The defaults are the ORIGINALS:
// changing them is an optimisation and must be measured against the frozen
// exact binary, never against the previous version alone.
// 8 TAPS, and measured against the thing that actually matters: not the distance to some exact model, but HOW AUDIBLE
// THE CHANGE IS against what already ships. For a change to a shipped product
// the current build IS the reference, and no oracle is needed.
//
// 18 knob points, real DI, aligned: median −82,86 dB, best −93,71, WORST
// −67,04 (drive 1,0 / tone 1,0 / lvl 1,0) — 7,0 dB below the port yardstick.
// Control that came free: the aligner recovered exactly +8 samples at all 18
// points, which is the latency difference (15 − 7). The instrument found the
// offset it was supposed to find.
//
// AND WHY THE TWO OBVIOUS YARDSTICKS COULD NOT DECIDE THIS, because both
// looked fine and both were blind:
//   · `barrido_taps.py` scores against `build/ref/run_os_exacto`, FROZEN at
//     commit af6606a (7-Aug). The MA150 diode and the 2SC1815 transistor landed
//     after that, so what it measures is today's model against the 7-Aug one,
//     which buries the filter. Symptom: 16, 12, 10, 8 and 6 taps all returned
//     the SAME number to the last decimal.
//   · The ANMR grid builds BOTH its reference and its candidate from the SAME
//     binary, so compiling with 8 taps puts 8 taps on both sides and the filter
//     CANCELS in the subtraction. Symptom: +0,00 dB in all 36 cells, both
//     engines: a yardstick built against itself cannot see the modelling.
// => Two different mechanisms, one identical symptom: a figure that does not
// move when it must.
//
// What this buys is LATENCY, 15 -> 7 samples (0,31 -> 0,15 ms at 48 kHz).
// It also buys CPU, and the line here used to deny it: "NOT CPU: the whole
// FIR is ~1,7 % of the DSP". That 1,7 % is retracted 110 lines below — it was a
// NET difference with the engine's reaction inside it, and the FIR measures
// 8,6 % of the DSP by profile. This file carried BOTH answers for an hour, and
// the stale one was the copy glued to `NLSC_OS_TAPS`, which is what anyone
// about to touch the taps reads FIRST, and a rule written twice diverges.
// => The NET of this change was −2,27 % of instructions; the deciding argument
// is still the latency, but not because the filter is cheap.
// And it moves the neural dataset's master, which compensates that latency.
// Reversible: `-DNLSC_OS_TAPS=16`.
#ifndef NLSC_OS_TAPS
#define NLSC_OS_TAPS 8
#endif
#ifndef NLSC_OS_BETA
#define NLSC_OS_BETA 7.0
#endif

// A4 (float32), FIRST HALF — the oversampler.
//
// `docs/OPTIMIZACIONES.md` §11 estimated A4 would buy 10-15 % and that it
// would come "more from cache than from lanes". THE CACHE TURNED OUT FALSE:
// measured with cachegrind, the engine's D1 miss rate is 0,0 % (29 712
// misses over the whole run) because its working set is 5,9 kB against
// 32 kB of L1d. It already fit. So the only thing A4 can buy on x86 is SIMD
// LANES, and that dictates WHERE to apply it:
//
//   profile of the day   newton 37,4 % · process 23,0 % · bjt 15,3 % · f 11,9 %
//
// `Plugin::process` is the oversampling: a FIR, the best-vectorising
// operation there is, and float32 doubles its lanes. The engine's 10x10 LU,
// by contrast, vectorises poorly. => Attack HERE first: half the benefit for
// a fraction of the risk, without touching the solver.
//
// The filter DESIGN stays in `double`: it runs once, offline, and the
// Kaiser window carries a series Bessel. What moves to float is the STORAGE
// of coefficients and delay lines, plus the inner loop. MIXED precision.
//
// With `NLSC_OS_F32=0` the output must be BIT-IDENTICAL to before this
// change. That is the gate that the refactor touched nothing else.
// Verified at 2x, 4x and 8x over 240 000 samples of real DI.
//
// ─────────────────────────────────────────────────────────────────────────────
// MEASURED, AND IT DOES NOT PAY — which is why the default is 0
//
// | yardstick | result |
// |---|---|
// | quality (vs the exact build, real DI 0,5 V) | −138 / −136 / −134 dB at 2/4/8x |
// | cost, 21 alternated pairs with sign test | −1,52 %, wins 11/21, p = 1,00 |
// | instructions of `Plugin::process` | −1,41 % (−0,32 % of the program) |
//
// Quality is irrelevant (−136 dB sits 65 dB below the 4x's own error). What
// fails is the benefit: there is no measurable difference.
//
// And the reason matters, because it invalidates §11's estimate:
//
//   1. THE CACHE WAS FALSE. Measured with cachegrind: D1 miss rate 0,0 %.
//      The working set is 5,9 kB against 32 kB of L1d — it already fit
//      whole, so shrinking it to 2,9 kB buys nothing.
//   2. THE 23 % OF `Plugin::process` WAS NOT THE FIR. `dk::Engine::process`
//      is INLINED in there, so that percentage is almost all engine. The FIR
//      is a small fraction, which is why floating it moves 1,4 %.
//
// => The switch stays written, measured and bit-identity-gated, because on
// NEON float32 genuinely doubles the lanes. But ARM is out of this project's
// scope, so today it stays OFF: shipping an unvalidated variant in exchange
// for an effect indistinguishable from zero is all risk and no prize.
// ─────────────────────────────────────────────────────────────────────────────
#ifndef NLSC_OS_F32
#define NLSC_OS_F32 0
#endif

// How many PARTIAL SUMS the decimation accumulator is split into. 4 is one
// AVX2 vector register of doubles, which is the shape the profile asked for;
// 1 restores the single serial accumulator that shipped before it, and is
// the A arm of the paired bench. See the long note on `bajar()`.
#ifndef NLSC_OS_ACC_LANES
#define NLSC_OS_ACC_LANES 4
#endif

// The same, for the INTERPOLATION accumulator (row 30). It is a SEPARATE
// switch on purpose: the two legs share the shape and NOT the consequence.
// `bajar()` is TERMINAL — its result leaves through a `float` port and
// nothing reads it back, which is why splitting it came out BIT-IDENTICAL.
// `subir()` FEEDS THE NONLINEAR CORE: a last-bit change goes into an iterative
// solver that can amplify it. The fidelity answer does NOT travel from one to
// the other, and it was measured separately
// (the width of the port decides whether a reassociation is visible at all).
// **0 BY MEASUREMENT — and not "it fell short": there was NO
// STALL to remove.** Two paired runs: **−1,02 % (9/21, p = 0,664)** and
// **+0,00 % (10/21, p = 1)**. And it costs: the output stops being
// bit-identical (worst **−98,24 dB**), which is paying something for nothing.
//
// THE PREMISE WAS REFUTABLE WITHOUT A BENCH, from that same morning's
// profile table. A stall's signature is **share of CYCLES above share of
// INSTRUCTIONS**, and the two legs say opposite things:
//     downsample()  5,01 % cycles / 3,45 % instructions  => 1,45x : STALLS
//     upsample()    2,28 % cycles / 2,86 % instructions  => 0,80x : HAS SLACK
// `upsample()` retires MORE instructions per cycle than the DSP's average.
// There was no latency to hide, so splitting its accumulator could buy nothing.
//
// => What led to opening it was `nls_os.h:343` being the file's **hottest**
// line (2,27 % of the DSP). A high share says **where the time goes, not
// whether it can be recovered**. `upsample()`'s four branches already gave the
// ILP; the chain of 8 inside them was never the bottleneck.
// The switch stays so it can be re-measured in a build, and `make decisiones`
// checks it remains 0.
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
    // THE FACTOR CAP, AND WHY IT IS PUBLIC.
    //
    // `subir()` writes N values into the CALLER's buffer, and nine places in
    // the repo declared it `osreal up[16]` with the 16 written by hand.
    // `prepare()` clamped from below (N<1 -> 1) and NOT from above, so
    // asking for os=32 smashed the stack: `free(): invalid pointer` and
    // abort.
    // And the crash was not the expensive part: the harness that invoked
    // it PRINTED an equal-looking number for that row, because it never
    // checked the exit code. An overflow that also produces a plausible
    // figure is the worst of both worlds.
    //
    // => The cap lives HERE, next to the code that enforces it, and callers
    // use it instead of repeating the literal.
    static constexpr int kMaxOS = 32;

    // `taps_fase` is the length of each polyphase branch; the prototype's is
    // `taps_fase * N`. With 16 and β = 7 the stopband rejection exceeds
    // 80 dB, which is where the filter stops ruling and the engine itself
    // takes over.
    //
    // 80 dB of rejection against a −60 dB target is ~20 dB of surplus,
    // and the filter is ALL of the plugin's latency (`tf_ − 1` base
    // samples). That is why both are compile-adjustable (lever D5):
    // `make TAPS=12`.
    //
    // SECOND CORRECTION — the "~1,7 %" below was a NET
    // difference, not the filter's cost, and it is off by 5x.
    //
    // What the old note measured, by DIFFERENCE at 4x (callgrind, only
    // `-DNLSC_OS_TAPS` changing):
    //     16 taps -> 1 223 350 607 Ir      8 taps -> 1 214 000 843 Ir
    // and concluded "halving removes 9,35 M, so the whole FIR is ~18,7 M
    // over ~1071 M of DSP: ~1,7 %". (An earlier claim of "14 % of the
    // profile" was already retracted; it came from a post-A7 profile.)
    //
    // A DIFFERENCE CANNOT MEASURE THIS FILTER, because the filter feeds
    // the engine: a worse anti-alias leaves more high-frequency content and
    // Newton works harder, so the saving is partly cancelled by an increase
    // nobody attributes. The control that proves it costs one build — go one
    // step further down. Measured here, same material, `perf stat`, three
    // reps each (spread ~100 instructions in 2e9):
    //     16 taps 2 078 327 328 Ir | 8 taps 2 031 108 708 | 4 taps 2 112 036 358
    //     16 -> 8: -47,2 M (-2,27 %)      8 -> 4: +80,9 M (+3,98 %)
    // => HALVING THE FILTER AGAIN MAKES THE PLUGIN MORE EXPENSIVE. The sign
    // flips, so the difference is measuring the engine, not the FIR.
    //
    // What the FIR actually costs, by PROFILE instead (`perf record` over
    // 60 runs, `-g` twin whose `.text` is byte-identical to the product,
    // share taken against the plugin's own samples, two independent runs
    // agreeing to 0,07 points):
    //     nls_os.h = 8,6 % of the DSP in CYCLES · 8,4 % in INSTRUCTIONS
    //     the two convolution loops alone = 7,3 % of the DSP
    // => The FIR is ~5x more of the DSP than this comment said.
    //
    // AND THE TWO LEGS ARE NOT SYMMETRIC, which is the useful part.
    // Both do 32 multiply-adds per base sample (`subir`: n_=4 branches of
    // tf_=8 · `bajar`: one dot product of L = tf_*n_ = 32):
    //     line 309 `bajar`  5,01 % of DSP in cycles, 3,45 % in instructions
    //     line 293 `subir`  2,28 %                   2,86 %
    //     ratio             2,19x in CYCLES          1,21x in INSTRUCTIONS
    // Same work, 2,2x the clock: `bajar` accumulates ONE serial chain of 32
    // while `subir` has four independent chains of 8, so the decimation leg
    // is latency-bound, not throughput-bound
    // (a floating-point reduction does not vectorise on its own).
    // => The expensive leg is the one NO lever in the catalogue names: D6
    // shortens the UPSAMPLING filter, which is the cheap one.
    //
    // => D5 was still applied for LATENCY and that decision stands. What
    // changes is the reason written beside it: shortening to 8 taps bought
    // -2,27 % of instructions NET, and that net already has the engine's
    // reaction inside it.
    //
    // Shortening is NOT decided by counting rejection dB: it is decided by
    // measuring the ACCUMULATED ANMR against the frozen exact binary, which
    // is what `harness/barrido_taps.py` does.
    void prepare(int N, int taps_fase = NLSC_OS_TAPS, double beta = NLSC_OS_BETA)
    {
        // Clamped from ABOVE and NOT IN SILENCE: clamping quietly would
        // turn an overflow into a measurement at a different factor than
        // believed.
        //
        // BUT IT DOES NOT PRINT FROM HERE, and the RT gate caught this:
        // the first version called `fprintf`, which puts the symbol into the
        // PRODUCT's .so. `stderr` is buffered => every `fprintf` takes the
        // FILE lock, and a write(2) blocks if nobody reads the other end.
        // That in THIS path `prepare()` is only called from `instantiate()`
        // does not save it: the gate looks at the ARTEFACT, not the flow,
        // and rightly so — fifteen harnesses share this same header.
        //
        // => It latches into a member and WHOEVER MAY PRINT publishes it,
        // which is exactly what the gate prescribes. `clamped()` is the
        // query.
        clamped_ = (N > kMaxOS);
        if (clamped_) N = kMaxOS;
        n_ = (N < 1) ? 1 : N;
        tf_ = taps_fase;
        const int L = tf_ * n_;
        h_.assign(L, 0.0);
        if (n_ == 1) { h_[0] = 1.0; reset(); return; }

        // Cutoff at 90 % of the BASE rate's Nyquist, referred to the
        // oversampled rate: fc = 0,5/N · 0,9. The 10 % margin is the
        // transition band; pushing to Nyquist demands a much longer filter
        // to win a band the circuit itself no longer uses.
        const double fc = 0.45 / double(n_);
        const double m = 0.5 * double(L - 1);
        const double i0b = besselI0(beta);
        double suma = 0.0;
        for (int i = 0; i < L; ++i) {
            const double x = double(i) - m;
            const double sinc = (std::fabs(x) < 1e-12)
                              ? 2.0 * fc
                              : std::sin(2.0 * M_PI * fc * x) / (M_PI * x);
            const double r = 2.0 * double(i) / double(L - 1) - 1.0;
            const double w = besselI0(beta * std::sqrt(1.0 - r * r)) / i0b;
            h_[i] = sinc * w;
            suma += h_[i];
        }
        // Normalise to unity DC gain. Interpolation additionally needs the
        // factor N, because inserting zeros splits the energy across the N
        // phases.
        for (double& v : h_) v /= suma;
        // Transposed polyphase branches: `hp_[f*tf_ + k]` = `h_[k*n_ + f]`,
        // so `subir`'s inner loop reads contiguous memory.
        hp_.assign(size_t(n_) * tf_, osreal(0));
        for (int f = 0; f < n_; ++f)
            for (int k = 0; k < tf_; ++k)
                hp_[size_t(f) * tf_ + k] = osreal(h_[k * n_ + f]);
        // The downsampling prototype, in working precision. It is `h_` as
        // is; copied rather than converted per tap inside the loop, which
        // would block vectorisation.
        hd_.assign(h_.size(), osreal(0));
        for (size_t k = 0; k < h_.size(); ++k) hd_[k] = osreal(h_[k]);
        // The state lines are SIZED here — prepare() is the only allocator —
        // and merely zeroed in reset(). The DOUBLE length keeps the window
        // contiguous (see `subir`).
        zu_.assign(size_t(2 * tf_), osreal(0));
        zd_.assign(2 * h_.size(), osreal(0));
        reset();
    }

    int factor()  const { return n_; }
    // Total round-trip delay, in BASE-rate samples.
    int latency() const { return (n_ == 1) ? 0 : (tf_ - 1); }

    // Did the last `prepare()` have to CLAMP the factor? If true, whatever
    // is being measured is NOT at the requested factor. Any harness that
    // takes the factor from the command line must check this and abort.
    bool clamped() const { return clamped_; }

    void reset()
    {
        // A plain fill loop, not `assign`: reset() is reached from run()
        // (a mode switch re-seeds the oversampler) and must not be able to
        // allocate even in principle. prepare() is the only allocator; it
        // sizes zu_/zd_ right before calling this.
        for (auto& v : zu_) v = osreal(0);
        for (auto& v : zd_) v = osreal(0);
        iu_ = id_ = 0;
    }

    // One base sample -> N oversampled samples.
    //
    // Lever D3. The previous version walked the circular buffer BACKWARDS
    // with a wraparound branch per tap, and read the coefficients with
    // stride `n_`. Neither vectorises, even with reassociation allowed: the
    // compiler cannot prove the accesses contiguous nor remove the branch.
    //
    // Now: the buffer is stored DOUBLED (each sample at `pos` and at
    // `pos+tf_`), so the window of the last `tf_` values — newest to oldest
    // — is ALWAYS contiguous with no branch. And the polyphase branches are
    // transposed into `hp_` so each one is contiguous.
    //
    // The ORDER of operations does not change, only the addressing. That
    // yields a strong control: the output must be BIT-IDENTICAL to before.
    // If it is not, the refactor changed something it should not have.
    void upsample(double x, osreal* fuera)
    {
        if (n_ == 1) { fuera[0] = osreal(x); return; }
        iu_ = (iu_ == 0) ? tf_ - 1 : iu_ - 1;
        zu_[iu_] = osreal(x);  zu_[iu_ + tf_] = osreal(x);
        const osreal* z = &zu_[iu_];
        for (int f = 0; f < n_; ++f) {
            const osreal* hf = &hp_[size_t(f) * tf_];
#if NLSC_OS_UP_ACC_LANES > 1
            // Row 30 — the SAME split as `bajar()`, for the same reason and
            // found the same way: after that one landed, `nls_os.h:343` became
            // the hottest LINE of the file, 2,27 % of the DSP.
            //
            // AND WHY IT WAS NOT OBVIOUS. `subir()` was the CHEAP leg, so
            // nobody looked: it has `n_` independent branches, which buys ILP
            // BETWEEN branches. But INSIDE one branch this is still a serial
            // reduction of `tf_` terms, exactly what made `bajar()` stall. The
            // branches hid the defect instead of curing it.
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
            fuera[f] = a * osreal(n_);
        }
    }

    // N oversampled samples -> one base sample.
    //
    // THE ACCUMULATOR IS SPLIT INTO `NLSC_OS_ACC_LANES` PARTIAL SUMS, and
    // the reason is a measurement, not a style preference. Profiled:
    // this leg and `subir()` do the SAME 32 multiply-adds per base sample, and
    // this one costs 5,01 % of the DSP against 2,28 % — 2,19× the clock for
    // 1,21× the instructions. It does not do more work: IT STALLS.
    //
    // The disassembly of the clone that runs (`isa_v3`, AVX2) said why. The old
    // single-accumulator loop compiled to `vmulpd` on `ymm` — four products at a
    // time, so it did vectorise — feeding FOUR CHAINED SCALAR `vaddsd` on the
    // same `xmm0`, plus `vunpckhpd`/`vextractf128` to unpack the vector. No FMA
    // is emitted at all, because a serial reduction cannot be fused. That is
    // ~4 cycles of latency per add, four chained per iteration, eight
    // iterations: pure dependency, not arithmetic.
    //
    // Which is why `D3`'s note that these loops "already vectorise (AVX, 32
    // bytes)" was TRUE OF THE MULTIPLY AND FALSE OF THE ACCUMULATION, and why
    // `-fopt-info-vec-optimized` could not tell: it answers yes/no about a loop
    // that does two different things.
    //
    // With one partial sum per lane the adds become independent, the compiler
    // keeps them in a vector register, and the horizontal sum is paid ONCE at
    // the end instead of once per iteration. It is the shape `subir()` already
    // has by construction — four branches of eight — which is exactly why
    // `subir()` is the cheap leg without being any cleverer.
    //
    // NOT BIT-IDENTICAL: it reassociates a 32-term float sum, so it is signed
    // with a sample-by-sample audio diff against the shipped build, never with a
    // null. Not to be confused with `A3` (`-fassociative-math`), which
    // measured WORSE: that was the global flag over the whole engine, letting
    // the compiler reassociate anything it liked. This is one accumulator, local
    // and deterministic.
    //
    // The tail loop is not decoration: `L = tf_ * n_` is a multiple of four
    // for every factor the product offers, but this header is shared by fifteen
    // harnesses that build it with other prototypes.
    double downsample(const osreal* dentro)
    {
        if (n_ == 1) return double(dentro[0]);
        const int L = int(h_.size());
        for (int f = 0; f < n_; ++f) {
            id_ = (id_ == 0) ? L - 1 : id_ - 1;
            zd_[id_] = dentro[f];  zd_[id_ + L] = dentro[f];
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
    int n_ = 1, tf_ = 16, iu_ = 0, id_ = 0;
    // `h_` stays DOUBLE: it is the DESIGN prototype (and the source of
    // `hp_`), and design is offline. What runs per sample are `hp_` (up) and
    // `hd_` (down), which do use `osreal`.
    std::vector<double> h_;
    std::vector<osreal> hp_, hd_, zu_, zd_;
};


// THE CLAMP WARNING LIVES HERE, NOT INSIDE `prepare()`.
//
// `prepare()` runs in the PRODUCT's .so, and no I/O may live there: the gate
// `tools/sin_es_en_rt.sh` reads the artefact's undefined symbols and goes
// red on merely SEEING `fprintf`, without asking which thread it runs on.
// Rightly so: fifteen harnesses share this header with the plugin.
//
// This function is `inline` and UNUSED IT IS NOT EMITTED, so the plugin —
// which never calls it — drags no `fprintf`. Measured with
// `nm -D --undefined-only` on the built .so, not assumed.
//
// Every harness that takes the factor from the command line MUST call it.
// A clamp nobody publishes is a measurement at a different factor than
// believed — the silent failure this mechanism exists to prevent.
inline bool warn_if_clamped(const Oversampler& ovs, int pedido, const char* quien)
{
    if (!ovs.clamped()) return false;
    std::fprintf(stderr,
        "⛔ %s: requested oversampling factor %d exceeds the cap %d and has been "
        "CLAMPED.\n   Whatever comes out is NOT at the requested factor.\n",
        quien, pedido, Oversampler::kMaxOS);
    return true;
}

} // namespace nlsc
