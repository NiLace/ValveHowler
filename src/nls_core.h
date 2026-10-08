// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// nls_core.h — all of the plugin's DSP, separated from the LV2 scaffolding.
//
// This core compiles twice — baseline and avx2/fma — in two translation units
// and two namespaces. Which one runs is chosen once, at instantiate(), and is
// reached through one virtual call per block. AVX2/FMA3 require a Haswell
// (2013) or later CPU; an earlier one would fault on them, so the baseline
// copy is always present.
//
// When compiled inside an ISA namespace, the system headers must already be
// included outside it, so the includes here are project-only: the system ones
// live in the .cpp that includes this.
#pragma once

// The ABI goes first and un-namespaced: `Plugin` inherits from
// `::nlsc::ICore`, which must resolve to the outer one.
#include "nls_iface.h"

#include "nls_cascada.h"
// The plugin runs the cascade only; the DK engine (`nls_dk.h`) is not part
// of it and is not included.
#include "nls_os.h"
// Variant changes fade through the dry path (1, the default); 0 applies them
// instantly.
#ifndef NLSC_FADE_MODE
#  define NLSC_FADE_MODE 1
#endif
#include "nls_variantes.h"
#include "nls_tone_law.h"

// `NLSC_TONE_W` selects the TONE knob's law. Its default lives here, where it
// is used: `nls_valvehowler.cpp` (the LV2 scaffolding) does not include this
// header, so a default placed there would never reach the unit that compiles
// the engine, and the build would silently use a linear law.
#ifndef NLSC_TONE_W
#  define NLSC_TONE_W 1
#endif

namespace nlsc {

// The ABI (`PortIndex`, `ICore`) lives in the real `::nlsc`, outside any ISA
// namespace; this keeps `PORT_DRIVE` and friends resolving when the header
// compiles inside one. Compiled un-namespaced it is a harmless self-import.
using namespace ::nlsc;

// Internal state precision. The LV2 ports are float by API; the engine state
// is double.
using Real = double;


// The circuit variants live in `nls_variantes.h`, the single source shared
// by the DSP and the GUI (two distinct .so files).

// The oversampling factor is fixed at 4x; there is no port for it. At 2x the
// model misses its fidelity target over most of the knob grid and the alias
// is audible on pinch harmonics; 16x costs about half a core in the worst
// block for an alias figure 8x already reaches.
inline constexpr int kOsFactor = 4;

// Implements `::nlsc::ICore` — see `nls_iface.h`. The leading `::` is
// required: inside the ISA namespace, `nlsc::ICore` would resolve to
// `isa_v3::nlsc::ICore`, which does not exist.
struct Plugin : ::nlsc::ICore {
    // Port pointers. The host sets them with connect_port() before run().
    const float* in   = nullptr;
    float*       out  = nullptr;
    float*       latency_out = nullptr;
    const float* ctl[PORT_COUNT] = { nullptr };

    double sample_rate = 48000.0;

    // Prepared in instantiate() (it designs its filter there), reset from
    // run() on a mode change.
    Oversampler os_;
    // The cascade's constructor allocates nothing (fixed arrays throughout), so
    // the object lives inside the plugin and nothing is allocated on the audio
    // thread.
    Cascade4    casc_;

    // Knob state already applied to the engine.
    double drive_ = -1.0, tone_ = -1.0, lvl_ = -1.0;   // −1 forces the first prepare
    int    idx_var_ = 0;             // variant applied to the engine (0 = 808)
    bool   pending_ = true;
    // A MODE change is waiting for the fade to bottom out.
    bool   pending_mode_ = false;
    // A variant change undone before the fade reaches dry is cancelled, not
    // applied. 0 re-applies the same circuit instead.
#ifndef NLSC_UNDO_CANCELS_MODE
#define NLSC_UNDO_CANCELS_MODE 1
#endif

    // ─────────────────────────────────────────────────────────────────────────
    // The footswitch (`lv2:enabled`), and the two things it must do right.
    //
    // 1. Latency does not vanish when bypassed. The plugin declares
    // `NLSC_OS_TAPS − 1` samples and the host always compensates them: it does
    // not re-query on `enabled` changes. If bypass passed `in[i]` through raw,
    // the signal would come out that many samples early relative to what the
    // host expects, and a parallel mix with the dry signal would comb-filter.
    // So the dry leaves delayed by the same amount, through this ring.
    //
    // 2. No hard switching. Jumping from wet to dry is an amplitude step, and
    // the engine is biased (its rest output is not 0), so the step is audible
    // even without signal. It fades over `kFadeSeconds`, and while fading the
    // engine keeps running — the wet signal being faded from has to exist.
    //
    // When the fade lands in bypass, the engine stops running entirely, which
    // is the footswitch's CPU saving. Its state stays frozen at the last point,
    // like a real pedal with its signal cut; on return, the fade in covers the
    // re-engagement transient.
    //
    // The ring size is a power of two and is bounded against the real latency:
    // the oversampler's taps are a build setting, so the declared latency is not
    // a product constant.
    static constexpr int    kDryRing = 128;         // > any declared latency
    static constexpr int    kMask    = kDryRing - 1;
    // In seconds, not samples, so the fade lasts the same at every sample rate.
    static constexpr double kFadeSeconds = 512.0 / 48000.0;   // ~10.7 ms at any rate
    // The bound is enforced at compile time. `latency()` equals `taps − 1`, and
    // the taps are a build setting: a ring shorter than the latency would wrap
    // and misalign the dry with no error. The `.ttl` declares the latency port's
    // `lv2:maximum 64`, so a build past it would publish a value its own
    // manifest calls out of range.
    static_assert(NLSC_OS_TAPS - 1 <= 64, "latency past the .ttl's lv2:maximum 64");
    static_assert(NLSC_OS_TAPS <= kDryRing,
                  "the dry ring does not cover the oversampler's latency: "
                  "raise kDryRing to the next power of two");

    float  dry_[kDryRing] = { 0.0f };
    int    dry_w_ = 0;
    double mix_ = 1.0;            // 1 = wet, 0 = dry. Starts active.
    // The first block after `activate()` does not fade: it jumps to whatever
    // the port asks, so an instance created in bypass does not open with a fade
    // nobody requested.
    bool   startup_ = true;
    // ─────────────────────────────────────────────────────────────────────────


    // Port-pointer routing, next to the code that uses the ports.
    void connect_port(uint32_t port, void* data) override
    {
        switch (port) {
        case ::nlsc::PORT_IN:      in  = static_cast<const float*>(data); break;
        case ::nlsc::PORT_OUT:     out = static_cast<float*>(data);       break;
        case ::nlsc::PORT_LATENCY: latency_out = static_cast<float*>(data); break;
        default:
            if (port < ::nlsc::PORT_COUNT)
                ctl[port] = static_cast<const float*>(data);
            break;
        }
    }

    void prepare(double sr) override
    {
        sample_rate = sr;
        knob_a_ = 1.0 - std::exp(-double(kKnobStep) / (kKnobTau * sr));
        os_.prepare(kOsFactor);
        // Deferred cache filling is requested here and only here: this object
        // reaches `Cascade4::prepare()` (through `apply_knobs()`) from inside
        // `process()`, i.e. the audio thread. Other users of the cascade do not
        // request it and build the table at once.
        casc_.defer_fill(true);
        pending_ = true;
        reset();
    }

    void reset() override
    {
        os_.reset();
        pending_ = true;      // the engine re-prepares on the first run()
        knob_countdown_ = 0;  // the glide's schedule restarts with the instance
        std::memset(dry_, 0, sizeof dry_);
        dry_w_  = 0;
        startup_ = true;       // the first block does NOT fade: jump to the target
    }

    // The DRIVE pot's law. The port is a knob position (0..1); the netlist
    // wants a resistance fraction. The TS GAIN pot is a 500K logarithmic
    // (audio, "A") taper; with a linear law the useful travel would bunch into
    // the first quarter of the knob.
    //
    // Shape: the Bourns PDB24 datasheet (24 mm rotary pot) labels its taper
    // curves by mid-travel percentage, (05A):A1 … (30A):A6; the audio standard
    // is A3 = 15 %. Real audio pots are made with two tracks overlapping at the
    // centre, a law of two straight segments, which is what the chart follows:
    //
    //     x ≤ ½ :  y = 2·m·x                    (gentle segment)
    //     x > ½ :  y = m + 2·(1−m)·(x−½)        (fast segment)
    //
    // Midpoint m = 0.161: it reproduces the published THD of a real TS with the
    // drive at the mechanical midpoint (Open Heart, 4.18 %), and lies within a
    // pot's typical ±20 % taper tolerance of the A3 spec.
    static constexpr double kTaperMid = 0.161;   // mid-travel fraction

    static double taper_log(double x)
    {
        if (x <= 0.0) return 0.0;
        if (x >= 1.0) return 1.0;
        return (x <= 0.5) ? 2.0 * kTaperMid * x
                          : kTaperMid + 2.0 * (1.0 - kTaperMid) * (x - 0.5);
    }

    // The TONE pot's "W" taper. The TS tone pot is not linear: ElectroSmash
    // calls it "G", the GGG schematic prints `20KG`, pedalparts `20KW`, and the
    // manufacturer's service manual lists a 20KG part.
    //
    // Shape: a symmetric S with the smallest slope at the centre — fast at
    // first, flat in the middle, fast again ("more control in the middle of the
    // sweep"). Source: a taper-curve chart whose curve 5 is labelled "G", "W" or
    // "S" taper. Fit: `0.5 + 0.5·sign(2x−1)·(2x−1)²`, a symmetric parabola:
    //
    //     x    chart     parabola
    //    0.1    0.185      0.180
    //    0.2    0.315      0.320
    //    0.3    0.400      0.420
    //    0.5    0.500      0.500   <- the only explicitly marked point
    //
    // The chart values are symmetrised (averaging x and 1−x); the upper-half
    // misfit (0.04-0.07) is within the imprecision of reading the chart.
    //
    // Limitations: no published table backs the numeric curve; what is
    // established is the shape, not the percentages. The curve passes through
    // (0.5, 0.5) by construction, so it does not move the midpoint.
    //
    // `NLSC_TONE_W=0` selects a linear law instead.
    static double taper_w(double x)
    {
        if (x <= 0.0) return 0.0;
        if (x >= 1.0) return 1.0;
        // Symmetric parabola: fits the chart within the error of reading it.
        const double u = 2.0 * x - 1.0;
        return 0.5 + 0.5 * (u >= 0.0 ? u * u : -u * u);
    }

    // `ToneLaw::Even` — the generated table (`nls_tone_law.h`), read with
    // linear interpolation. 65 points over the travel, so each segment spans
    // 0.0156 of knob and the interpolation error stays far below the table's own
    // 0.069 dB deviation from its ideal line.
    //
    // It is not the default and not a model of the real pot: the pot's G taper
    // stays on the faithful rows. It exists because the circuit concentrates the
    // tone's whole effect in the last quarter of the pot, and no catalogue pot
    // undoes that — the law does. Evaluated in `apply_knobs`, i.e. once per
    // glide step (every `kKnobStep` samples, 8 by default) while a knob moves;
    // no per-sample cost.
    static double taper_even(double x)
    {
        if (x <= 0.0) return tone_law::kT[0];
        if (x >= 1.0) return tone_law::kT[tone_law::kN - 1];
        const double u = x * double(tone_law::kN - 1);
        const int    i = int(u);
        const double f = u - double(i);
        return tone_law::kT[i] + (tone_law::kT[i + 1] - tone_law::kT[i]) * f;
    }

    // The inverse of `taper_w`, for the knob glide on the `ToneLaw::Even` rows.
    static double taper_w_inv(double y)
    {
        if (y <= 0.0) return 0.0;
        if (y >= 1.0) return 1.0;
        const double v = 2.0 * y - 1.0;
        return 0.5 + 0.5 * (v >= 0.0 ? std::sqrt(v) : -std::sqrt(-v));
    }

    // Control ports are sanitised before touching the netlist. A host is not
    // obliged to clamp: a console host accepts any typed value, a saved session
    // can carry a corrupt one, and a host that interpolates parameter changes
    // can overshoot the range. With `tone = 5` the netlist would hold a
    // resistance of −80 kΩ. Costs three comparisons per block.
    static double sanitise(double x, double fallback)
    {
        if (!std::isfinite(x)) return fallback;
        return (x < 0.0) ? 0.0 : (x > 1.0 ? 1.0 : x);
    }

    // Rebuilds the engine's matrices with the current knobs. The pots are
    // netlist resistors (`RGAIN`, `RTONE`, `RLVL`), so they enter the system
    // matrix rather than being a gain applied at the end. The rebuild allocates
    // nothing, so it may run on the audio thread. It runs once per glide step
    // (every `kKnobStep` samples) while a knob moves, and once when the engine
    // is (re)initialised: first block, rate change, or a variant change landing
    // under the fade.
    //
    // Receives the knobs already sanitised (done where the ports are read, see
    // `process`). What is stored in `drive_/tone_/lvl_` is what the next block
    // compares against; storing a value that differs from the one compared would
    // make a port pinned out of range read as "changed" every block and
    // re-prepare continuously.
    //
    // `reinit` distinguishes the two reasons for arriving here:
    //   true  — startup, rate change, or a variant change: the stored state is
    //           at the old rate or describes the other circuit, and is invalid.
    //           A variant change enters under the fade (see `process()`), so the
    //           reinit happens with the output on the dry path.
    //   false — only a potentiometer moved: the matrices re-tune and the
    //           circuit keeps the charge it had, like the real pedal.
    void apply_knobs(double drive, double tone, double level,
                     int idx_var, bool reinit, double tone_param = -1.0)
    {
        // The pots' physical parameters. The variant is a circuit (component
        // values), passed to the cascade by index: it re-tunes the stages and
        // keeps the capacitors' state.
        struct { double gain, tone, lvl; } nl;
        nl.gain = taper_log(drive);
#if !NLSC_TONE_W
        nl.tone = tone;            // linear law
#else
        // The row selects the law: `ToneLaw::Pot` is the real pot's taper and is
        // used on every faithful row; `ToneLaw::Even` is the derived law, offered on
        // its own row. The two variants above (linear, and the quadratic
        // verification build) deliberately ignore the row, so each forces one law.
        nl.tone = (kVariants[idx_var].tone_law == ToneLaw::Even)
                      ? taper_even(tone)
                      : taper_w(tone);   // W taper: symmetric log/antilog S
#endif
        // `tone_param` >= 0 is the pot's parameter already computed by the
        // knob glide mid-way on an Even row (see `tone_glide`).
        if (tone_param >= 0.0) nl.tone = tone_param;
        // The LEVEL is inverted: in the netlist `lvl=0` is level at maximum
        // (0.856 Vpp) and `lvl=1` leaves the output at 8.8 nV, so the knob position
        // must be inverted.
        // The law is linear, per the manufacturer's service manual's parts list:
        //
        //     VARIABLE RESISTOR  16φ 500KA   PMXB155R504ASC   <- DISTORTION
        //     VARIABLE RESISTOR  16φ  20KG   PMXB155R203GSC   <- TONE
        //     VARIABLE RESISTOR  16φ 100KB   PMXB155R104BSC   <- LEVEL
        //
        // and its schematic repeats "100KB" next to the LEVEL knob. The taper letter
        // sits inside the part number, and the A and the G on the other two pots
        // confirm the reading of the B: B = linear.
        //
        // The law applies to the position and the inversion afterwards: the
        // divider's attenuation is (1 − lvl), so `lvl = 1 − taper(position)`.
        nl.lvl = 1.0 - level;      // linear: 100K B pot (service manual)
        const double fs_int = sample_rate * double(kOsFactor);
        // The cascade is prepared with the physical parameters (`nl`), not the port
        // positions, so the knob laws are defined in one place. Its fixed filters
        // (`nls_fijos_s.h`) discretise here at the real rate, so any host rate is
        // supported. The variant reaches the cascade as a circuit
        // (`kVariants[idx_var].circuit`), not as a row index: the cascade's
        // coefficient banks are indexed by circuit. Per-sample cost is zero —
        // everything resolves in `prepare()`.
        // The gesture is declared before re-tuning: `reinit` tells a set-up or mode
        // change apart from a knob move, which stage 2 cannot know on its own.
        casc_.table_gesture(!reinit);
        casc_.prepare(fs_int, nl.gain, nl.tone, nl.lvl,
                      kVariants[idx_var].circuit, reinit);

        drive_ = drive; tone_ = tone; lvl_ = level;
        idx_var_ = idx_var;
        pending_ = false;
    }

    // The dispatch, in one place: the three `process` paths (wet, fading and
    // bypass) call here, so they cannot run different engines.
    inline double motor(double x)
    {
        return casc_.process_modelled(x).out;
    }

    static double read_port(const float* p, double def)
    {
        return p ? double(*p) : def;
    }

    static int variant_index(double v)
    {
        if (!std::isfinite(v)) return 0;
        const int i = int(std::lround(v));
        return (i < 0) ? 0 : (i >= kNumVariants ? kNumVariants - 1 : i);
    }

    // Real-time contract: no malloc, no exceptions, no locks.
    void process(uint32_t n_samples) override
    {
        if (!in || !out) return;

        // --- knobs, once per block ---------------------------------------
        // Read before the audio, not mid-block, which keeps the output
        // bit-identical under any block slicing. Sanitising happens here, at the
        // read point, so what is compared and what is stored are the same value
        // (see `apply_knobs`).
        const double drive  = sanitise(read_port(ctl[PORT_DRIVE], 0.5), 0.5);
        const double tone   = sanitise(read_port(ctl[PORT_TONE],  0.5), 0.5);
        const double level  = sanitise(read_port(ctl[PORT_LEVEL], 0.5), 0.5);
        // The fallback is for an unconnected port, and must equal the
        // default the `.ttl` declares: otherwise a host that skips the port
        // runs a different product than the manifest says.
        const int    idx_var = variant_index(read_port(ctl[PORT_VARIANT], 0.0));

        // Variant changes enter under the fade. A knob re-tunes live and keeps the
        // state; a variant does not: re-tuned live, the bias network would
        // re-settle on its own (`C11` 47 uF, tau ~235 ms) for about two seconds,
        // producing neither variant, and the switch would click. So the output
        // fades to the dry path, the variant is applied with `reinit` — which
        // re-seeds the new variant's rest point — and the output fades back to wet.
        // A knob does not fade: it glides (a one-pole, `knob_tick`) with the current
        // variant, and lands exactly on the port value.
        bool mode_change = (idx_var != idx_var_);
#if !NLSC_FADE_MODE
        // Verification variant, never in the release build: applies the variant
        // instantly, without the fade.
        if (!pending_ && mode_change) {
            apply_knobs(drive, tone, level, idx_var, true);
            knob_land(drive, tone, level);
            os_.reset();
            // Clearing the flag keeps the pending-mode path below from adding a fade on
            // top of the instant change.
            mode_change = false;
        }
#endif
        if (pending_) {
            // The first `prepare` does not fade: there is nothing to leave.
            apply_knobs(drive, tone, level, idx_var, true);
            knob_land(drive, tone, level);
            os_.reset();
            pending_mode_ = false;
        } else {
            // The variant lands when the ramp bottoms out, and at the block boundary:
            // `apply_knobs` rebuilds matrices, which does not belong inside the sample
            // loop.
            if (NLSC_UNDO_CANCELS_MODE && pending_mode_ && !mode_change) {
                // The change was undone before the ramp reached dry: nothing to apply, so
                // the same circuit is not re-initialised.
                pending_mode_ = false;     // the ramp turns back up
            } else if (pending_mode_ && mix_ <= 0.0) {
                apply_knobs(drive, tone, level, idx_var, true);
                knob_land(drive, tone, level);
                os_.reset();
                pending_mode_ = false;
            } else if (mode_change) {
                pending_mode_ = true;      // start the ramp down; apply on arrival
            }
        }
        knob_target_[0] = drive;  knob_target_[1] = tone_glide(tone, idx_var_);
        knob_target_[2] = level;  tone_port_ = tone;

        // Stage 4's cache is filled per sample inside `SubQ2`, which keeps the
        // output independent of the host's block size. This per-block call exists
        // only when `NLSC_E4_FILL_PER_SAMPLE=0`.
#if !NLSC_E4_FILL_PER_SAMPLE
        casc_.advance_cache();
#endif

        // The host wants the latency in base-rate samples: the oversampler's
        // taps − 1. Published every block because the taps are
        // compile-configurable, not because it moves at run time.
        if (latency_out) *latency_out = float(os_.latency());

        const int N = kOsFactor;
        // The buffer uses the oversampler's precision (`osreal`: double, or float
        // with `NLSC_OS_F32=1`), not the engine's. With float it is the
        // mixed-precision boundary: the FIR works in float and the solver stays
        // in double, with one conversion per oversampled sample.
        osreal up[Oversampler::kMaxOS];                       // the max factor; no allocation

        // --- the footswitch, decided once per block ----------------------
        //
        // The dry's delay must equal the latency just published: the taps are a
        // build setting, and a mismatched delay would comb-filter.
        const int lat = os_.latency();
        const bool active  = read_port(ctl[PORT_ENABLED], 1.0) >= 0.5;
        // A pending mode change ramps down like a bypass: same path, which
        // is why there is no second fade that could diverge.
        const double target = (active && !pending_mode_) ? 1.0 : 0.0;
        if (startup_) { mix_ = target; startup_ = false; }
        const double step = 1.0 / (kFadeSeconds * sample_rate);

        // The block is split into spans that end on absolute multiples of
        // kKnobStep, so the glide steps at the same samples whatever the host's
        // block size and the output stays bit-identical across block sizes.
        for (uint32_t done = 0; done < n_samples; ) {
            if (knob_countdown_ == 0) {
                knob_tick();
                knob_countdown_ = kKnobStep;
            }
            const uint32_t len = std::min<uint32_t>(n_samples - done, knob_countdown_);
            run_span(in + done, out + done, len, N, up, lat, target, step);
            done += len;
            knob_countdown_ -= len;
        }
    }

    // One span of the block: the footswitch's three regimes, per sample.
    void run_span(const float* src, float* dst, uint32_t n_samples, int N,
                  osreal* up, int lat, double target, double step)
    {
        if (mix_ == target && target == 1.0) {
            // ══ PURE WET PATH ════════════════════════════════════════════
            // Kept separate from the general `mix*y + (1-mix)*dry` with mix = 1, so
            // the wet output is exact by construction rather than by an argument
            // about rounding. The only addition is pushing the dry into the ring,
            // which does not touch the output.
            for (uint32_t i = 0; i < n_samples; ++i) {
                // A non-finite input sample is replaced by 0: fed to the engine it would
                // make the capacitors' state non-finite for good, and the output would
                // stay muted until the next re-prepare. Costs one comparison per
                // base-rate sample.
                const float xin = src[i];         // read before writing: in-place
                const double x = std::isfinite(xin) ? double(xin) : 0.0;
                dry_[dry_w_] = float(x);
                dry_w_ = (dry_w_ + 1) & kMask;
                os_.upsample(x, up);
                for (int k = 0; k < N; ++k) up[k] = osreal(motor(double(up[k])));
                const double y = os_.downsample(up);
                // A non-finite output would propagate down the host's chain. The engine
                // keeps its previous solution if Newton fails to converge; this is the
                // last guard.
                dst[i] = std::isfinite(y) ? float(y) : 0.0f;
            }
        } else if (mix_ == target) {
            // ══ SETTLED BYPASS ═══════════════════════════════════════════
            // The engine does NOT run: this is the footswitch's CPU saving.
            // Its state stays frozen at the last point, like a real pedal
            // with the signal cut; the fade-in covers the re-engagement
            // transient.
            for (uint32_t i = 0; i < n_samples; ++i) {
                const float xin = src[i];
                const double x = std::isfinite(xin) ? double(xin) : 0.0;
                dry_[dry_w_] = float(x);
                dry_w_ = (dry_w_ + 1) & kMask;
                dst[i] = dry_[(dry_w_ + kDryRing - 1 - lat) & kMask];
            }
        } else {
            // ══ FADING ═══════════════════════════════════════════════════
            // The engine keeps running while fading, both directions: fading
            // towards the dry needs the wet being left behind.
            for (uint32_t i = 0; i < n_samples; ++i) {
                const float xin = src[i];
                const double x = std::isfinite(xin) ? double(xin) : 0.0;
                dry_[dry_w_] = float(x);
                dry_w_ = (dry_w_ + 1) & kMask;
                os_.upsample(x, up);
                for (int k = 0; k < N; ++k) up[k] = osreal(motor(double(up[k])));
                const double y = os_.downsample(up);
                const double wet = std::isfinite(y) ? y : 0.0;
                const double dry_sample = double(dry_[(dry_w_ + kDryRing - 1 - lat) & kMask]);
                dst[i] = float(mix_ * wet + (1.0 - mix_) * dry_sample);
                mix_ += (target > mix_) ? step : -step;
                if (mix_ > 1.0) mix_ = 1.0;
                if (mix_ < 0.0) mix_ = 0.0;
            }
        }

    }

    // The knob glide. A knob is a resistor inside the circuit, so moving it
    // re-tunes the matrices, and a step change would click. Each knob glides
    // toward its port as a one-pole of kKnobTau, applied every kKnobStep
    // samples — each application is one `apply_knobs()` (~3.3 us), and only
    // while a knob moves — and lands exactly on the port once within
    // kKnobSnap, so a settled knob gives the same engine state bit for bit.
    // On a reinit (first block, `activate`, a variant change under the fade)
    // the knobs jump instead: there is nothing to glide from.
    // The glide runs in a logit domain (NLSC_KNOB_LOGIT) because the circuit is
    // most sensitive near the ends of a pot's travel. The constants trade the
    // click against CPU while a knob moves: a re-tune every 8 samples costs
    // about half of one every 4, and a 70 ms pole keeps the worst step (a
    // whole-travel jump at a low level) within the sharpness of a 5 ms one-pole:
    // 95 % of a full travel in ~0.09 s, exact landing after 0.4-0.7 s. Still: free.
#ifndef NLSC_KNOB_STEP
#define NLSC_KNOB_STEP 8
#endif
#ifndef NLSC_KNOB_TAU
#define NLSC_KNOB_TAU 0.070
#endif
    static constexpr uint32_t kKnobStep = NLSC_KNOB_STEP;  // samples between applications
    static constexpr double   kKnobTau  = NLSC_KNOB_TAU;   // s, one-pole glide
    static constexpr double   kKnobSnap = 1.0e-4;   // land exactly within this
#ifndef NLSC_KNOB_LOGIT
#define NLSC_KNOB_LOGIT 1
#endif
#ifndef NLSC_KNOB_P0
#define NLSC_KNOB_P0 0.002
#endif
    static constexpr double   kP0 = NLSC_KNOB_P0;   // the logit's end scale (see knob_tick)
    // On the `ToneLaw::Even` rows the tone glides in the W law's domain. The
    // glide runs on the knob position and the row's law comes after it; the
    // Even law puts 63 % of the pot between knob 0.266 and 0.281, so a glide in
    // position would cross it in a few steps. On an Even row the glide state is
    // taper_w_inv(taper_even(knob)), so the pot follows the same physical path
    // as on a W row. 0 glides in position.
#ifndef NLSC_KNOB_EVEN_W_DOMAIN
#define NLSC_KNOB_EVEN_W_DOMAIN 1
#endif
    static bool tone_even(int idx_var)
    {
#if NLSC_KNOB_EVEN_W_DOMAIN && NLSC_TONE_W
        return kVariants[idx_var].tone_law == ToneLaw::Even;
#else
        (void)idx_var;
        return false;
#endif
    }
    static double tone_glide(double tone, int idx_var)
    {
        return tone_even(idx_var) ? taper_w_inv(taper_even(tone)) : tone;
    }
    double   tone_port_      = 0.5;                  // the tone port, for landing
    double   knob_a_         = 0.0;                  // the glide's pole, set in prepare()
    double   knob_now_[3]    = { 0.5, 0.5, 0.5 };   // drive, tone, level as applied
    double   knob_target_[3] = { 0.5, 0.5, 0.5 };   // this block's ports
    uint32_t knob_countdown_ = 0;

    void knob_land(double drive, double tone, double level)
    {
        knob_now_[0] = drive;  knob_now_[1] = tone_glide(tone, idx_var_);  knob_now_[2] = level;
        tone_port_ = tone;
    }

    void knob_tick()
    {
        if (knob_now_[0] == knob_target_[0] && knob_now_[1] == knob_target_[1]
            && knob_now_[2] == knob_target_[2]) return;
        const double a = knob_a_;
        for (int k = 0; k < 3; ++k) {
            const double d = knob_target_[k] - knob_now_[k];
            if (std::fabs(d) <= kKnobSnap) { knob_now_[k] = knob_target_[k]; continue; }
#if NLSC_KNOB_LOGIT
            // The glide runs in z = ln(p0 + p) − ln(p0 + 1 − p), not in p: at an end
            // of a pot one leg is ~0 ohm, and the first 1.6 % of travel takes it to
            // hundreds of ohms — the tone's corner falls from infinity to kHz in one
            // step. In z the steps near either end are tiny and grow towards the
            // middle, where the circuit is less sensitive.
            const double z  = std::log(kP0 + knob_now_[k]) - std::log(kP0 + 1.0 - knob_now_[k]);
            const double zt = std::log(kP0 + knob_target_[k]) - std::log(kP0 + 1.0 - knob_target_[k]);
            const double zn = z + a * (zt - z);
            const double e  = std::exp(zn);
            knob_now_[k] = (e * (kP0 + 1.0) - kP0) / (1.0 + e);
#else
            knob_now_[k] += a * d;
#endif
        }
        if (tone_even(idx_var_)) {
            // Landed: the port through the row's own law, bit for bit the
            // settled engine. Mid-glide: the pot parameter of the glide state.
            const bool landed = (knob_now_[1] == knob_target_[1]);
            apply_knobs(knob_now_[0], tone_port_, knob_now_[2], idx_var_, false,
                        landed ? -1.0 : taper_w(knob_now_[1]));
        } else {
            apply_knobs(knob_now_[0], knob_now_[1], knob_now_[2], idx_var_, false);
        }
    }
};

} // namespace nlsc
