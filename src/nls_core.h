// nls_core.h — ALL of the plugin's DSP, separated from the LV2 scaffolding.
//
// WHY IT EXISTS: this core compiles TWICE — baseline and avx2/fma — in two
// translation units and two namespaces, and which one runs is chosen ONCE PER
// BLOCK. Measured: `-mavx2 -mfma` buys −16,84 % of clock for −126,7 dB, but
// AVX2/FMA3 are Haswell (2013) and an earlier CPU SIGILLs.
//
// Doing it with a per-SAMPLE attribute was measured and came out NEGATIVE
// (cascade +4,09 % slower): at 192 000 calls per second the IFUNC costs more
// than the FMA buys, mostly because it blocks inlining.
//
// WHEN COMPILED INSIDE AN ISA NAMESPACE, the SYSTEM headers must already be
// included OUTSIDE. That is why the includes here are project-only: the
// system ones live in the .cpp that includes this.
#pragma once

// The ABI goes FIRST and un-namespaced: `Plugin` inherits from
// `::nlsc::ICore`, which must resolve to the OUTER one.
#include "nls_iface.h"

#include "nls_cascada.h"
// `nls_dk.h` IS NOT INCLUDED HERE. The DK is the bench's arbiter, not a
// mode of the plugin: every harness that needs it includes it directly.
// Pulling it in here compiled 74,5 kB of unreachable per-sample code into
// the shipped `.so` AND kept `prepare()` reachable, which is worse than
// dead: see `apply_knobs`.
#include "nls_os.h"
// THE MODE-CHANGE FADE. At 0 the mode applies instantly — the old
// behaviour — and that is the MUTANT `make conmutacion` calibrates against.
// The product ships at 1.
#ifndef NLSC_FADE_MODE
#  define NLSC_FADE_MODE 1
#endif
#include "nls_variantes.h"
#include "nls_tone_law.h"

// THIS MACRO'S DEFAULT LIVES HERE, WHERE IT IS USED — and it sat
// THIRTEEN DAYS in the wrong file.
//
// `NLSC_TONE_W` governs the TONE knob's law, and its `#define` lived in
// `nls_valvehowler.cpp`. That file is the LV2 SCAFFOLDING: since the ISA dispatch
// moved the engine into its own translation unit (`nls_isa_tu.cpp`), it **does not include `nls_core.h`**. The macro never
// reached the unit that compiles the engine => `#if !NLSC_TONE_W` took the
// LINEAR branch and the product shipped WITHOUT the potentiometer's taper.
//
// And it raised neither an error nor a warning: the binary compiles, runs
// and sounds. All that changes is the knob's LAW, which no fidelity yardstick
// looks at: a null runs on the PHYSICAL parameter and the port carries a
// knob POSITION, so the law between the two sits outside every null.
//
// WHY THE GATE DID NOT SEE IT, which is the expensive part: `make mandos`
// judges the tone knob at positions **0 / 0,5 / 1**, and `taper_w` is the
// IDENTITY at all three. The gate could not tell linear from W even when
// healthy — and its MUTANT is compiled with `-DNLSC_MUTANT_TONE` on the
// command line, which DOES reach this unit, so the mutant failed and the gate
// looked calibrated. A control that goes red on a branch other than the one
// that ships calibrates nothing.
// => New gate: `tools/tone_branch.sh`, which PREPROCESSES this unit and
// demands that the live branch be the declared one.
#ifndef NLSC_TONE_W
#  define NLSC_TONE_W 1
#endif

namespace nlsc {

// The ABI (`PortIndex`, `ICore`) lives in the REAL `::nlsc`, outside any
// ISA namespace. When this header compiles inside one, this is what keeps
// `PORT_DRIVE` and friends resolving.
// Compiled un-namespaced it is a self-import, which is harmless.
using namespace ::nlsc;

// Internal state precision. Compile-parametrised: dropping to float is the
// SECOND rung of the loosening ladder, and only once the numerically robust
// topologies are in place. The LV2 ports are float by API, not by choice.
using Real = double;


// The circuit variants live in `nls_variantes.h`, the SINGLE source shared
// by the DSP and the GUI (two distinct .so files: see that file's header and
// why duplicating it would raise no error).

// The port's four factors. The port carries the REAL FACTOR (2/4/8/16), not
// an index — so a value stored in a session keeps meaning the same if another
// factor is ever added.
inline constexpr int kFactores[4] = { 2, 4, 8, 16 };
inline constexpr int kNumFactores = 4;

// 2x IS NOT OFFERED — a product decision, and MEASURED.
//
// It fails the port yardstick: −58,4 dB (808) and −58,5 (OD9) against a
// ≤ −60 target, and on the 808's 36-point knob grid it breaks out 28 times
// of 36, with BOTH engines. A factor that fails the yardstick is not offered
// to the user in an unlabelled dropdown.
//
// RE-VERIFIED after the diode, the transistor and the cache landed, because
// halving the rate was the whole prize of the ADAA lever and the premise
// predated all three. It holds, and by a wide margin: on
// the pinch-harmonic bound the alias is AUDIBLE in 4 of 6 cells at 2x (worst
// +9,3 dB against a -10 criterion, i.e. 19,3 dB short), where 4x sits at -13,1
// with 3,1 dB to spare. The cell the old note named reproduces to 0,4 dB:
// +3,6 dB at 2.637 Hz then, +4,0 now.
// => The ladder is ~17-22 dB per doubling, so no anti-aliasing scheme bolted
// onto the memoryless stages closes it -- the alias that decides is born in
// stage 2, inside the implicit core.
//
// But it is NOT deleted, literally: `kFactores` keeps all four, all four
// `Oversampler`s still prepare in `instantiate()`, and the 2x path still
// compiles. The only change is WHERE the index search starts. Re-offering it
// is this line, not git archaeology.
//
// `-DNLSC_OS_2X=1` puts it back on offer — what any harness measuring the
// 2x rows of `docs/TABLA_MAESTRA.md` must use.
// Recompiling the harness is not enough: the BUNDLE must recompile too,
// because the core does the trimming, not the `.ttl`.
//
// That the trim lives HERE and not in a `.ttl` clamp is deliberate: a
// host may write anything into a control port, including values below the
// manifest's `lv2:minimum`. With the filter only in the TTL, a careless host
// would still run 2x and the plugin would deliver a fidelity it does not
// announce — silently, as always.
#ifndef NLSC_OS_2X
#define NLSC_OS_2X 0
#endif
inline constexpr int kOsPrimero = (NLSC_OS_2X ? 0 : 1);

// THE FACTOR THAT SHIPS, and it is no longer selectable. It is declared by
// INDEX and the index is checked to still be the 4x one: if anybody reorders
// `kFactores`, this does not quietly follow — it fails to compile.
inline constexpr int kOsFijo = 1;
static_assert(kFactores[kOsFijo] == 4,
              "kOsFijo must point at 4x: the .ttl used to declare `lv2:default 4`");

// AND 16x LEAVES THE MENU TOO. Same shape as the 2x trim above, mirrored to
// the other end, and the same reason: a factor that is offered has to be one
// whose behaviour we can stand behind.
//
// PRICED FIRST, on the .so that ships, through its ports, on an idle machine,
// three repeats: at 16x the cascade averages 11,63 % of a core and its WORST
// BLOCK runs 47,7-50,4 %. Worst block is what drops audio in real time, so a
// single instance eats half a core on an i7-8700K -- against a project whose
// stated reference is fitting in a Raspberry Pi 5. What it buys is 6,1 dB of
// alias (-43,3 at 8x to -49,4 at 16x) on a criterion 8x already clears by 33 dB.
//
// Null and ANMR at 16x were deliberately never measured: once the price
// decides, measuring the prize more finely is spending for its own sake.
//
// Nothing is deleted here either: `kFactores` still carries all four and all
// four oversamplers still prepare. `-DNLSC_OS_16X=1` puts it back on offer.
// And, as with 2x, the trim is in the CORE and not only in the `.ttl`: a host
// can write past `lv2:maximum`, and with the filter only in the manifest it
// would run 16x and pay for it silently.
#ifndef NLSC_OS_16X
#define NLSC_OS_16X 0
#endif
// Exclusive bound, so it reads like the loop that uses it.
inline constexpr int kOsUltimo = (NLSC_OS_16X ? kNumFactores : kNumFactores - 1);

// Implements `::nlsc::ICore` — see `nls_iface.h`. The leading `::` is NOT
// optional: inside the ISA namespace, `nlsc::ICore` would resolve to
// `isa_v3::nlsc::ICore`, which does not exist.
struct Plugin : ::nlsc::ICore {
    // Port pointers. The host sets them with connect_port() before run().
    const float* in   = nullptr;
    float*       out  = nullptr;
    float*       latency_out = nullptr;
    const float* ctl[PORT_COUNT] = { nullptr };

    double sample_rate = 48000.0;

    // ALL FOUR OVERSAMPLERS PREPARE IN instantiate(), not in run().
    // `Oversampler::prepare()` calls `assign()` on std::vector, i.e. it
    // ALLOCATES, and allocating on the audio thread breaks the real-time
    // contract. Four prepared filters cost a few KB; switching between them
    // is changing an index.
    Oversampler os_[kNumFactores];
    // The cascade is ALWAYS constructed even when unused: its constructor
    // allocates nothing (fixed arrays throughout), and keeping it alive
    // avoids any allocation on the audio thread when switching engines.
    Cascade4    casc_;

    // Knob state already applied to the engine — how we know whether the
    // matrices must be rebuilt.
    // 1 = 4x, which is what the `.ttl` declares. It once sat at 2 (8x),
    // i.e. code and manifest disagreed.
    int    idx_os_ = 1;              // 4x by default (lever D1, ANMR yardstick)
    double drive_ = -1.0, tone_ = -1.0, lvl_ = -1.0;   // −1 forces the first prepare
    int    idx_var_ = 0;             // variant applied to the engine (0 = 808)
    unsigned seed_ = 0;           // 0 = the ideal specimen (nominal)
    bool   pending_ = true;
    // A MODE change is waiting for the fade to bottom out.
    bool   pending_mode_ = false;

    // ─────────────────────────────────────────────────────────────────────────
    // THE FOOTSWITCH (`lv2:enabled`), and the TWO things it must do right.
    //
    // 1. LATENCY DOES NOT VANISH WHEN STOMPED. The plugin declares
    // `NLSC_OS_TAPS − 1` samples — 7 today — and the host compensates ALWAYS:
    // it does not re-ask on `enabled` changes. If bypass spat `in[i]` raw,
    // the signal would come out that many samples EARLY relative to what the
    // host expects, and anyone mixing in parallel with the dry gets a comb
    // filter: exactly the defect the latency port exists to prevent. => the
    // dry leaves DELAYED by the same amount, through this ring.
    // Do not spell the number on its own: it read 15 here until the FIR
    // went to 8 taps, and the comment outlived the change. The quantity is
    // `NLSC_OS_TAPS − 1`; the compile gate below is what enforces the bound.
    //
    // 2. NO HARD SWITCHING. Jumping from wet to dry is an amplitude step,
    // and the engine is biased (its rest output is not 0): the step is
    // audible even without signal. It fades over `kFadeSeconds`, and while
    // fading the engine KEEPS RUNNING — the wet signal being faded from has
    // to exist.
    //
    // When the fade lands in bypass, the engine stops running entirely,
    // and that is the footswitch's CPU saving. Its state stays FROZEN at the
    // last point, like a real pedal with its signal cut; on return, the fade
    // in covers the re-engagement transient.
    //
    // The ring size is a POWER OF TWO and is bounded against the real
    // latency: the oversampler's taps are compile-configurable
    // (`make TAPS=`), so the declared latency is not a product constant.
    static constexpr int    kDryRing = 128;         // > any declared latency
    static constexpr int    kMask    = kDryRing - 1;
    // In SECONDS, not samples: as `512.0` samples the fade
    // lasted 10,7 ms at 48 kHz but 5,3 ms at 96 k and 2,7 ms at 192 k — a
    // constant tuned to one rate is an approximation at every other one.
    static constexpr double kFadeSeconds = 512.0 / 48000.0;   // ~10,7 ms at ANY rate
    // THE BOUND IS A COMPILE GATE, not a comment. `latency()` equals
    // `taps−1`, and taps are chosen with `make TAPS=`: with a ring shorter
    // than the latency the delay would wrap and the dry would come out
    // misaligned with NO error — the expensive failure mode. Here the build
    // refuses. (Same family as the `kMaxOS` overflow.)
    static_assert(NLSC_OS_TAPS <= kDryRing,
                  "the dry ring does not cover the oversampler's latency: "
                  "raise kDryRing to the next power of two");

    float  dry_[kDryRing] = { 0.0f };
    int    dry_w_ = 0;
    double mix_ = 1.0;            // 1 = wet, 0 = dry. Starts active.
    // The FIRST block after `activate()` does not fade: it jumps to
    // whatever the port asks. Without this, instantiating the plugin already
    // stomped would open with half a second of downward ramp — a fade nobody
    // requested.
    bool   startup_ = true;
    // ─────────────────────────────────────────────────────────────────────────


    // Port-pointer routing, moved here from the ABI's connect_port: the
    // indices belong to the CORE, so the decision lives with their user.
    void connect_port(uint32_t puerto, void* dato) override
    {
        switch (puerto) {
        case ::nlsc::PORT_IN:      in  = static_cast<const float*>(dato); break;
        case ::nlsc::PORT_OUT:     out = static_cast<float*>(dato);       break;
        case ::nlsc::PORT_LATENCY: latency_out = static_cast<float*>(dato); break;
        default:
            if (puerto < ::nlsc::PORT_COUNT)
                ctl[puerto] = static_cast<const float*>(dato);
            break;
        }
    }

    void prepare(double sr) override
    {
        sample_rate = sr;
        for (int i = 0; i < kNumFactores; ++i) os_[i].prepare(kFactores[i]);
        // HERE, AND ONLY HERE, DEFERRED CACHE FILLING IS REQUESTED. This
        // object knows what no other piece knows: that its `apply_knobs()` —
        // and therefore `Cascade4::prepare()` — reaches it from INSIDE
        // `process()`, i.e. the audio thread. A harness does not request it
        // and keeps building the table at once, which is what makes its
        // figures measure the cache and not the Newton.
        casc_.defer_fill(true);
        pending_ = true;
        reset();
    }

    void reset() override
    {
        for (int i = 0; i < kNumFactores; ++i) os_[i].reset();
        pending_ = true;      // the engine re-prepares on the first run()
        std::memset(dry_, 0, sizeof dry_);
        dry_w_  = 0;
        startup_ = true;       // the first block does NOT fade: jump to the target
    }

    // Rebuilds the engine's matrices with the current knobs.
    //
    // WHY A KNOB MOVE FORCES A REBUILD, and it is no oversight: in the DK
    // formulation the potentiometers are NETLIST RESISTORS (`RGAIN`,
    // `RTONE`, `RLVL`), so they enter the system matrix. They are not a gain
    // multiplied at the end.
    //
    // Costs ~0,25 ms measured, and allocates NOTHING (the engine is all
    // static), so it may run on the audio thread. It happens AT MOST ONCE
    // PER BLOCK and only when something changed: with knobs still the cost
    // is zero, and while moving it is one 64-sample block (1,33 ms at
    // 48 kHz) taking a 19 % extra. Acceptable, and measurable if it ever
    // hurts.
    //
    // THIS COMMENT USED TO BE HALF TRUE: it accounted the COST and said
    // nothing of the STATE, while prepare also reset the twelve capacitors
    // to rest. No longer: through the knob path the matrices re-tune and the
    // state is kept (see the `reinit` parameter and `Engine::prepare`'s
    // comment). The cost even DROPS, because the skipped `solveDc()` is the
    // one that carried up to 300 Newton iterations.
    // THE DRIVE POT'S LAW. The port is a KNOB POSITION (0..1); the netlist
    // wants a RESISTANCE FRACTION. They are not the same, and confusing them
    // was a real defect: this used to be a raw `nl.gain = drive`.
    //
    // The TS's GAIN is a 500K LOGARITHMIC pot (`VARIANTES.md` §6, settled by
    // four sources). With a linear law, the whole useful travel bunches into
    // the first quarter of the knob and the pedal matches the real one at NO
    // intermediate position.
    //
    // Not theory: an EXTERNAL MEASUREMENT exposed it. Open Heart publishes
    // THD of a real TS with the drive at the MECHANICAL midpoint; our model
    // gave 16,21 % against their measured 4,18 %. Reproducing their 4,18 %
    // demands a resistance fraction of ~0,161 — exactly a standard A taper.
    // A null would NEVER have caught this: the null runs with `gain` = the
    // resistance fraction, so the question "what does the knob at 50 % mean?"
    // never even gets asked.
    //
    // THE SHAPE of the law comes from a MANUFACTURER DATASHEET, the
    // strongest arbiter in the house — above ngspice.
    //
    // Bourns PDB24 (24 mm rotary; its declared applications literally include
    // "electric guitars"). Its taper chart labels the curves (05A):A1 …
    // (30A):A6, i.e. THE NUMBER IS THE MID-TRAVEL PERCENTAGE. The audio
    // standard is A3 = "15A" = 15 %.
    //
    // Our `m` MEASURED against a real TS's THD gave 0,161, which lands
    // within ~1 point of the A3 spec (15 %) — and a pot taper's typical
    // tolerance is on the order of ±20 %. Two independent routes (a THD
    // measurement and a manufacturer sheet) agree.
    //
    // AND THE SHAPE MATTERS, not just the midpoint. A pure exponential
    // with the same midpoint falls SHORT in the upper half: at 90 % rotation
    // it would give 71 % against the chart's ~82 %. Real audio pots are made
    // with TWO TRACKS overlapping at the centre — a law of two straight
    // segments, which is what the chart follows:
    //
    //     x ≤ ½ :  y = 2·m·x                    (gentle segment)
    //     x > ½ :  y = m + 2·(1−m)·(x−½)        (fast segment)
    //
    //     turn    exponential   TWO SEGMENTS  A3 chart
    //     0,25       4,9 %         8,1 %       ~6 %
    //     0,50      16,1 %        16,1 %       15 %
    //     0,75      41,7 %        58,1 %      ~50 %
    //     0,90      70,8 %        83,2 %      ~82 %
    //
    // Both models agree at 0, ½ and 1 — the three points Open Heart
    // measures — so changing the shape does NOT touch the THD validation and
    // does bring the rest of the travel closer to the sheet.
    // The chart's intermediate readings are APPROXIMATE (±5 points): six
    // overlapping curves on a small graph. What is exact is the LABEL, and
    // the label is the spec.
    static constexpr double kTaperMedio = 0.161;   // mid-travel fraction

    static double taper_log(double x)
    {
        if (x <= 0.0) return 0.0;
        if (x >= 1.0) return 1.0;
        return (x <= 0.5) ? 2.0 * kTaperMedio * x
                          : kTaperMedio + 2.0 * (1.0 - kTaperMedio) * (x - 0.5);
    }

    // THE TONE POT'S "W" TAPER.
    //
    // The TS's tone pot is NOT linear, and that is settled as FACT
    // (`VERIFICACION_NETLIST.md` §11): ElectroSmash calls it "G", the GGG
    // schematic prints `20KG` and pedalparts `20KW` — and builders of TS
    // clones consistently use W there.
    //
    // SHAPE: a SYMMETRIC S with the SMALLEST slope AT THE CENTRE — fast at
    // first, flat in the middle, fast again. That is what ElectroSmash's
    // "more control in the middle of the sweep" means.
    //
    // SOURCE: a taper-curve chart where curve 5 is labelled literally
    // "G", "W" or "S" taper — the three letters our three sources use for
    // this pot. Points read off the chart:
    //     0,1 -> 0,25 · 0,2 -> 0,38 · 0,3 -> 0,42 · 0,5 -> 0,50 · 0,7 -> 0,62
    //     0,8 -> 0,75 · 0,9 -> 0,88
    // FIT: `0,5 + 0,5·sign(2x−1)·(2x−1)²`, a symmetric parabola.
    //     x    chart     parabola
    //    0,1    0,185      0,180
    //    0,2    0,315      0,320
    //    0,3    0,400      0,420
    //    0,5    0,500      0,500   <- the ONLY explicitly marked point
    // The chart values carry IMPOSED symmetry (averaging x and 1−x): read
    // by eye they give f(0,1)=0,25 and f(0,9)=0,88, summing 1,13 instead of
    // 1. The upper-half misfit (0,04-0,07) sits within that reading
    // imprecision, and its effect is second order: the whole taper is worth
    // 0,4 dB in the stable magnitude.
    //
    // AND TAPER NOMENCLATURE IS NOT CONSISTENT, verified against two
    // charts: a 100K W WITH CENTER DETENT shows the OPPOSITE curve (steep at
    // the centre, flat at the ends), which suits a blend control. The letter
    // alone is not enough: look at the curve.
    //
    // LIMITATIONS, to read before trusting this:
    //  · The NUMERIC curve is unbacked. Published taper tables contradict
    //    each other and the qualitative description; the catalogues that
    //    print them return 403. What is established is the SHAPE (symmetric
    //    log/antilog S), not the percentages.
    //  · It passes through (0,5 , 0,5) by construction, so it does NOT
    //    change the midpoint — exactly where our only useful external
    //    reference sits. The 0,4 dB peak deviation at mid-knob REMAINS: this
    //    does not fix it and must not be sold as fixing it.
    //  · The points it DOES change (0,25 and 0,75) are the ones we have
    //    nothing to validate against. It ships on the source and on build
    //    experience, not because a measurement demands it.
    //
    // Reversible: `make NLSC_TONE_W=0` reverts to linear.
    static double taper_w(double x)
    {
        if (x <= 0.0) return 0.0;
        if (x >= 1.0) return 1.0;
        // SYMMETRIC PARABOLA. Forcing the curve through the drive's
        // `taper_log` (piecewise linear) lacked shape and missed by up to
        // 0,14; this fits the chart within the error of reading it by eye.
        const double u = 2.0 * x - 1.0;
        return 0.5 + 0.5 * (u >= 0.0 ? u * u : -u * u);
    }

    // `ToneLaw::Even` — the generated table (`nls_tone_law.h`), read with a
    // straight linear interpolation. 65 points over the travel, so each
    // segment spans 0,0156 of knob and the interpolation error stays far
    // under the 0,069 dB the generator reports against its ideal line.
    //
    // It is NOT the default and it is NOT a fidelity fix: the pot's G taper
    // stays on the faithful rows. This one exists because the circuit
    // concentrates the tone's whole effect in the last quarter of the pot,
    // and no pot in a catalogue undoes that — the law does.
    // Cost: evaluated in `apply_knobs`, i.e. once per BLOCK when a knob
    // moved. ZERO per sample.
    static double taper_even(double x)
    {
        if (x <= 0.0) return tone_law::kT[0];
        if (x >= 1.0) return tone_law::kT[tone_law::kN - 1];
        const double u = x * double(tone_law::kN - 1);
        const int    i = int(u);
        const double f = u - double(i);
        return tone_law::kT[i] + (tone_law::kT[i + 1] - tone_law::kT[i]) * f;
    }

    // Control ports are SANITISED before touching the netlist.
    //
    // A host is not obliged to clamp: `jalv` lets you type `set tone 5` at
    // its console, a saved session can carry a corrupt value, and a host
    // that interpolates parameter changes can overshoot the range. With
    // `tone = 5` the netlist ends up with a resistance of −80 kΩ: garbage or
    // silent output, no diagnostic, until the value comes back.
    //
    // And this was already house policy: `factor_index` literally
    // says an out-of-range value gets treated, not propagated. This was the
    // exception, not the rule. Costs three comparisons per BLOCK, and only
    // when something changed.
    static double sanitise(double x, double porDefecto)
    {
        if (!std::isfinite(x)) return porDefecto;
        return (x < 0.0) ? 0.0 : (x > 1.0 ? 1.0 : x);
    }

    // Receives the knobs ALREADY SANITISED (done where the ports are
    // read, see `process`). Sanitising in here would be a subtle failure
    // worse than the one it fixes: what is stored in `drive_/tone_/lvl_` is
    // what next block's comparison sees to decide on re-preparing, so if the
    // SANITISED value were stored while the comparison sees the RAW one, a
    // port pinned out of range would read "changed" EVERY block ->
    // `prepare()` every block -> the 12 capacitors reset to rest 344 times a
    // second. A continuous click, plus 0,25 ms extra of audio thread.
    // `reinit` distinguishes the TWO reasons for arriving here:
    //   true  — startup, rate change, or a MODE change (`os`, `variant`,
    //           `engine`): the stored state is at the old rate or describes
    //           the other circuit, and is invalid.
    //           That is NO LONGER A CLICK: the mode enters under the fade
    //           (see `process()`), so the reinit happens with the output on
    //           the dry and is inaudible. `make conmutacion`.
    //   false — only a potentiometer moved: the matrices re-tune and the
    //           circuit keeps the charge it had, like the real pedal.
    void apply_knobs(double drive, double tone, double level, int idx_os,
                        int idx_var, bool reinit)
    {
        mna::Netlist nl;
        // THE VARIANT IS A COMPONENT VALUE, so it enters through the
        // netlist and leaves by the same path as the pots: matrices re-tune
        // and the twelve capacitors' state is KEPT. Changing two resistors
        // does not change the topology, so the sparsity pattern and the
        // generated elimination code stay the same.
        nl.rout_ser   = circuit_of(idx_var).rout_ser;
        nl.rout_shunt = circuit_of(idx_var).rout_shunt;
        nl.seed = seed_;      // before stamping: `build()` consumes it
        nl.gain = taper_log(drive);
#if !NLSC_TONE_W
        nl.tone = tone;            // LINEAR — the pre-taper behaviour
#elif defined(NLSC_MUTANT_TONE)
        // CALIBRATION MUTANT — QUADRATIC law instead of linear. It exists
        // to force `knob_law.py` RED on the tone axis: that harness used
        // to judge the law by the CORNER (a moving window, since the peak
        // shifts) over three points, so it would have approved almost any
        // law.
        nl.tone = tone * tone;
#else
        // THE ROW DECIDES THE LAW. `ToneLaw::Pot` is the real pot's taper
        // and ships on every faithful row; `ToneLaw::Even` is the derived
        // law, offered on its own row. The two measurement arms above
        // (linear, and the calibration mutant) do NOT consult the row on
        // purpose: they force one law to measure it, and a row that changed
        // it under them would leave the instrument measuring something else.
        nl.tone = (kVariants[idx_var].tone_law == ToneLaw::Even)
                      ? taper_even(tone)
                      : taper_w(tone);   // W taper: symmetric log/antilog S
#endif
        // THE LEVEL IS INVERTED, and it is measured: in the netlist
        // `lvl=0` is LEVEL AT MAXIMUM (0,856 Vpp) and `lvl=1` leaves the
        // output at 8,8 nV (docs/LINEALIDAD_ETAPAS.md §8, which literally
        // says the parameter layer must invert it). Wiring it straight gives
        // a backwards knob and a mute plugin at full level.
        // THE LAW IS LINEAR, and a PRIMARY MANUFACTURER SOURCE settles it.
        // The comment here used to say "audio, Keen rules" and ended in
        // "re-verify at the first datum that touches this". The datum came:
        // the manufacturer's service manual, whose parts list gives the three
        // pots with their full part numbers —
        //
        //     VARIABLE RESISTOR  16φ 500KA   PMXB155R504ASC   <- DISTORTION
        //     VARIABLE RESISTOR  16φ  20KG   PMXB155R203GSC   <- TONE
        //     VARIABLE RESISTOR  16φ 100KB   PMXB155R104BSC   <- LEVEL
        //
        // — and its schematic repeats "100KB" next to the LEVEL knob. The
        // taper letter sits INSIDE the part number, so there is no glyph to
        // guess, and all three coexist in one document with the A and the G
        // exactly where expected: that disambiguates the B.
        //
        // => B = LINEAR. Keen said "audio" and was wrong; ElectroSmash copied
        // that sentence even though its OWN BOM said "100K Lin"; and an
        // earlier arbitration sided with Keen. It also fits the builders who
        // report that with B100K the pedal responds like the original and
        // with A100K it does not.
        //
        // No null would ever have seen this: the null runs on the PHYSICAL
        // parameter, and this is the law between knob POSITION and that
        // parameter. Second time this defect class appears in this plugin —
        // the first was the DRIVE.
        //
        // Order matters: the law applies to the POSITION and the inversion
        // afterwards. The divider's attenuation is (1 - lvl), so for the
        // attenuation to follow the law we need `lvl = 1 - taper(position)`.
#ifdef NLSC_MUTANT_LEVEL
        // CALIBRATION MUTANT — removes the inversion on purpose, to force
        // `harness/knob_law.py` RED: three greens from a harness that has
        // never failed are not a pass.
        nl.lvl = level;
#else
        nl.lvl = 1.0 - level;      // LINEAR: 100K B pot (service manual)
#endif
        const double fs_int = sample_rate * double(kFactores[idx_os]);
        // THE DK'S `prepare()` USED TO RUN HERE ON EVERY KNOB BLOCK, for an
        // output nobody reads. The guard was `if (reinit || idx_eng_ == 0)`, and
        // once `engine` left the plugin `idx_eng_` was pinned at 0 (the DK), so
        // the condition was ALWAYS TRUE: a 29x29 inverse, the junction tables and
        // `compute_max_step` per knob move. Measured on this machine: 0,052 ms
        // median (`make coste-prepare`) against 0,0033 ms for the cascade's own
        // (`make coste-prepare-casc`) — 94 % of the knob path was waste.
        // => The DK is gone from the core. It stays in `nls_dk.h` for the bench,
        // where it is the cascade's positive control and the arbiter of every
        // null grid.

        // THE CASCADE, prepared in the SAME place and with the SAME
        // physical parameters (`nl`), not with the port positions: if each
        // engine derived its own parameter from the knob, an A/B would
        // measure the two laws instead of the two architectures.
        // THERE IS NO RATE RESTRICTION any more. The cascade's fixed
        // filters ship in `s` (`nls_fijos_s.h`) and discretise here at the
        // REAL rate, like the tone and the level. This used to be
        // `casc_ok_ = (fs_int == e234::kFs)`, with the plugin falling back
        // to the DK away from 48 kHz x 4, the only tabulated rate.
        // THE VARIANT REACHES THE CASCADE. This `prepare` once did not
        // receive it, which is why the `variant` port came out BIT-IDENTICAL
        // with the cascade: the manifest announced two circuits and the
        // engine delivered one. Per-sample cost is ZERO — everything
        // resolves in `prepare()`.
        // ROW -> CIRCUIT IS TRANSLATED HERE. The cascade dispatches its
        // BANK, and a bank belongs to a CIRCUIT: passing the row index sent
        // every row other than 0 and 1 to the 808's bank.
        // THE GESTURE IS DECLARED BEFORE RE-TUNING. `reinit` already tells the
        // two reasons for arriving here apart: set-up or mode change, against
        // "the user has moved a knob". Stage 2 cannot know which it is and does
        // not need to: the CALLER decides.
        casc_.tabla_gesto(!reinit);
        if (true)
            casc_.prepare(fs_int, nl.gain, nl.tone, nl.lvl,
                          kVariants[idx_var].circuit, reinit);

        drive_ = drive; tone_ = tone; lvl_ = level; idx_os_ = idx_os;
        idx_var_ = idx_var;
        pending_ = false;
    }

    // THE DISPATCH, in one place. The three `process` loops (wet, fading
    // and bypass) call HERE, so they cannot diverge: a different engine on
    // one of the three paths is exactly the defect class that raises no
    // error and only sounds wrong when stomping.
    inline double motor(double x)
    {
        // Was a two-way dispatch on `idx_eng_`. One engine ships now, so the
        // branch is gone rather than pinned: a branch nobody can take is dead
        // code that still has to be read and maintained.
        return casc_.process_modelled(x).out;
    }

    static double read_port(const float* p, double def)
    {
        return p ? double(*p) : def;
    }

    // The port carries the real factor; it maps to the nearest of the four
    // indices. A value off the list (a host interpolating the enum) must NOT
    // crash: it snaps to the closest.
    static int factor_index(double v)
    {
        // Starts at `kOsPrimero`, not 0: the factors below it exist in
        // `kFactores` but are NOT OFFERED (see its comment). A host writing
        // 2 with the offer trimmed snaps to 4x — exactly the policy already
        // applied to any other off-list value.
        int mejor = kOsPrimero; double dmin = 1e18;
        for (int i = kOsPrimero; i < kOsUltimo; ++i) {
            const double d = std::fabs(v - double(kFactores[i]));
            if (d < dmin) { dmin = d; mejor = i; }
        }
        return mejor;
    }

    // The variant port is an INDEX (no physical value to preserve as in
    // `oversampling`), so it rounds and clamps. Same policy as the rest: an
    // out-of-range value gets TREATED, not propagated — unclamped here it
    // would be a read past `kVariants`.
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

        // --- knobs, ONCE per block ---------------------------------------
        // Read before processing, with any `prepare` done here. Doing it
        // before the audio (not mid-block) is what keeps the output
        // BIT-IDENTICAL under any block slicing: with knobs still, the state
        // does not depend on where the boundaries fall.
        // Sanitising happens HERE, at the read point, and not inside
        // `apply_knobs`: that way what is compared and what is stored are
        // the same magnitude, and a port pinned out of range clamps once and
        // stops "changing" (see `apply_knobs`'s comment).
        const double drive  = sanitise(read_port(ctl[PORT_DRIVE], 0.5), 0.5);
        const double tone   = sanitise(read_port(ctl[PORT_TONE],  0.5), 0.5);
        const double level  = sanitise(read_port(ctl[PORT_LEVEL], 0.5), 0.5);
        // The fallback is for an UNCONNECTED port, and must equal the
        // default the `.ttl` declares: otherwise a host that skips the port
        // runs a different product than the manifest says.
        // `oversampling` left the plugin: the factor is FIXED at 4x.
        // `factor_index` is kept because the BENCH still sweeps that axis by
        // compile flag, and because the index is then derived from the same
        // place as before instead of being written by hand.
        const int    idx_os = kOsFijo;
        const int    idx_var = variant_index(read_port(ctl[PORT_VARIANT], 0.0));
        // THE DEFAULT IS 1 (CASCADE) — a product decision: the cascade is
        // always the default engine; the DK is the HQ mode.
        //
        // And the default must MATCH what the `.ttl` declares: an
        // unconnected port runs what the manifest says, nothing else.
        //
        // What used to block it, and no longer does: the cascade once did
        // NOT implement the variants — `variant` 0 and 1 gave BIT-IDENTICAL
        // output (max|delta| = 0,0000e+00) — so the manifest announced two
        // circuits and the engine delivered one. Today `make puertos`
        // measures `variant@engine1 -> 6,52e-01` and carries its own axis
        // with the engine PINNED to cascade, so that hole cannot come back
        // hidden by a default choice.
        // `engine` and `seed` LEFT THE PLUGIN. There is
        // one engine now — the cascade — so there is no selector to read and
        // no mode to fade for. See `nls_iface.h` for the measured reason.

        // MODE CHANGES ENTER UNDER THE FADE.
        //
        // A KNOB re-tunes hot and keeps the state. A MODE — `os`, `variant`,
        // `engine` — does not:
        //   · `os` changes the internal rate, so the stored state (capacitor
        //     voltages at the old rate) is invalid => a reinit was needed,
        //     and that was an audible CLICK;
        //   · `variant` did NOT reinit, and sounded WORSE than it looked:
        //     measured, switching 808->OD9 jumps 2,4x the signal's typical
        //     slope AND spends ~2 SECONDS in which the output is neither
        //     variant, because the bias network re-settles on its own
        //     (`C11` 47 uF => tau ~235 ms, and 2 s is 8,5 tau). The DK does
        //     the same and WORSE, so it was not the cache or the cascade.
        //
        // => ONE MECHANISM fixes both: fade to the DRY (the fade the
        // footswitch already had), apply the mode with `reinit` — which
        // re-seeds the new variant's rest instead of letting the model
        // integrate its way there — and fade back to wet. The transient
        // stops existing and the jump lands under the ramp.
        // And a KNOB does NOT fade: those keep applying instantly, with
        // the CURRENT mode, or `make mandos` would measure a ramp instead of
        // a law.
        // `seed` is a MODE, not a knob: it re-draws every component value, so
        // applying it hot re-tunes the matrices under the OLD state — measured
        // as a 33,2x jump against this gate's 0,5x cap, worse than the pre-fade
        // `variant` (2,4x). It enters under the fade like the other three.
        // `seed` ONLY WAKES A MODE CHANGE ON THE DK, because
        // that is the contract the `.ttl` already declares: *"it only acts on
        // the DK engine — NOT the default"*, since the cascade runs on
        // precomputed header coefficients and has nothing to re-seed. Waking
        // it there cost a full fade + `reinit`: a ~21 ms gap and the state
        // teleported to rest, for a port that does nothing on that engine.
        // Switching TO the DK still picks the new seed up — `idx_eng`
        // changes, so `mode_change` fires and `seed_` is updated there.
        // ONE mode is left: `variant`. The engine, the seed and the
        // oversampling factor are gone, and their arms of this condition with
        // them. The MECHANISM does not simplify away with them: the fade is
        // still the same path, and it is what avoids the 2,4x click when the
        // circuit is switched.
        bool mode_change = (idx_var != idx_var_);
#if !NLSC_FADE_MODE
        // CALIBRATION MUTANT for `make conmutacion` — NEVER SHIPPED. It
        // applies the mode INSTANTLY, the old behaviour, to force that gate
        // RED. Four greens from a harness that has never failed are not a
        // pass.
        if (!pending_ && mode_change) {
            apply_knobs(drive, tone, level, idx_os, idx_var, true);
            os_[idx_os_].reset();
            // The mutant reproduces the OLD behaviour and nothing else: without
            // this line the pending-mode path below would still see the change
            // and ADD a fade on top of the instant jump it just made.
            mode_change = false;
        }
#endif
        if (pending_) {
            // The first `prepare` does not fade: there is nothing to leave.
            apply_knobs(drive, tone, level, idx_os, idx_var, true);
            os_[idx_os_].reset();
            pending_mode_ = false;
        } else {
            // The mode lands when the ramp bottoms out, and at the BLOCK
            // BOUNDARY: `apply_knobs` rebuilds matrices, and that does not
            // belong inside the sample loop.
            if (pending_mode_ && mix_ <= 0.0) {
                apply_knobs(drive, tone, level, idx_os, idx_var, true);
                os_[idx_os_].reset();
                pending_mode_ = false;
            } else if (mode_change) {
                pending_mode_ = true;      // start the ramp down; apply on arrival
            }
            // `seed` is NOT in this condition: it is a mode now (see above) and
            // enters through the pending path when the fade bottoms out.
            if (drive != drive_ || tone != tone_ || level != lvl_) {
                apply_knobs(drive, tone, level, idx_os_, idx_var_, false);
            }
        }

        // STAGE 4's CACHE FILL, ONE SLICE PER BLOCK.
        //
        // It sits HERE — OUTSIDE the knob `if` — because the fill must
        // advance in EVERY block, not only when something is touched: right
        // after moving the variant dropdown nothing else moves, and that is
        // exactly when filling is needed. Inside the `if`, the table would
        // never return until the next gesture.
        //
        // Bounded, deterministic cost — no threads, no allocation: ~21 us
        // over a 1,333 ms block, and the full table returns in ~267 ms.
        // Meanwhile the Newton runs, the same path already used outside the
        // box.
        // COUNTING PER SAMPLE FIXES NOTHING — tried and REVERTED.
        //
        // The fill advances `NLSC_E4_CACHE_NODES_PER_BLOCK` nodes per CALL,
        // i.e. per block, making the pace depend on the host's block size.
        // Making it proportional to samples was tried and made it WORSE:
        // max|dif| from 3,923e-06 to 7,659e-06, and from failing one test to
        // two. => The reason is structural, not accounting: a ONE-block
        // reference fills the table before any slicing, however you count.
        // Progressive filling and block bit-identity are INCOMPATIBLE, and
        // the way out is a DECISION (build at once and eat a first-block
        // spike, or declare the compromise), not a one-line fix. The gate
        // watches it with a threshold.
        // With `NLSC_E4_FILL_PER_SAMPLE` the fill is carried by
        // `SubQ2` INSIDE `process`, between samples, and this call is
        // redundant: keeping it would fill TWICE and the schedule would
        // depend on the block again.
#if !NLSC_E4_FILL_PER_SAMPLE
        casc_.advance_cache();
#endif

        // The host wants the latency in base-rate samples.
        //
        // THIS USED TO SAY "NOT constant across modes: 0 at 1x and taps−1
        // from 2x", and in the PRODUCT it is constant: `kFactores` is
        // {2,4,8,16} and `factor_index` snaps any port value to the
        // nearest of those four, so 1x — the only one that would give 0 — is
        // unreachable from the plugin. All four factors publish `taps−1` (7 with
        // today's 8 taps; a hand-written number here already sent a gate red once).
        // It is still published every block for robustness (taps are
        // compile-configurable), NOT because the knob moves it.
        if (latency_out) *latency_out = float(os_[idx_os_].latency());

        const int N = kFactores[idx_os_];
        // The buffer uses the OVERSAMPLER's precision (`osreal`, float
        // under A4), not the engine's. It is the mixed-precision boundary:
        // the FIR works in float — where it doubles lanes — and the solver
        // stays in double, with one conversion per oversampled sample that
        // costs an instruction and never shows in the profile.
        osreal up[Oversampler::kMaxOS];                       // the max factor; no allocation

        // --- THE FOOTSWITCH, decided ONCE per block ----------------------
        //
        // The dry's delay must be EXACTLY the latency just published, not a
        // hand-written 15: taps are configurable (`make TAPS=`) and a
        // mismatched delay would comb-filter exactly the case this code
        // exists to prevent.
        const int lat = os_[idx_os_].latency();
        const bool activo  = read_port(ctl[PORT_ENABLED], 1.0) >= 0.5;
        // A pending mode change ramps down like a bypass: same path, which
        // is why there is no second fade that could diverge.
        const double destino = (activo && !pending_mode_) ? 1.0 : 0.0;
        if (startup_) { mix_ = destino; startup_ = false; }
        const double step = 1.0 / (kFadeSeconds * sample_rate);

        if (mix_ == destino && destino == 1.0) {
            // ══ PURE WET PATH ════════════════════════════════════════════
            // The usual loop, arithmetic INTACT. Deliberately separate
            // instead of the general `mix*y + (1-mix)*dry` with mix = 1:
            // even though IEEE gives the same number, a separate path makes
            // the shipped route the SAME as before by construction, not by
            // an argument about rounding.
            // The only addition is pushing the dry into the ring, which does
            // not touch the output.
            for (uint32_t i = 0; i < n_samples; ++i) {
                // SANITISE THE INPUT, and it is not paranoia: without this
                // one non-finite host sample MUTES THE TRACK FOREVER. The
                // engine feeds it into `s_` (the 12 capacitors' state) and
                // there is no way back: `p[]` is NaN every sample, Newton
                // returns -1 forever and the output belt writes 0.0f — mute
                // until the user moves a knob and forces `prepare()`. One
                // unstable upstream plugin or one uninitialised buffer is
                // enough. Costs one comparison per BASE sample.
                const float xin = in[i];         // read BEFORE writing: in-place
                const double x = std::isfinite(xin) ? double(xin) : 0.0;
                dry_[dry_w_] = float(x);
                dry_w_ = (dry_w_ + 1) & kMask;
                os_[idx_os_].upsample(x, up);
                for (int k = 0; k < N; ++k) up[k] = osreal(motor(double(up[k])));
                const double y = os_[idx_os_].downsample(up);
                // Safety net: a non-finite in the output buffer propagates
                // down the host's chain and is fiendish to trace back here.
                // The engine has its own net (keeps the previous solution if
                // Newton fails to converge); this is the last belt.
                out[i] = std::isfinite(y) ? float(y) : 0.0f;
            }
        } else if (mix_ == destino) {
            // ══ SETTLED BYPASS ═══════════════════════════════════════════
            // The engine does NOT run: this is the footswitch's CPU saving.
            // Its state stays frozen at the last point, like a real pedal
            // with the signal cut; the fade-in covers the re-engagement
            // transient.
            for (uint32_t i = 0; i < n_samples; ++i) {
                const float xin = in[i];
                const double x = std::isfinite(xin) ? double(xin) : 0.0;
                dry_[dry_w_] = float(x);
                dry_w_ = (dry_w_ + 1) & kMask;
                out[i] = dry_[(dry_w_ + kDryRing - 1 - lat) & kMask];
            }
        } else {
            // ══ FADING ═══════════════════════════════════════════════════
            // The engine KEEPS RUNNING while fading, both directions: fading
            // towards the dry needs the wet being left behind.
            for (uint32_t i = 0; i < n_samples; ++i) {
                const float xin = in[i];
                const double x = std::isfinite(xin) ? double(xin) : 0.0;
                dry_[dry_w_] = float(x);
                dry_w_ = (dry_w_ + 1) & kMask;
                os_[idx_os_].upsample(x, up);
                for (int k = 0; k < N; ++k) up[k] = osreal(motor(double(up[k])));
                const double y = os_[idx_os_].downsample(up);
                const double mojada = std::isfinite(y) ? y : 0.0;
                const double seca = double(dry_[(dry_w_ + kDryRing - 1 - lat) & kMask]);
                out[i] = float(mix_ * mojada + (1.0 - mix_) * seca);
                mix_ += (destino > mix_) ? step : -step;
                if (mix_ > 1.0) mix_ = 1.0;
                if (mix_ < 0.0) mix_ = 0.0;
            }
        }

#ifdef NLSC_MUTANT
        // HARNESS CALIBRATION, never in the installed binary. Perturbs the
        // ALREADY-PROCESSED output by one ULP — the smallest error float32
        // can express — to prove the test KNOWS how to go red.
        // This used to REPLACE the processing with `nextafter(in)`. With
        // DSP inside that calibrated nothing: the mutant would have passed
        // the block-invariance test as well as the good one, both being
        // deterministic. It has to be a PERTURBATION of what comes out.
        for (uint32_t i = 0; i < n_samples; ++i)
            out[i] = std::nextafter(out[i], 2.0f);
#endif
    }
};

} // namespace nlsc
