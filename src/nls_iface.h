// nls_iface.h — the ABI shared by the LV2 scaffolding and the TWO cores.
//
// THIS FILE IS NEVER WRAPPED IN AN ISA NAMESPACE.
//
// The DSP core compiles TWICE — baseline and avx2/fma — inside two different
// namespaces, so the linker cannot merge the two copies. Everything the LV2
// scaffolding needs to see must live OUTSIDE those namespaces, or the
// connect_port switch would end up talking about `isa_v3::nlsc::PORT_IN`,
// which is not the same type.
//
// Nothing from the DSP belongs here: no templates, no `inline`, no engine
// types. It is a call interface, and its only cost is ONE indirection per
// block (375/s at 128 samples and 48 kHz) — which is the point: dispatching
// per SAMPLE, at 192 000 calls/s, measured 4,09 % SLOWER on the cascade.
#pragma once

#include <cstdint>

namespace nlsc {

enum PortIndex : uint32_t {
    PORT_IN           = 0,
    PORT_OUT          = 1,
    PORT_DRIVE        = 2,
    PORT_TONE         = 3,
    PORT_LEVEL        = 4,
    // `model` and `clipping` were REMOVED before release: they were declared
    // in the TTL, visible and movable by the user, and read by code that did
    // nothing with them — a user-facing defect, not a pending task. If circuit
    // variants ever need them, they are created anew.
    //
    // Removing them MOVED the indices of `oversampling` and `latency`. That
    // breaks saved sessions in general — but the plugin was not yet published,
    // and the alternative was shipping two dead knobs. Once there are users,
    // this door is closed for good.
    // `oversampling` LEFT THE PLUGIN TOO. The factor is FIXED at 4x and is not
    // selectable, because the knob had no correct answer to offer:
    //   · 2x FAILS the port yardstick in 28 of 36 cells, worst -50,1 dB.
    //   · 8x costs +92 % of CPU and buys 3,1 dB of ANMR — the binding axis —
    //     because the cascade saturates while the engine keeps falling to -41,60.
    //   · 4x passes with 0 of 36 and +2,30 dB of margin.
    // A knob with a single correct answer is not a knob, it is a trap.
    //
    // THIS is the removal that RENUMBERS (latency 6->5, enabled 7->6,
    // variant 8->7), which is why the GUI stopped keeping its own copy of the
    // indices BEFORE it happened: with the old copy its `PORT_ENABLED = 7`
    // would have pointed at `variant`, and pressing the footswitch would have
    // changed circuit with no error at all.
    //
    // The BENCH does not lose the axis: it selects the factor with a COMPILE
    // FLAG (`EXTRA_DEFS=-DNLSC_CASC_KHZ=<48·N>`), which is how the knob sweep
    // already produced its 8x row.
    PORT_LATENCY      = 5,   // output: the host needs it to compensate
    // These two are the 3PDT footswitch and the variant selector from the
    // design handoff.
    //
    // They go AT THE END on purpose: indices 0..6 do not move, so neither the
    // frozen reference nor any harness goes out of alignment.
    //
    // And they are NOT dead knobs like `model`/`clipping` above: BOTH act on
    // the output, and `harness/puerta_puertos.py` verifies that by RUNNING the
    // harness, not by reading the source. In this plugin that is the
    // requirement for declaring a port, not a recommendation.
    PORT_ENABLED      = 6,   // lv2:enabled — the footswitch
    PORT_VARIANT      = 7,   // which circuit ships the output: 808 or 9/9RI
    // `engine` AND `seed` LEFT THE PLUGIN TOO.
    //
    // The DK solver is not deleted — it moves to the BENCH, where it is what it
    // always really was: the cascade's positive control and the arbiter of
    // every null grid. It just stops being a user-facing mode.
    //
    // Measured on the 4x grid, which is why the mode earned nothing: the
    // cascade wins on all three yardsticks — null worst -62,3 against -60,4
    // (and +2,30 dB of margin to the bar against +0,40), ANMR worst -30,10
    // against -29,50, and half the CPU (3,07 % against 6,18 %).
    //
    // `seed` goes with it because it only ever ACTED on the DK: the cascade
    // runs on precomputed header coefficients and has nothing to re-seed. The
    // unit-to-unit variation it provided (~16 % at the tone corner, ~1,5 dB at
    // the peak) is logged as future work ON THE CASCADE, not lost.
    //
    // They were the LAST two indices on purpose: removing them renumbers
    // NOTHING. `oversampling` is the one that will, and that is a separate
    // step with its own guard.
    PORT_COUNT        = 8
};

// The core, seen from outside. One entry point per thing the LV2 life cycle
// needs to do.
struct ICore {
    virtual ~ICore() = default;
    virtual void prepare(double rate)                  = 0;
    virtual void reset()                               = 0;
    virtual void process(uint32_t n_samples)           = 0;
    virtual void connect_port(uint32_t puerto, void* dato) = 0;
};

// The two factories, one per translation unit. They return nullptr when out
// of memory: the host discards the instance, which is instantiate()'s
// contract.
ICore* make_base();
ICore* make_v3();

// Reads CPUID. Called ONCE, in instantiate().
bool has_v3();

} // namespace nlsc
