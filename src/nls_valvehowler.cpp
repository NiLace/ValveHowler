// Valve Howler — an overdrive modelled from the circuit.
//
// The oversampling factor is a user-facing QUALITY SWITCH (2/4/8/16x), not a
// constant. The three criteria are measured and pull in different
// directions (docs/FIDELIDAD_VS_SOBREMUESTREO.md):
//     aliasing with guitar ..... 2x     (docs/SOBREMUESTREO.md)
//     fidelity (output null) ... 8x     first factor to reach −80 dB
//     real time ................ 2x     the «8x costs 217 % of one core on the dev
//                                        box» that stood here was stale twice over: it
//                                        named no machine (it was the LAPTOP) and it
//                                        priced an engine that no longer exists. On the
//                                        i7-8700K the shipped cascade at 8x is 5,54 %,
//                                        and at the 4x it ships, 2,61 %.
// No single value satisfies all three, so the choice is exposed instead of
// hidden.
//
// THE DEFAULT IS 4x, decided on the ANMR yardstick over real material; the
// `.ttl` states it: "8x adds 21 dB of margin for DOUBLE the CPU: not
// justified". The −80 dB criterion above is SUPERSEDED by that measurement —
// it stays written because it explains where the switch came from.
//
// Mounted from the first commit, because retrofitting is what costs:
//   - Entry point wrapped in extern "C".
//   - Precision parametrised by typedef, NOT hard-wired.
//   - FTZ/DAZ armed at instantiation.
//   - run() with no allocation, no exceptions, no locks.

// `NLSC_TONE_W`'s default USED TO LIVE HERE, and this file does not
// include the core — the ISA dispatch moved the engine into its own
// translation unit, so the macro never reached the code that reads it and the
// product shipped a LINEAR tone law for thirteen days. The default now lives
// in `nls_core.h`, next to its only use. Do not restore a copy here:
// `-DNLSC_TONE_W=0` on the command line still reaches both units.

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <lv2.h>

// THIS BLOCK GOES BEFORE EVERYTHING ELSE: `arm_denormal_flush()` consults
// this macro, and below it its body would come out EMPTY with no warning.
#if defined(__SSE2__) || defined(__x86_64__)
#  include <pmmintrin.h>
#  include <xmmintrin.h>
#  define NLSC_HAVE_X86_DENORMAL_CTRL 1
#endif

// THIS FILE HAS NO DSP LEFT.
//
// The core compiles TWICE in `nls_isa_tu.cpp` — baseline and avx2/fma — and
// here the only thing chosen is which one, ONCE, in `instantiate()`.
// Everything below talks to `nlsc::ICore`, an interface with nothing of the
// engine inside.
#include "nls_iface.h"

namespace nlsc {


// Flush denormals to zero.
//
// THIS COMMENT USED TO CLAIM TWO FALSE THINGS, and both were measured. It
// said "without this the CPU spikes as the signal decays to silence", and
// that it is armed per instance "because the host does not guarantee which
// thread runs run()" — the second was also a non-sequitur, since arming in
// `instantiate()` does not answer that.
//
// WHAT IS MEASURED: in this engine the guard sustains nothing. 60 s of exact
// digital silence produce ZERO denormals, because a physically modelled
// circuit does not decay to zero: it settles at its rest point (|y| pinned
// at 5,66e-08 and the states at their DC). A chord's tail does not drive the
// state towards denormals; it drives it towards the bias. Zero denormals as
// well over 5,76 M samples of real material.
//
// AND WHY IT IS NOT ARMED IN `process()`, which would be the only place
// that guarantees the audio thread: the MXCSR belongs to the THREAD, not to
// the instance, so re-arming per block imposes FTZ/DAZ on the HOST's thread —
// stomping on the rest of the graph and on Ardour's explicit "Denormal
// Protection" preference. Measured: it buys nothing (+1,95 %, 13 of 21
// pairs, i.e. noise) in a scenario that additionally requires the host to
// inject denormals. It stays where it is: cheap, and it steps on nobody.
static inline void arm_denormal_flush()
{
#ifdef NLSC_HAVE_X86_DENORMAL_CTRL
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif
}

// THE DISPATCH, and it happens ONCE PER INSTANCE.
//
// Checking `__AVX2__` would be wrong: that is what the COMPILER supported,
// not what the machine running has. `__builtin_cpu_supports` reads CPUID at
// run time.
// BOTH are required: AVX2 and FMA3 arrived together with Haswell, but they
// are queried separately and a hypervisor can mask just one.
bool has_v3()
{
    // THE NEGATIVE LEG OF THIS MECHANISM — without it, it does not exist.
    //
    // On a modern machine `has_v3()` is ALWAYS true, so the BASELINE path is
    // exercised by nobody: it could be broken and no gate would see it — and
    // it is exactly the path that will run on pre-2013 CPUs, where we cannot
    // test.
    //
    // `NLSC_FORCE_BASE=1` in the environment forces the baseline engine.
    // HARNESS-ONLY: the installed plugin never sees that variable in a
    // normal session, and if it is set, the harness says so, not the audio.
    if (const char* f = std::getenv("NLSC_FORCE_BASE"))
        if (f[0] == '1') return false;
#if defined(__x86_64__)
    return __builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma");
#else
    return false;
#endif
}

} // namespace nlsc

// ---------------------------------------------------------------------------
// LV2 life cycle
// ---------------------------------------------------------------------------

static LV2_Handle instantiate(const LV2_Descriptor*, double rate,
                              const char*, const LV2_Feature* const*)
{
    nlsc::arm_denormal_flush();
    nlsc::ICore* self = nlsc::has_v3() ? nlsc::make_v3() : nlsc::make_base();
    if (self) self->prepare(rate);
    return static_cast<LV2_Handle>(self);   // nullptr => the host discards the instance
}

static void connect_port(LV2_Handle instance, uint32_t port, void* data)
{
    static_cast<nlsc::ICore*>(instance)->connect_port(port, data);
}

static void activate(LV2_Handle instance)
{
    // activate() may run on a different thread than instantiate(), and the
    // denormal control is PER THREAD: re-arming here is not redundant.
    nlsc::arm_denormal_flush();
    static_cast<nlsc::ICore*>(instance)->reset();
}

static void run(LV2_Handle instance, uint32_t n_samples)
{
    // THE ONLY dispatch indirection: 375 calls per second at 128 samples
    // and 48 kHz. The per-SAMPLE attempt (192 000/s) cost more than it
    // bought.
    static_cast<nlsc::ICore*>(instance)->process(n_samples);
}

static void deactivate(LV2_Handle) {}

static void cleanup(LV2_Handle instance)
{
    delete static_cast<nlsc::ICore*>(instance);
}

static const void* extension_data(const char*) { return nullptr; }

static const LV2_Descriptor descriptor = {
    "https://nylarea.com/plugins/valvehowler",
    instantiate, connect_port, activate, run, deactivate, cleanup, extension_data
};

extern "C" LV2_SYMBOL_EXPORT
const LV2_Descriptor* lv2_descriptor(uint32_t index)
{
    return (index == 0) ? &descriptor : nullptr;
}
