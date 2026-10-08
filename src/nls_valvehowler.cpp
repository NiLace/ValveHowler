// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// Valve Howler — an overdrive modelled from the circuit.
//
// This file is the LV2 scaffolding: descriptor, ports and the one-time choice
// between the two builds of the engine (generic x86-64, or AVX2/FMA when the
// CPU has it). The DSP lives in `nls_core.h`, compiled twice by
// `nls_isa_tu.cpp`. The oversampling factor is fixed at 4x (`kOsFactor`).
//
//   - Entry point wrapped in extern "C".
//   - Precision parametrised by typedef, not hard-wired.
//   - FTZ/DAZ for the duration of each DSP call, restored on exit.
//   - run() with no allocation, no exceptions, no locks.

// Engine macros (e.g. `NLSC_TONE_W`) take their defaults in the engine's own
// headers: this file does not include the core, so a default placed here
// would never reach it. `-D` on the command line reaches both units.

#include <cstdint>
#include <cstdlib>
#include <lv2.h>

// This block goes before everything else: `DenormalGuard` consults this
// macro, and placed after it the guard's body would compile empty with no
// warning.
#if defined(__SSE2__) || defined(__x86_64__)
#  include <pmmintrin.h>
#  include <xmmintrin.h>
#  define NLSC_HAVE_X86_DENORMAL_CTRL 1
#endif

// The core compiles twice in `nls_isa_tu.cpp` (baseline and avx2/fma), and
// here the only thing chosen is which one, once, in `instantiate()`.
// Everything below talks to `nlsc::ICore`, an interface with nothing of the
// engine inside.
#include "nls_iface.h"

namespace nlsc {


// Flush denormals to zero for the duration of a call, and give the thread
// back as it was found.
//
// The MXCSR belongs to the thread, not to the instance. Leaving it set would
// change the floating-point mode of whatever thread the host used for the
// call (its main or GUI thread, or a worker) for good, and would still not
// guarantee the audio thread that runs `run()`.
//
// So the mode is set on entry to each LV2 call that runs DSP and restored on
// exit. It costs two MXCSR accesses per block, and the host's own denormal
// setting is back in place the moment the call returns.
//
// Digital silence produces no denormals in this engine, because a physically
// modelled circuit settles at its rest point instead of decaying to zero. The
// guard is a net for what the host feeds in, not a cost fix.
class DenormalGuard {
public:
    DenormalGuard() noexcept
    {
#ifdef NLSC_HAVE_X86_DENORMAL_CTRL
        saved_ = _mm_getcsr();
        _mm_setcsr(saved_ | 0x8040u);   // FTZ (bit 15) | DAZ (bit 6)
#endif
    }
    ~DenormalGuard() noexcept
    {
#ifdef NLSC_HAVE_X86_DENORMAL_CTRL
        _mm_setcsr(saved_);
#endif
    }
    DenormalGuard(const DenormalGuard&) = delete;
    DenormalGuard& operator=(const DenormalGuard&) = delete;
private:
#ifdef NLSC_HAVE_X86_DENORMAL_CTRL
    unsigned int saved_ = 0;
#endif
};

// The ISA dispatch, once per instance.
//
// `__AVX2__` would say what the compiler supported, not what the running
// machine has; `__builtin_cpu_supports` reads CPUID at run time.
// Both are required: AVX2 and FMA3 arrived together with Haswell, but they
// are queried separately and a hypervisor can mask just one.
bool has_v3()
{
    // `NLSC_FORCE_BASE=1` in the environment forces the baseline engine, so
    // the baseline path can be exercised on a machine that has AVX2/FMA.
    // A normal session never sets it.
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
    nlsc::DenormalGuard fp;
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
    // The same mode as `run()` while the state is rebuilt, then restored.
    nlsc::DenormalGuard fp;
    static_cast<nlsc::ICore*>(instance)->reset();
}

static void run(LV2_Handle instance, uint32_t n_samples)
{
    // The only dispatch indirection, once per block: 375 calls per second
    // at 128 samples and 48 kHz.
    nlsc::DenormalGuard fp;
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
