// nls_iface.h — the ABI shared by the LV2 scaffolding and the two cores.
//
// This file is never wrapped in an ISA namespace. The DSP core compiles twice
// — baseline and avx2/fma — inside two different namespaces, so the linker
// cannot merge the two copies. Everything the LV2 scaffolding needs must live
// outside those namespaces, or connect_port would refer to
// `isa_v3::nlsc::PORT_IN`, which is a different type.
//
// Nothing from the DSP belongs here: no templates, no `inline`, no engine
// types. It is a call interface whose cost is one indirection per block
// (375/s at 128 samples and 48 kHz); dispatching per sample would cost more
// than the vector code gains.
#pragma once

#include <cstdint>

namespace nlsc {

// The port indices are fixed, together with their `lv2:symbol`s
// (`lv2/frozen_symbols.txt`): a host stores automation by symbol, and a
// renumbering breaks saved sessions. This is the single definition: the GUI
// and the C++ test programs include this file, and the Makefile extracts the
// indices for the C test programs with a text match on the `PORT_X = N`
// lines, so keep one enumerator per line in that form.
enum PortIndex : uint32_t {
    PORT_IN           = 0,
    PORT_OUT          = 1,
    PORT_DRIVE        = 2,
    PORT_TONE         = 3,
    PORT_LEVEL        = 4,
    PORT_LATENCY      = 5,   // output: the host needs it to compensate
    // Both of the following act on the output.
    PORT_ENABLED      = 6,   // lv2:enabled — the footswitch
    PORT_VARIANT      = 7,   // which circuit and tone law: see `nls_variantes.h`
    PORT_COUNT        = 8
};

// The core, seen from outside. One entry point per thing the LV2 life cycle
// needs to do.
struct ICore {
    virtual ~ICore() = default;
    virtual void prepare(double rate)                  = 0;
    virtual void reset()                               = 0;
    virtual void process(uint32_t n_samples)           = 0;
    virtual void connect_port(uint32_t port, void* data) = 0;
};

// The two factories, one per translation unit. They return nullptr when out
// of memory: the host discards the instance, which is instantiate()'s
// contract.
ICore* make_base();
ICore* make_v3();

// Reads CPUID. Called once, in instantiate().
bool has_v3();

} // namespace nlsc
