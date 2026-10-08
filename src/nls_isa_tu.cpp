// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// nls_isa_tu.cpp — the core, compiled twice with different flags.
//
// Built once per instruction set:
//
//     -DNLSC_ISA_NS=isa_base -DNLSC_ISA_FACTORY=make_base   $(ARCH)
//     -DNLSC_ISA_NS=isa_v3   -DNLSC_ISA_FACTORY=make_v3     $(ARCH_V3)
//
// What keeps the linker from merging the two copies is the namespace, not
// visibility: `nlsc::Plugin::process` becomes `isa_base::nlsc::Plugin::process`
// and `isa_v3::nlsc::Plugin::process`, which are distinct symbols. Without it
// the ODR is violated silently and both routes run the same version.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <new>
#include <utility>

// System headers are included here, outside the ISA namespace: included
// inside it, `std::` would land in the namespace and nothing would compile.
#if defined(__SSE2__) || defined(__x86_64__)
#  include <pmmintrin.h>
#  include <xmmintrin.h>
#  define NLSC_HAVE_X86_DENORMAL_CTRL 1
#endif

#include "nls_iface.h"          // the ABI, in ::nlsc, never namespaced

namespace NLSC_ISA_NS {
#include "nls_core.h"
}

// The object is built inside the ISA namespace, in a function that is not
// inlined: the constructor carries vector code when this unit is compiled for
// v3, and it has to stay within the v3 copy's own symbols. The check on the
// built binary requires every AVX2/FMA instruction to lie inside an `isa_v3`
// symbol; inlined into the ::nlsc factory, that code would sit outside every
// clone. It would still be safe at run time, since the factory is only called
// after `has_v3()`, but it would escape that check.
namespace NLSC_ISA_NS { namespace nlsc {
__attribute__((noinline)) ::nlsc::ICore* create()
{
    return new (std::nothrow) Plugin();
}
} }

namespace nlsc {
ICore* NLSC_ISA_FACTORY()
{
    return NLSC_ISA_NS::nlsc::create();
}
}
