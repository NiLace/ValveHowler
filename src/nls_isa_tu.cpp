// nls_isa_tu.cpp — the CORE, compiled TWICE with different flags.
//
// Built once per instruction set:
//
//     -DNLSC_ISA_NS=isa_base -DNLSC_ISA_FACTORY=make_base   $(ARCH)
//     -DNLSC_ISA_NS=isa_v3   -DNLSC_ISA_FACTORY=make_v3     $(ARCH_V3)
//
// What keeps the linker from merging the two copies is the NAMESPACE, not
// visibility: `dk::Engine::newton` becomes `isa_base::nlsc::dk::Engine::newton`
// and `isa_v3::nlsc::dk::Engine::newton`, which are distinct symbols. Without
// this the ODR is violated in silence: it compiles, it links, and BOTH routes
// end up running the SAME version.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <new>
#include <utility>
#include <vector>

// SYSTEM HEADERS GO UP HERE, OUTSIDE. If one lands inside the ISA
// namespace, `std::` ends up inside it and nothing compiles. The list comes
// from a grep over every `src/nls_*.h`; if another appears, the error is
// immediate and clear.
#if defined(__SSE2__) || defined(__x86_64__)
#  include <pmmintrin.h>
#  include <xmmintrin.h>
#  define NLSC_HAVE_X86_DENORMAL_CTRL 1
#endif

#include "nls_iface.h"          // the ABI, in ::nlsc, never namespaced

namespace NLSC_ISA_NS {
#include "nls_core.h"
}

namespace nlsc {
ICore* NLSC_ISA_FACTORY()
{
    return new (std::nothrow) NLSC_ISA_NS::nlsc::Plugin();
}
}
