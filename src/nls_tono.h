// nls_tono.h — stage 3 (tone) with the knob as a parameter, not a constant.
//
// The coefficients come from `nls_tono_coef.h`, which gives them as explicit
// functions of the knob (exact SymPy algebra over the netlist, NJM4558 opamp
// included), so the whole travel of the tone knob is modelled.
//
// All the cost lives in `prepare()`. Per sample this is an order-4 IIR in
// transposed direct form II, the cost of two biquad sections; the knob costs
// no CPU.
//
// Direct form rather than biquad sections: building SOS would require
// solving the order-4 polynomial's roots in `prepare()`, and direct form and
// SOS agree to −163 to −218 dB across the five knob positions and both rates;
// the poles are not close enough for the usual numerical argument against
// the direct form to apply.
//
// The discretisation is the standard bilinear, no prewarp, factor 2*fs, the
// same convention as the fixed coefficient sets. At tone=0,5 the error
// against the exact analog transfer is −64,9 dB at 192 kHz and −77,0 dB at
// 384 kHz: that is the bilinear's own error at those rates.
#pragma once

#include "nls_filtro_param.h"
#include "nls_tono_coef.h"

namespace nlsc {

class VariableTone {
public:
    // `tone` is the physical parameter (the pot's track fraction), not the
    // knob position: the row's knob law (`taper_w` or `taper_even`) is
    // applied before this, in `nls_core.h` (`apply_knobs`).
    void prepare(double fs, double tone, bool reinit = true)
    {
        f_.prepare(fs, tone, tone::kB, tone::kA, tone::kScale, reinit);
    }
    void reset() { f_.reset(); }
    double process(double x) { return f_.process(x); }

private:
    // The knob degree comes from the header, not from a literal 2 here: if
    // the generator ever emitted degree 3, a hard-wired 2 would read the
    // table misaligned and nothing would fail visibly.
    ParamFilter<tone::kDegreeNum, tone::kDegreeDen, tone::kKnobDeg> f_;
};

// What the rail injects into `n14` (through `R8` towards `n9`), also a
// function of the knob, because the rail enters before the tone network and
// the network filters it.
class RailN14Variable {
public:
    void prepare(double fs, double tone, bool reinit = true)
    {
        f_.prepare(fs, tone, tone::kBr, tone::kAr, tone::kScale, reinit);
    }
    void reset() { f_.reset(); }
    double process(double x) { return f_.process(x); }

private:
    ParamFilter<tone::kDegreeNumR, tone::kDegreeDenR, tone::kKnobDegR> f_;
};

// The current `n7` feeds into the rail (`VRA`), also a function of the
// knob. `VRA` is physically `(n9/n7)/R8`, and `n9` is loaded by the tone
// network, so it depends on `tone`. It is the same network as
// `RailN14Variable` in the opposite direction: not what the rail does to a
// node, but what the node feeds the rail.
class RailVraVariable {
public:
    void prepare(double fs, double tone, bool reinit = true)
    {
        f_.prepare(fs, tone, tone::kBv, tone::kAv, tone::kScale, reinit);
    }
    void reset() { f_.reset(); }
    double process(double x) { return f_.process(x); }

private:
    ParamFilter<tone::kDegreeNumV, tone::kDegreeDenV, tone::kKnobDegV> f_;
};

} // namespace nlsc
