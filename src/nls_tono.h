// nls_tono.h — stage 3 (TONE) with the knob as a PARAMETER, not a constant.
//
// WHAT IT REPLACES, AND WHY
// -------------------------
// `Stage3` used to be a fixed-coefficient cascade (the `kE3` sets),
// discretised for tone=0,5. Measured: moving the knob to the ends with that
// costs up to +17,2 dB of ANMR — loudly audible. Here the coefficients come
// from `nls_tono_coef.h`, which gives them as EXPLICIT functions of the knob
// (exact SymPy algebra over the netlist, NJM4558 opamp included).
//
// ALL THE COST LIVES IN `prepare()`. Per sample this is an order-4 IIR in
// transposed direct form II: exactly what the two biquad sections cost
// before. The knob costs no CPU.
//
// WHY DIRECT FORM AND NOT BIQUAD SECTIONS
// ---------------------------------------
// Because building SOS would require solving the order-4 polynomial's roots
// in `prepare()`, and that is delicate code that must earn its place. It was
// MEASURED before deciding: direct form vs SOS gives −163 to −218 dB across
// the five knob positions and both rates — the usual numerical argument
// against the direct form does not apply here; the poles are not close
// enough to matter.
//
// THE DISCRETISATION
// ------------------
// Standard bilinear, no prewarp, factor 2*fs — the SAME convention as
// `gen_coef_etapa1.a_sos`, which produced the sets already in the repo.
// Verified to match them: at tone=0,5 the error against the exact analog
// transfer is −64,9 dB at 192 kHz and −77,0 dB at 384 kHz, the SAME figures
// `nls_e234_coef_192k.h` and `nls_e234_coef.h` give. So this does not move
// the nominal point: what it adds is the rest of the travel.
//
// That −64,9 dB at 4x is the BILINEAR's error, shared by both. It is not
// a loss introduced by this change; it is a figure the previous pipeline
// never measured.
#pragma once

#include "nls_filtro_param.h"
#include "nls_tono_coef.h"

namespace nlsc {

class VariableTone {
public:
    // `tone` is the PHYSICAL parameter (the pot's track fraction), not the
    // knob position: the knob's law (the W taper) lives in the LV2 wrapper
    // and is validated separately with `make mandos`. A null runs on the
    // physical parameter, which is why it is blind to the law.
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
    ParamFilter<tone::kGradoNum, tone::kGradoDen, tone::kKnobDeg> f_;
};

// What the RAIL injects into `n14` (through `R8` towards `n9`), also a
// function of the knob — because the rail enters BEFORE the tone network and
// the network filters it.
// Replaces `rail::kN14`, a fixed tone=0,5 filter that was wrong by
// −13,6 dB at tone=0 and −7,4 dB at tone=1.
class RailN14Variable {
public:
    void prepare(double fs, double tone, bool reinit = true)
    {
        f_.prepare(fs, tone, tone::kBr, tone::kAr, tone::kScale, reinit);
    }
    void reset() { f_.reset(); }
    double process(double x) { return f_.process(x); }

private:
    ParamFilter<tone::kGradoNumR, tone::kGradoDenR, tone::kKnobDegR> f_;
};

// THE CURRENT `n7` FEEDS INTO THE RAIL (`VRA`), also a function of the
// knob. `VRA` is physically `(n9/n7)/R8`, and `n9` is loaded by the TONE
// NETWORK => it depends on `tone`. The cascade used to carry it as a bank
// CONSTANT frozen at tone=0,5, while the tone itself was parametrised.
//
// Third case of the same family in this repo: `VR -> n19` frozen at lvl=0
// (+19,5 dB when fixed), `VR -> n14` frozen at tone=0,5 (−13,6 dB of error
// at the end stop), and this one — the SAME network, in the opposite
// direction: not what the rail does to a node, but what the node feeds the
// rail.
//
// BOUNDED BEFORE IT WAS WRITTEN: giving the chain the ORACLE's rail, `n7`
// goes from −56,3 to −74,0 dB at tone=0 (+17,7) and only +1,7 at tone=0,5.
class RailVraVariable {
public:
    void prepare(double fs, double tone, bool reinit = true)
    {
        f_.prepare(fs, tone, tone::kBv, tone::kAv, tone::kScale, reinit);
    }
    void reset() { f_.reset(); }
    double process(double x) { return f_.process(x); }

private:
    ParamFilter<tone::kGradoNumV, tone::kGradoDenV, tone::kKnobDegV> f_;
};

} // namespace nlsc
