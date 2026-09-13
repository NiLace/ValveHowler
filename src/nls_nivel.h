// nls_nivel.h — the LEVEL network (E4G1 = n19/n14) with the knob as a
// PARAMETER.
//
// WHAT IT REPLACES
// ----------------
// The fixed `kE4G1` from `nls_e234_coef*.h`, discretised at lvl=0. Measured:
// turning the volume down with that cost −6,3 dB of ANMR at lvl=0,25, −2,5
// at 0,50 and +3,4 at 0,75. The pedal's volume knob made the distance to the
// circuit audible, and more so the lower it went.
//
// Q2's base enters as `Rb || Cb`, fitted ONCE against the four positions
// simultaneously — two numbers for the whole travel, not a table. That works
// because the pot sits BEFORE the base, so the load Q2 presents does not
// depend on the knob. Residual against ngspice: −62,8 to −63,5 dB — passes
// the PORT yardstick.
//
// AND THE REST POINT IS NOT NEEDED — MEASURED, not assumed. The first
// version of this file warned that Q2's rest point was pending "because the
// base current drops across the pot". FALSE: an `.op` at five `lvl`
// positions gives `n19`, `n17`, `n14` and `vr` IDENTICAL to the seventh
// digit. The reason is the question that decides this class of defect —
// WHICH pot carries DC — and this one does not: C8 blocks it, so Q2's base
// current arrives through R12 from the rail. The DRIVE pot did carry DC
// (RGAIN has no series capacitor), which is why that one did move the rest.
//
// AND THE MISSING PIECE: `RailN19Variable`. The residual that remained —
// absolute, constant while the signal drops 4,2x, living at 1-10 Hz — was
// NOT Q2's waveshaper (ruled out by measurement, −85 dB) nor the rail's
// value (ruled out by upper bound). It was the OTHER transfer of this same
// network: `v(n19)/v(VR)`, which used to ship as a FIXED filter measured at
// lvl=0 and which changes 58x along the pot.
// => The signature explains itself: the rail enters through `R12`, AFTER the
// pot, so its error does not attenuate with the knob (the parallel-path
// version of a knob moving corners). Upper bound of fixing it, measured
// BEFORE writing it: giving the stage the oracle's `n19`, the output goes
// from −40,3 to −91,2 dB.
#pragma once

#include "nls_filtro_param.h"
#include "nls_level_coef.h"
#include "nls_level_coef_v9ri.h"   // the variant's bank, see `prepare`

namespace nlsc {

// THE LEVEL NETWORK WITH Q2'S BASE OPEN.
//
// The one below (`VariableLevel`) carries the base as a LINEAR admittance
// with FITTED `rpi`, `beta` and `Cb`, and that is its ceiling: measured
// against the oracle's `n19` with the REAL signal it tops out at −63,4 dB,
// because under excitation `rpi` depends on the current and the base stops
// being an impedance.
//
// This one fits nothing. The network is PASSIVE and the solution is exact
// superposition:
//
//     n19 = H_sen(s;lvl)·n14  +  H_rail(s;lvl)·vr  +  Z(s;lvl)·ib
//
// The base current `ib` is computed per sample by stage 4's subsystem, so
// there is nothing to estimate. Measured: −80,1 dB with `ib` delayed one
// sample (−87,1 with the loop solved).
//
// Only valid where `ib` EXISTS, i.e. with the subsystem. With the
// tabulated waveshaper that current does not exist and `VariableLevel` is
// used instead.
class OpenBaseLevel {
public:
    static_assert(level::kGradoNumO == level_v9ri::kGradoNumO &&
                  level::kGradoDenO == level_v9ri::kGradoDenO &&
                  level::kKnobDegO == level_v9ri::kKnobDegO &&
                  level::kGradoNumZ == level_v9ri::kGradoNumZ,
                  "both open-base banks must share the same degrees");

    // `J` is the variant's bank (`nls_juegos.h`). The template instantiates
    // in the ONE dispatch, in `Cascade4::prepare`: there is no `if` here that
    // could fall through to the wrong circuit.
    template <class J>
    void prepare_bank(double fs, double lvl, bool reinit)
    {
        sen_.prepare(fs, lvl, J::nBo, J::nAo, J::ScaleN, reinit);
        z_.prepare(fs, lvl, J::nBz, J::nAo, J::ScaleN, reinit);
        rail_.prepare(fs, lvl, J::nBro, J::nAo, J::ScaleN, reinit);
    }

    void reset() { sen_.reset(); rail_.reset(); z_.reset(); }
    double preset_dc(double x) { z_.preset_dc(0.0); rail_.preset_dc(0.0); return sen_.preset_dc(x); }

    // The THREE terms come from the SAME matrix: that is what guarantees they
    // describe one network. `dev_n14` is the input's deviation; `dvr` the
    // rail's, UNFILTERED; `dib` the base current's against its rest.
    double process(double dev_n14, double dvr, double dib)
    {
        return sen_.process(dev_n14) + rail_.process(dvr) + z_.process(dib);
    }

    // THE LOOP, CLOSED INSTEAD OF DELAYED.
    //
    // `ib` is THIS sample's base current, so it depends on `n19`, which is
    // what this method computes: the loop used to close with the PREVIOUS
    // sample's `ib`. The written justification was a loop gain of 0,0009 —
    // measured at `lvl = 0`, while `|Z(s;lvl)|` is 24x larger at
    // mid-travel with the signal at half. Measured: that version loses in 27
    // of 36 points of the knob grid.
    //
    // And it closes EXACTLY and almost free because the output is AFFINE
    // in `ib`:
    //
    //     n19 = [H_sen·n14 + H_rail·vr + Z's state]  +  b0(Z)·ib
    //            \_______________ a0 _____________/     \_ kz _/
    //
    // The two paths that do NOT depend on `ib` advance here, and what
    // remains is a straight line that Q2's Newton solves as a second
    // equation — with the SAME `bjt()` call per iteration, which is where
    // the exponentials are. The overhead is a 2x2, not one more evaluation.
    //
    // `close_ib` advances `z_` ONCE, with the current already solved.
    // Without that call Z's state freezes and the term vanishes.
    double parte_sin_ib(double dev_n14, double dvr)
    {
        return sen_.process(dev_n14) + rail_.process(dvr) + z_.estado();
    }
    double ib_gain() const { return z_.direct_gain(); }
    void close_ib(double dib) { (void)z_.process(dib); }

private:
    ParamFilter<level::kGradoNumO, level::kGradoDenO, level::kKnobDegO> sen_;
    ParamFilter<level::kGradoNumZ,  level::kGradoDenO, level::kKnobDegO> z_;
    ParamFilter<level::kGradoNumRo, level::kGradoDenO, level::kKnobDegO> rail_;
};

class VariableLevel {
public:
    // THE VARIANT: `R14` and `R15` hang off Q2's emitter (`Ze(s)` is R13
    // in parallel with the R14 + C9 + R15 branch), so they enter `Ze(s)` and
    // with it the load the base presents to the pot. The emitter itself,
    // R13, is 10 k in both models. Measured in ngspice: `n19/n14`
    // separates by +0,07 dB flat and +0,28 dB at 10 Hz between the two
    // variants. It is only read in `prepare()`, so choosing the table
    // suffices.
    static_assert(level::kGradoNum == level_v9ri::kGradoNum &&
                  level::kGradoDen == level_v9ri::kGradoDen &&
                  level::kKnobDeg == level_v9ri::kKnobDeg,
                  "both level banks must share the same degrees");

    template <class J>
    void prepare_bank(double fs, double lvl, bool reinit)
    {
        f_.prepare(fs, lvl, J::nB, J::nA, J::ScaleN, reinit);
    }
    void reset() { f_.reset(); }
    double process(double x) { return f_.process(x); }

private:
    ParamFilter<level::kGradoNum, level::kGradoDen, level::kKnobDeg> f_;
};

// What the RAIL injects into `n19` (through `R12`), also as a function of
// the knob. Same network, same fitted base, different right-hand side:
// exact superposition.
// Replaces `rail::kN19`, a fixed lvl=0 filter.
class RailN19Variable {
public:
    static_assert(level::kGradoNumR == level_v9ri::kGradoNumR &&
                  level::kGradoDenR == level_v9ri::kGradoDenR &&
                  level::kKnobDegR == level_v9ri::kKnobDegR,
                  "both level-rail banks must share the same degrees");

    template <class J>
    void prepare_bank(double fs, double lvl, bool reinit)
    {
        f_.prepare(fs, lvl, J::nBr, J::nAr, J::ScaleN, reinit);
    }
    void reset() { f_.reset(); }
    double process(double x) { return f_.process(x); }

private:
    ParamFilter<level::kGradoNumR, level::kGradoDenR, level::kKnobDegR> f_;
};

} // namespace nlsc
