// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// nls_nivel.h — the level network (E4G1 = n19/n14) with the knob as a
// parameter.
//
// The volume pot changes this transfer along its travel, so it is a
// parametric filter in `lvl` rather than a fixed one.
//
// Q2's base enters as `Rb || Cb`, fitted once against four knob positions
// simultaneously: two numbers for the whole travel, not a table. That works
// because the pot sits before the base, so the load Q2 presents does not
// depend on the knob. Residual against ngspice: −62,8 to −63,5 dB.
//
// Q2's rest point does not depend on the knob: this pot carries no DC (C8
// blocks it), so Q2's base current arrives through R12 from the rail, and an
// `.op` at five `lvl` positions gives `n19`, `n17`, `n14` and `vr` identical
// to the seventh digit. The drive pot, by contrast, does carry DC (RGAIN has
// no series capacitor) and does move the rest point.
//
// The rail's transfer into this network, `v(n19)/v(VR)`, also changes with the
// knob (58x along the pot), and is `RailN19Variable`. The rail enters through
// `R12`, after the pot, so its contribution does not attenuate with the knob.
#pragma once

#include "nls_filtro_param.h"
#include "nls_level_coef.h"
#include "nls_level_coef_v9ri.h"   // the variant's bank, see `prepare`

namespace nlsc {

// The level network with Q2's base open.
//
// `VariableLevel` below carries the base as a linear admittance with fitted
// `rpi`, `beta` and `Cb`; under excitation `rpi` depends on the current and
// the base stops being an impedance, which bounds its accuracy (~−63 dB).
//
// This one fits nothing. The network is passive and the solution is exact
// superposition:
//
//     n19 = H_sen(s;lvl)·n14  +  H_rail(s;lvl)·vr  +  Z(s;lvl)·ib
//
// The base current `ib` is computed per sample by stage 4's subsystem, so
// there is nothing to estimate (−87 dB against ngspice with the loop solved).
//
// Only valid where `ib` exists, i.e. with the subsystem. With the
// tabulated waveshaper that current does not exist and `VariableLevel` is
// used instead.
class OpenBaseLevel {
public:
    static_assert(level::kDegreeNumO == level_v9ri::kDegreeNumO &&
                  level::kDegreeDenO == level_v9ri::kDegreeDenO &&
                  level::kKnobDegO == level_v9ri::kKnobDegO &&
                  level::kDegreeNumZ == level_v9ri::kDegreeNumZ,
                  "both open-base banks must share the same degrees");

    // `J` is the variant's bank (`nls_juegos.h`). The template instantiates
    // in the one dispatch, in `Cascade4::prepare`: there is no `if` here that
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

    // The three terms come from the same matrix: that is what guarantees they
    // describe one network. `dev_n14` is the input's deviation; `dvr` the
    // rail's, unfiltered; `dib` the base current's against its rest.
    double process(double dev_n14, double dvr, double dib)
    {
        return sen_.process(dev_n14) + rail_.process(dvr) + z_.process(dib);
    }

    // The loop, closed within the sample.
    //
    // `ib` is this sample's base current, so it depends on `n19`, which is
    // what this method computes. `|Z(s;lvl)|` grows ~24x from `lvl = 0` to
    // mid-travel, so a one-sample delay in this loop is not negligible.
    //
    // It closes exactly and almost free because the output is affine in `ib`:
    //
    //     n19 = [H_sen·n14 + H_rail·vr + Z's state]  +  b0(Z)·ib
    //            \_______________ a0 _____________/     \_ kz _/
    //
    // The two paths that do not depend on `ib` advance here, and what
    // remains is a straight line that Q2's Newton solves as a second
    // equation, with the same `bjt()` call per iteration (where the
    // exponentials are). The overhead is a 2x2, not one more evaluation.
    //
    // `close_ib` advances `z_` once, with the current already solved.
    // Without that call Z's state freezes and the term vanishes.
    double part_without_ib(double dev_n14, double dvr)
    {
        return sen_.process(dev_n14) + rail_.process(dvr) + z_.state();
    }
    double ib_gain() const { return z_.direct_gain(); }
    void close_ib(double dib) { (void)z_.process(dib); }

private:
    ParamFilter<level::kDegreeNumO, level::kDegreeDenO, level::kKnobDegO> sen_;
    ParamFilter<level::kDegreeNumZ,  level::kDegreeDenO, level::kKnobDegO> z_;
    ParamFilter<level::kDegreeNumRo, level::kDegreeDenO, level::kKnobDegO> rail_;
};

class VariableLevel {
public:
    // The variant: `R14` and `R15` hang off Q2's emitter (`Ze(s)` is R13
    // in parallel with the R14 + C9 + R15 branch), so they enter `Ze(s)` and
    // with it the load the base presents to the pot. The emitter itself,
    // R13, is 10 k in both models. In ngspice `n19/n14` separates by
    // +0,07 dB flat and +0,28 dB at 10 Hz between the two variants. It is
    // only read in `prepare()`, so choosing the table suffices.
    static_assert(level::kDegreeNum == level_v9ri::kDegreeNum &&
                  level::kDegreeDen == level_v9ri::kDegreeDen &&
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
    ParamFilter<level::kDegreeNum, level::kDegreeDen, level::kKnobDeg> f_;
};

// What the rail injects into `n19` (through `R12`), also as a function of
// the knob. Same network, same fitted base, different right-hand side:
// exact superposition.
class RailN19Variable {
public:
    static_assert(level::kDegreeNumR == level_v9ri::kDegreeNumR &&
                  level::kDegreeDenR == level_v9ri::kDegreeDenR &&
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
    ParamFilter<level::kDegreeNumR, level::kDegreeDenR, level::kKnobDegR> f_;
};

} // namespace nlsc
