// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) The NL Sounds contributors
// nls_filtro_param.h — an IIR whose coefficients are a function of a knob.
//
// The machinery shared by `nls_tono.h` (stage 3) and `nls_nivel.h`
// (stage 4). The delicate part is the bilinear expansion, and it lives in one
// place so a sign cannot differ between two copies.
//
// The tone and level coefficient tables give `b(s)` and `a(s)` with each
// coefficient as a polynomial in the knob, with `s` in units of `kScale` and
// a monic denominator. Here:
//
//   1. the knob polynomial is evaluated by Horner        -> b(s), a(s)
//   2. the bilinear  s = c*(1-z^-1)/(1+z^-1) is applied  -> B(z), A(z)
//   3. everything is normalised by A[0]
//
// All of that is `prepare()`. Per sample this is an IIR in transposed
// direct form II, the same cost as fixed biquad sections.
//
// Direct form rather than biquad sections: building SOS would require solving
// the polynomial's roots in `prepare()`, and direct form and SOS agree to
// −163 to −218 dB at every knob position and all three rates, so the usual
// numerical argument against the direct form does not apply here.
#pragma once

namespace nlsc {

// NB = numerator degree in s · NA = denominator degree in s
// NM = degree of the knob polynomials
template <int NB, int NA, int NM>
class ParamFilter {
public:
    // `table_b` is [NB+1][NM+1] and `table_a` is [NA+1][NM+1], from highest
    // to lowest power of s, and within each row from highest to lowest power
    // of the knob.
    // `reinit` — re-tuning is not restarting.
    // Pots are netlist resistors: moving one forces the coefficients to be
    // rebuilt, but not the state to be dropped. With `reinit = false` the
    // filter keeps the samples it had, which is what the real circuit does;
    // zeroing the state on every knob move would click.
    // `reinit = true` is the default: on a rate change the stored state
    // belongs to another discretisation and is invalid.
    void prepare(double fs, double knob,
                 const double (&table_b)[NB + 1][NM + 1],
                 const double (&table_a)[NA + 1][NM + 1],
                 double scale, bool reinit = true)
    {
        double bs[NB + 1], as[NA + 1];
        for (int k = 0; k <= NB; ++k) bs[k] = horner(table_b[k], knob);
        for (int k = 0; k <= NA; ++k) as[k] = horner(table_a[k], knob);

        // The bilinear multiplies numerator and denominator by (1+z^-1)^NA,
        // so both end up of degree NA in z^-1.
        const double c = 2.0 * fs / scale;
        double Bz[NA + 1] = {0}, Az[NA + 1] = {0};
        double ck = 1.0;
        for (int k = 0; k <= NA; ++k) {
            // (1 - z^-1)^k * (1 + z^-1)^(NA-k), expanded by convolution.
            // Computed rather than tabulated: a wrong sign in a hand-written
            // table would break nothing visibly, it would just detune.
            double term[NA + 1] = {0};
            term[0] = 1.0;
            int degree = 0;
            for (int j = 0; j < k; ++j)         degree = conv(term, degree, -1.0);
            for (int j = 0; j < NA - k; ++j)    degree = conv(term, degree, +1.0);
            // `bs`/`as` run from highest to lowest power of s: the s^k entry
            // sits at position (degree - k).
            const double bk = (k <= NB) ? bs[NB - k] : 0.0;
            const double ak = as[NA - k];
            for (int i = 0; i <= NA; ++i) {
                Bz[i] += bk * ck * term[i];
                Az[i] += ak * ck * term[i];
            }
            ck *= c;
        }
        const double g = 1.0 / Az[0];
        for (int i = 0; i <= NA; ++i) { b_[i] = Bz[i] * g; a_[i] = Az[i] * g; }
        if (reinit) reset();
    }

    void reset() { for (int i = 0; i < NA; ++i) z_[i] = 0.0; }

    // The steady state for a constant input `x`. Without this, starting
    // with a non-zero rest injects a transient of hundreds of milliseconds,
    // visible whole in the band below 35 Hz.
    // Transposed direct form II: in steady state `y = H(1)*x`, and the
    // states fall out backwards from their own recurrence.
    double preset_dc(double x)
    {
        double sb = 0.0, sa = 0.0;
        for (int i = 0; i <= NA; ++i) { sb += b_[i]; sa += a_[i]; }
        const double y = x * sb / sa;
        z_[NA - 1] = b_[NA] * x - a_[NA] * y;
        for (int i = NA - 2; i >= 0; --i)
            z_[i] = b_[i + 1] * x - a_[i + 1] * y + z_[i + 1];
        return y;
    }

    // The affine split of `process`, so a loop can be closed.
    //
    // A transposed direct form II gives `y = b0*x + z0`: the output is
    // affine in this sample's input and everything else already sits in the
    // state. When this filter's input depends, in turn, on its own output
    // (the `Z(s)·ib` case in stage 4), that turns the loop into an exact
    // scalar equation instead of forcing a one-sample delay.
    //
    // Neither of the two advances the state: whoever solves the loop calls
    // `process()` afterwards with the right input, exactly once. Calling
    // twice would advance the filter two samples.
    double direct_gain() const { return b_[0]; }
    double state() const { return z_[0]; }

    double process(double x)
    {
        const double y = b_[0] * x + z_[0];
        for (int i = 0; i < NA - 1; ++i)
            z_[i] = b_[i + 1] * x - a_[i + 1] * y + z_[i + 1];
        z_[NA - 1] = b_[NA] * x - a_[NA] * y;
        return y;
    }

private:
    static double horner(const double (&c)[NM + 1], double x)
    {
        double v = c[0];
        for (int i = 1; i <= NM; ++i) v = v * x + c[i];
        return v;
    }

    // Multiplies `p` (degree `g`, in z^-1) by (1 + sign*z^-1). Returns the
    // new degree. In place and back to front, so it does not overwrite what
    // is still needed.
    static int conv(double (&p)[NA + 1], int g, double sign_)
    {
        p[g + 1] = sign_ * p[g];
        for (int i = g; i >= 1; --i) p[i] = p[i] + sign_ * p[i - 1];
        return g + 1;
    }

    double b_[NA + 1] = {0}, a_[NA + 1] = {0}, z_[NA] = {0};
};

// A fixed filter that discretises itself in `prepare()`.
//
// It is `ParamFilter` with degree zero in the knob: the `s` coefficients are
// constants and the bilinear runs at the real rate, so the cascade's fixed
// blocks are not tied to one internal rate.
template <int NB, int NA>
class FixedFilter {
public:
    void prepare(double fs, const double (&tb)[NB + 1][1],
                 const double (&ta)[NA + 1][1], double scale, bool reinit = true)
    {
        f_.prepare(fs, 0.0, tb, ta, scale, reinit);
    }
    void reset() { f_.reset(); }
    double preset_dc(double x) { return f_.preset_dc(x); }
    double process(double x) { return f_.process(x); }

private:
    ParamFilter<NB, NA, 0> f_;
};

} // namespace nlsc
