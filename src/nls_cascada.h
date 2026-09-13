// The CASCADE of the four stages — the cheap architecture, complete.
//
// WHAT THIS IS AND IS NOT
// -----------------------
// This is the DEFAULT engine of the shipped plugin; the DK — which solves
// the ten nonlinear ports together and reads −70,8 dB against the circuit —
// ships alongside as the HQ mode. This one exists to answer with a
// measurement the question `docs/MODELAR_POR_BLOQUES.md` left open: what a
// block architecture truly loses with ALL FOUR stages, not one of four.
//
// THE DECOMPOSITION, AND WHY SPLITTING HERE IS LEGITIMATE
// -------------------------------------------------------
//     in --[1: Q1 buffer]--> n4 --[2: gain+diodes]--> n7
//        --[3: tone]--> n14 --[4: LEVEL + Q2 buffer]--> out
//
// The boundaries are NOT arbitrary: `n7` and `n14` are opamp outputs, and
// with the ideal opamp an opamp output is a VOLTAGE SOURCE. Whatever the
// next stage draws does not change its value => the split approximates
// nothing beyond the opamp itself. That is the difference with the
// discarded cascade, which split by the signal without justification.
//
// WHAT THIS ARCHITECTURE CANNOT REPRESENT, WRITTEN IN ADVANCE
// -----------------------------------------------------------
// 1. **The shared VR rail**, which moves 10,5 mV rms — more than the signal
//    node itself. Here VR is a constant.
// 2. **The backward coupling** between stages (0,1 % from Q1 and 1,1 % from
//    opamp B, measured in `MODELAR_POR_BLOQUES.md` §1bis).
// 3. **The real opamp**: finite gain, GBW pole, slew and clipping.
//
// Written before measuring so the result can contradict them (do not put
// the conclusion inside the instrument).
//
// INTER-STAGE INTERFACE: **DEVIATION**, NOT ABSOLUTE VOLTAGE
// -------------------------------------------------------------
// Each stage delivers the deviation from ITS output node's rest, and the
// next adds its input node's rest. In absolutes, the chain would carry the
// DC difference between the rests each stage was fitted at —
// `nls_stage1.h` brings its own from a drive-0,5 run while this measures at
// drive 1,0, and those 0,8 mV are BOOKKEEPING error, not architecture. What
// does survive (and must) is the rest displacement the signal itself
// produces, which is the effect under discussion.

#pragma once
#ifndef NLSC_E4_STARTUP_PROBE
#define NLSC_E4_STARTUP_PROBE 0
#endif
#ifndef NLSC_WS_RECENTER
#define NLSC_WS_RECENTER 0
#endif

#include <cmath>
#include <cstdio>
#include <cstdlib>

// RATE SELECTOR — only for the CASCADE bench, never for the plugin.
//
// The coefficients are discretised at one concrete rate. Measuring the
// cascade at 8x, 4x and 2x takes three sets, chosen here. With nothing
// defined the usual one is taken and the binary is BIT-IDENTICAL: a
// selector, not a change.
//   -DNLSC_CASC_KHZ=192  ->  4x      -DNLSC_CASC_KHZ=96  ->  2x
// RETIRED: `-DNLSC_CASC_DIG` (sets fitted in the DIGITAL domain, the
// bilinear-warping control). Product decision.
// That control ALREADY ANSWERED: the 96 kHz set redone without the
// bilinear gave −39,1 dB vs −38,9 => 0,2 dB, no contamination
// (`CASCADA_CUATRO_ETAPAS.md` §5).
// And it was broken in THREE places: it did not compile (five constants
// missing from the `e234*dig` sets), it could not regenerate (the digital
// `E4G1` fit comes out UNSTABLE), and stage 1's three `dig` sets carried
// poles OUTSIDE the circle (|z| = 1,0095 · 1,00004 · 1,0024).
// => If the question ever reopens, the generators keep their `--digital`
// flag: build the instrument anew — do not resurrect the corpse.
#if defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 96
#  include "nls_e234_coef_96k.h"
#elif defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 192
#  include "nls_e234_coef_192k.h"
#else
#  include "nls_e234_coef.h"
#endif
// THIS USED TO BE INCLUDED UNCONDITIONALLY — the other half of the 4x
// defect.
// `rn3_`/`rn14_`/`rn19_` are what the rail INJECTS into the stages — the
// return path of the same loop — and once existed only discretised at
// 384 kHz, so at 4x and 2x the cascade ran with the 8x time constants.
// Nobody saw it because the file compiles the same at any rate.
// TODAY the danger is gone by another route: `rn3_`/`rn14_`/`rn19_`
// prepare from the BANK's s-domain fits (`J::N3_B`, `tone::`, `level::`)
// and discretise at the REAL rate in `prepare()` — the code no longer reads
// any symbol from these headers' `rail::` namespace. The selector survives
// while harnesses sweep it; all it selects is a dead initialiser.
#if defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 96
#  include "nls_rail_n3_96k.h"
#elif defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 192
#  include "nls_rail_n3_192k.h"
#else
#  include "nls_rail_n3.h"
#endif
// Which variant uses the subsystem. These are flags so the four
// combinations can be measured with the usual binary, not knobs to taste:
// the split above is MEASURED.
#ifndef NLSC_E4_OPEN_BASE
#  define NLSC_E4_OPEN_BASE 1
#endif
// THE 808 SOLVES Q2 (`NLSC_E4_SUB_808`) — SHIPPED, and verified ON.
//
// What it does: stage 4 solves Q2's subsystem with a scalar Newton instead
// of the tabulated waveshaper, so Q2's base stops being a fitted impedance
// where the subsystem already computes the real current.
//
// What stands, measured:
//   · cost +0,24 core points (3,91 -> 4,15 %), 0 of 21 pairs, p = 9,54e-07 —
//     the earlier +50,75 % rejection predated the stage-4 cache, which
//     replaced exactly the Newton that was expensive;
//   · pinched-harmonic ANMR +0,20 dB, passing its criterion by 19,10;
//   · the additive `n19` term: 143,80 -> 27,41 µV (14,4 dB) — the finding
//     that uncovered all of this;
//   · better than the tabulated path in 85 of 100 grid points against the
//     DK (a valid RELATIVE comparison);
//   · worst BLOCK 11,11 -> 13,72 % (+2,61 points) — the real-time figure to
//     watch.
//
// A claim that did NOT survive: "the 4 points failing the port yardstick
// get fixed" was read off a grid whose header says `vara: motor@8x` — a DK
// comparison, while the port's −60 dB threshold is against NGSPICE. Two
// different yardsticks. Measured at the point it claimed to fix (drive 0 ·
// tone 1 · lvl 0), against ngspice: −70,0 without the subsystem, −70,3 with
// it — 0,3 dB, and it already passed.
//
// The declared compromise it drags in (via the cache): slice filling makes
// the TRANSIENT depend on the host's block size, so exact block
// bit-identity gives way to a bounded ~1e-05 (−100 dB) difference during
// the ~267 ms of filling, and only if the host changes block size. Building
// the table at once would cost a 4-6 ms first-block spike against a
// 1,333 ms block — an xrun at startup, which is unacceptable where −100 dB
// is not. Not a hidden defect: the gate MEASURES it against a declared
// cap and goes red if the number rises.
//
// This switch once sat at 0 for days while its comment said SHIPPED — the
// decision was made, the measurement done, the comment written, and the
// code unchanged. A comment claiming "shipped" proves nothing: the proof is
// the VALUE, and it is checked (`make decisiones`).
// `=0` returns to the tabulated waveshaper and is kept as the CONTROL.
#ifndef NLSC_E4_SUB_808
#  define NLSC_E4_SUB_808 1
#endif
#ifndef NLSC_E4_SUB_V9RI
#  define NLSC_E4_SUB_V9RI 1
#endif
// THE `ib` LOOP, CLOSED INSTEAD OF DELAYED. See `SubQ2::process_loop`
// and `OpenBaseLevel::parte_sin_ib`. At 0 it returns to the one-sample
// delay, the CONTROL this is measured against.
#ifndef NLSC_E4_CLOSED_LOOP
#  define NLSC_E4_CLOSED_LOOP 1
#endif
// STAGE 4's SOLUTION CACHE — SHIPPED, with all three ship conditions
// measured axis by axis (the criterion: it saves CPU and worsens NOTHING,
// audio included, with certainty):
//
//   1. The FALLBACK exists: outside the box it SOLVES instead of clamping,
//      and the grid against ngspice reads 0 of 36 over −60 dB, worst
//      −61,10 — identical to the Newton's own worst point, with 22 points
//      better and NONE worse. (An earlier box, sized on the DI alone,
//      clipped at node 84 of a 0..31 axis under the grid's hard sine and
//      suspended 4 of 36 — the box is sized on the stimulus that bites,
//      and the read never clips: it falls through.)
//   2. BOTH yardsticks measured: with real DI −28…−29 % of CPU (21/21
//      pairs, p = 9,5e−07) and with the hard sine at maxed knobs it TIES
//      (+0,00 %, 10/21, p = 1) — where the table cannot reach, the box test
//      is paid and it solves, which makes it an ACCELERATOR, not a bet.
//   3. And the non-signal axis: `prepare()` no longer builds the table on
//      the audio thread. The `variant` port cost 4,33-6,25 ms against a
//      1,333 ms block and now costs 0,0021 ms, same as without the cache.
//      See `advance_cache()` and `defer_fill()`.
//
// Also measured: ANMR 77 better of 100 and 2 worse within ONE digit of the
// instrument · absolute alias identical · toll on the 808 +2,06 % at
// p = 0,383, i.e. no difference.
// What it DOES cost, said out loud: +204,8 kB per instance.
// Gate: `make cache-rodajas`, inside `make test`, with a negative leg.
//
//   0 = Newton, the original — still the CONTROL everything is measured
//       against (`NLSC_E4_CACHE=0`)
//   1 = DESIGN B — table of the OPEN system `(n19, ieq) -> (n17, ib)`, with
//       the affine loop `n19 = a0 + kz·ib` closed OUTSIDE by fixed point.
//       Knob-independent: one table serves the whole travel.
//   2 = DESIGN A — table of the CLOSED system `(a0, ieq) -> (n17, ib)`, one
//       access. Depends on `kz`, i.e. on LEVEL, so it rebuilds on every
//       `prepare()`.
// The rail is PREDICTED. `=0` returns to the one-sample delay, the
// CONTROL that measured what the prediction buys.
#ifndef NLSC_CASC_RAIL_PREDICTED
#  define NLSC_CASC_RAIL_PREDICTED 1
#endif
#ifndef NLSC_E4_CACHE
#  define NLSC_E4_CACHE 1
#endif
// The grid, CHOSEN BY MEASUREMENT (worst point = `lvl` 0, where the
// output is smallest and the relative error largest):
//
//   512x32  131 kB  -77,6 dB      800x64  410 kB  -85,2 dB
//   800x32  205 kB  -82,0 dB     1024x48  393 kB  -87,9 dB
//
// 800x32 is 204,8 kB in `float` with two outputs => fits the 256 kB of L2
// per core, sitting 10,5 dB below the plugin's own null (−71,5), i.e.
// adding ~0,37 dB to the total. Fitting in L2 does not say it is
// cheaper: that is a clock measurement.
//
// RE-BALANCED SINCE, AND THE TABLE ABOVE IS PRE-FIX. Those grid
// figures were measured while the variant shunt hung off Q2's EMITTER, which
// threw the emitter around and made the `ieq` axis carry an excursion ~25x
// larger than the real circuit has. With the shunt on its real node (`R15`)
// the `ieq` travel collapses to ~1,5 nodes of 31, while the `n19` axis --
// whose half-span was sized on ONE input level, 0,5 V of DI -- leaves its box
// as soon as the input gets hot:
//
//     input     n19 outside          ieq outside
//     0,5 V     0                    0
//     0,8 V     49 %  (99,6 nodes)   0
//     2,0 V     74 %  (565 nodes)    0 / 72 % on the 808
//
// Half to three quarters of samples falling to the Newton is the cache not
// being there at all. So the nodes move to the axis that needs them:
// 1900 x 8 over ±2,70 V, SAME `n19` resolution, and the table drops to
// **121,6 kB**. Measured: 0 % outside everywhere up to 2,0 V on both variants
// and three rates, fidelity unchanged over the 36 knob points at both 0,5 V
// and 1,6 V of input (worst −96,9 dB, same as before).
//
// AND THE SPEED-UP IS NOT THE SIZE, IT IS THE STRIDE. `kCN2` sets the byte
// distance between the two rows a bilinear read touches: `kCN2 * 2 * 4`. At 32
// that is **256 B -- four cache lines apart**; at 8 it is **64 B, the adjacent
// line**. Counters, three repeats, hot DI: instructions −0,72 % (stable to the
// digit) but **L1 d-cache load misses −58 %** (641k -> 258k). Paired clock:
// −2,72 % nominal DI (10/11, p = 0,012), −3,07 % sustained 1,2 V sine (15/15,
// p = 6,1e−05). Taken with load average 1,5, not a rested machine; the sign
// is what the pairing establishes, and the counters do not care about load.
#ifndef NLSC_E4_CACHE_N1
#  define NLSC_E4_CACHE_N1 1900
#endif
#ifndef NLSC_E4_CACHE_N2
#  define NLSC_E4_CACHE_N2 8
#endif
// Iterations of design B's affine loop. The measured loop gain is
// `kz·dib/dvbe` ~ 0,09 at the worst `lvl` (kz = −24 080 Ω at mid-travel,
// 24x the `lvl` 0 value), so two passes leave the term at ~1e-4 of its
// 2,4 mV.
//
// LOWERING IT TO 1 IS MEASURED AND REJECTED. It buys −11,38 % of clock
// (3,34 -> 2,96 % of core, 21 of 21 pairs, p = 9,5e-07; the `IT=3` control
// moves the other way) and costs 9-11 dB of ANMR across the WHOLE `level`
// axis — and only there: the `drive` and `tone` axes do not move. The null
// against ngspice drops from −72,5 to −60,7 dB, above the port's line.
// => The reason sits three lines up: `kz` is 24x larger at mid-travel, so
// the loop gain is too, and one iteration leaves 9 % of the term instead of
// 0,8 %. "Converges in one iteration" was true about the term's SIZE and
// false about what it costs the yardstick.
// And the closing figure is the WHOLE GRID against ngspice (36 cells, 4x,
// v808): the product leaves 0 of 36 above −60 dB (median −69,15, worst
// −62,40) and with `IT=1` it is 25 of 36, median −59,15, worst −55,50. The
// DK column comes out IDENTICAL in both arms. => It is not "borderline": it
// fails. A 12-point per-AXIS sweep read −60,7 and missed the worst case,
// which lives in the INTERACTION.
#ifndef NLSC_E4_IEQ_CEILING
#  define NLSC_E4_IEQ_CEILING 0
#endif
#ifndef NLSC_E4_CACHE_IT
#  define NLSC_E4_CACHE_IT 2
#endif
// The BOX, in volts AROUND THE REST, so it follows the knobs and the rate
// on its own. Measured over the 36 points with 5 s of DI: `n19` travels
// 4,268 −1,044 / +0,833 and `vc` travels 0,072 V peak to peak => these are
// those numbers with margin. Outside the box it CLIPS and COUNTS
// (`out_of_box()`): an edge that degrades silently is what turns a table
// into a trap.
// Store the DEVIATION from the follower's line instead of absolute `n17`.
// MEASURED: it buys nothing while interpolation rules (400x16: +0,46 dB)
// and buys 11,7 dB once the grid reaches `float`'s floor (1600x64: −82,4
// without it, −94,1 with it). The floor is real and sits at ~−82 dB, not
// where estimated (−72): the estimate served to decide it needed measuring,
// not as a figure.
#ifndef NLSC_E4_CACHE_LINE
#  define NLSC_E4_CACHE_LINE 1
#endif
// THE SLICE: how many grid nodes solve PER BLOCK while the table fills.
// It is what turns the 4-6 ms rebuild into a bounded per-block cost, and
// the knob that invalidated it into nothing.
//
// The whole grid is `800 x 32 = 25 600` nodes and building it at once cost
// 4,3 ms => ~168 ns per node. While filling, the Newton runs — what ran
// before the cache existed.
//
// THE VALUE COMES FROM A MEASURED CURVE, not an estimate (`run_lv2` over
// 5 s of DI, 128-sample blocks at 48 kHz):
//
//   slice    fill               mean CPU    WORST BLOCK
//     128    200 blk · 533 ms     4,48 %      9,48 %
//     512     50 blk · 133 ms     4,13 %     11,96 %   <- the chosen one
//    2048     12 blk ·  33 ms     4,13 %     20,51 %
//
// => 512 fills 4x faster and lowers the mean, and its worst block (11,96 %)
// sits barely above the 10,98 % the replaced Newton cost. 2048 buys no more
// mean and doubles the peak: all cost.
// If 32-sample-or-smaller blocks ever need support, this is what gets
// touched — not the cache.
#ifndef NLSC_E4_CACHE_NODES_PER_BLOCK
#  define NLSC_E4_CACHE_NODES_PER_BLOCK 512
#endif

// SHIPPED: the cache fill advances PER SAMPLE. It recovers the 9RI's
// EXACT block bit-identity (1,522e−05 -> 0,0)
// paying no clock (+0,30 %, p = 0,383) and not moving the null (−82,8 dB
// identical). => The formerly declared compromise is no longer needed.
// `=0` returns to the per-block fill, kept as the CONTROL.
#ifndef NLSC_E4_FILL_PER_SAMPLE
#  define NLSC_E4_FILL_PER_SAMPLE 1
#endif
#ifndef NLSC_E4_CACHE_NODES_PER_SAMPLE
#  define NLSC_E4_CACHE_NODES_PER_SAMPLE 2
#endif

// TIGHTENING THE BOX IS FREE: going from ±1,30/±0,10 to ±1,10/±0,045 —
// what `n19` and `vc` actually travel across the 36 points with 5 s of DI —
// bought 8,1 dB on the SAME grid (−61,9 -> −70,0). A loose box does not
// fail: it just throws away resolution, quietly.
// 1,10 WAS SIZED AT ONE INPUT LEVEL and the comment above says so
// itself: "what `n19` travels with 5 s of DI" -- that DI peaks at 0,5 V. At
// 0,8 V the axis is already 49 % outside. 2,70 covers up to 2,0 V of input,
// which is where the plugin's own criterion sits (A-v, `OPTIMIZACIONES.md`).
#ifndef NLSC_E4_CACHE_DN19
#  define NLSC_E4_CACHE_DN19 2.70
#endif
// THE `vc` AXIS IS OFF-CENTRE ON PURPOSE.
//
// Centred on the STATIC rest, the counter said the axis's bottom half is
// never stepped on and the top runs out: travel measured in NODES
// 14,8 .. 38,3 of a 0..31 box (at 96 kHz; equally off-centre at 44,1 and
// 48 kHz). The reason was already written: under signal the rest point
// DISPLACES, and `vc` is precisely C9's DC. In volts the real travel is
// `q_n17 + [-0,002, +0,066]`.
// => Declared asymmetric. The SPAN does not change (0,09 V => same
// resolution): what changes is where it sits.
#ifndef NLSC_E4_CACHE_VC_LO
#  define NLSC_E4_CACHE_VC_LO (-0.030)
#endif
// Widened with the axis 4x coarser: with 8 nodes the span has to cover the
// 808 at 2,0 V, which was the only case still leaving on this axis.
#ifndef NLSC_E4_CACHE_VC_HI
#  define NLSC_E4_CACHE_VC_HI (+0.240)
#endif
// THE BOX PROBE STAYS OFF IN THE PRODUCT.
// The PER-AXIS accounting and the `ieq` axis's travel are what size the
// box, and therefore what `make cache-caja` needs — but they cost a
// multiply-add PER SAMPLE on the hot path, and the master table's cost
// figures were taken without them.
// => Enabled ONLY in the harness (the Makefile's `CAJA_DEFS`). With it off
// the product binary does not change.
#ifndef NLSC_E4_CACHE_PROBE
#  define NLSC_E4_CACHE_PROBE 0
#endif
// `E4GS` IS NOT APPLIED ON THE SUBSYSTEM PATH.
//
// `E4GS` is the `n17/n19` shape normalised at 8 kHz — the "residual load
// step" the TABULATED waveshaper needs, because `poly()` is a MEMORYLESS
// curve and does not carry it. The subsystem, instead, SOLVES `n17` with
// the real network (R13, R14, C9, R15) and its state in `C9`: it already
// carries it. Applying it again counts the step TWICE.
//
// Measured, and it is the whole output defect: feeding ngspice's `n17`
// through the plugin's chain, `gs_`->`g2_` reads −64,51 dB and `g2_` alone
// −142,23. The output stage imposed a FLOOR at −64,5 dB that made any
// upstream improvement invisible — and explains why giving the cascade the
// oracle's exact `n4` lifted every internal node to DK level while the
// output stayed pinned.
// At 0 it applies again, which is the CONTROL.
#ifndef NLSC_E4_GS_IN_SUB
#  define NLSC_E4_GS_IN_SUB 0
#endif
// STAGE 4's NEWTON ITERATION CAP — chosen by MEASURING, with the
// criterion explicit: truncation is an IMPLEMENTATION error and must stay
// BELOW the MODEL's error, or improving the model would buy nothing.
// Deviation against the converged model, on ADVERSE material (full-scale
// steps, 82 Hz and 5 kHz sines, 3 kHz square):
//
//   iter=1 -> −37,3 dB   iter=2 -> −48,9   iter=3 -> −55,5   iter=4 -> −63,2
//
// And why NOT 1, which on the sine null came out 0,7 dB BETTER (−54,2):
// that is not the model converging, it is a lucky truncation — on adverse
// material it falls 26 dB.
// Four is nearly free: with the predictor the Newton exits on tolerance
// in 2-3 passes, so the fourth is a CAP, not a cost (ratio 0,818 vs 0,823
// for three, 0,851 without the predictor).
// IT STAYS AT 4 — but for a DIFFERENT reason than first written.
//
// The first table gave −48,9 dB for 2 iterations, hence 4. That table
// does NOT reproduce: the plugin rebuilt from its own commit and run with
// today's harness gives cap 1 at −83,2 dB where it said −37,3 => the object
// did not change; the then-instrument NEVER TRAVELLED. Now it does:
// `harness/presupuesto_iter.py`.
//
// Re-measured with HARD material (2 V segments), sweeping five knob
// positions and publishing the WORST:
//
//   cap 1 -> −63,6 dB (fails by 3,5)   cap 2 -> −83,3 dB (+16,2 margin)
//   cap 3 -> −115,7                        cap 4 -> −155,3
//
// The criterion is unchanged: truncation is IMPLEMENTATION error and goes
// BELOW the MODEL's error, today −67,1 dB (not the −53,5 of then). With 2
// there are 16 dB to spare => on FIDELITY, 2 would do.
//
// BUT IT DOES NOT COME DOWN, BECAUSE IT SAVES NOTHING. Measured paired
// on the product .so, 4 vs 2: +1,63 % (if anything SLOWER), wins 6 of 21
// pairs, p = 0,078 — not significant. The "saves 10 %" once logged came
// from the same irreproducible table.
// It squares with the measurement next door: under normal signal the cap
// does not bite (0 escapes of 192 000), so lowering it removes no work — it
// removes a cap that was not in use. Trading fidelity for ZERO is not a
// trade, it is a risk (the simplest form of instructions-are-not-clock:
// here there are not even fewer instructions to remove).
// Under normal signal the cap does not bite: 0 escapes of 192 000 samples
// at 4x with 0,35 V (`newton_escapes()` counter).
// THIS IS NOT THE OVERSAMPLING FACTOR. 2x still FAILS (null −57,3 against
// a −60 yardstick, and the pinched harmonic at 2x reads +9,2 dB against a
// −10 criterion). Two different things, confused once.
// SOFTWARE PIPELINING PROBE (N2) — NEVER SHIPPED.
//
// N2 proposes breaking the INTER-SAMPLE dependency chain: on iteration `i`
// stage 1 does sample `i`, stage 2 does `i-1`, stage 3 `i-2`, stage 4
// `i-3`. The plan logged it as bit-exact, and IT CANNOT BE: the rail
// assembles from ALL FOUR stages' outputs of the SAME sample (`s.n4`,
// `s.n7`, `s.n14`, `dev_n19_`, `dev_n3_`, `n11`) and feeds back into all
// four of the next. Pipelined, those six contributions stop belonging to
// one sample: the rail stage 1 of `i` sees would need stage 4 of `i-1`,
// which has not run yet. The chain is not just the signal — it is the LOOP.
//
// => This probe exists to put a NUMBER on the two questions, where they are
// paid, instead of debating them:
//   1. the real ceiling — the `coste_etapas` bench compares the WHOLE
//      `Cascade4` against four loose stages WITHOUT the rail, two arms that
//      do not model the same thing; here the arm carries ALL the rail work
//      and lacks only the chain;
//   2. the fidelity price of the rail mixing samples.
// Its output is INCORRECT on purpose. Not a mode — an instrument.
#ifndef NLSC_CASC_SEGMENTED
#  define NLSC_CASC_SEGMENTED 0
#endif
#ifndef NLSC_E4_ITER
#  define NLSC_E4_ITER 4
#endif
#ifndef NLSC_E4_TOL
#  define NLSC_E4_TOL 1e-9
#endif
#include "nls_variantes.h"
#include "nls_mna.h"      // the DK engine's `bjt()`, for stage 4's subsystem
#include "nls_fijos_s.h"
#include "nls_stage1.h"
#include "nls_reposo_drive.h"
// THE SECOND BANK. Included ALWAYS, not under `#if`: the variant is
// chosen LIVE, so both banks of the current rate must be compiled at once.
// (`nls_stage1.h` already brings its own plus `fijos`/`reposo`.)
#if defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 96
#  include "nls_e234_coef_v9ri_96k.h"
#elif defined(NLSC_CASC_KHZ) && NLSC_CASC_KHZ == 192
#  include "nls_e234_coef_v9ri_192k.h"
#else
#  include "nls_e234_coef_v9ri_384k.h"
#endif
#include "nls_stage2.h"
#include "nls_tono.h"
#include "nls_nivel.h"
// `nls_juegos.h` goes LAST: it enumerates symbols from every namespace
// above, and each bank DECLARES which stage-4 subsystem it uses.
// `VRA` PARAMETRISED BY TONE: off until the measurement says it enters.
// See `nls_tono.h::RailVraVariable` and the generated header.
#ifndef NLSC_VRA_PARAM
#define NLSC_VRA_PARAM 0
#endif

// TONE COST PROBE (see its use below). Off = the product.
#ifndef NLSC_TONE_COST_PROBE
#  define NLSC_TONE_COST_PROBE 0
#endif
namespace nlsc {
inline constexpr bool kBaseAbierta = (NLSC_E4_OPEN_BASE != 0);
inline constexpr bool kSubEn808  = (NLSC_E4_SUB_808  != 0);
inline constexpr bool kSubEnV9ri = (NLSC_E4_SUB_V9RI != 0);
inline constexpr bool kClosedLoop = (NLSC_E4_CLOSED_LOOP != 0);
inline constexpr bool kGsEnSub    = (NLSC_E4_GS_IN_SUB != 0);
}
#include "nls_juegos.h"

namespace nlsc {

// The "a variant without a bank does not compile" guard lives in
// `nls_juegos.h`.



// Stage 3 — the tone control. LINEAR, measured at −156 dBc
// (`docs/LINEALIDAD_ETAPAS.md` §6-bis), so it is a filter and nothing more.
//
// THE KNOB IS A PARAMETER, not a constant. This used to be a fixed-`kE3`
// SOS cascade discretised at tone=0,5 — and measured, moving the knob to
// the ends cost up to +17,2 dB of ANMR. Now the coefficients come from
// `nls_tono.h`, derived by exact algebra over the netlist with the NJM4558
// inside.
// At tone=0,5 it reproduces the old `kE3` EXACTLY (−64,9 dB at 192 kHz
// and −77,0 at 384 kHz against the analog — their own figures): the nominal
// point does not move; the rest of the travel is what gets added.
// And the per-sample cost is unchanged: order 4 in direct form against
// the two biquad sections of before.
class Stage3 {
public:
    void prepare(double fs, double tone, bool reinit = true)
    { h_.prepare(fs, tone, reinit); }
    void reset() { h_.reset(); }
    double process(double dev_n7) { return h_.process(dev_n7); }
private:
    VariableTone h_;
};

// Stage 4 — the LEVEL pot and the Q2 output buffer. Wiener-Hammerstein,
// like stage 1 and for the same measured reason: Q2's distortion is FLAT in
// frequency (0,1 dB over two and a half decades) => memoryless.
class Stage4 {
public:
    Stage4()
    {
        for (int i = 0; i < e234::kWS2_N; ++i) ws2_[i] = e234::kWS2[i];
#if NLSC_WS_RECENTER
        ws2_[e234::kWS2_N - 1] = 0.0;   // see NLSC_WS_RECENTER in nls_stage1.h
#endif
        reset();
    }

    // `g1_` (the LEVEL pot's network) IS PARAMETRIC. It was a fixed
    // `kE4G1` set discretised at lvl=0, and measured, that cost
    // −6,3 / −2,5 / +3,4 dB of ANMR while lowering the volume.
    // And the REST needs no parametrisation: measured, `lvl` does not move
    // it (identical to the seventh digit at five positions), because C8
    // blocks DC between the pot and Q2's base. See `nls_nivel.h`.
    // THE VARIANT. Here lives the PHYSICAL difference between the 808 and
    // the 9/9RI: `R14` (series) and `R15` (the output load, after the
    // coupling cap). Both models share Q2's emitter (10 k) and therefore its
    // quiescent point; what moves is the load on `n17` and the output
    // divider `E4G2` — 0,10 dB, Keen's "admittedly very subtle".
    // This used to say `R13` is the emitter and the waveshaper changes
    // shape by −14,4 dB. That described a modelling defect, since corrected.
    // `kWS2` is read PER SAMPLE => COPIED. The filters only in `prepare()`
    // => choosing the table suffices.
    static_assert(fijos::kE4GS_NB == fijos_v9ri::kE4GS_NB &&
                  fijos::kE4GS_NA == fijos_v9ri::kE4GS_NA &&
                  fijos::kE4G2_NB == fijos_v9ri::kE4G2_NB &&
                  fijos::kE4G2_NA == fijos_v9ri::kE4G2_NA,
                  "both stage-4 banks must share the same filter orders");
    static_assert(e234::kWS2_N <= kWSMax && e234_v9ri::kWS2_N <= kWSMax,
                  "kWSMax is too small for Q2's waveshaper");

    // ═══════════════════════════════════════════════════════════════════════
    // STAGE 4 AS A SOLVED SUBSYSTEM
    // ═══════════════════════════════════════════════════════════════════════
    //
    // The TABULATED waveshaper is a static function of `n19`, and for the
    // 9/9RI variant that is not enough: measured against the oracle's `n17`
    // with the REAL signal, the polynomial stops at −31,1 dB and this gives
    // −70,0 dB.
    //
    // And why it had been dismissed: it was compared against the
    // EXTRACTION SWEEP (`q2_curva.cir`), not the signal. Both curves
    // measured over their overlap are NOT the same function: they separate
    // by up to 23 mV in the cutoff region. A model is judged in ITS regime,
    // and this stage's regime is the whole chain, not a sine injected at
    // `n13`.
    //
    // The circuit, and why ONE unknown suffices: Q2 is a follower with the
    // collector at 9 V, the emitter at `n17`, `R13` to ground and
    // `R14+C9+R15` towards the output. That branch, seen from `n17`, is
    // (R14+R15) in series with C9 => one unknown (`n17`) and one state (C9's
    // voltage).
    //
    //   Ie(vbe) = n17/R13 + (n17 − vc)·geq
    //
    // The transistor model is the SAME `bjt()` as the DK engine — not a
    // copy: duplicating it is the recipe for the two engines drifting apart
    // unannounced.
    struct SubQ2 {
        // From the netlist (`harness/spice/full.inc`): `R13` (Q2's emitter)
        // and C9 do not depend on the variant; `R14` (series) and `R15` (the
        // output load) DO, and they are the axis. `kR13` is the only fixed
        // resistor here; the two variant ones arrive through `prepare()`.
        static constexpr double kR13 = 10.0e3;
        static constexpr double kC9  = 10.0e-6;
        static constexpr double kVCC = 9.0;
        // The follower's gain, for the predictor. Measured on Q2's curve:
        // 0,94 on the 9/9RI and 0,98 on the 808 — an intermediate value
        // serves both, since it only decides where the Newton STARTS.
        static constexpr double kGanSeg = 0.96;
        // The Newton's budget. Swept and chosen BY MEASUREMENT (null and
        // cost), not by habit.
        static constexpr int    kMaxIter   = NLSC_E4_ITER;
        // Observability: how often the budget ran out, of how many samples.
        long unconverged_ = 0;
        long samples_ = 0;
#ifdef NLSC_E4_LOOPGAIN_PROBE
        // Here and not with the cache counters: those live INSIDE
        // `#if NLSC_E4_CACHE`, and this probe has to be able to run with the
        // cache OFF, which is the only way it sees every sample and not just
        // the ones that fall outside the box.
        double lg_max_ = 0.0, lg_sum_ = 0.0;
        long   lg_n_ = 0;
#endif
        static constexpr double kTolNewton = NLSC_E4_TOL;

        // Q2's JUNCTION CHARGES — `NLSC_E4_CHARGES`.
        //
        // Measured: `n19`'s error is `Z(s;lvl)·delta_ib`, and `delta_ib` is
        // the DISPLACEMENT current this subsystem did not model, because its
        // four `bjt()` calls pass `with_charges = false`. Against ngspice:
        // `delta_ib` = 1,428 nA rms, in QUADRATURE at every harmonic and
        // growing with frequency. Bound at the output, with the oracle's
        // `ib` injected: −68,9 -> −84,8 dB.
        //
        // WHY `with_charges` DOES NOT SIMPLY SWITCH ON: this class's cache
        // is a table `(n19, ieq) -> (n17, ib)`, valid because with the
        // charges off Q2 is MEMORYLESS. Giving Q2 its own state would demand
        // two more history inputs and the table would stop being a function
        // — and that table buys −28…−29 % of CPU.
        //
        // => Instead `ib` is corrected OUTSIDE the solve: the static part is
        // solved by the Newton (or the table) as always, and on top the
        // displacement current adds via a trapezoidal COMPANION model, the
        // same scheme as `nls_mna.h`.
        // Neglecting its feedback on the emitter node is SECOND order: the
        // missing current is 1,428 nA against the ~37 uA `R13` draws from
        // that node — 38 ppm. In `ib` it is FIRST-order error instead,
        // because `ib` leaves the block multiplied by `Z(s;lvl)`.
        // That is a magnitude quotient, not an A/B: the null arbitrates.
// SHIPPED IN MODE 2, a product decision.
//   0 = no charges (the original)     · 1 = depletion + DIFFUSION (control)
//   2 = depletion ONLY  <- THE PRODUCT
// Measured: mode 2 matches mode 1 on EVERY yardstick and costs no clock
// (+0,31 %, p = 0,383, against mode 1's +15,17 % in the SAME run).
// Mode 1 is KEPT: the control proving the diffusion charge is not needed
// here (0,1 dB, opposite sign, for the whole 15 %).
        // The displacement current at given voltages, WITHOUT touching the
        // state: callable inside the Newton as often as needed.
        // With `NLSC_E4_CHARGES == 0` it is zero and the compiler erases it.
// A MACRO'S DEFAULT GOES **BEFORE** ITS FIRST USE, and it nearly slipped
// through. `idesp_de()` sat ABOVE this `#define`, so inside it the macro
// did not exist yet: the preprocessor took the `#else` branch and the
// function returned 0 IN SILENCE. The plugin compiled, ran, and the null
// came out EXACTLY the baseline's — the lever looked useless. Seen only
// because the headline figure was re-measured after the refactor.
// With `-DNLSC_E4_CHARGES=2` on the command line it DID work — the worst
// way to have it: the harness read −82,8 and the product −68,9.
// And the block-invariance cap was NOT touched: the cause was fixed
// (the box decision is taken with the STATIC `v`), and the 9RI sits at
// 3,82e−06 against its declared 5,0e−06 cap.
// `NLSC_E4_RB` — Q2's BASE RESISTANCE, which is the root cause of the solver
// not landing on the TABULATED rest point.
//
// THE DEFECT. The rest point comes from ngspice's `.op`, so it is the
// physically correct point, and it is: ngspice gives `v(n17) = 3,577195 V`
// where the table carries `3,57719494`, seven digits. But this subsystem
// evaluated the junction at the base TERMINAL, and `.model QBUF` carries
// **RB = 50 ohm**. At `ib = 6,803e-07 A` that is **34,0 uV** of missing drop,
// and through `gm = 13,8 mS` it is **4,70e-07 A** of imbalance in node `n17`'s
// KCL. Measured on the tabulated rest, with no signal and no ngspice:
// `f1 = 4,70e-07 A`, and the `n17` that DOES satisfy the KCL sits
// **+33,5..+34,0 uV** higher across the whole drive travel — which is the
// `w(0) = +35,0 uV` that digital silence had measured by a completely
// different route.
//
// WHY THE DK SOLVER DOES LAND. Because its MNA **does** build that resistance
// (`nls_mna.h`, `R(kQ[i].b, kQ[i].bi, q_.RB)`: it has an INTERNAL base node).
// It was not a difference of tolerance or of discretisation: it was a branch.
//
// HOW IT ENTERS, AND WHY IT IS FREE. The level network imposes
// `n19 = a0 + kz*ib` and the internal base is `u = n19 - RB*ib`, so
// **`u = a0 + (kz - RB)*ib`**: the SAME form. The 2x2 Newton solves in `u` with
// `kz' = kz - RB`, the junction is evaluated the same way, and `n19` is
// recovered at the end with the original `kz`. Same number of unknowns, same
// cache table (it is indexed by the voltage the JUNCTION sees, so not a byte
// moves) and one FMA per sample.
// At 0 the binary must come out BIT-IDENTICAL: `rb_` is 0 and every formula
// below collapses to the previous one. That is the control arm.
#ifndef NLSC_E4_RB
#define NLSC_E4_RB 1
#endif
#ifndef NLSC_E4_CHARGES
#define NLSC_E4_CHARGES 2
#endif
        // The displacement's derivatives. WITHOUT THIS Newton does not
        // close the loop: `f2` carried the term and the Jacobian did not, so
        // it stopped on step size with a 1e−5 residual where the product
        // gives EXACT ZERO, and `make lazo-ib` went red — rightly.
        // In mode 2 they are CONSTANT (the effective depletion), so they
        // come at zero cost. In mode 1 they are the same: the DIFFUSION
        // charge's derivative is neglected, so its Jacobian is approximate
        // and Newton converges somewhat slower. A CONTROL, not the product.
        double didesp_dv() const
        {
#if NLSC_E4_CHARGES
            return gh_ * (jbe_.cj0 + jbc_.cj0);
#else
            return 0.0;
#endif
        }
        double didesp_dx() const
        {
#if NLSC_E4_CHARGES
            return -gh_ * jbe_.cj0;
#else
            return 0.0;
#endif
        }
        double idesp_de(double vbe, double vbc) const
        {
#if NLSC_E4_CHARGES == 2
            return gh_ * ((jbe_.cj0 * vbe - qbe_ant_) + (jbc_.cj0 * vbc - qbc_ant_));
#elif NLSC_E4_CHARGES
            const mna::Engine::QOut o =
                mna::Engine::bjt(qm_, jbe_, jbc_, vbe, vbc, true);
            return gh_ * ((o.qbe - qbe_ant_) + (o.qbc - qbc_ant_));
#else
            (void)vbe; (void)vbc; return 0.0;
#endif
        }
        static constexpr bool kCharges = (NLSC_E4_CHARGES != 0);
        // Only mode 1 asks `bjt()` for the charges; 2 computes them apart.
        static constexpr bool kExactCharges = (NLSC_E4_CHARGES == 1);
#if NLSC_E4_CHARGES
        double gh_ = 0.0;                       // 1/h (backward Euler)
        double qbe_ant_ = 0.0, qbc_ant_ = 0.0;
        double idesp_ = 0.0;

        // Advances the companion with charges ALREADY computed. One copy
        // of the formula: both paths (Newton and cache) call here.
        // BACKWARD EULER, **NOT** TRAPEZOIDAL — and it is no preference.
        //
        // The trapezoidal companion is `i_n = (2/h)(q_n - q_{n-1}) - i_{n-1}`,
        // whose homogeneous solution is `(-1)^n`: an UNDAMPED Nyquist mode.
        // Inside an MNA that is harmless because the companion conductance
        // `(2/h)·c` enters the Jacobian and damps it; here the term computes
        // APART and feeds back through `kz` with one sample of delay, so
        // nothing holds it. MEASURED: it diverges at sample 134 (NaN at
        // t = 3,5e-4 s).
        //
        // => `i_n = (q_n - q_{n-1})/h`. First order, but applied to a term
        // worth 1,6 % of `ib`: the rule's error is ~(omega·h/2) OF THAT
        // 1,6 %, i.e. 1,3e-4 of the total at 1 kHz and 384 kHz.
        // And it has no current state of its own: one variable fewer and
        // no mode that could ring.
        void advance_charges(double qbe, double qbc)
        {
            idesp_ = gh_ * ((qbe - qbe_ant_) + (qbc - qbc_ant_));
            qbe_ant_ = qbe; qbc_ant_ = qbc;
        }
        // The CACHE path does not evaluate the transistor, so here it must
        // be paid. ONE evaluation per sample, against the 2-3 iterations
        // the table saves: which is why the table stays worth it.
        // MODE 2 — DEPLETION ONLY, and why it can suffice.
        //
        // Mode 1 demands a FULL transistor evaluation on the cache path,
        // exactly where there was none: nearly all of the +15,28 % of clock
        // lives there. But the RULING term does not need it:
        //   · `Cjc` sees `vbc = n19 − 9 V`, i.e. `n19`'s WHOLE travel;
        //   · `Cbe` sees `vbe = n19 − n17`, and the follower cancels ~96 %
        //     of that travel, so it contributes ~25x less.
        // => With depletion alone (`q = cj0_eff·v`, TWO multiplies and not
        // one exponential) most of the correction should remain.
        // A magnitude PREDICTION, not a measurement: the null and the
        // clock measure it, which is why both modes coexist until there are
        // numbers.
        void advance_charges_from(double vbe, double vbc)
        {
#if NLSC_E4_CHARGES == 2
            advance_charges(jbe_.cj0 * vbe, jbc_.cj0 * vbc);
#else
            const mna::Engine::QOut o =
                mna::Engine::bjt(qm_, jbe_, jbc_, vbe, vbc, true);
            advance_charges(o.qbe, o.qbc);
#endif
        }
        void seed_charges(double n19_q, double n17_q)
        {
#if NLSC_E4_CHARGES == 2
            qbe_ant_ = jbe_.cj0 * (n19_q - n17_q);
            qbc_ant_ = jbc_.cj0 * (n19_q - kVCC);
#else
            const mna::Engine::QOut o = mna::Engine::bjt(
                qm_, jbe_, jbc_, n19_q - n17_q, n19_q - kVCC, true);
            qbe_ant_ = o.qbe; qbc_ant_ = o.qbc;
#endif
            idesp_ = 0.0;                          // no displacement at rest
        }
#endif

        // `reiniciar` — and its absence was a CLICK.
        //
        // `prepare()` is ALSO called on a knob move, and this `reset()` was
        // unconditional: every move dropped the subsystem's state. It is
        // literally re-tuning-is-not-restarting — pots are netlist
        // resistors, so a knob forces new matrices but NOT dropping state.
        //
        // `make test` caught it when the subsystem went on for the 808: a
        // knob delta of 1e-07 => output 6,759e-05 against the control's
        // 2,980e-08. No fidelity yardstick sees it; it is the host contract.
        // And it was NOT a new defect: the 9/9RI already ran the
        // subsystem, so this click WAS SHIPPING on that variant. The gate
        // missed it because it tests the default variant, which was the
        // tabulated one.
        void prepare(double fs, double r15, double r14, double n17_rest,
                     double n19_rest, double kz = 0.0, bool reinit = true)
        {
            n19_rest_ = n19_rest;
            // CORRECTED together with the netlist and the DK's
            // MNA. The variant axis is `R15` — the output load — and the
            // emitter is 10 k in both models. R15 enters this model by two
            // DIFFERENT roles, and it is not double counting:
            //   · `g13_` — the DC path from `n17` to ground, i.e. R13. Fixed.
            //   · `rs`   — the LOAD the output branch puts on `n17`:
            //              R14 + R15 in series with C9.
            // The transfer `n17 -> out` is NOT here: `E4G2`, a generated
            // coefficient, carries `R15/(R14+R15)`.
            // This fix was first measured against a DK that still stamped
            // the old topology, and it came out WORSE — 1,0447 -> 1,1563 of
            // gain. It was the arbiter, not the fix
            // -- when a control fails, look at the control first.
            g13_ = 1.0 / kR13;
            const double rs = r14 + r15;
            const double gc = 2.0 * kC9 * fs;         // trapezoidal
            geq_ = 1.0 / (rs + 1.0 / gc);
            inv_gc_ = 1.0 / gc;
            k_vc_ = 1.0 / (2.0 * kC9 * fs);
#if NLSC_E4_CHARGES
            gh_ = fs;
#endif
            qm_.derive();
            // RB AND THE INTERNAL BASE AT REST, and they go HERE -- before
            // `jbe_`/`jbc_` -- because the depletion capacitance is evaluated
            // at the JUNCTION voltage, not at the terminal's. `ib_rest()`
            // solves the fixed point, and with `rb_ = 0` it returns the
            // usual value on the first pass.
            // `bjt(..., false)` does not touch `jbe_`/`jbc_`, so calling it
            // before they are initialised is correct — and that is what makes
            // this order possible.
            rb_ = (NLSC_E4_RB) ? qm_.RB : 0.0;
            u_rest_ = n19_rest - rb_ * ib_rest(n19_rest, n17_rest);
#if NLSC_E4_CHARGES
            // WITHOUT THIS IT COMES OUT NaN, AND THE FIELD'S OWN COMMENT SAID SO:
            // `jbe_`/`jbc_` sat at ZERO because this subsystem never asked
            // for charges (`with_charges = false`), so `qjunc` ran on an
            // uninitialised junction. They fill with the engine's SAME `init`,
            // from the SAME model table.
            // CONSTANT DEPLETION DOES NOT HOLD HERE, and it is measured.
            //
            // `qjunc` uses `c = cj0`, and its justification is literal: "over
            // a CLIPPER's travel `u^(-m)` only moves ±20 % around 1 (VJ =
            // 1 V, M = 0,4, v ∈ −0,7…0,7)". Q2's BASE-COLLECTOR junction
            // does not live there: it rests at −4,73 V, where `cj0` is 1,85x
            // the real capacitance (3,638 vs 1,970 pF).
            // Measured before fixing: with raw `cj0` the correction comes
            // out 1,64x what is needed and the residual changes quadrature
            // (−91,1° -> +88,2°), which is the signature of overshooting.
            //
            // => The EXACT capacitance evaluates ONCE, in `prepare()`, at the
            // rest point, and serves as the constant. Over the signal's
            // travel that capacitance moves ±0,7 % (1,955-1,984 pF), so the
            // approximation stays constant — at the RIGHT value.
            // Hot-path cost: ZERO. One `pow` in `prepare()`.
            // The formula lives in `nls_mna.h`, NEXT TO `qjunc` — the other
            // writing of the same law; they cannot unify (one runs per
            // sample with range reduction, the other once with `pow`), which
            // is why they sit ten lines apart.
            // `NLSC_CJC_SCALE` — sensitivity PROBE, 1,0 by default. The
            // service manual asks for a 2SC1815BL and our card is a 2N3904:
            // at VCB = 10 V Toshiba publishes Cob = 2,0-3,5 pF and our card
            // gives 1,60 => short by 25 % to 119 %. This lets that be
            // MEASURED into the audio instead of opined.
            // At 1,0 the binary is bit-identical: a probe, not a change.
#ifndef NLSC_CJC_SCALE
#define NLSC_CJC_SCALE 1.0
#endif
            jbe_.init(mna::Engine::depletion_cap(u_rest_ - n17_rest, qm_.CJE, qm_.VJE,
                                 qm_.MJE, qm_.FC) * double(NLSC_CJC_SCALE),
                      qm_.VJE, qm_.MJE, qm_.FC);
            jbc_.init(mna::Engine::depletion_cap(u_rest_ - kVCC, qm_.CJC, qm_.VJC,
                                 qm_.MJC, qm_.FC) * double(NLSC_CJC_SCALE),
                      qm_.VJC, qm_.MJC, qm_.FC);
#endif
            if (reinit) reset(n17_rest);
#if NLSC_E4_CACHE
            // The cache is indexed by the voltage the JUNCTION sees, not the
            // terminal's: `solve_node()` evaluates at `p1 - x`. With RB that is
            // `u`, so the table is the SAME and all that moves is the centre of its
            // caja — 34 uV sobre un eje de cientos de mV.
            build_cache(u_rest_, n17_rest, kz - rb_);
#else
            (void)kz;
#endif
        }

#if NLSC_E4_CACHE
        // ─────────────────────────────────────────────────────────── EL CACHE
        static constexpr int kCN1 = NLSC_E4_CACHE_N1;
        static constexpr int kCN2 = NLSC_E4_CACHE_N2;
        static constexpr int kCIt = NLSC_E4_CACHE_IT;

        long out_of_box() const { return out_of_box_; }
        // AND PER AXIS, because "13 % outside" says NOTHING until which is
        // known: clipping the `ieq` axis (which enters linearly) is not
        // clipping `n19`'s. `exceso_*` is in NODES, i.e. grid steps.
        long outside_p()   const { return fuera_p_; }
        long outside_ieq() const { return fuera_i_; }
        double exceso_p()   const { return exceso_p_; }
        double exceso_ieq() const { return exceso_i_; }
        double w_min() const { return w_min_; }   // in NODES: 0 .. kCN2-1 is inside
        double w_max() const { return w_max_; }

#ifdef NLSC_E4_PLANOS_PROBE
        // PROBE (not product, guarded): dumps the table's TWO PLANES so
        // they can be judged apart off-line.
        //
        // The question it exists for: the `n17` plane is stored as a DEVIATION
        // from the follower's line (see the fill), and the `ib` plane is stored
        // RAW. A table that holds a coefficient quantises coarser than one that
        // holds a value, so what limits the grid may be a plane that nobody
        // factored. That decides whether the `ieq` axis can be widened, which
        // is an open decision in `docs/TABLA_MAESTRA.md`.
        //
        // It prints the GEOMETRY too. A plane read without the axes that
        // index it cannot be refitted, and refitting is the whole point.
        void dump_planes(std::FILE* f) const
        {
            // EVERY geometry line carries its own `#`. They used to share one:
            // `"# N1\t%d\nN2\t%d\n"` puts the comment mark on the first line
            // only, so `N2`, `h1`, `h2`, `g` and `p0` came out looking like DATA
            // rows and the first reader of this dump died on them. It had never
            // run — the probe had no invoker.
            std::fprintf(f, "# N1\t%d\n# N2\t%d\n", kCN1, kCN2);
            std::fprintf(f, "# lo1\t%.17g\n# h1\t%.17g\n", c_lo1_, c_h1_);
            std::fprintf(f, "# lo2\t%.17g\n# h2\t%.17g\n", c_lo2_, c_h2_);
            std::fprintf(f, "# x0\t%.17g\n# g\t%.17g\n# p0\t%.17g\n",
                         c_x0_, c_g_, c_p0_);
            std::fprintf(f, "j\tk\tp1\tieq\tdev_n17\tib\n");
            for (int j = 0; j < kCN1; ++j)
                for (int k = 0; k < kCN2; ++k) {
                    const size_t c = (size_t(j) * size_t(kCN2) + size_t(k)) * 2;
                    std::fprintf(f, "%d\t%d\t%.17g\t%.17g\t%.9g\t%.9g\n",
                                 j, k, c_lo1_ + c_h1_ * double(j),
                                 c_lo2_ + c_h2_ * double(k),
                                 double(tab_[c]), double(tab_[c + 1]));
                }
        }
        bool built() const { return construida_; }
#endif


        // Solves the OPEN system to true convergence. Called only when
        // BUILDING: the hot path solves nothing.
        void solve_node(double n19, double ieq, double& x, double& ib) const
        {
            const double vbc = n19 - kVCC;
            for (int it = 0; it < 200; ++it) {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, n19 - x, vbc, false);
                const double f  = (o.ib + o.ic) - x * g13_ - (x * geq_ - ieq);
                const double df = -(o.dic_be + o.dib_be) - g13_ - geq_;
                double step = f / df;
                if (step >  0.3) step =  0.3;
                if (step < -0.3) step = -0.3;
                x -= step;
                if (step < 1e-15 && step > -1e-15) break;
            }
            ib = mna::Engine::bjt(qm_, jbe_, jbc_, n19 - x, vbc, false).ib;
        }

        // The CLOSED system (design A): both unknowns at once, with
        // `process_loop`'s same 2x2 but converged.
        void solve_node_loop(double a0, double kz, double ieq,
                                double& x, double& v, double& ib) const
        {
            for (int it = 0; it < 200; ++it) {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, v - x, v - kVCC, false);
                const double gm = o.dib_be + o.dic_be;
                const double gcb = o.dib_bc + o.dic_bc;
                const double f1 = (o.ib + o.ic) - x * g13_ - (x * geq_ - ieq);
                const double f2 = a0 + kz * o.ib - v;
                const double j11 = -gm - g13_ - geq_;
                const double j12 = gm + gcb;
                const double j21 = -kz * o.dib_be;
                const double j22 = kz * (o.dib_be + o.dib_bc) - 1.0;
                const double det = j11 * j22 - j12 * j21;
                const double inv = (det > -1e-300 && det < 1e-300) ? 0.0 : 1.0 / det;
                double px = (f1 * j22 - f2 * j12) * inv;
                double pv = (j11 * f2 - j21 * f1) * inv;
                if (px >  0.3) px =  0.3;
                if (px < -0.3) px = -0.3;
                if (pv >  0.3) pv =  0.3;
                if (pv < -0.3) pv = -0.3;
                x -= px; v -= pv;
                if (px < 1e-15 && px > -1e-15 && pv < 1e-15 && pv > -1e-15) break;
            }
            ib = mna::Engine::bjt(qm_, jbe_, jbc_, v - x, v - kVCC, false).ib;
        }

        void build_cache(double n19_rep, double n17_rep, double kz)
        {
            // REBUILDING HERE IS AN XRUN, AND IT IS MEASURED: building
            // the table costs 4,1 ms (design B) / 5,2 ms (design A) against
            // the 1,333 ms of a 64-sample block at 48 kHz. And `prepare()`
            // is called on a knob move.
            //
            // => Design B depends on NO knob: its table only changes with
            // `fs` or the VARIANT (and the rest, which moves 1,9 mV over the
            // whole drive travel — nothing against the ±1,1 V box). So it
            // compares against what was built and BAILS.
            // Design A DOES depend (`kz` is the LEVEL), which is why it
            // pays the xrun on every knob move: that is what rules it out,
            // not its per-sample cost, the best of the three.
            //
            // AND THE GUARD WAS NOT ENOUGH. It survives the KNOBS — the
            // rest moves 1,9 mV against a 10 threshold — but it also
            // compares `g13_`, R13's conductance, which is exactly what
            // distinguishes the variants => the `variant` port invalidated it
            // and rebuilt ON THE AUDIO THREAD. MEASURED with
            // `make coste-prepare-casc`: `variant` 4,33-6,25 ms and the OS
            // factor 4,34-5,06 ms against 0,002 and 0,013 without the
            // cache. And the worst
            // case was entering THE 808 — the variant that never reads the
            // table.
            // => It no longer builds here: the geometry is fixed (scalar
            // arithmetic) and the table left PENDING. While it is,
            // `in_box()` says no and THE NEWTON RUNS — exactly what the
            // out-of-box failure path already did, carrying 83 % of the hard
            // stimulus's samples while tying on clock. The fill goes in
            // slices via `advance_cache()`.
#if NLSC_E4_CACHE == 2
            const bool mismo_kz = (kz == c_kz_hecho_);   // table A carries it inside
#else
            const bool mismo_kz = true;                  // table B ignores it
#endif
            const bool mismos_parametros =
                mismo_kz && geq_ == c_geq_done_ && g13_ == c_g13_hecho_ &&
                n19_rep > c_p0_hecho_ - 1e-2 && n19_rep < c_p0_hecho_ + 1e-2;
            // `cache_pend_` BELONGS IN THE GUARD, and it is no ornament:
            // without it, a `prepare()` with the SAME parameters mid-fill
            // re-arms the cursor at zero. With a knob the host interpolates,
            // that is a fill that never ends and a table that never comes
            // back — no error raised, the Newton running forever at the old
            // cost. A SILENT failure, the expensive kind.
            if ((construida_ || cache_pend_) && mismos_parametros) {
                return;
            }
            construida_ = true;
            c_geq_done_ = geq_; c_g13_hecho_ = g13_;
            c_kz_hecho_ = kz;    c_p0_hecho_ = n19_rep;
            // The box is ANCHORED TO THE REST POINT, not to absolute
            // constants: it follows the knobs (which move the rest) and the
            // rate (which moves `geq`) on its own, instead of inheriting the
            // box of the machine where it was measured.
            c_lo1_ = n19_rep - NLSC_E4_CACHE_DN19;
            const double hi1 = n19_rep + NLSC_E4_CACHE_DN19;
            c_lo2_ = geq_ * (n17_rep + (NLSC_E4_CACHE_VC_LO));
            const double hi2 = geq_ * (n17_rep + (NLSC_E4_CACHE_VC_HI));
            c_h1_ = (hi1 - c_lo1_) / double(kCN1 - 1);
            c_h2_ = (hi2 - c_lo2_) / double(kCN2 - 1);
            c_inv1_ = 1.0 / c_h1_;
            c_inv2_ = 1.0 / c_h2_;
            out_of_box_ = 0;
            // WHAT IS STORED IS THE DEVIATION FROM A LINE, NOT `n17`.
            //
            // Storing `n17` in `float` sets a FLOOR no grid can lower:
            // `n17` is ~3,7 V and `float`'s relative eps is 6e-8, i.e.
            // 2,2e-7 V of FIXED quantisation. At `lvl = 0` the plugin's
            // output has 9,1e-4 V rms => that is -72 dB, and measured it came
            // out -70,9: refining 400x16 to 3200x256 moved nothing because
            // the limit was not the interpolation. The follower's line is
            // subtracted BEFORE storing and added back on read: the
            // deviation is millivolts, so the same `float` gives four more
            // orders of room.
#if NLSC_E4_CACHE_LINE
            c_x0_ = n17_rep;
            c_p0_ = n19_rep;
            c_g_  = kGanSeg;
#else
            c_x0_ = 0.0; c_p0_ = 0.0; c_g_ = 0.0;   // ABSOLUTE `n17` is stored
#endif
#if NLSC_E4_CACHE == 2
            // DESIGN A KEEPS BUILDING ALL AT ONCE, on purpose: its
            // predictor reads `ib_`, which is LIVE STATE, so slicing it
            // would build each column from a different `ib_` — columns from
            // different instants in one table. Not fixed because A is
            // already ruled out by another measured reason: its table
            // depends on `kz`, i.e. on LEVEL, so it rebuilds on a knob move.
            for (int k = 0; k < kCN2; ++k) {
                const double ieq = c_lo2_ + c_h2_ * double(k);
                double x = n17_rep;
                double v = c_lo1_;
                for (int j = 0; j < kCN1; ++j) {
                    const double p1 = c_lo1_ + c_h1_ * double(j);
                    double ib = 0.0;
                    v = p1 + kz * ib_;                 // loop predictor
                    solve_node_loop(p1, kz, ieq, x, v, ib);
                    const size_t c = (size_t(j) * size_t(kCN2) + size_t(k)) * 2;
                    tab_[c]     = float(x - (c_x0_ + c_g_ * (p1 - c_p0_)));
                    tab_[c + 1] = float(ib);
                }
            }
#else
            (void)kz;
            // Here only the cursor is ARMED. `construida_` stays false
            // until `advance_cache()` finishes, and meanwhile `in_box()`
            // sends every sample to the Newton.
            construida_    = false;
            cache_pend_    = true;
            cur_k_ = 0; cur_j_ = 0;
            cur_x_ = n17_rep;      // `tab_` stores deviations: no good as a start
            c_n17_rep_ = n17_rep;  // each new column's warm start

            // AND DEFERRAL IS REQUESTED BY WHOEVER KNOWS IT IS ON THE
            // AUDIO THREAD — the plugin core. By DEFAULT it builds all at
            // once, right here.
            //
            // The reason is not convenience: were the fill ALWAYS deferred,
            // any harness driving `Cascade4` directly — and there are
            // several: `run_cascada_spice`, `run_cascada4_fin`, `cache_caja`,
            // the cost ones — would NEVER fill the table, because none has a
            // block boundary from which to call `advance_cache()`. They
            // would not error: they would report the NEWTON's figures
            // labelled as the cache's.
            //
            // Draining the cursor here does EXACTLY the same work, in the
            // same order and with the same warm start as the old all-at-once
            // loop => the table comes out bit-identical, and that is the gate.
            if (!deferred_fill_) {
                while (!advance_cache(kCN1)) { }
            }
#endif
        }

        // THE SLICED FILL — what turns an xrun into nothing.
        //
        // Called ONCE per block from `Plugin::process`, not per sample.
        // Each call solves at most `nodos` grid points and remembers where
        // it was. The per-block cost is bounded and DETERMINISTIC, with no
        // threads and no allocation: the table is a fixed-size member.
        //
        // The warm start survives because `cur_x_` travels between slices:
        // node `j-1`'s solution is node `j`'s starting point, as in the
        // all-at-once loop. Cutting a column in half costs nothing — only
        // `x` must come along.
        // And slicing is safe because `solve_node()` is `const` and
        // reads not one byte of per-sample state: only `qm_`, `jbe_`,
        // `jbc_`, `g13_` and `geq_`, which `prepare()` fixes. If a knob
        // changes them mid-fill, `build_cache()` re-arms the cursor from
        // zero and the half-built table is discarded whole.
        //
        // Returns true if THIS call left the table FINISHED.
        bool advance_cache(int nodes)
        {
#if NLSC_E4_CACHE == 1
            if (!cache_pend_) return false;
            while (nodes-- > 0) {
                const double ieq = c_lo2_ + c_h2_ * double(cur_k_);
                const double p1  = c_lo1_ + c_h1_ * double(cur_j_);
                double ib = 0.0;
                solve_node(p1, ieq, cur_x_, ib);
                const size_t c = (size_t(cur_j_) * size_t(kCN2) + size_t(cur_k_)) * 2;
                tab_[c]     = float(cur_x_ - (c_x0_ + c_g_ * (p1 - c_p0_)));
                tab_[c + 1] = float(ib);
                if (++cur_j_ >= kCN1) {          // column finished
                    cur_j_ = 0;
                    cur_x_ = c_n17_rep_;         // next one starts from rest
                    if (++cur_k_ >= kCN2) {      // table finished
                        cache_pend_ = false;
                        construida_ = true;
                        return true;
                    }
                }
            }
            return false;
#else
            (void)nodes;
            return false;
#endif
        }

        // Observability: how many fill slices remain, so "the table comes
        // back" is CHECKABLE and not an assumption.
        void defer_fill(bool v) { deferred_fill_ = v; }

        // THE FILL, ADVANCED PER SAMPLE — `NLSC_E4_FILL_PER_SAMPLE`.
        //
        // WHY THE PER-BLOCK ONE BREAKS INVARIANCE: `Plugin::process` calls
        // `advance_cache()` ONCE per block, so how many nodes are done at
        // sample `n` depends on how the host slices — and the reference run,
        // in ONE block, fills the table at once. That was the declared
        // compromise (3,8e−06, later 1,5e−05 with the charges).
        //
        // The earlier attempt failed narrowly: node count proportional to
        // SAMPLES but still called per BLOCK, so the slicing still ruled
        // (3,923e−06 -> 7,659e−06, and from failing one test to two). The
        // document concluded "progressive fill and block bit-identity are
        // incompatible" — and they are NOT: what must move is not the HOW
        // MUCH but the WHEN.
        //
        // => Here it advances INSIDE `process`, i.e. BETWEEN samples. The
        // schedule becomes a function of the SAMPLE INDEX and stops seeing
        // the slicing: one 1024 block and sixteen of 64 fill exactly alike,
        // because both process the same samples in the same order.
        // TOTAL work is unchanged: 2 nodes per internal sample is the
        // same rate as 512 per 64-block at 4x (25.600 nodes / 12.800
        // samples).
        void advance_fill_per_sample()
        {
#if NLSC_E4_CACHE == 1 && NLSC_E4_FILL_PER_SAMPLE
            if (deferred_fill_ && cache_pend_)
                advance_cache(NLSC_E4_CACHE_NODES_PER_SAMPLE);
#endif
        }
        bool cache_pending() const { return cache_pend_; }
        long cache_nodes_left() const
        {
            if (!cache_pend_) return 0;
            return long(kCN2 - cur_k_ - 1) * long(kCN1) + long(kCN1 - cur_j_);
        }

        // IS THIS POINT INSIDE? Asked BEFORE reading; outside it falls to
        // the Newton instead of clipping.
        //
        // Clamping was the historical failure: the table returned the
        // edge node and carried on, and the grid against ngspice sank to
        // −33,7 dB at max tone + max level (4 of 36 points failing at 4x,
        // against 0 of 36 without the cache) with the MEDIAN unmoved. The
        // box had been sized with the DI, and with the grid's sine the
        // `ieq` axis reaches node 84 of a 0..31 box.
        //
        // => A table is an ACCELERATOR, not a substitute: where it does not
        // reach, solve. The per-sample cost stops being constant, and that
        // is deliberate — a constant cost that sometimes LIES is worthless.
        bool in_box(double p1, double ieq) const
        {
            // A HALF-BUILT TABLE IS NOT "INSIDE" EITHER. Same answer as
            // outside the box — solve — which is why slice filling needs no
            // new path and no new test: it reuses the fallback that already
            // carries 83 % of the hard stimulus's samples. It sits here
            // and not at the read site because this is the ONLY choke:
            // `process_loop` asks up to `kCIt` times per sample and all must
            // see the same thing.
            if (!construida_) return false;
            const double u = (p1 - c_lo1_) * c_inv1_;
            if (!(u >= 0.0 && u <= double(kCN1 - 1))) return false;
            const double w = (ieq - c_lo2_) * c_inv2_;
            return (w >= 0.0 && w <= double(kCN2 - 1));
        }

        // ACCOUNTING GOES ON THE FAILURE PATH, ONCE PER SAMPLE.
        // It CANNOT live inside `in_box()`: `process_loop` calls that up
        // to `kCIt` times per sample and it would be counting the fixed
        // point's TRIALS, not samples.
        void note_outside(double p1, double ieq)
        {
#if NLSC_E4_CACHE_PROBE
            const double u = (p1 - c_lo1_) * c_inv1_;
            if (u < 0.0)                   { ++fuera_p_; exceso_p_ = std::max(exceso_p_, -u); }
            else if (u > double(kCN1 - 1)) { ++fuera_p_; exceso_p_ = std::max(exceso_p_, u - double(kCN1 - 1)); }
            const double w = (ieq - c_lo2_) * c_inv2_;
            if (w < 0.0)                   { ++fuera_i_; exceso_i_ = std::max(exceso_i_, -w); }
            else if (w > double(kCN2 - 1)) { ++fuera_i_; exceso_i_ = std::max(exceso_i_, w - double(kCN2 - 1)); }
#else
            (void)p1; (void)ieq;
#endif
        }

        // The `ieq` axis's TRAVEL is logged ALWAYS, inside and out: it is
        // the magnitude the box is SIZED with, and logging it only inside
        // leaves it saturated at the edge — exactly what read "31,00 of a
        // 0..31 box" while the stimulus reached node 84.
        void note_range(double ieq)
        {
#if NLSC_E4_CACHE_PROBE
            const double w = (ieq - c_lo2_) * c_inv2_;
            w_min_ = std::min(w_min_, w); w_max_ = std::max(w_max_, w);
#else
            (void)ieq;
#endif
        }

        // Bilinear read: four accesses, two outputs, all in `float`.
        // Called only with `in_box()` true: the index clamp below is a
        // SAFETY NET against a NaN, not an operating mode.
        void read_cache(double p1, double ieq, double& x, double& ib)
        {
            // NOTHING IS COUNTED HERE ANY MORE. These clamps used to
            // count box exits, and since `in_box()` decides before entry
            // this point is only reached from INSIDE: the per-axis counters
            // read 0 and the travel saturated at the edge exactly where
            // 83 % of the samples left. A counter that only counts the good
            // cases is not a counter — the count happens on the FAILURE
            // path, in `note_outside()`.
            // => What remains is the SAFETY NET against a NaN, which is what
            // the comment above always said: not an operating mode.
            double u = (p1 - c_lo1_) * c_inv1_;
            int j = int(u);
            if (j < 0)             { j = 0; }
            else if (j > kCN1 - 2) { j = kCN1 - 2; }
            const float fu = float(u - double(j));
#if NLSC_E4_IEQ_CEILING
            // CEILING PROBE — NOT A CANDIDATE, IT GIVES WRONG NUMBERS.
            // Pins the cache's second axis so the compiler removes its WHOLE
            // path (the axis computation, the clamps, half the loads and four
            // of the six lerps). What it measures is the MAXIMUM the
            // second-axis lever could buy, not what a correct implementation
            // would: this one loses the `ieq` dependence entirely, and with it
            // the fidelity.
            (void)ieq; const int k = 0; const float fw = 0.0f;
#else
            double w = (ieq - c_lo2_) * c_inv2_;
            int k = int(w);
            if (k < 0)             { k = 0; }
            else if (k > kCN2 - 2) { k = kCN2 - 2; }
            const float fw = float(w - double(k));
#endif
            const size_t c = (size_t(j) * size_t(kCN2) + size_t(k)) * 2;
            const size_t d = c + size_t(kCN2) * 2;
            const float a0x = tab_[c],     a0i = tab_[c + 1];
            const float a1x = tab_[c + 2], a1i = tab_[c + 3];
            const float b0x = tab_[d],     b0i = tab_[d + 1];
            const float b1x = tab_[d + 2], b1i = tab_[d + 3];
            const float mx = (a0x + (a1x - a0x) * fw);
            const float nx = (b0x + (b1x - b0x) * fw);
            const float mi = (a0i + (a1i - a0i) * fw);
            const float ni = (b0i + (b1i - b0i) * fw);
            x  = c_x0_ + c_g_ * (p1 - c_p0_) + double(mx + (nx - mx) * fu);
            ib = double(mi + (ni - mi) * fu);
        }
#endif  // NLSC_E4_CACHE


#if NLSC_E4_CHARGES
        // The PHYSICAL base current: static + displacement. It is what
        // leaves the block towards `Z(s;lvl)`, where the null measures it.
        double ib() const { return ib_ + idesp_; }
        double ib_estatica() const { return ib_; }
        double idesp() const { return idesp_; }
#else
        double ib() const { return ib_; }
#endif
#ifdef NLSC_E4_IB_PROBE
        // PROBE (not product). The cache table is `(n19, ieq) -> (n17,
        // ib)`, so `ib` depends on TWO variables: the question is not
        // whether it depends on `ieq` but HOW MUCH, which decides whether
        // `ib = g(n19)` alone can carry the open-base fix to the 808's
        // TABULATED path.
        double ieq_probe() const { return ieq_probe_; }
        double ieq_probe_ = 0.0;
#endif

        // The base current AT REST, with the SAME `bjt()` as the loop:
        // computing it apart (or tabulating it) is the recipe for the two
        // to drift.
        double ib_rest(double n19_q, double n17_q)
        {
            qm_.derive();
            // With RB the junction is not at the terminal but at `n19 - RB*ib`,
            // so `ib` is a FIXED POINT. Its loop gain is `RB*dib/dvbe ~
            // 50 * gm/BF ~ 1,4e-3`, so three steps are already plenty; four are
            // taken, and with `rb_ = 0` the loop returns the previous value
            // on the FIRST pass, bit-identical.
            double ib = 0.0;
            for (int k = 0; k < 4; ++k) {
                const double u = n19_q - rb_ * ib;
                ib = mna::Engine::bjt(qm_, jbe_, jbc_, u - n17_q,
                                      u - kVCC, false).ib;
            }
            return ib;
        }

#ifdef NLSC_E4_REST_PROBE
        // SUBSYSTEM REST PROBE — the KIRCHHOFF residual at the TABULATED
        // point, with no signal and no ngspice. Constants against constants.
        //
        // THE QUESTION: the rest point in `nls_reposo_drive*.h` comes from
        // ngspice's `.op`, so it is the physically correct point. But the solver
        // here does not land on it (`w(0) = +35 uV`, `dev_n19` up to 1,3 mV).
        // => Does that point satisfy the equations THIS solver solves?
        //
        // At rest the two equations of the 2x2 Newton simplify on their own:
        //   f2 = a0 + kz*(ib + idesp) - v.  With `a0 = n19_q - kz*ib0` and
        //        `ib == ib0` by construction (same function, same arguments),
        //        what is left is **f2 = kz*idesp**, and `seed_charges()` leaves
        //        `idesp = 0` exactly at the tabulated point => **f2 = 0**.
        //   f1 = (ib+ic) - n17*g13 - (n17*geq - ieq).  The C9 branch does not
        //        conduct at DC (`vc_ = n17`, `i_prev_ = 0`)
        //        => **f1 = ie - n17/R13**.
        // => ALL the residual lives in f1: node n17's KCL.
        //
        // Returns the imbalance in amperes and, by scalar Newton over the SAME
        // `bjt()`, the `n17` that does cancel it.
        struct RestResid { double f1, ie, i_r13, n17_fix, dn17; int it; };
        RestResid residuo_en_reposo(double n19_q, double n17_q)
        {
            qm_.derive();
            auto ie_de = [&](double n17) {
                const mna::Engine::QOut o = mna::Engine::bjt(
                    qm_, jbe_, jbc_, n19_q - n17, n19_q - kVCC, false);
                return o.ib + o.ic;
            };
            RestResid r{};
            r.ie    = ie_de(n17_q);
            r.i_r13 = n17_q * g13_;
            r.f1    = r.ie - r.i_r13;
            // The n17 that satisfies the KCL, with the SAME transistor model.
            double x = n17_q;
            for (r.it = 0; r.it < 60; ++r.it) {
                const mna::Engine::QOut o = mna::Engine::bjt(
                    qm_, jbe_, jbc_, n19_q - x, n19_q - kVCC, false);
                const double f  = (o.ib + o.ic) - x * g13_;
                const double df = -(o.dic_be + o.dib_be) - g13_;
                const double p  = f / df;
                x -= p;
                if (p < 1e-14 && p > -1e-14) break;
            }
            r.n17_fix = x;
            r.dn17    = x - n17_q;
            return r;
        }
        double g13_probe() const { return g13_; }
        double geq_probe() const { return geq_; }
        double rb_probe() const { return qm_.RB; }
        // THE SAME residual but with the INTERNAL BASE: the junction is
        // evaluated at `n19 - RB*ib` rather than at the terminal. A fixed point,
        // because the loop gain is `RB*dib/dvbe ~ 50 * gm/BF ~ 1,4e-3` and it
        // converges with room to spare.
        // THE RESIDUAL OF THE SHIPPED CONFIGURATION: it uses `rb_`, that is,
        // what the binary really carries. The other two pin RB to 0 and to
        // `qm_.RB` so the three can be COMPARED; this is the one that can
        // SCORE, because an arbiter has to compile the configuration that
        // ships.
        RestResid residuo_en_reposo_producto(double n19_q, double n17_q)
        {
            const double guarda = rb_;
            qm_.derive();
            RestResid r{};
            double ib = 0.0;
            for (int k = 0; k < 8; ++k) {
                const double u = n19_q - guarda * ib;
                const mna::Engine::QOut o = mna::Engine::bjt(
                    qm_, jbe_, jbc_, u - n17_q, u - kVCC, false);
                ib = o.ib; r.ie = o.ib + o.ic;
            }
            r.i_r13 = n17_q * g13_;
            r.f1 = r.ie - r.i_r13;
            double x = n17_q, ibx = ib;
            for (r.it = 0; r.it < 80; ++r.it) {
                const double u = n19_q - guarda * ibx;
                const mna::Engine::QOut o = mna::Engine::bjt(
                    qm_, jbe_, jbc_, u - x, u - kVCC, false);
                ibx = o.ib;
                const double pp = ((o.ib + o.ic) - x * g13_)
                                / (-(o.dic_be + o.dib_be) - g13_);
                x -= pp;
                if (pp < 1e-14 && pp > -1e-14) break;
            }
            r.n17_fix = x; r.dn17 = x - n17_q;
            return r;
        }
        RestResid residuo_en_reposo_rb(double n19_q, double n17_q, int pasos = 8)
        {
            qm_.derive();
            RestResid r{};
            double ib = 0.0;
            for (int k = 0; k < pasos; ++k) {
                const double u = n19_q - qm_.RB * ib;
                const mna::Engine::QOut o = mna::Engine::bjt(
                    qm_, jbe_, jbc_, u - n17_q, u - kVCC, false);
                ib = o.ib;
                r.ie = o.ib + o.ic;
            }
            r.i_r13 = n17_q * g13_;
            r.f1    = r.ie - r.i_r13;
            double x = n17_q, ibx = ib;
            for (r.it = 0; r.it < 80; ++r.it) {
                const double u = n19_q - qm_.RB * ibx;
                const mna::Engine::QOut o = mna::Engine::bjt(
                    qm_, jbe_, jbc_, u - x, u - kVCC, false);
                ibx = o.ib;
                const double f  = (o.ib + o.ic) - x * g13_;
                const double df = -(o.dic_be + o.dib_be) - g13_;
                const double pp = f / df;
                x -= pp;
                if (pp < 1e-14 && pp > -1e-14) break;
            }
            r.n17_fix = x;
            r.dn17    = x - n17_q;
            return r;
        }
#endif

        void reset(double n17_rest)
        {
            x_ = vc_ = n17_rest;                    // at rest nothing flows in C9
            i_prev_ = 0.0;
            // `n19_prev_` is the Newton's variable, and with RB that variable
            // is the INTERNAL base: it is seeded with `u_rest_`, not with the
            // terminal.
            n19_prev_ = u_rest_;
            // `ib_` is `process_loop`'s PREDICTOR, so starting it at zero
            // injects a `kz·ib0` jump on the first sample. It is seeded with
            // the rest point, and with the loop's SAME `bjt()`.
            ib_ = ib_rest(n19_rest_, n17_rest);
#if NLSC_E4_CHARGES
            // The charges belong to the JUNCTION, so they are seeded at the
            // INTERNAL base: seeding them at the terminal would leave
            // `idesp != 0` at rest, and that is exactly the t=0 step being
            // hunted here.
            seed_charges(u_rest_, n17_rest);
#endif
        }

        double process(double n19_abs)
        {
            // ORACLE INJECTION path (`--ib` exact), not the one that ships.
            // With RB the junction is not at the terminal here either:
            // `u = n19 - RB*ib`, a fixed point of gain ~1,4e-3 seeded with the
            // previous sample's `ib_`, so converged with room to spare in one
            // step. With `rb_ = 0` it is the assignment it always was.
            n19_abs -= rb_ * ib();
#if NLSC_E4_CACHE == 1
            // The guard is NOT cosmetic: `advance_fill_per_sample()` is
            // defined INSIDE `#if NLSC_E4_CACHE == 1`, so without it the
            // CONTROL arm (`-DNLSC_E4_CACHE=0`) does not compile. A control
            // that does not compile gives no warning: it fails the day it
            // is needed.
            advance_fill_per_sample();
#endif
            const double ieq = geq_ * (vc_ + i_prev_ * inv_gc_);
#ifdef NLSC_E4_IB_PROBE
            ieq_probe_ = ieq;   // PROBE: the SECOND variable `ib` depends on
#endif
#if NLSC_E4_CACHE == 1
            note_range(ieq);
            // DESIGN B: the table IS this system => inside the box there is
            // no Newton. Outside, it falls to it without touching the state.
            if (in_box(n19_abs, ieq)) {
                read_cache(n19_abs, ieq, x_, ib_);
#if NLSC_E4_CHARGES
                // No Newton here, so the transistor evaluation is paid:
                // one per sample, against the 2-3 iterations saved.
                advance_charges_from(n19_abs - x_, n19_abs - kVCC);
#endif
                n19_prev_ = n19_abs;
                ++samples_;
                const double i_branch = x_ * geq_ - ieq;
                vc_ += (i_branch + i_prev_) * k_vc_;
                i_prev_ = i_branch;
                return x_;
            }
            ++out_of_box_;
            note_outside(n19_abs, ieq);
#endif
            const double vbc = n19_abs - kVCC;
            // THE FOLLOWER'S PREDICTOR. Starting the Newton from the
            // previous sample is starting from an OLD value; starting from
            // `x + g·Δn19` is starting where the circuit is going to be,
            // because an emitter follower has near-constant gain (0,94-0,97
            // measured). Costs one multiply-add and saves whole iterations,
            // which is where the exponentials live.
            // `g` is NOT tuned: it is the follower's gain, and if it is
            // wrong all that happens is the Newton takes one more iteration
            // — the solution, the fixed point, never changes.
            x_ += kGanSeg * (n19_abs - n19_prev_);
            n19_prev_ = n19_abs;
            // Scalar Newton. Starts from the previous sample, which at
            // 192 kHz sits microvolts away: 2-3 typical iterations.
            // THE NEWTON'S BUDGET, and it is not a style detail. With 30
            // iterations and 1e-13 V tolerance, measured: the paired
            // cascade/engine ratio went from 0,678 to 1,049 — the cascade
            // stopped being cheaper than the DK, its only reason to exist.
            // 1e-13 V on a 1 V signal asks 13 digits of a node whose model
            // is worth −70 dB: spending iterations below the model's
            // error.
            int it = 0;
            for (; it < kMaxIter; ++it) {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, n19_abs - x_, vbc, false);
                const double f  = (o.ib + o.ic) - x_ * g13_ - (x_ * geq_ - ieq);
                const double df = -(o.dic_be + o.dib_be) - g13_ - geq_;
                double step = f / df;
                // The step is clamped: without it a distant start throws
                // the exponential to infinity and the engine never returns.
                if (step >  0.3) step =  0.3;
                if (step < -0.3) step = -0.3;
                x_ -= step;
                if (step < kTolNewton && step > -kTolNewton) break;
            }
            // ESCAPE COUNTER. The DK counts its own, which is how its
            // belt was caught; this stage used to run up to `kMaxIter` and
            // leave on tolerance WITHOUT MARKING non-convergence, so "the
            // cap does not bite" was indistinguishable from "it always
            // bites". Only exhausting the budget WITHOUT meeting tolerance
            // is counted: leaving via the `break` is convergence.
            // No new branches on the normal path: `it == kMaxIter` is
            // only true when the loop ran out.
            if (it == kMaxIter) ++unconverged_;
            ++samples_;
            // The last sample's BASE current. Already computed: all that
            // was needed was not to throw it away.
            {
                // `kCharges` instead of `false`: this evaluation was
                // already made, so on the Newton path the charges are free.
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, n19_abs - x_, vbc, kExactCharges);
                ib_ = o.ib;
#ifdef NLSC_E4_LOOPGAIN_PROBE
                // PROBE (guarded, not product): D'ANGELO'S ENTRY CONDITION.
                //
                // `Publication VI` of his thesis turns an implicitly defined
                // nonlinear SISO system into an EXPLICIT equivalent, and it is
                // the only method of that family that handles STATEFUL
                // components -- which is what this subsystem is. Its published
                // viability condition is that linearising around ALL operating
                // points gives a SUB-UNITY feedback gain at every frequency.
                //
                // Here that number is closed-form. The implicit equation is
                //     f(x) = i(n19 - x) + ieq - x*(g13 + geq) = 0,
                // whose fixed-point map is x <- (i(n19-x) + ieq)/(g13 + geq),
                // so the loop gain is |di/dvbe| / (g13 + geq) -- the
                // transistor's transconductance against the branch's total
                // conductance.
                //
                // IT IS EVALUATED HERE, AFTER THE LOOP, ON PURPOSE. Inside
                // the Newton it would measure the TRIALS -- points the solver
                // evaluates and then rejects -- and the condition is about
                // the OPERATING POINT
                // -- a probe inside the solver measures the trials.
                // And it is free: this `bjt()` call already happened, and
                // `QOut` already carries the derivatives. Nothing is added but
                // a divide behind a `#ifdef`.
                {
                    const double gl = std::fabs(o.dib_be + o.dic_be) / (g13_ + geq_);
                    if (gl > lg_max_) lg_max_ = gl;
                    lg_sum_ += gl; ++lg_n_;
                }
#endif
#if NLSC_E4_CHARGES == 2
                advance_charges_from(n19_abs - x_, vbc);
#elif NLSC_E4_CHARGES
                advance_charges(o.qbe, o.qbc);
#endif
            }
            const double i_branch = x_ * geq_ - ieq;
            vc_ += (i_branch + i_prev_) * k_vc_;
            i_prev_ = i_branch;
            return x_;
        }

        // THE SAME SUBSYSTEM, WITH `n19` ALSO UNKNOWN.
        //
        // `process()` receives `n19` ready-made, which forces its caller to
        // have used the PREVIOUS sample's base current. Here `n19` enters
        // as the LINE the level network imposes,
        //
        //     n19 = a0 + kz * ib
        //
        // and is solved together with `n17`. The cost is a 2x2 instead of
        // a division: THE SAME single `bjt()` call per iteration, which is
        // where the exponentials live.
        // That does NOT make it free, and the figure is measured, not
        // assumed: paired on the i7-8700K, +5,49 % over the delayed version
        // (5,46 -> 5,76 % of one core, 0/21 pairs, p = 9,5e-7). What the
        // 2x2 saves is the second round of exponentials iterating outside
        // would cost.
        //
        // The two equations, with `vbe = v - x` and `vbc = v - VCC`:
        //   f1 = (ib + ic) - x*g13 - (x*geq - ieq)          [KCL at n17]
        //   f2 = a0 + kz*ib - v                             [the level network]
        // and the Jacobian comes whole from the four derivatives `bjt()`
        // ALREADY returns — no finite differences, no fitted parameter.
        //
        // `kz = 0` leaves it EXACTLY in the old case with `n19 = a0`: f2
        // pins `v` and the 2x2 reduces to the scalar. That is the control
        // that compares the two branches without changing code path.
        double process_loop(double a0, double kz, double& n19_out)
        {
            // THE CHANGE OF VARIABLE. The level network imposes
            // `n19 = a0 + kz*ib_total`, and Q2's INTERNAL base is
            // `u = n19 - RB*ib_total` => **`u = a0 + (kz - RB)*ib_total`**.
            // Same form, same unknown: the Newton solves in `u` with `kzp` and
            // `n19` is recovered at the end with the real `kz`. With
            // `rb_ = 0`, `kzp == kz` and all of this is the previous
            // arithmetic, bit for bit.
            const double kzp = kz - rb_;
#if NLSC_E4_CACHE == 1
            // The guard is NOT cosmetic: `advance_fill_per_sample()` is
            // defined INSIDE `#if NLSC_E4_CACHE == 1`, so without it the
            // CONTROL arm (`-DNLSC_E4_CACHE=0`) does not compile. A control
            // that does not compile gives no warning: it fails the day it
            // is needed.
            advance_fill_per_sample();
#endif
            const double ieq = geq_ * (vc_ + i_prev_ * inv_gc_);
#ifdef NLSC_E4_IB_PROBE
            ieq_probe_ = ieq;   // PROBE: the SECOND variable `ib` depends on
#endif
#if NLSC_E4_CACHE
            note_range(ieq);
            // Work happens on COPIES and commits only if ALL the reads
            // fell inside: if any leaves, the state must stay intact so the
            // Newton below starts exactly where it would have without the
            // cache.
            {
                double xt = x_, ibt = ib_;
                bool dentro = true;
#  if NLSC_E4_CACHE == 2
                // DESIGN A: the table ALREADY carries the loop (built with
                // this `kz`), so it is ONE read and done.
                double v = 0.0;
                if (in_box(a0, ieq)) { read_cache(a0, ieq, xt, ibt); v = a0 + kzp * ibt; }
                else dentro = false;
#  else
                // DESIGN B: the table is the open system's and the affine
                // loop closes OUTSIDE by fixed point. Loop gain is ~0,09 at
                // the worst `lvl`, so it converges in one iteration — and
                // the step count is a CONSTANT, not a `while`.
                // THE FIXED POINT CARRIES THE DISPLACEMENT INSIDE —
                // MEASURED, not taste. The alternatives, on the 9RI's block
                // invariance (declared cap 5,0e−06):
                //   · displacement INSIDE (this)              -> 1,52e−05
                //   · box decided with the static `v`         -> 1,68e−04
                //   · no charges (baseline)                   -> 3,82e−06
                // => Taking it out leaves `x` and `n19` describing different
                // voltages and worsens 11x. It stays inside.
                double v = a0 + kzp * (ibt + idesp_de(n19_prev_ - xt, n19_prev_ - kVCC));
                for (int it = 0; it < kCIt && dentro; ++it) {
                    if (!in_box(v, ieq)) { dentro = false; break; }
                    read_cache(v, ieq, xt, ibt);
                    v = a0 + kzp * (ibt + idesp_de(v - xt, v - kVCC));
                }
#  endif
                if (dentro) {
                    x_ = xt; ib_ = ibt;
#if NLSC_E4_CHARGES
                    // `idesp_` is pinned with the fixed point's `v` and
                    // THEN `v` is recomputed with it: `n19 = a0 + kz·ib()`
                    // holds by CONSTRUCTION, as before the charges. What
                    // remains is the fixed point's truncation, which `kCIt`
                    // governs — the same treatment `ib` already had.
                    advance_charges_from(v - x_, v - kVCC);
                    v = a0 + kzp * (ib_ + idesp_);
#endif
                    ++samples_;
                    n19_prev_ = v;
                    // `v` is the INTERNAL base; what leaves is the TERMINAL,
                    // which is what `dev_n19` is measured against and what the
                    // waveshaper eats. It is recovered from the DEFINITION of
                    // the change of variable (`terminal = internal + RB*ib`) and
                    // NOT by recomputing from `a0`: that way, with `rb_ = 0`,
                    // this line is `n19_out = v` BIT FOR BIT, which is what
                    // makes the control arm usable.
                    n19_out = v + rb_ * ib();
                    const double i_branch = x_ * geq_ - ieq;
                    vc_ += (i_branch + i_prev_) * k_vc_;
                    i_prev_ = i_branch;
                    return x_;
                }
                ++out_of_box_;
                note_outside(v, ieq);
            }
#endif
            // Predictor: `v` from the previous sample's current (exactly
            // what the delayed version did, so it starts from ITS answer),
            // and `x` with the follower's gain.
            double v = a0 + kzp * ib_;
            x_ += kGanSeg * (v - n19_prev_);
            int it = 0;
            for (; it < kMaxIter; ++it) {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, v - x_, v - kVCC, false);
                const double gm = o.dib_be + o.dic_be;       // d(ib+ic)/d(vbe)
                const double gc = o.dib_bc + o.dic_bc;       // d(ib+ic)/d(vbc)
                const double f1 = (o.ib + o.ic) - x_ * g13_ - (x_ * geq_ - ieq);
                // THE DISPLACEMENT CURRENT ENTERS **INSIDE** THE LOOP.
                //
                // Putting it outside, with the previous sample's value,
                // stops closing the line `n19 = a0 + kz·ib` — and `make
                // lazo-ib` catches that: it demands the loop residual sit
                // 100x below the delayed branch. With the delay the ratio
                // fell to 1,97: the gate went RED, rightly.
                // Here it is nearly free because in mode 2 the charge is
                // `cj0_eff·v`: the Euler companion only needs the PREVIOUS
                // sample's `q`, a constant during this Newton.
                const double f2 = a0 + kzp * (o.ib + idesp_de(v - x_, v - kVCC)) - v;
                const double j11 = -gm - g13_ - geq_;        // df1/dx
                const double j12 = gm + gc;                  // df1/dv
                // Both displacement derivatives enter the Jacobian.
                // Without them Newton stops on step size with the loop's
                // residual at 2,6e−4 where the product gives EXACT ZERO.
                const double j21 = -kzp * o.dib_be + kzp * didesp_dx();   // df2/dx
                const double j22 = kzp * (o.dib_be + o.dib_bc) - 1.0
                                 + kzp * didesp_dv();                    // df2/dv
                const double det = j11 * j22 - j12 * j21;
                // The determinant CANNOT vanish in this circuit (j11 and
                // j22 are negative and dominate), but a tiny `det` from an
                // absurd excursion would launch the step to infinity as the
                // exponential used to. Clamped like the step.
                const double inv = (det > -1e-300 && det < 1e-300) ? 0.0 : 1.0 / det;
                double px = (f1 * j22 - f2 * j12) * inv;
                double pv = (j11 * f2 - j21 * f1) * inv;
                // The step is clamped as in the scalar case: without this a
                // distant start throws the exponential to infinity and the
                // engine never returns.
                if (px >  0.3) px =  0.3;
                if (px < -0.3) px = -0.3;
                if (pv >  0.3) pv =  0.3;
                if (pv < -0.3) pv = -0.3;
                x_ -= px;
                v  -= pv;
                if (px < kTolNewton && px > -kTolNewton &&
                    pv < kTolNewton && pv > -kTolNewton) break;
            }
            if (it == kMaxIter) ++unconverged_;
            ++samples_;
            {
                const mna::Engine::QOut o =
                    mna::Engine::bjt(qm_, jbe_, jbc_, v - x_, v - kVCC, kExactCharges);
                ib_ = o.ib;
#ifdef NLSC_E4_LOOPGAIN_PROBE
                // D'Angelo's entry condition, ON THE PATH THAT ACTUALLY
                // RUNS. The twin probe in `process()` measured ZERO samples:
                // `process()` is not the hot entry, `process_loop` is --- it is
                // the symbol the profile names at 26,7 %. A probe on a dead
                // path reports a green verdict over nothing, which reads
                // exactly like a pass.
                {
                    const double gl = std::fabs(o.dib_be + o.dic_be) / (g13_ + geq_);
                    if (gl > lg_max_) lg_max_ = gl;
                    lg_sum_ += gl; ++lg_n_;
                }
#endif
#if NLSC_E4_CHARGES == 2
                advance_charges_from(v - x_, v - kVCC);
#elif NLSC_E4_CHARGES
                advance_charges(o.qbe, o.qbc);
#endif
            }
            n19_prev_ = v;
            // `v` is the INTERNAL base (the Newton's variable); the TERMINAL is
            // what leaves, because `dev_n19` is measured against `q_n19_abs_` —
            // ngspice's `n19` — and it is what Q2's waveshaper eats.
            // By the DEFINITION of the change of variable, not by recomputing
            // from `a0`: with `rb_ = 0` this is `n19_out = v` bit for bit, and
            // without that the control arm would differ by the Newton's
            // residual.
            n19_out = v + rb_ * ib();
            const double i_branch = x_ * geq_ - ieq;
            vc_ += (i_branch + i_prev_) * k_vc_;
            i_prev_ = i_branch;
            return x_;
        }

    private:
        mna::QModel qm_{};
        mna::Engine::Junc jbe_{}, jbc_{};   // no charges: `with_charges = false`
        double g13_ = 0.0, geq_ = 0.0, inv_gc_ = 0.0, k_vc_ = 0.0;
        double x_ = 0.0, vc_ = 0.0, i_prev_ = 0.0, n19_prev_ = 0.0;
        double ib_ = 0.0;
        double n19_rest_ = 0.0;
        // Q2's base resistance. At 0 every formula below collapses to the one
        // that preceded it, which is the CONTROL arm.
        double rb_ = 0.0;
        double u_rest_ = 0.0;      // INTERNAL base at rest = n19_rest - RB*ib0
#if NLSC_E4_CACHE
        // FIXED size: `prepare()` is also called on a knob move, and no
        // memory gets requested there (the audio-thread rule: no heap).
        float tab_[size_t(kCN1) * size_t(kCN2) * 2] = {};
        double c_lo1_ = 0.0, c_lo2_ = 0.0, c_h1_ = 1.0, c_h2_ = 1.0;
        double c_inv1_ = 1.0, c_inv2_ = 1.0;
        double c_x0_ = 0.0, c_p0_ = 0.0, c_g_ = 0.0;  // the line subtracted on store
        bool   construida_ = false;
        // THE SLICE-FILL CURSOR: five scalars, and with them the table
        // stops being built on the audio thread.
        bool   cache_pend_ = false;      // geometry fixed, content half-done
        bool   deferred_fill_ = false;  // set ONLY by the plugin core
        int    cur_k_ = 0, cur_j_ = 0;   // where it was (column, node)
        double cur_x_ = 0.0;             // the warm start, BETWEEN slices
        double c_n17_rep_ = 0.0;         // rest each new column starts from
        double c_geq_done_ = 0.0, c_g13_hecho_ = 0.0, c_kz_hecho_ = 0.0, c_p0_hecho_ = 0.0;
        long out_of_box_ = 0, fuera_p_ = 0, fuera_i_ = 0;
        double exceso_p_ = 0.0, exceso_i_ = 0.0;
        double w_min_ = 1e300, w_max_ = -1e300;
#endif
    };

    // SEPARATE FROM `prepare()`: see the note at `Stage1::set_bank`.
    // Called BEFORE `set_rest()`, which subtracts `fit_n19_`/`fit_n17_`.
    // The bank is COPIED to members because it is read per sample.
    template <class J>
    void set_bank()
    {
        ws2_n_ = J::WS2_N;
        for (int i = 0; i < ws2_n_; ++i) ws2_[i] = J::WS2[i];
#if NLSC_WS_RECENTER
        ws2_[ws2_n_ - 1] = 0.0;   // see NLSC_WS_RECENTER in nls_stage1.h
#endif
        umin_ = J::WS2_UMIN;  umax_ = J::WS2_UMAX;
        fit_n19_ = J::Fit_n19;  fit_n17_ = J::Fit_n17;
        use_sub_ = J::use_sub;      // declared by the BANK, not an `idx == 1`
    }

    template <class J>
    void prepare_bank(double fs, double lvl, bool reinit)
    {
        set_bank<J>();
        g1_.prepare_bank<J>(fs, lvl, reinit);
        g1o_.prepare_bank<J>(fs, lvl, reinit);
        // `kz` enters HERE because the cache's design A depends on it (and
        // therefore on LEVEL): always passed, and the Newton ignores it.
        sub_.prepare(fs, r15_, r14_, q_n17_, q_n19_abs_, g1o_.ib_gain(),
                     reinit);
        ib0_ = sub_.ib_rest(q_n19_abs_, q_n17_);
        if (reinit) ib_prev_ = ib0_;
        gs_.prepare(fs, J::E4GS_B, J::E4GS_A, J::ScaleF, reinit);
        g2_.prepare(fs, J::E4G2_B, J::E4G2_A, J::ScaleF, reinit);
        // There is no self-seeding option here, and it was removed after being
        // measured rather than on taste.
        //
        // It seeded the rest point with THE MODEL'S OWN FIXED POINT instead of
        // with ngspice's `.op`. Both halves of the bargain:
        //   · PRICE: the null grid is identical in **36 of 36 cells**.
        //   · PRIZE: over digital silence the injected step comes out **83,6x**
        //     cleaner at sample 0, **1,1x** at sample 5, and by 0,4 s the sign
        //     INVERTS (rest offset +2,5e-7 against +1,7e-7 V). So it does not
        //     remove the transient: it delays it by ONE sample and leaves a
        //     permanent error in exchange.
        // => Both below what any published yardstick can see.
        //
        // And the "5,5x cleaner start-up" was never this flag's: the naive
        // version — this one — buys 2,15e-04 -> 1,95e-04, and the 5,5x comes
        // from applying the correction in `set_rest()`, because
        // `sub_.prepare()` TAKES `q_n17_` AS AN ARGUMENT and a later correction
        // leaves the subsystem holding the old rest point.
        // => That needs a TWO-PASS `prepare()` — measuring needs `prepare`,
        // applying has to come before it — which is a cost decision about
        // `prepare()` and is still open.
        //
        // It is deleted rather than kept switched off, against the usual habit
        // here, because its whole purpose was to BOUND those two numbers and
        // they are now measured and written down: the instrument did its job.
        // Control on the removal: the product `.so` comes out md5-IDENTICAL.
    }

#ifdef NLSC_E4_REST_PROBE
    // The TABULATED rest point this stage received, and the residual it leaves
    // in the solver's KCL. A probe: it is not on the shipped path.
    double q_n17_probe()  const { return q_n17_; }
    double q_n19_probe()  const { return q_n19_abs_; }
    double ib0_probe()    const { return ib0_; }
    double kz_probe()     const { return g1o_.ib_gain(); }
    auto   residuo_reposo() { return sub_.residuo_en_reposo(q_n19_abs_, q_n17_); }
    auto   residuo_reposo_rb() { return sub_.residuo_en_reposo_rb(q_n19_abs_, q_n17_); }
    auto   residuo_reposo_producto() { return sub_.residuo_en_reposo_producto(q_n19_abs_, q_n17_); }
    SubQ2& sub_probe() { return sub_; }
    double g13_probe()    const { return sub_.g13_probe(); }
#endif

    void reset()
    {
        g1_.reset(); g1o_.reset(); gs_.reset(); g2_.reset();
        out_of_range_ = 0; u_min_ = 1e9; u_max_ = -1e9;
    }

    // Input: `n14`'s deviation. Output: `out`'s deviation (resting at 0).
    // `du` is the rail's contribution ALREADY FILTERED (tabulated path)
    // and `dvr` the rail's UNFILTERED deviation (subsystem path, which
    // filters it with the OPEN-base transfer). Both travel because they
    // belong to two different networks, not two spellings of one.
#ifdef NLSC_E4_IB_PROBE
    double ib_probe()  const { return sub_.ib(); }
    double probe_ieq() const { return sub_.ieq_probe(); }
#endif

// THE ORACLE'S `ib` — `NLSC_CASC_IB_ORACLE` (measurement rig).
//
// Measured: `n19`'s error is `Z(s;lvl)·delta_ib`, the displacement current
// `SubQ2` did not model (`bjt(..., false)`). What was needed before
// deciding on a fix was the BOUND at the OUTPUT, and this measures it: the
// stage is given ngspice's `ib` EXACTLY where it leaves the block.
//
// Substituted ONLY at the two sites where `ib` leaves `SubQ2`:
//   · `n19 = a0 + kz·ib`  (the term `Z` multiplies), and
//   · `close_ib`, which advances `Z`'s state with the solved current.
// `n17`'s solve stays the model's. On purpose: it is exactly the scope of
// the repair that WOULD BE MADE, so this bound prices that repair, not an
// ideal model.
//
// Behind a RUNTIME flag besides the macro: compiled but not injecting,
// the dump must come out bit-identical.
#ifndef NLSC_CASC_IB_ORACLE
#define NLSC_CASC_IB_ORACLE 0
#endif
#if NLSC_CASC_IB_ORACLE
    void inject_ib(double ib_absoluta_oraculo)
    {
        ib_inj_ = ib_absoluta_oraculo;
        use_ib_inj_ = true;
    }
    double ib_inj_ = 0.0;
    bool   use_ib_inj_ = false;
#endif
    // THE LOOP, IN ONE PLACE — with the DISPLACEMENT current.
    //
    // The level network imposes `n19 = a0 + kz·ib`, and `ib` is the
    // PHYSICAL current: static + displacement. `process_loop` solves the
    // static part, so the displacement term enters through `a0`.
    //
    // `close_ib` receives the SAME current as `n19`: otherwise `Z`'s
    // state and the line using it would speak of two different currents.
    double process_loop_(double a0, double kz, double& n19_abs)
    {
        // `sub_.ib()` is already the PHYSICAL current (static +
        // displacement) and the loop closes with it INSIDE the Newton, so
        // there is nothing to correct out here: `n19` and `Z`'s state speak
        // of the SAME current.
        const double w = sub_.process_loop(a0, kz, n19_abs) - q_n17_;
#if NLSC_E4_STARTUP_PROBE
        // DIAGNOSTIC ONLY: what does stage 4 inject at t=0?
        // With silence and every state at rest, BOTH of these must be zero.
        // Whichever is not is the step the high-passes then wash out.
        {
            // MEMBER, not `static`: a `static` with state is shared by every
            // instance of the plugin, and `make test` refuses it — correctly.
            const double dib = sub_.ib() - ib0_;
            if (nprb_ < 6 || nprb_ == 19200 || nprb_ == 57599)
                std::fprintf(stderr, "  [e4 %6ld] w=%+.6e  dib=%+.6e  a0=%+.6e  n19_abs-q=%+.6e\n",
                             nprb_, w, dib, a0, n19_abs - q_n19_abs_);
            ++nprb_;
        }
#endif
        g1o_.close_ib(sub_.ib() - ib0_);
        last_w_ = w;
        return w;
    }

    double process(double dev_n14, double* dev_n19 = nullptr, double du = 0.0,
                   double dvr = 0.0)
    {
        // THE WAVESHAPER'S INPUT, and the model's ceiling lives here.
        //
        // The TABULATED path uses `g1_`, carrying Q2's base as a fitted
        // LINEAR impedance (`rpi`, `beta`, `Cb`). Measured against the
        // oracle's `n19` with the REAL signal, that model tops out at
        // −63,4 dB: under excitation the base is not an impedance, because
        // `rpi` depends on the current.
        //
        // The SUBSYSTEM path needs none of it: the base current is ALREADY
        // computed per sample, so the network goes OPEN-base and the current
        // enters as a source — EXACT superposition, not one fitted
        // parameter. Measured: −80,1 dB with the current delayed one sample,
        // against −87,1 solving the loop. => +16,7 dB.
        //
        // SUPERSEDED LATER — THE FIGURE WAS CORRECT BUT PARTIAL.
        // This used to say: "one sample of delay closes an algebraic loop, and that
        // is not done without checking its gain: measured, 0,0009 => cannot
        // oscillate". That gain holds AT `lvl = 0`, and `|Z(s;lvl)|` is 24x
        // larger at mid-travel while the signal halves => the delay's error
        // against the signal multiplies ~50x. It did not oscillate, no: it
        // ate 9 dB of null on the 9/9RI, which SHIPS this path.
        // => The loop is no longer delayed — it is CLOSED (`process_loop`,
        // and the `make lazo-ib` gate). The delay survives as the CONTROL,
        // under `NLSC_E4_CLOSED_LOOP=0`.
        // THE CLOSED-LOOP PATH. Kept apart with its own `return` on
        // purpose: the other three paths stay BYTE FOR BYTE as they were,
        // which is what allows comparing them.
        //
        // `n19` and `n17` solve TOGETHER. The level network gives a line,
        //     n19 = a0 + kz·ib      with a0 = q_n19 + [H_sen·n14 + H_rail·vr +
        //                                              Z's state] − kz·ib0
        // and Q2's Newton takes it as the second equation. `close_ib`
        // advances `Z`'s state ONCE, with the current already solved.
        if (use_sub_ && kBaseAbierta && kClosedLoop) {
            const double kz = g1o_.ib_gain();
            const double a0 = q_n19_abs_ + g1o_.parte_sin_ib(dev_n14, dvr)
                            - kz * ib0_;
            double n19_abs = 0.0;
            double w_loop;
#if NLSC_CASC_IB_ORACLE
            if (use_ib_inj_) {
                // With EXACT `ib` the loop is no loop: `n19` is direct,
                // and `SubQ2` only has to solve `n17` at that voltage.
                n19_abs = a0 + kz * ib_inj_;
                w_loop  = sub_.process(n19_abs) - q_n17_;
                g1o_.close_ib(ib_inj_ - ib0_);
            } else {
                w_loop = process_loop_(a0, kz, n19_abs);
            }
#else
            w_loop = process_loop_(a0, kz, n19_abs);
#endif
            // STRUCTURAL INVARIANT, in volts: how much this sample
            // violates the algebraic relation `n19 = a0 + kz·ib` the network
            // imposes. It is what the one-sample delay broke, and measuring
            // it is ORTHOGONAL to the null — no ngspice, no reference.
            loop_resid_ = (a0 + kz * sub_.ib()) - n19_abs;
            ib_prev_ = sub_.ib();
            const double u_loop = (n19_abs - q_n19_abs_) + off_in_;
            if (u_loop < u_min_) u_min_ = u_loop;
            if (u_loop > u_max_) u_max_ = u_loop;
            if (dev_n19) *dev_n19 = u_loop;
            w_ = w_loop;
            (void)du;
            // NOT `gs_`: the subsystem already solves `n17` with its own
            // network. See the `NLSC_E4_GS_IN_SUB` macro above.
            return g2_.process(kGsEnSub ? gs_.process(w_loop) : w_loop);
        }
        double u;
        if (use_sub_ && kBaseAbierta) {
            u = g1o_.process(dev_n14, dvr, ib_prev_ - ib0_) + off_in_;
        } else if (use_sub_) {
            // CONTROL (not product): the subsystem with the OLD level
            // (base as a fitted linear impedance). It prices the change over
            // the knob's WHOLE travel, where one point proved not enough.
            u = g1_.process(dev_n14) + du + off_in_;
        } else {
            u = g1_.process(dev_n14) + du + off_in_;
        }
        if (u < u_min_) u_min_ = u;
        if (u > u_max_) u_max_ = u;
        if (dev_n19) *dev_n19 = u;

        // THE WAVESHAPER GUARD. Extrapolating a polynomial does not
        // degrade, it BLOWS UP: +41 dB measured in
        // `WAVESHAPER_ETAPAS_1_4.md` §4. Outside the fitted travel the curve
        // continues along the endpoint's TANGENT, which is bounded, and it
        // is COUNTED — an uncounted guard is a guard nobody knows fired.
        double w;
        // WHICH MODEL STAGE 4 USES — CHOSEN PER VARIANT, and the split
        // comes from MEASURING the two things that matter:
        //
        //   variant  | cascade null, table      | subsystem  | casc/engine ratio
        //   ---------|--------------------------|------------|-----------------
        //   808      |        −64,5 dB          |  −65,0 dB  | 0,678 -> 0,907
        //   9/9RI    |     −27,7 dB          |  −53,5 dB  | 0,678 -> 0,907
        //
        // => For the 808 the subsystem buys 0,5 dB and costs +34 % of
        // clock: not paid. For the 9/9RI it buys 25,8 dB, and there it is.
        // The branch is ONE per sample and perfectly predicted; folding
        // this into one path behind a pointer would be the per-sample
        // dispatch failure.
        if (use_sub_) {
            // The subsystem works in ABSOLUTE terms and returns `n17`;
            // here it becomes a deviation against TODAY's rest, which is
            // what `gs_`/`g2_` expect. And NO `off_out_` correction: that
            // exists to move between the FIT's frame and today's, and the
            // subsystem has no fit frame.
            w = sub_.process(q_n19_abs_ + u - off_in_) - q_n17_;
            // The SAME invariant for the delayed branch, hence
            // comparable: there `n19` was computed with the PREVIOUS
            // sample's `ib`, so the violation is exactly
            // `kz·(ib_now − ib_before)`.
            loop_resid_ = g1o_.ib_gain() * (sub_.ib() - ib_prev_);
            ib_prev_ = sub_.ib();
            w_ = w;
            return g2_.process(kGsEnSub ? gs_.process(w) : w);
        }
        if (u < umin_)      { ++out_of_range_; w = tangent(umin_, u); }
        else if (u > umax_) { ++out_of_range_; w = tangent(umax_, u); }
        else                          { w = poly(u); }
        w_ = w;

        return g2_.process(gs_.process(w - off_out_ * rest::kOffOutput));
    }

    // TODAY's rest against the one Q2's curve was fitted at (drive 1,0).
    // The gain pot moves `n19` by 1,38 mV and `n17` by 1,37 mV between the
    // knob's ends; the waveshaper must evaluate in ITS frame and its output
    // return to today's. See `nls_reposo_drive.h`. At zero = as before.
    // `kFit_*` BELONGS TO THE VARIANT. It is the rest Q2's curve was
    // fitted at, and between the 808 and the 9/9RI it separates by 998 mV.
    // With the 808's on the variant, the waveshaper's input arrives a volt
    // displaced, the guard fires on 62 % of samples and the null stops at
    // −11,9 dB — exactly what was measured on forgetting it.
    void set_rest(double q_n19, double q_n17)
    {
        off_in_  = q_n19 - fit_n19_;
        off_out_ = q_n17 - fit_n17_;
        q_n19_abs_ = q_n19;
        q_n17_     = q_n17;
#if defined(NLSC_E4_Q_N17_TRIM)
        // DIAGNOSTIC: shifts the TABULATED rest of `n17` by a
        // fixed amount. `w = sub_.process_loop(...) - q_n17_` must be zero at
        // rest and measures +35 uV at t=0, settling at +405 uV. If the start-up
        // bump minimises at some trim, the offset IS this mismatch.
        q_n17_ += (NLSC_E4_Q_N17_TRIM);
#endif
    }

    // The netlist values the VARIANT changes, for the subsystem.
    void set_resistencias(double r15, double r14) { r15_ = r15; r14_ = r14; }

    // PROBE: the last sample's absolute `n17`. `w_` is stored by
    // `process()`.
    //
    // THE FRAME DEPENDS ON THE PATH. This used to always return
    // `fit_n17_ + w_`, valid only on the TABULATED path, where `w_` is the
    // waveshaper's output in its FIT frame. On the SUBSYSTEM's,
    // `w_ = n17 − q_n17_`: a deviation against TODAY's rest.
    // The two rests coincide at drive 1,0 — where the curve was fitted
    // and where ALL the early measurements sat, so the error hid. Away from
    // there the probe lied by millivolts, silently.
    double n17_abs() const { return (use_sub_ ? q_n17_ : fit_n17_) + w_; }

    double ws_umin() const { return umin_; }
    double ws_umax() const { return umax_; }

    // Today's rest steady state — see `ParamFilter::preset_dc`.
    void startup()
    {
        // `ParamFilter` needs no `preset_dc` here: its zero state IS the
        // steady state for zero input, which is what this call asked for.
        g1_.reset(); g1o_.reset(); ib_prev_ = ib0_;
        g2_.preset_dc(gs_.preset_dc(poly(off_in_) - off_out_ * rest::kOffOutput));
    }

    // PROBE (not product): evaluates the waveshaper at a GIVEN `n19`
    // instead of the one the pot network computes. It is the UPPER BOUND of
    // fixing everything BEFORE Q2's waveshaper — pot, base load and rail
    // included — and therefore what says whether paying for it is worth it.
    // The `g1_` network keeps running with its usual input so its STATE
    // does not drift: a probe that changes the filter's state stops
    // measuring the same system from the second sample on.
    // CLOSED — IT MEASURED A RETIRED ENGINE, WITH NO ERROR RAISED.
    //
    // This body is a HAND COPY of `process()` from when stage 4 was
    // `g1_` + `poly()` + `gs_` + `g2_`. Since then `process()` gained Q2's
    // SUBSYSTEM (`sub_`), the OPEN BASE (`g1o_`) and the CLOSED `ib` loop —
    // the SHIPPED path, which this body never mentions. So an `n19`-oracle
    // run would have yielded a perfectly credible upper bound OF THE
    // RETIRED ENGINE.
    //
    // Not deleted: closed out loud, like `NLSC_CASC_RAIL_COMPOSED`. And
    // not "fixed" by copying the new body again, which is what created the
    // problem: when the `n19` cut is needed, it is done by INJECTION inside
    // the one `process()`, as the `n3`/`n6`/`n4`/`n7`/`n14` ones are built.
    //
    // The hard part, and why it is not done in passing: on the shipped
    // path `n19` and `n17` solve TOGETHER by Newton over the line
    // `n19 = a0 + kz·ib`, so "injecting n19" is passing that line with ZERO
    // slope (`a0 = oracle n19`, `kz = 0`) and letting the solver do
    // the rest. Written here so the next reader need not re-derive it.
    // THE BODY IS DELETED, not left dead: an obsolete body nobody can run
    // is still a body somebody COPIES. The abort remains and, above, the
    // recipe for doing it right.
    [[noreturn]] double process_con_n19(double, double, double* = nullptr)
    {
        std::fprintf(stderr, "Stage4::process_con_n19 is CLOSED: its body was a "
                             "copy of the RETIRED engine (no Q2 subsystem, no "
                             "closed `ib` loop). Aborting rather than measuring "
                             "SOMETHING ELSE. See the comment above.\n");
        std::exit(2);
    }

    // The guard's statistics zero AFTER the pre-run: otherwise they count
    // the startup transient, which is not what is being judged.
    void reset_estadisticas() { out_of_range_ = 0; u_min_ = 1e9; u_max_ = -1e9; }

    long out_of_range() const { return out_of_range_; }
    double u_min() const { return u_min_; }
    double u_max() const { return u_max_; }
#if NLSC_E4_CACHE
    // The cache BOX's counter, published outward: an edge that clips and
    // does not say so is exactly what turns a table into a trap.
    long out_of_box() const { return sub_.out_of_box(); }
    long outside_p()   const { return sub_.outside_p(); }
    long outside_ieq() const { return sub_.outside_ieq(); }
    double exceso_p()   const { return sub_.exceso_p(); }
    double exceso_ieq() const { return sub_.exceso_ieq(); }
    double w_min() const { return sub_.w_min(); }
    double w_max() const { return sub_.w_max(); }
#ifdef NLSC_E4_PLANOS_PROBE
    void dump_planes(std::FILE* f) const { sub_.dump_planes(f); }
    bool cache_built() const { return sub_.built(); }
#endif
#endif

#ifdef NLSC_E4_LOOPGAIN_PROBE
    // Next to the Newton observability and NOT with the cache accessors: those
    // live inside `#if NLSC_E4_CACHE`, and this probe is compiled with the
    // cache OFF.
    double sub_lg_max() const  { return sub_.lg_max_; }
    double sub_lg_mean() const { return sub_.lg_n_ ? sub_.lg_sum_ / double(sub_.lg_n_) : 0.0; }
    long   sub_lg_n() const    { return sub_.lg_n_; }
#endif

    // Subsystem Newton observability (see `SubQ2::process`).
    long newton_escapes() const { return sub_.unconverged_; }
    long newton_samples() const { return sub_.samples_; }
    // The cache's slice filling lives in the subsystem; this only forwards.
#if NLSC_E4_CACHE
    void defer_fill(bool v)        { sub_.defer_fill(v); }
    bool advance_cache(int nodes)        { return sub_.advance_cache(nodes); }
    bool cache_pending() const        { return sub_.cache_pending(); }
    long cache_nodes_left() const  { return sub_.cache_nodes_left(); }
#endif
    // PROBE: the `ib` loop's violation on the last sample, in volts.
    // With the loop CLOSED it must sit at the Newton's floor; with `ib`
    // delayed it equals `kz·Δib`, which is what got fixed.
    double loop_resid() const { return loop_resid_; }

private:
    double poly(double u) const
    {
        double w = 0.0;
        for (int i = 0; i < ws2_n_; ++i) w = w * u + ws2_[i];
        return w;
    }
    double tangent(double u0, double u) const
    {
        double d = 0.0;                     // derivative by Horner
        for (int i = 0; i < ws2_n_ - 1; ++i)
            d = d * u0 + ws2_[i] * (ws2_n_ - 1 - i);
        return poly(u0) + d * (u - u0);
    }

    VariableLevel     g1_;
    OpenBaseLevel  g1o_;   // the OPEN-base one, for the subsystem
    FixedFilter<fijos::kE4GS_NB, fijos::kE4GS_NA> gs_;
    FixedFilter<fijos::kE4G2_NB, fijos::kE4G2_NA> g2_;
    long out_of_range_ = 0;

    // COPY of the active variant's waveshaper: read PER SAMPLE. The
    // constructor leaves the 808's, the shipped one.
    double last_w_ = 0.0;   // the loop's own `w`, read back by `prepare()`
#if NLSC_E4_STARTUP_PROBE
    long   nprb_ = 0;       // diagnostic sample counter — never a `static`
#endif
    double ws2_[kWSMax] = {0.0};
    int    ws2_n_ = e234::kWS2_N;
    double umin_  = e234::kWS2_UMIN;
    double umax_  = e234::kWS2_UMAX;
    double w_ = 0.0;          // last waveshaper output, for the probe
    double loop_resid_ = 0.0; // violation of `n19 = a0 + kz·ib`, in volts
    double ib_prev_ = 0.0, ib0_ = 0.0;   // base current and its rest
    double q_n19_abs_ = 0.0, q_n17_ = 0.0;
    double r15_ = 10.0e3, r14_ = 100.0;   // 808 by default: R15 10k, R14 100
    bool   use_sub_ = false;              // set by `set_bank`
    SubQ2  sub_{};
    double fit_n19_ = rest::kFit_n19;
    double fit_n17_ = rest::kFit_n17;
    double u_min_ = 1e9, u_max_ = -1e9;
    double off_in_ = 0.0, off_out_ = 0.0;
};

// The four chained stages.
struct Outputs { double n4, n7, n14, out; };

// The `n3`/`n6` probe (see the member of the same name): without the
// flag it is `nullptr` and stage 1 writes nothing. The product compiles
// that way.
#ifdef NLSC_CASC_PROBE_N36
#  define kProbeN6 (&n6_abs_probe_)
#else
#  define kProbeN6 nullptr
#endif

// GUARD: `NLSC_CASC_CORR` CANNOT BE TURNED ON WHILE E2C IS THE IDENTITY.
//
// The opamp correction is MEASURED AND OFF (it wins 24,8 dB in true small signal
// and loses 9,2 dB on real DI), so `gen_coef_e234.py` deliberately takes its OFF
// branch and emits `E2C` as the identity — `kE2C_B` is bit-for-bit `kE2C_A` in
// BOTH variant sets. Turning the switch on in that state applies `H(z) = 1` and
// reports «no change», which reads as a REFUTATION of the post-filter when it
// only means the post-filter is not there.
//
// Costs nothing in the product: the switch is undefined, so this block does
// not exist in the shipped translation unit and the `.so` is md5-identical.
// Its negative arm is free and is the point: building with -DNLSC_CASC_CORR
// MUST fail today, with this message.
#ifdef NLSC_CASC_CORR
constexpr bool nlsc_coef_pair_is_identity(const double (&b)[3][1],
                                          const double (&a)[3][1])
{
    for (int i = 0; i < 3; ++i)
        if (b[i][0] != a[i][0]) return false;
    return true;
}
static_assert(!nlsc_coef_pair_is_identity(fijos::kE2C_B, fijos::kE2C_A),
              "NLSC_CASC_CORR is ON but E2C is the IDENTITY filter for the 808 set: "
              "the coefficients come from the generator's OFF branch, so this would "
              "measure nothing. The route back exists: `make resp-e2` writes the "
              "candidate, and copying it to harness/spice/resp_e2.dat then re-running "
              "`make coeficientes` emits a real E2C -- deliberately, because that "
              "changes VERSIONED headers.");
static_assert(!nlsc_coef_pair_is_identity(fijos_v9ri::kE2C_B, fijos_v9ri::kE2C_A),
              "NLSC_CASC_CORR is ON but E2C is the IDENTITY filter for the 9/9RI set: "
              "same cause and same fix as the 808 assertion above.");
#endif

class Cascade4 {
public:
    // The hoisted `lp_rail` coefficients start CONSISTENT with the default
    // `fs_`, so the filter is never run with zeroes if something processes
    // before the first `prepare_bank`. Seeded by the SAME method that keeps
    // them in step, so there is no second copy of the formula to diverge.
    Cascade4() { lp_set_rate(); }

    // `tone` carries NO default value, on purpose. A silent default is
    // exactly the hole that bit twice in one day: a parameter the harness
    // does not feed takes its default and the number looks good. A caller
    // that does not know which tone it wants must decide and write it.
    // `reiniciar` — RE-TUNING IS NOT RESTARTING. Moving a pot forces the
    // coefficients and matrices to be rebuilt, but NOT the state to be
    // dropped: with `false` the circuit keeps the charge it had, like the
    // real pedal. With `true` (startup or a RATE change) everything resets
    // and the rest point is solved. The DK engine had this case fixed and
    // gated in `make test`; the cascade did not, and without it every knob
    // move would CLICK.
    // `variante` — the parameter that was missing, and why the `variant`
    // port came out MUTE with the cascade: the manifest advertised two
    // circuits and the engine delivered one.
    // Everything it decides resolves HERE; the process loop never looks.
    // THE CASCADE'S ONLY PER-VARIANT DISPATCH.
    //
    // There used to be EIGHT `if (variante == 1)` across three files, and
    // its `else`s fell through to the 808. Now the variant resolves ONCE,
    // here, and everything below is a template over the bank
    // (`nls_juegos.h`).
    // => Adding a variant is a `struct` there and a `case` here — and missing
    // either one, IT DOES NOT COMPILE.
    // UPPER BOUND OF FIXING STAGE 2. Like the existing `n19`-oracle
    // mode: the whole chain runs but stage 3 receives the ORACLE's `n7`.
    // What remains is what touching stage 2 does NOT fix — i.e. the maximum
    // that improving the opamp linearisation can buy.
    // Not a model: oracle injection, which is why it decides whether
    // paying is worth it BEFORE writing it (same discipline as B6's
    // bound).
// BEHIND A MACRO, off by default: the injection puts a BRANCH in the hot
// path and the product .so stopped coming out bit-identical. A measurement
// variant does not get paid for in the shipped binary.
#ifndef NLSC_CASC_N7_ORACLE
#define NLSC_CASC_N7_ORACLE 0
#endif
#if NLSC_CASC_N7_ORACLE
    double n7_inj_ = 0.0;
    bool   use_n7_inj_ = false;
#endif

// UPPER BOUND OF FIXING **STAGE 1**.
//
// WHY IT IS NEEDED, AND IT IS A CORRECTION: the `n7`-oracle bound was
// read as "what fixing stage 2 wins", and it is NOT — injecting the
// oracle's `n7` replaces EVERYTHING upstream, i.e. stages 1 AND 2. While
// stage 2 dominated it made no difference; since stage 3's load entered it
// contributes ~0,1 dB, and that bound then mostly measures STAGE 1 without
// saying so. With both injections the split reads by difference:
//     n4-oracle  -> stages 2,3,4 remain
//     n7-oracle  -> stages   3,4 remain
// Same treatment as its sibling: behind a macro and off by default, so
// the product `.so` keeps coming out bit-identical.
#ifndef NLSC_CASC_N4_ORACLE
#define NLSC_CASC_N4_ORACLE 0
#endif
#if NLSC_CASC_N4_ORACLE
    double n4_inj_ = 0.0;
    bool   use_n4_inj_ = false;
#endif

// AND STAGE 1's BISECTION: the ORACLE's `n3`.
//
// `n4`-oracle says stage 1 is worth +5,6 dB, but not WHAT that figure is
// made of. Stage 1 is four things in series: the input network `H1`, the
// RAIL's coupling into `n3` via `R2`, Q1's WAVESHAPER, and the output
// filters `Hs`/`H2`. Injecting `n3` cuts that list in half:
//     n3-oracle -> waveshaper + Hs + H2 remain   (the stage's LOWER half)
//     n4-oracle -> nothing of stage 1 remains
// => The difference between the two is what `H1` + rail coupling are worth.
//
// Unlike the other two, this one CANNOT let the stage run and overwrite
// the result: `n3` is an INTERNAL node, so everything below must run
// ON the injected value. `process_con_n3()` is called INSTEAD of
// `process()` — calling both would advance `Hs`'s and `H2`'s state twice, a
// silent error exactly the size of a result.
#ifndef NLSC_CASC_N3_ORACLE
#define NLSC_CASC_N3_ORACLE 0
#endif
#if NLSC_CASC_N3_ORACLE
    double n3_inj_ = 0.0;
    bool   use_n3_inj_ = false;
#endif

// STAGE 1's THIRD CUT: the ORACLE's `n6`.
//
// `n3` and `n4` left the split half-done: of the stage's +4,50 dB, 0,96
// are born upstream of `n3` and 3,54 between `n3` and `n4` — and TWO
// different things live there, Q1's waveshaper (with `Hs`) and the output
// coupling `H2`. Without this cut, attributing those 3,54 dB would be
// striking out two candidates and keeping one — exactly what one retracted
// attribution already cost.
//     n3-oracle -> waveshaper + Hs + H2 remain
//     n6-oracle -> ONLY H2 remains
//     n4-oracle -> nothing of stage 1 remains
// Same treatment as its siblings: behind a macro and off by default, so
// the product `.so` keeps coming out BIT-IDENTICAL.
#ifndef NLSC_CASC_N6_ORACLE
#define NLSC_CASC_N6_ORACLE 0
#endif
#if NLSC_CASC_N6_ORACLE
    double n6_inj_ = 0.0;
    bool   use_n6_inj_ = false;
#endif

// THE CUT BETWEEN STAGES 3 AND 4: the ORACLE's `n14`.
//
// Measured with `n7`, in the deciding cell 67,9 % of the error's energy
// sits behind `n7` — i.e. in stage 3 (tone) and stage 4 (level/output)
// together. `n14` is the exact border and splits that 67,9 %:
//     n7-oracle  -> stages 3 and 4 remain
//     n14-oracle -> ONLY stage 4 remains
// And it is the CHEAP cut of the two that were missing: `dev_n14` is a
// scalar entering `e4_.process()`, so injecting it does not touch the
// Newton. The `n19` one lives INSIDE the loop solving `n19` and `n17`
// together — another story.
// Off by default: the product `.so` keeps coming out bit-identical.
#ifndef NLSC_CASC_N14_ORACLE
#define NLSC_CASC_N14_ORACLE 0
#endif
#if NLSC_CASC_N14_ORACLE
    double n14_inj_ = 0.0;
    bool   use_n14_inj_ = false;
#endif

// THE ORACLE's RAIL: `NLSC_CASC_VR_ORACLE`.
//
// The 808/9RI gap is born on `n19`'s path, and that path is a
// SUPERPOSITION of three knob-parametrised transfers
//     n19 = H_sen(s;lvl)·n14 + H_rail(s;lvl)·vr + Z(s;lvl)·ib
// The rail term is the suspect because `R12` enters AFTER the pot, so its
// error does NOT attenuate with the knob — exactly the signature (worse at
// high `lvl`, where the signal drops 4,2x and the rail does not).
//
// But "the rail term" is TWO different things and the `NLSC_NO_RN19` arm
// does not separate them: (a) the `vr` VALUE the rail loop produces and (b)
// the `H_rail(s;lvl)` FIT. Injecting the oracle's `vr` leaves (b) alone:
//     vr-oracle, gap remains  -> it is the parametric FIT
//     vr-oracle, gap closes   -> it is the rail's VALUE, upstream
//
// It sits behind an EXECUTION flag besides the macro, like
// `NLSC_E2_N9_ORACLE`: compiled and uninjected, the dump must come out
// IDENTICAL byte for byte — the instrumented binary is its own control.
#ifndef NLSC_CASC_VR_ORACLE
#define NLSC_CASC_VR_ORACLE 0
#endif
#if NLSC_CASC_VR_ORACLE
    double vr_inj_ = 0.0;
    bool   use_vr_inj_ = false;
#endif

    // Belt state (see `process`). `fs_`…`circuit_` are the anchor.
    double gain_ = 1.0, tone_ = 0.5, lvl_ = 0.0;   // `fs_` already exists below
    int    circuit_ = 0;        // CIRCUIT, not the selector row
    int    belt_streak_ = 0;
public:
    long   belt_broken_ = 0;   // samples with the output broken
    // Stage 4 Newton observability, so "the cap does not bite" is
    // CHECKABLE and not an assumption.
#ifdef NLSC_E4_REST_PROBE
    // PROBE access to stage 4, for the rest residual. Not product.
    Stage4& etapa4_probe() { return e4_; }
#endif
    long newton_escapes() const { return e4_.newton_escapes(); }
    // Stage 2's probes, forwarded outwards. The counters existed with no way
    // to read them from a harness, and a counter nobody can read observes
    // nothing.
    long   e2_no_conv()   const { return e2_.no_conv(); }
    double e2_v5() const { return e2_.probe_v5(); }
    double e2_v7() const { return e2_.probe_v7(); }
    double e2_v2() const { return e2_.probe_v2(); }
    double e2_w_extremo() const { return e2_.w_extremo(); }
    long newton_samples() const { return e4_.newton_samples(); }
    double loop_resid() const { return e4_.loop_resid(); }
    // THE CACHE FILL, ONCE PER BLOCK — not per sample.
    // `Plugin::process` calls it at the block boundary, the LOWEST
    // frequency that still contains the work: per sample would be 192.000
    // calls per second to move a cursor.
    // ONLY THE PLUGIN CORE TURNS IT ON — the only one that knows
    // `prepare()` reaches it from the audio thread. A harness does NOT, and
    // therefore keeps measuring the FULL table as always.
    void defer_fill(bool v)
    {
#if NLSC_E4_CACHE
        e4_.defer_fill(v);
#endif
        // Stage 2's table is deferred by the SAME switch, on purpose: both are
        // caches that `prepare()` builds from the audio thread, and the one who
        // knows that path is reached is the plugin core, not a harness. Two
        // switches for one decision are two places to forget one of them.
#if NLSC_E2_TABLA
        e2_.tabla_diferida(v);
#endif
#if !NLSC_E4_CACHE && !NLSC_E2_TABLA
        (void)v;
#endif
    }
    // "The USER asked for this re-tune" — said by `apply_knobs()`, which is the
    // only caller that tells a gesture from a set-up. It exists with the table
    // switched off too, so the core does not have to know whether it is
    // compiled in.
    void tabla_gesto(bool v)
    {
#if NLSC_E2_TABLA
        e2_.tabla_gesto(v);
#else
        (void)v;
#endif
    }
    // Stage 2 table observability, forwarded so that "it never fires" is
    // CHECKABLE from outside rather than assumed. Without this forwarding the
    // accessors were dead code and the "0 of 192.000" figure could not be
    // reproduced from the tree.
    long tabla_fuera_caja() const
    {
#if NLSC_E2_TABLA
        return e2_.tabla_fuera_caja();
#else
        return 0;
#endif
    }
    long tabla_relleno_no_conv() const
    {
#if NLSC_E2_TABLA
        return e2_.tabla_relleno_no_conv();
#else
        return 0;
#endif
    }
    bool advance_cache(int nodes = NLSC_E4_CACHE_NODES_PER_BLOCK)
    {
#if NLSC_E4_CACHE
        return e4_.advance_cache(nodes);
#else
        (void)nodes; return false;
#endif
    }
    bool cache_pending() const
    {
#if NLSC_E4_CACHE
        return e4_.cache_pending();
#else
        return false;
#endif
    }
    long cache_nodes_left() const
    {
#if NLSC_E4_CACHE
        return e4_.cache_nodes_left();
#else
        return 0;
#endif
    }
    long   rescues_ = 0;         // times it returned to rest

    // THE PARAMETER IS THE **CIRCUIT**, NOT THE SELECTOR ROW — and the
    // old name (`variant`) cost a defect the moment the two stopped being the
    // same thing. Since a row is (circuit × knob law), passing the ROW index
    // sends any new row to the `default`, i.e. the 808's bank: the 9/9RI row
    // with taper ran the WRONG CIRCUIT, and the DK did not, because it
    // translated. Two engines, two circuits, no error. Caught by the control
    // in `reparto_tono.py --control`.
    // => The caller translates with `kVariants[row].circuit`; here it arrives
    // translated, and the name says so.
    void prepare(double fs, double gain, double tone, double lvl,
                 int circuit = 0, bool reinit = true)
    {
        // Stored so the belt below can RE-ANCHOR. Five scalars: the
        // price of an ABSOLUTE anchor to return to.
        gain_ = gain; tone_ = tone; lvl_ = lvl; circuit_ = circuit;   // `fs_` set by `prepare_bank`
        belt_streak_ = 0;
        // The `default` is the SAFETY NET for a corrupt index, not the
        // general case: adding a circuit does not compile until its `case` is
        // added, and `nls_juegos.h`'s `static_assert` sees to that.
        switch (circuit) {
        case 1:  prepare_bank<Bank9ri>(fs, gain, tone, lvl, reinit); break;
        default: prepare_bank<Bank808>(fs, gain, tone, lvl, reinit); break;
        }
    }

    template <class J>
    void prepare_bank(double fs, double gain, double tone, double lvl, bool reinit)
    {
        // FIRST the bank, because the `set_rest()` calls below subtract
        // constants that depend on it.
        e1_.set_bank<J>();
        e4_.set_bank<J>();
        // The two resistors that ARE the variant, from the netlist. One
        // source: `nls_variantes.h`, the same table the GUI and `.ttl` read.
        e4_.set_resistencias(kCircuits[J::idx].rout_shunt, kCircuits[J::idx].rout_ser);

        // THE REST FOLLOWS THE KNOB. See `nls_reposo_drive.h`:
        // the gain pot moves `n7` by 6,09 mV and `VR` by 1,90 mV between the
        // ends, and pinning them to drive 1,0's cost 12 dB in the
        // closed-drive weak-signal corner. Evaluated ONCE, here.
        // AND THEY ARE THE VARIANT'S: between the 808 and the 9/9RI `n17`
        // moves +998 mV and `n19` +947 mV. Crossing them injects a volt of
        // DC that the rail loop integrates.
        q_vr_  = rest::ev(J::Rvr,  gain);
        q_n3_  = rest::ev(J::Rn3,  gain);
        q_n4_  = rest::ev(J::Rn4,  gain);
        q_n7_  = rest::ev(J::Rn7,  gain);
        q_n11_ = q_vr_;   // `n11` is opamp B's output, resting at VR
        q_n14_ = rest::ev(J::Rn14, gain);
        q_n19_ = rest::ev(J::Rn19, gain);
        const double q_n6  = rest::ev(J::Rn6,  gain);
        const double q_n17 = rest::ev(J::Rn17, gain);

        // The conductances each node injects into the rail with.
        g_n3_ = J::G_n3;  g_n4_ = J::G_n4;  g_n11_ = J::G_n11;  g_n19_ = J::G_n19;
        // The output's quiescent point is the VARIANT's, like every other q_*.
        // It was hard-wired to e234::kQ_out (the 808's) in three places; benign
        // today because both variants publish 0.0, silently wrong for any future
        // variant whose output does not rest at zero.
        q_out_ = J::Q_out;

        // PROBE (not product): UPPER BOUND of correcting STAGE 1's rest,
        // sibling of `NLSC_SHIFT_N19`. Measured motive: `n3`/`n4`'s slow
        // residual correlates with the ENVELOPE at -0,944 and -0,950 — the
        // high-level corner's signature — and this stage's rest evaluates
        // ONCE in `prepare()`, from the KNOB alone.
        // The two axes are SEPARATE on purpose: the residual means differ
        // not just in magnitude but in SIGN (`n3` -0,4896 mV, `n4` +0,1255),
        // so one common offset cannot correct both.
        // Undefined, the path is BIT-IDENTICAL to before.
#if defined(NLSC_SHIFT_E1_N3) || defined(NLSC_SHIFT_E1_N6)
#  ifndef NLSC_SHIFT_E1_N3
#    define NLSC_SHIFT_E1_N3 0.0
#  endif
#  ifndef NLSC_SHIFT_E1_N6
#    define NLSC_SHIFT_E1_N6 0.0
#  endif
        e1_.set_rest(q_n3_ + (NLSC_SHIFT_E1_N3), q_n6 + (NLSC_SHIFT_E1_N6), q_n4_);
#else
        e1_.set_rest(q_n3_, q_n6, q_n4_);
#endif
        // PROBE (not product): UPPER BOUND of correcting the BIAS
        // DISPLACEMENT. Under signal the real circuit shifts its working
        // point (+10,0 mV at `n19`, measured) and the cascade evaluates the
        // waveshaper centred on the STATIC rest.
#ifdef NLSC_SHIFT_N19
        e4_.set_rest(q_n19_ + (NLSC_SHIFT_N19), q_n17 + (NLSC_SHIFT_N19));
#else
        e4_.set_rest(q_n19_, q_n17);
#endif

        e2_.prepare(fs, gain, q_vr_, q_n4_, 1e-3, reinit);
        e3_.prepare(fs, tone, reinit);
        e4_.prepare_bank<J>(fs, lvl, reinit);
        // The rail's injection into each stage depends on the SAME knob as
        // its signal network, for the same reason: same network, other
        // source.
        rn14_.prepare(fs, tone, reinit);
        rn19_.prepare_bank<J>(fs, lvl, reinit);
        // The FIXED filters also discretise here, at the real rate: that
        // is what frees the cascade from the three tabulated rates.
        e1_.prepare_bank<J>(fs, reinit);
        rn3_.prepare(fs, J::N3_B,  J::N3_A,  J::ScaleF, reinit);
#if NLSC_VRA_PARAM
        vra_.prepare(fs, tone, reinit);
#else
        vra_.prepare(fs, J::VRA_B, J::VRA_A, J::ScaleF, reinit);
#endif
        vrb_.prepare(fs, J::VRB_B, J::VRB_A, J::ScaleF, reinit);
        vrc_.prepare(fs, J::VRC_B, J::VRC_A, J::ScaleF, reinit);
        vrp_.prepare(fs, J::VRP_B, J::VRP_A, J::ScaleF, reinit);
        c2_.prepare(fs,  J::E2C_B, J::E2C_A, J::ScaleF, reinit);
        fs_ = fs;
        lp_set_rate();   // the ONLY writer of `fs_`: keep the hoist in step
        if (reinit) {
            e1_.reset(); c2_.reset(); e3_.reset(); e4_.reset();
            lp_x_ = lp_y_ = 0.0;
            dvr_ = dev_n3_ = dev_n19_ = 0.0;
            // NEW STATE ENTERS THE RESET, and forgetting it shows on no
            // fidelity yardstick: the HOST CONTRACT catches it. Without
            // this line `make test` gave FIVE failures — bit-identity in
            // blocks, in-place, out-of-range ports, NaN — because `dvr_ant_`
            // survived `activate()` and the second run started from another
            // state. A perfectly nulling `.so` can still break the contract.
            dvr_ant_ = 0.0;
#if NLSC_CASC_SEGMENTED
            seg_n4_ = seg_n7_ = seg_n14_ = 0.0;
#endif
            rn3_.reset(); rn14_.reset(); rn19_.reset();
            vra_.reset(); vrb_.reset(); vrp_.reset(); vrc_.reset();
        }
        // AND AT THE END, the steady state today's rest deserves: the
        // `reset()`s above leave everything at zero, which is only rest if
        // the rest input is zero — and since the rest follows the knob it no
        // longer is.
        if (reinit) { e1_.startup(); e4_.startup(); }
    }

    // STAGE 2's LINEAR CORRECTION — the idea, and why it does NOT hold.
    //
    // The idea: the term the cascade discards is the opamp's finite gain,
    // which in small signal is a LINEAR error, so a filter corrects it:
    //
    //     Hcorr(s) = (real circuit's n7/n4) / (ideal model's n7/n4)
    //
    // with `H_ideal` MEASURED on this same `Stage2` (`harness/resp_e2.cpp`),
    // not hand-derived.
    //
    // MEASURED, AND OFF BY DEFAULT: IT DOES NOT WORK ON REAL MATERIAL.
    //
    // | | no correction | with correction |
    // |---|---:|---:|
    // | true small signal (0,2 mV, diodes off) | −32,7 dB | **−57,5 dB** |
    // | real DI (the diodes clip)              | **−38,9 dB** | −29,7 dB |
    //
    // It wins 24,8 dB where it was derived and loses 9,2 dB where it
    // matters. The reason is physics in one sentence: the ideal-opamp error
    // depends on LEVEL. Closed-loop gain with the diodes off is ~97; when
    // they clip it collapses, so the LOOP gain rises and the ideal-opamp
    // hypothesis gets BETTER. A correction sized with the diodes off
    // over-corrects exactly when they conduct.
    //
    // => A FIXED post-filter cannot fix an error that moves with level. The
    // correct fix puts the finite gain INSIDE stage 2's loop (and with it
    // stage 3's linear load on `n7`) — still split-compatible, but another
    // model, not a filter.
    //
    // Kept compilable (`-DNLSC_CASC_CORR`) because its measurement is the
    // argument.
    //
    // AND THE SWITCH CANNOT SILENTLY DO NOTHING. Because the
    // correction is measured and OFF, `gen_coef_e234.py` emits `E2C` as the
    // IDENTITY — `kE2C_B` is bit-for-bit `kE2C_A` in BOTH coefficient sets. So
    // turning this on today applies `H(z) = 1` and returns «0 dB of change»,
    // which reads as «the post-filter does not help» when it actually means
    // "there is no post-filter" -- and a yardstick that is never run is not a
    // yardstick.
    // The guard (just above `class Cascade4`) makes the compiler say so
    // instead. And re-running the measurement is not one step: the generator's
    // real branch needs `spice/resp_e2.dat`. **The route back exists** —
    // `make resp-e2` — and it deliberately writes a `.CANDIDATO`
    // file: `*.dat` is gitignored, so leaving it under the name the generator
    // reads would make VERSIONED headers depend on a LOCAL file, real here and
    // identity everywhere else. The DECISION («measured and off, it loses 9,2 dB
    // on real DI») is not in question.
    // It costs NOTHING in the product: the switch is undefined, so the
    // `static_assert` is not even instantiated and the `.so` stays md5-identical.
    double correct(double dev_n7)
    {
#ifdef NLSC_CASC_CORR
        return c2_.process(dev_n7);
#else
        return dev_n7;
#endif
    }

    // UPPER BOUND of "modelling the rail": it receives the oracle's REAL
    // `VR`. Not a model — oracle injection, like the `aislado` mode — and it
    // says what would be won BEFORE writing the rail model.
    Outputs process(double in, double vr)
    {
        rail_ = vr;
        return process_(in);
    }

    // THE CASCADE'S BELT — and it comes from a MEASUREMENT.
    //
    // The abuse harness once exercised only the DK, while the cascade is the
    // DEFAULT engine. On its first exercise: ALL FOUR abuses break it, and
    // three leave it at NaN FOREVER (2,5 V of square at 2x leaves it at
    // 1,7e32; 5 V, 20 V and the 20 V step, at NaN). It recovers with neither
    // silence nor normal signal. The DK survives all four.
    //
    // Third time this workshop meets the same failure, and the rule says
    // it: EVERY FALLBACK NEEDS AN ABSOLUTE ANCHOR, not a relative one. Here
    // the absolute anchor is `prepare()` with `reinit`, which solves the
    // rest from scratch — the only state that ALWAYS is valid.
    //
    // The trigger is NOT just `!isfinite`: the 2,5 V case sits at 1,7e32,
    // finite and equally broken. It also bounds by MAGNITUDE, with a
    // physically impossible threshold (the pedal runs at 9 V; 100 V is not
    // a legitimate transient, it is a diverged engine).
    //
    // And `kRescue` CONSECUTIVE samples are required, like the DK: an
    // isolated event must not cause a click. The counter zeroes as soon as
    // one sample comes out sane, so on the normal path this costs one
    // comparison.
    static constexpr int    kRescue = 128;
    static constexpr double kAbsurd = 100.0;   // volts

    // The belt itself, shared by EVERY entry point. It used to live only in
    // process() — the rail-inert instrument entry — while the shipped path went
    // through process_modelled() unprotected: measured on the shipped path with
    // the gate's flags, four abuses left the engine at NaN forever, and the only
    // thing between that and the host was a per-sample isfinite that mutes
    // without healing the state.
    inline Outputs belt_(Outputs s)
    {
        if (!(std::fabs(s.out) < kAbsurd)) {    // catches NaN, inf and absurd
            ++belt_broken_;
            // While the engine is broken, emit the quiescent point, not garbage.
            // Re-emitting the last good value would be a RELATIVE anchor, which
            // is exactly what self-perpetuates.
            s = {q_n4_, q_n7_, q_n14_, q_out_};
            if (++belt_streak_ >= kRescue) {
                prepare(fs_, gain_, tone_, lvl_, circuit_, true);   // ABSOLUTE anchor
                ++rescues_;
            }
        } else {
            belt_streak_ = 0;
        }
        return s;
    }

    Outputs process(double in)
    {
        rail_ = q_vr_;
        return belt_(process_(in));
    }

    // UPPER BOUNDS by oracle injection, one per cut:
    //   · `n19`-oracle — the whole chain with the MODELLED rail, but Q2's
    //     waveshaper receives the ORACLE's `n19`. What remains is what the
    //     waveshaper itself and the networks BEHIND it contribute.
    //   · `n7`-oracle — stage 3 receives the ORACLE's `n7`. Upper bound of
    //     improving stage 2.
    //   · `n9`-oracle — stage 2's LOAD receives the ORACLE's `n9`. Upper
    //     bound of modelling well the load stage 3 hangs off `n7` — today
    //     `R7` in series with (`C5` ∥ `R8`), without the tone network.
    // `n9` travels ABSOLUTE, not in deviation: stage 2's KCL is the
    // circuit's and `Ieq = G7·n9` follows from it.
    Outputs process_con_n9(double in, double n9_absoluta_oraculo)
    {
#if NLSC_E2_N9_ORACLE
        e2_.set_n9_orac(n9_absoluta_oraculo);
        return process_modelled(in);
#else
        (void)in; (void)n9_absoluta_oraculo;
        std::fprintf(stderr, "n9-oracle mode without -DNLSC_E2_N9_ORACLE=1: "
                             "aborting rather than measuring SOMETHING ELSE\n");
        std::exit(2);
#endif
    }

    Outputs process_con_n7(double in, double dev_n7_oraculo)
    {
#if NLSC_CASC_N7_ORACLE
        n7_inj_ = dev_n7_oraculo;
        use_n7_inj_ = true;
        const Outputs s = process_modelled(in);
        use_n7_inj_ = false;
        return s;
#else
        (void)in; (void)dev_n7_oraculo;
        std::fprintf(stderr, "n7-oracle mode without -DNLSC_CASC_N7_ORACLE=1: "
                             "aborting rather than measuring SOMETHING ELSE\n");
        std::exit(2);
#endif
    }

    // The whole chain, but stage 1 starts from the ORACLE's `n3`.
    // What remains is Q1's WAVESHAPER plus `Hs`/`H2`. The difference
    // against `n4`-oracle prices `H1` and the rail coupling.
    Outputs process_con_n3(double in, double n3_absoluta_oraculo)
    {
#if NLSC_CASC_N3_ORACLE
        n3_inj_ = n3_absoluta_oraculo;
        use_n3_inj_ = true;
        const Outputs s = process_modelled(in);
        use_n3_inj_ = false;
        return s;
#else
        (void)in; (void)n3_absoluta_oraculo;
        std::fprintf(stderr, "⛔ n3-oracle mode without -DNLSC_CASC_N3_ORACLE=1: "
                             "abort rather than measure something ELSE\n");
        std::exit(2);
#endif
    }

    // The whole chain, but stage 4 is given the ORACLE's `n14`. What
    // remains is stage 4 ALONE. The difference against `n7`-oracle prices
    // stage 3 (the tone) by itself.
#if NLSC_CASC_VR_ORACLE
    // Pins the coming sample's ABSOLUTE `vr`. Called BEFORE
    // `process_modelled()`: the rail is consumed at the sample's start.
    void inject_vr(double vr_absoluto_oraculo)
    {
        vr_inj_ = vr_absoluto_oraculo;
        use_vr_inj_ = true;
    }
#endif

    Outputs process_con_n14(double in, double dev_n14_oraculo)
    {
#if NLSC_CASC_N14_ORACLE
        n14_inj_ = dev_n14_oraculo;
        use_n14_inj_ = true;
        const Outputs s = process_modelled(in);
        use_n14_inj_ = false;
        return s;
#else
        (void)in; (void)dev_n14_oraculo;
        std::fprintf(stderr, "n14-oracle mode without -DNLSC_CASC_N14_ORACLE=1: "
                             "aborting rather than measuring SOMETHING ELSE\n");
        std::exit(2);
#endif
    }

    // The whole chain, but `H2` feeds from the ORACLE's `n6`. What
    // remains of stage 1 is ONLY the output coupling (`C2` against `R4`).
    // The difference against `n3`-oracle prices the waveshaper and `Hs`.
    Outputs process_con_n6(double in, double n6_absoluta_oraculo)
    {
#if NLSC_CASC_N6_ORACLE
        n6_inj_ = n6_absoluta_oraculo;
        use_n6_inj_ = true;
        const Outputs s = process_modelled(in);
        use_n6_inj_ = false;
        return s;
#else
        (void)in; (void)n6_absoluta_oraculo;
        std::fprintf(stderr, "n6-oracle mode without -DNLSC_CASC_N6_ORACLE=1: "
                             "aborting rather than measuring SOMETHING ELSE\n");
        std::exit(2);
#endif
    }

    // The whole chain, but stage 2 is given the ORACLE's `n4`. UPPER
    // BOUND of improving STAGE 1. See the macro's comment above: it is the
    // half `n7`-oracle was missing to attribute.
    Outputs process_con_n4(double in, double dev_n4_oraculo)
    {
#if NLSC_CASC_N4_ORACLE
        n4_inj_ = dev_n4_oraculo;
        use_n4_inj_ = true;
        const Outputs s = process_modelled(in);
        use_n4_inj_ = false;
        return s;
#else
        (void)in; (void)dev_n4_oraculo;
        std::fprintf(stderr, "⛔ n4-oracle mode without -DNLSC_CASC_N4_ORACLE=1: "
                             "abort rather than measure something ELSE\n");
        std::exit(2);
#endif
    }

    // CLOSED along with its `Stage4` sibling, and for TWO reasons,
    // not one: (a) it calls `Stage4::process_con_n19`, which measures a
    // retired engine; (b) this body is itself a HAND COPY of
    // `process_modelled()` and had already diverged — it calls stage 4
    // WITHOUT the rail injection into `n19` (`rn19_.process(drv)`) that the
    // normal path passes.
    // => Two copies of one body, two distinct divergences. That is the whole
    // written-twice-diverges argument in one place.
    [[noreturn]] Outputs process_con_n19(double, double)
    {
        std::fprintf(stderr, "⛔ Cascade4::process_con_n19 is CLOSED: it was a "
                             "HAND COPY of process_modelled() and had diverged "
                             "(it called stage 4 without the rail injection into "
                             "`n19`). Aborting. See the comment above.\n");
        std::exit(2);
    }

    // THE MODELLED RAIL — the only route left, and the one that works.
    //
    // `VR` is not AC ground: it is a NODE, and its KCL is exact:
    //
    //     dVR · (Gtot + s·C11) = Σ dn_i / R_i
    //
    // checked against ngspice's AC at −173,5 dB (what an identity must
    // give). And the split says nearly everything comes from one place: at
    // 300 Hz `n9` feeds 85,4 %, `n18` 9,1 %, and `n4`/`n11`/`n19`/`n3` 6,5 %
    // between the four. Since `n9` and `n18` are LINEAR functions of `n7`
    // and `n14`, their two contributions ship already composed with the
    // rail's pole (`kVRA`, `kVRB`); the rest enters through the loose pole
    // (`kVRP`).
    //
    // There is an algebraic loop — `VR` depends on `n7`, which depends on
    // `VR` — broken with ONE sample of delay. At 384 kHz that is 2,6 µs
    // against a rail pole at 2,12 Hz: five orders of magnitude.
    Outputs process_modelled(double in)
    {
    // THE RAIL IS PREDICTED, NOT DELAYED.
    //
    // The algebraic loop `VR -> n7 -> VR` used to break with ONE sample of
    // delay — a FIRST-order error in `h`. It was defended with "2,6 µs
    // against a 2,12 Hz rail pole, five orders, it is noise" — literally the
    // argument that already fell on stage 4's `ib` loop, where taken at one
    // knob extreme it came out 50x optimistic. A slow pole does NOT bound a
    // delay's error: the loop gain times the signal's SLOPE does, and that
    // one is not slow.
    //
    // Truly closing it — like `ib`'s — is impossible here: the loop passes
    // through stage 2, which is NONLINEAR, so the rail would have to be an
    // unknown of its Newton. But cancelling the first order does not require
    // solving the loop: a linear prediction cancels it, in THREE
    // instructions.
    //
    // MEASURED, and that is why it ships. It buys, at the OUTPUT over the
    // 36 points: worst fixed part 87,34 -> 74,25 µV, median 6,12 -> 4,77, and
    // it stops dominating in 6 of 36 points to dominate in 2. It costs 3,00
    // instructions per sample (+0,134 %) and the clock cannot tell
    // (+0,52 %, p = 0,281, 31 pairs). It moves neither the null against
    // ngspice (worst −61,10 identical, 0/36 over −60), nor the ANMR
    // (−15,3), nor the alias at all — the real risk, since predicting
    // amplifies high-frequency noise.
    // The loop gain goes from 0,1496 to 0,4489 worst case (Nyquist), far
    // below 1.
    //
    // `NLSC_CASC_RAIL_PREDICTED=0` returns to the delay, kept because it is
    // the CONTROL everything here is measured with — not out of doubt.
    // And this PREDICTS, not solves: it cancels the first order and
    // leaves the ~36 µV floor intact, which is MODEL error.
#if NLSC_CASC_RAIL_PREDICTED
        const double dvr_pred = dvr_ + (dvr_ - dvr_ant_);
        dvr_ant_ = dvr_;
        rail_ = q_vr_ + dvr_pred;
#else
        rail_ = q_vr_ + dvr_;      // the CONTROL: one sample of delay
#endif
#if NLSC_CASC_VR_ORACLE
        // The EXACT rail. `dvr_` is still computed below with the
        // model: what gets overridden is what the stages CONSUME this
        // sample, which is what must be isolated. Without `use_vr_inj_`
        // this touches nothing.
        if (use_vr_inj_) rail_ = vr_inj_;
#endif
        const Outputs s = process_(in);
        // The NEXT sample's rail, from what just left each stage.
        // The CURRENTS each node feeds the rail are summed and the node's
        // impedance applies ONCE: `dVR·(Gtot + s·C11) = Σ n_k/R_k`, the
        // node's exact KCL (checked against ngspice at −173,5 dB).
        // Composing the pole inside each contribution, the fits came out
        // with poles at |z| = 1 and the cascade sank from −47,3 to −6,0 dB.
        // A DIAGNOSTIC PROBE THAT NO LONGER APPLIES — hence this branch
        // is CLOSED with `#error`.
        //
        // What it diagnosed: the 192k and 96k banks followed the OLD
        // convention ("kVRA and kVRB, already composed with the rail pole")
        // while the 384k one said "they are CURRENTS (no pole)". Since this
        // code always applies `vrp_` to the SUM, with those banks the rail
        // pole (DC gain 1595,7) entered TWICE: `n7`'s loop gain went from
        // 0,145 to 241 and the 4x diverged.
        //
        // The real fix was made: the banks are regenerated. Checked with
        // `harness/rail_ganancia.py`: all THREE banks declare "they are
        // CURRENTS", their DC gains are IDENTICAL (kVRA 9,090887e-05 ·
        // kVRB 8,797041e-07 · kVRP 1,595692e+03) and `n7`'s loop gain is
        // 0,145 at 384k, 192k and 96k.
        //
        // WHY IT CLOSES INSTEAD OF BEING DELETED OR LEFT: with TODAY's
        // banks this branch would apply the rail pole one time too FEW,
        // giving a wrong model with NO error. And `EXTRA_DEFS` lets any
        // macro be defined, so it is not hypothetical: the system already
        // allows it. The text stays because it explains a x1663 divergence
        // that cost a day.
#if defined(NLSC_CASC_RAIL_COMPOSED)
#  error "NLSC_CASC_RAIL_COMPOSED probes a RETIRED convention. Today's banks carry kVRA/kVRB as CURRENTS at all three rates - check with 'python3 harness/rail_ganancia.py' - so this branch would apply the rail pole one time too few and give a wrong model with NO error."
#endif
#if 0
        dvr_ = vra_.process(s.n7 - q_n7_)
             + vrb_.process(s.n14 - q_n14_)
             + vrp_.process(g_n4_  * (s.n4 - q_n4_)
                          + g_n11_ * (e2_.n11() - q_n11_)
                          + g_n19_ * dev_n19_
                          + g_n3_  * dev_n3_);
#else
        dvr_ = kVrG * vrp_.process(kVrA * vra_.process(s.n7 - q_n7_)
                          + vrb_.process(s.n14 - q_n14_)
                          + g_n4_  * (s.n4 - q_n4_)
                          + g_n11_ * (e2_.n11() - q_n11_)
                          + vrc_.process(dev_n19_)
                          + g_n3_  * dev_n3_);
#endif
        // The belt wraps the SHIPPED path too. A broken sample also poisons the
        // rail update above (dvr_ goes NaN with it); that is fine because the
        // belt's absolute anchor — prepare(reiniciar) — re-seeds dvr_/dvr_ant_
        // and every filter along with the stages.
        return belt_(s);
    }

    Outputs process_(double in)
    {
        // Stage 1: `nls_stage1.h` returns ABSOLUTE `n4` over ITS own rest
        // (`kVr`), so it converts to deviation by subtraction.
        // HOW THE RAIL ENTERS `n4`, and it is not 1:1.
        // `n4` sees VR through `R4` (10 k) and `n6` through `C2` (1 µF), so
        //     n4 = VR·1/(1+sR4C2) + n6·sR4C2/(1+sR4C2)
        // The second term already sits INSIDE stage 1's `H2` fit. What must
        // be added is the first: a one-pole low-pass at 15,9 Hz on the
        // rail's motion.
        // Adding `drail` raw (the first version's move) is right at DC and
        // FALSE above 16 Hz — and it showed: `n4`'s 35-200 and 200-1k bands
        // WORSENED from −46,6 to −41,7 dB.
        const double drail = lp_rail(rail_ - q_vr_);
        double n3_abs = 0.0;
// THE TWO INJECTIONS ARE INDEPENDENT, NOT AN `#elif`. The first version
// chained `#if N3 / #elif N4`, so compiling both at once — exactly what
// comparing their bounds in ONE run requires — ERASED the `n4` one, and the
// `n4`-oracle mode degenerated into `modelado` with NO error. The ordering
// caught it: `n4`-oracle must fall strictly between `modelado` and
// `n7`-oracle, and it came out identical to `modelado`.
#if NLSC_CASC_N3_ORACLE || NLSC_CASC_N4_ORACLE || NLSC_CASC_N6_ORACLE
        double dev_n4;
#if NLSC_CASC_N3_ORACLE
        // `n3`-oracle: stage 1 runs FROM `n3`, not from the input, so
        // everything below (waveshaper + `Hs` + `H2`) acts ON the injected
        // value. Exclusive with the normal path: calling both would advance
        // `Hs`/`H2`'s state twice.
        if (use_n3_inj_) {
            n3_abs = n3_inj_;
            // THE `n6` PROBE'S POINTER WAS MISSING HERE, and the failure
            // was SILENT: `process_con_n3()` defaults it to `nullptr`, so
            // on the `n3`-oracle path the probe kept the value of the LAST
            // sample that took the normal path — i.e. the startup rest. The
            // file's `n6` column came out with 2.674 mV of bias and a
            // 0,00 dB null, exactly the number that reads as "the model is
            // broken" instead of "the probe does not measure".
            dev_n4 = e1_.process_con_n3(n3_inj_, kProbeN6) - e1_.rest_n4() + drail;
        } else
#endif
#if NLSC_CASC_N6_ORACLE
        // `n6`-oracle: the stage runs WHOLE — `H1`, waveshaper and `Hs`
        // advance and `n3` comes out right, which the rail model needs —
        // and only `H2`'s input is substituted. Exclusive with the normal
        // path for the same reason as its sibling: `H2` advances once, not
        // twice.
        if (use_n6_inj_) {
            dev_n4 = e1_.process_con_n6(in, rn3_.process(rail_ - q_vr_), n6_inj_,
                                        &n3_abs, kProbeN6) - e1_.rest_n4() + drail;
        } else
#endif
        {
            dev_n4 = e1_.process(in, rn3_.process(rail_ - q_vr_), &n3_abs,
                                 kProbeN6) - e1_.rest_n4() + drail;
        }
#if NLSC_CASC_N4_ORACLE
        // Stage 1 KEEPS running (its state must advance); all that
        // changes is what stage 2 sees — as with `n7`-oracle.
        if (use_n4_inj_) dev_n4 = n4_inj_;
#endif
#else
        const double dev_n4 = e1_.process(in, rn3_.process(rail_ - q_vr_), &n3_abs,
                                          kProbeN6) - e1_.rest_n4() + drail;
#endif
        dev_n3_ = n3_abs - q_n3_;
#ifdef NLSC_CASC_PROBE_N36
        n3_abs_probe_ = n3_abs;
#endif

#if NLSC_CASC_SEGMENTED
        // Stage `k` consumes what `k-1` produced on the PREVIOUS sample.
        // Same work, same instructions, same register pressure — only the
        // intra-sample dependency disappears.
        const double seg_n4 = seg_n4_;   seg_n4_ = dev_n4;
#endif
        // Stage 2: works in absolutes (its equations are the circuit's).
        e2_.set_vr(rail_);
#if NLSC_E2_COST_PROBE
        // COST PROBE, NEVER SHIPPED. It answers "how much would a
        // 4th-order method multiply the clock by" — one needing >=2
        // non-linear solves per sample — BEFORE implementing it.
        //
        // The second solve starts from the SAME state, not the converged
        // one: from converged the Newton leaves in one iteration and the
        // probe would measure an empty call. Hence copy and restore.
        //
        //   =1  control: copy and restore, ONE solve  -> isolates the copy's cost
        //   =2  probe:   copy, solve, restore, solve
        {
            const Stage2 guarda = e2_;
#if NLSC_E2_COST_PROBE == 2
            (void) e2_.process(q_n4_ + dev_n4);
#endif
            asm volatile("" : : "r"(&guarda) : "memory");
            e2_ = guarda;
        }
#endif
#if NLSC_CASC_N7_ORACLE
        double dev_n7 = correct(e2_.process(q_n4_ + dev_n4) - q_n7_);
        // Stage 2 KEEPS running (its state must advance); all that
        // changes is what stage 3 sees.
        if (use_n7_inj_) dev_n7 = n7_inj_;
#else
#if NLSC_CASC_SEGMENTED
        const double dev_n7 = correct(e2_.process(q_n4_ + seg_n4) - q_n7_);
#else
        const double dev_n7 = correct(e2_.process(q_n4_ + dev_n4) - q_n7_);
#endif
#endif

        // The rail enters stage 3 through `R8` (10 k) towards `n9`, and
        // stage 4 through `R12` (510 k) towards `n19`. The fits were made
        // with VR pinned, so these are the missing summands.
        const double drv = rail_ - q_vr_;
#if NLSC_CASC_N14_ORACLE
        // TONE COST PROBE — and it works by COPY AND RESTORE, not by
        // skipping the filter.
        //
        // THE FIRST ATTEMPT SKIPPED `rn14_` and `vra_` to measure "what they
        // would cost less", and the cheap arm came out **4x SLOWER**: without
        // the filter the rail enters raw, the operating point runs away and
        // the Newton needs many more iterations. A probe that breaks the model
        // measures the BROKEN MODEL, not the lever — the same result
        // `NLSC_E2_ITER_CAP=1` already gave ("a broken model is dearer, not
        // cheaper").
        // => Here the cost of ONE EXTRA PASS over a copy of the state is
        // measured, which is `NLSC_E2_COST_PROBE`'s pattern: the result does
        // not change and the only thing added is the work being priced.
        //   =1  one extra pass of `rn14_`  -> its marginal cost
        double dev_n14 = e3_.process(dev_n7) + rn14_.process(drv);
#if NLSC_TONE_COST_PROBE
        {
            RailN14Variable copia = rn14_;
            (void) rn14_.process(drv);
            asm volatile("" : : "r"(&copia) : "memory");
            rn14_ = copia;
        }
#endif
        // Stage 3 KEEPS running (its state must advance); the only change
        // is what stage 4 sees — as with `n4` and `n7`.
        if (use_n14_inj_) dev_n14 = n14_inj_;
#else
#if NLSC_CASC_SEGMENTED
        const double seg_n7 = seg_n7_;   seg_n7_ = dev_n7;
        const double dev_n14 = e3_.process(seg_n7) + rn14_.process(drv);
#else
        const double dev_n14 = e3_.process(dev_n7) + rn14_.process(drv);
#endif
#endif
        // ATTRIBUTION PROBE (not a product mode): switch off the rail's
        // injection into `n19` to see how much of that node's error is ITS.
        // The rail enters AFTER the level pot, so its error does not
        // attenuate with the knob — exactly the measured signature (constant
        // absolute error while the signal drops 4,2x).
#ifdef NLSC_NO_RN19
        const double dev_out = e4_.process(dev_n14, &dev_n19_, 0.0);
        (void)drv;
#else
#  if NLSC_CASC_SEGMENTED
        const double seg_n14 = seg_n14_;  seg_n14_ = dev_n14;
        const double dev_out = e4_.process(seg_n14, &dev_n19_, rn19_.process(drv), drv);
#  else
        const double dev_out = e4_.process(dev_n14, &dev_n19_, rn19_.process(drv), drv);
#  endif
#endif

        return {q_n4_ + dev_n4, q_n7_ + dev_n7,
                q_n14_ + dev_n14, q_out_ + dev_out};
    }

    // Each stage fed by the ORACLE's output instead of the previous one.
    // Returns, per boundary, what THAT stage alone produces. It is the only
    // way to know where the loss is BORN: in chain mode, stage 1's error
    // reaches stage 4 amplified by everything in between, and the final
    // number does not say whose it is.
    Outputs process_isolated(double in, double ref_n4, double ref_n7, double ref_n14,
                            double vr = e234::kQ_vr, double ref_n3 = 0.0)
    {
        e2_.set_vr(vr);
        // With `ref_n3` > 0 stage 1 splits in two: it is GIVEN the oracle's
        // `n3` node and only its waveshaper and output network get judged.
        // THE RAIL WAS MISSING HERE, falsifying the attribution: chain
        // mode adds to `n4` the rail filtered by `R4`·`C2` and isolated mode
        // did not, so stage 1 came out worse than it is and looked like the
        // floor. An instrument that does not apply to a block what it
        // applies in the chain measures another product.
        const double drail = lp_rail(vr - q_vr_);
        const double dev_n4  = (ref_n3 > 0.0
                                ? e1_.process_con_n3(ref_n3)
                                : e1_.process(in, rn3_.process(vr - q_vr_)))
                               - e1_.rest_n4() + drail;
        const double dev_n7  = correct(e2_.process(ref_n4) - q_n7_);
        // THE RAIL WAS MISSING AGAIN, in stages 3 and 4. The SAME defect
        // written three lines up for stage 1: chain mode adds to `n14` and
        // `n19` what the rail injects through `R8` and `R12`, and isolated
        // mode did not => both stages came out WORSE than they are, and
        // `n19`'s attribution ran against an instrument measuring another
        // product. A lesson written in the comment next door does not
        // protect the code next door.
        // With `vr` at rest (pure `aislado` mode) `drv` is 0 and this is
        // bit-identical to what there was: a correction, not a mode change.
        const double drv     = vr - q_vr_;
        const double dev_n14 = e3_.process(ref_n7 - q_n7_) + rn14_.process(drv);
        // The FOURTH argument was missing while the chain path passes it:
        // `dvr` fell to its 0.0 default, so stage 4's open-base network never
        // saw the rail in isolated mode and the stage measured worse than it
        // is — the third instance of the instrument defect described above.
        // With `vr` at rest `drv` is 0, so this is bit-identical to what it did.
        const double dev_out = e4_.process(ref_n14 - q_n14_, &dev_n19_,
                                           rn19_.process(drv), drv);
        return {q_n4_ + dev_n4, q_n7_ + dev_n7,
                q_n14_ + dev_n14, q_out_ + dev_out};
    }

    Stage1& stage1() { return e1_; }   // observability: the waveshaper guard
    Stage4& stage4() { return e4_; }
#if NLSC_CASC_IB_ORACLE
    void inject_ib(double ib) { e4_.inject_ib(ib); }
#endif

    // PROBE (not product): the last sample's ABSOLUTE `n19` and `n17`.
    // They judge stage 4 against the oracle BY BOUNDARY instead of looking
    // only at `out` — the only thing that attributes a chain's error.
    double n19_abs() const { return q_n19_ + dev_n19_; }
    double n17_abs() const { return e4_.n17_abs(); }
    Stage2& stage2() { return e2_; }
    // Quiescent point of n4, so an oracle-injection harness can turn an
    // ABSOLUTE node from the reference into the DEVIATION this class takes.
    double q_n4() const { return q_n4_; }
    // `n7`'s rest, which the harness needs to turn ngspice's ABSOLUTE
    // into the DEVIATION `process_con_n7()` expects. Without it the
    // harness would carry its own copy of the rest — and a constant
    // written twice diverges.
    double q_n7() const { return q_n7_; }
    // Same for `n14`, which `process_con_n14()` expects as a DEVIATION.
    double q_n14() const { return q_n14_; }

    // PROBE: the LAST sample's `n19` deviation. Exists to measure the
    // null at stage 4's INTERNAL boundary without pulling the block out of
    // the chain — which cannot be done here, because the rail closes a
    // global loop.
    double dev_n19() const { return dev_n19_; }
    // The RAIL, ABSOLUTE. Exposed because it turned out to be the
    // chain's shared error source with no way to look at it: the harness
    // dumped `n4 n7 n14 out` and the rail travelled inside all four.
    double rail() const { return rail_; }
    // ABSOLUTE `n11`. Exposed for the same reason as the rail: it
    // carries 61 % of node VR's DC current and was the only loop term no
    // reference dumped.
    double n11() const { return e2_.n11(); }

private:
    // TODAY's rests (`prepare` sets them from the knob; see above).
    // The conductances into the rail, from the active variant's bank.
    // Read PER SAMPLE, so they go to members like the waveshaper.
    double g_n3_  = e234::kG_n3,  g_n4_  = e234::kG_n4;
    double g_n11_ = e234::kG_n11, g_n19_ = e234::kG_n19;
    double q_vr_  = e234::kQ_vr,  q_n3_ = e234::kQ_n3, q_n4_  = e234::kQ_n4;
    double q_n7_  = e234::kQ_n7,  q_n11_ = e234::kQ_n11, q_n14_ = e234::kQ_n14;
    double q_out_ = e234::kQ_out;
    double q_n19_ = e234::kQ_n19;
    // MEASUREMENT SCALAR on the RAIL loop's gain. Measured: the DK
    // moves the rail +15,08 mV of DC with the DI and the cascade +13,81 =>
    // 8,3 % short, and the defect is CONSTANT over 4x of level
    // (−8,39 / −8,32 / −8,24 %) — a GAIN error, not a missing non-linear
    // term. This exists to check that with a single run.
    // Defaults to 1,0 => the product `.so` comes out bit-identical.
    // SCALAR ON `n7`'s TERM ALONE (the `n9` branch via `R8`).
    // Hypothesis: `kVRA` comes from `(n9/n7)/R8` measured with VR PINNED =
    // 0,9091 (10/11). With VR free the real DC quotient is 0,9643
    // (measured with the DK's `n9`), because VR moves WITH `n7`. If that
    // is the −8,3 % source, scaling by 0,9643/0,9091 = 1,0607 should
    // close it.
    static constexpr double kVrA =
#ifdef NLSC_VR_GA
        NLSC_VR_GA;
#else
        1.0;
#endif
    static constexpr double kVrG =
#ifdef NLSC_VR_GAIN
        NLSC_VR_GAIN;
#else
        1.0;
#endif
    double rail_ = e234::kQ_vr;

    // PROBE: absolute `n3` and `n6`, for the per-node profile against
    // ngspice (`make perfil-spice`). `run_cascada_spice` once wrote the rest
    // CONSTANT into those two columns, so stage 1 — where the real distance
    // to the DK lives, 10,8 dB — could not be bisected against the truth.
    // Behind a flag: `n6_out` is an optional pointer inside stage 1, and
    // always passing it would add one store per sample to the PRODUCT path
    // for pure instrumentation. The plugin compiles without this and comes
    // out bit-identical.
#ifdef NLSC_CASC_PROBE_N36
    double n3_abs_probe_ = 0.0, n6_abs_probe_ = 0.0;
public:
    double n3_probe() const { return n3_abs_probe_; }
    double n6_probe() const { return n6_abs_probe_; }
private:
#endif

    // One-pole low-pass (R4·C2 = 10 ms), trapezoidal at the internal rate.
    // Coefficients HOISTED to `prepare_bank`: they depend only
    // on `fs_`, and computing them here cost two divisions per INTERNAL sample
    // — eight per base sample at 4x — that never change. Same expression, so
    // the result is bit-identical by construction, and it was checked that way.
    // `lp_set_rate()` is the ONE place that writes them: anything that
    // changes `fs_` has to call it, which is why it is a method and not two
    // lines in `prepare_bank`: hoisting is only safe if nothing mutates the
    // inputs.
    void lp_set_rate()
    {
        // The REAL rate, not `e234::kFs`: that constant belongs to the
        // tabulated coefficient bank and no longer rules here.
        const double T = 1.0 / fs_, tau = 10e3 * 1e-6;
        lp_b0_ = T / (T + 2.0 * tau);
        lp_a1_ = (T - 2.0 * tau) / (T + 2.0 * tau);
    }
    double lp_rail(double x)
    {
        const double b0 = lp_b0_, a1 = lp_a1_;
        const double y = b0 * (x + lp_x_) - a1 * lp_y_;
        lp_x_ = x; lp_y_ = y;
        return y;
    }
    double lp_x_ = 0.0, lp_y_ = 0.0;
    double lp_b0_ = 0.0, lp_a1_ = 0.0;   // set by `lp_set_rate()`
    double fs_ = e234::kFs;
    double dvr_ = 0.0, dev_n3_ = 0.0, dev_n19_ = 0.0;
    double dvr_ant_ = 0.0;      // the previous sample, to predict the rail
#if NLSC_CASC_SEGMENTED
    // The N2 probe's pipeline registers. They start at ZERO because they
    // are DEVIATIONS over rest, which is a stage's value at rest.
    double seg_n4_ = 0.0, seg_n7_ = 0.0, seg_n14_ = 0.0;
#endif
    FixedFilter<fijos::kN3_NB, fijos::kN3_NA> rn3_;
    // `rail::kN14` is not used any more either, for the SAME reason: it
    // was a fixed tone=0,5 filter and the rail enters through `R8` BEFORE
    // the tone network, so what it injects gets filtered by the knob
    // (−13,6 dB of error at tone=0, −7,4 at tone=1).
    // `rail::kN3` DOES stay fixed, and that is MEASURED, not assumed:
    // `VR -> n3` does not depend on drive (−101 to −113 dB at all five
    // positions). Two of the three filters were wrong, not all three — and
    // the right one is the control.
    RailN14Variable rn14_;
    // `rail::kN19` IS NOT USED any more. It was a FIXED filter fitted at
    // lvl=0, and `VR -> n19` changes 58x along the level pot (0,0128 ->
    // 0,742 at 1 kHz, measured in ngspice). Since the rail enters through
    // `R12`, AFTER the pot, its error does not attenuate with the knob:
    // lowering the volume drops the signal and the error stays => it
    // dominated stage 4's residual (+13,5 dB at lvl=0,75). It now comes from
    // the SAME algebra as `E4G1`.
    // The constant remains in `nls_rail_n3.h` because its generator emits
    // the three together and their bit-identical regeneration is the control
    // that the invocation is right; but NOBODY should use it again.
    RailN19Variable rn19_;
#if NLSC_VRA_PARAM
    RailVraVariable vra_;
#else
    FixedFilter<fijos::kVRA_NB, fijos::kVRA_NA> vra_;
#endif
    FixedFilter<fijos::kVRB_NB, fijos::kVRB_NA> vrb_;
    // `n19`'s BRANCH, AND WHY IT IS A FILTER AND NOT A CONSTANT. It
    // used to enter through the scalar `G_n19` = 1/R12, and that left the
    // rail 1,190 % low in DC, identical at all 27 knob positions, because
    // `n19` reaches the rail also through `C8` into the LEVEL pot's `n18`,
    // not just via `R12`.
    //
    // Measured: that branch's admittance moves 15,7 dB between DC and the
    // high band, while its three passive siblings (`n3`, `n4`, `n11`) come
    // out FLAT at 0,0 dB — the control that the census does not flag
    // everything.
    //
    // And its price was MEASURED before writing it, because the
    // cascade's budget is frozen: 9,00 instructions per sample (+0,402 %)
    // and ZERO distinguishable clock (41 paired pairs, p = 0,755).
    FixedFilter<fijos::kVRC_NB, fijos::kVRC_NA> vrc_;
    FixedFilter<fijos::kVRP_NB, fijos::kVRP_NA> vrp_;

    Stage1 e1_;
    Stage2 e2_;
    FixedFilter<fijos::kE2C_NB, fijos::kE2C_NA> c2_;
    Stage3 e3_;
    Stage4 e4_;
};

} // namespace nlsc
