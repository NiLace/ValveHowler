# Changelog

All notable changes to **Valve Howler** are recorded here, newest first. Versions
follow the **LV2 versioning rules** rather than semantic versioning: there is
deliberately no major version, because the plugin URI *is* the major version. A
release is `minorVersion.microVersion`.

> **How "1.0.0" maps onto two numbers.** LV2 can express only two, so they are split
> the way the specification already splits them: the leading **1** is the URI, which
> *is* the major version and is what makes it permanent, and the **0.0** is
> `lv2:minorVersion 1 ; lv2:microVersion 0`. "1.0.0" and LV2's "1.0" are the same
> statement written twice, not two versions.

---

## 1.0.0 — 2026-09-12

**First stable release.** This is the version that fixes the plugin's identity: the
URI, the port set and their names can no longer change.

### Heads-up, if you used a pre-release

The 0.1 and 0.2 snapshots were a `minorVersion 0` pre-release series, which the
specification gives a hard meaning — *"Hosts SHOULD NOT expect such a plugin to
remain compatible with any future version"*. Three things a saved session could have
referred to are gone, so a project built on 0.1 or 0.2 will not reload unchanged:

- **The oversampling control is gone, and the factor is fixed at 4x.** The knob had
  no correct answer to offer. Measured on the 36-point knob grid: **2x fails the
  fidelity bar in 28 of those 36 points**, worst case −50.1 dB against a −60 dB
  target; **8x costs 92 % more CPU and buys 3.1 dB** of the aliasing score that
  actually binds; **4x passes at all 36 points** with 2.30 dB of margin. A control
  with one correct setting is not a control.
- **The reported latency is now 7 samples**, where the pre-release reported 15. The
  host compensates for it either way, but a project that bounced stems against the
  old value will not line up sample-for-sample with a re-render.
- **The engine selector, the unit seed, and two dead knobs (`model`, `clipping`) are
  gone.** The last two were declared in the manifest, movable by the user, and read
  by code that did nothing with them, which is a user-facing defect rather than an
  unfinished feature.

### The port set is frozen, and this is what freezes it

`lv2:minorVersion` goes 0 to 1, which is LV2's own machine-readable statement that
these **eight** ports and their symbols can never change under this URI:

`in` · `out` · `drive` · `tone` · `level` · `latency` · `enabled` · `variant`

The specification is explicit: *"All versions of a plugin with a given URI MUST have
the same set of mandatory ports with respect to `lv2:symbol` and `rdf:type`."* Three
independent sources were made to agree on that set before it was frozen: the port
enumeration in the source, the manifest, and `lv2info` run against the **installed**
bundle rather than the build directory.

### The plugin is named Valve Howler

Renamed from its working name. Everything that resolves to something followed: the
permanent URI `https://nylarea.com/plugins/valvehowler`, the bundle, both binaries
and the repository. The timing was the point — a plugin URI can be changed up to the
first release and never after, and nothing outside the workshop carried the old name.

The rename was proved not to have touched the DSP: rendered output is **bit-identical**
to a build of the parent commit at three operating points (drive/tone/level at
0.2/0.3/0.8, 0.5/0.5/0.5 and 1.0/0.9/0.2). Negative arm: two different knob settings
do produce different files, so the comparison is not blind.

### Measured state

Null against ngspice running the same netlist, the fidelity bar being −60 dB or
better:

| variant | oversampling | null vs ngspice | aliasing (ANMR) |
|---|---|---:|---:|
| OD-8 | 4x | **−70.5 dB** | −36.95 |
| OD-9 | 4x | **−71.6 dB** | −37.70 |

CPU figures are deliberately not quoted here: they describe one machine and one knob
position, and a number without both of those attached compares with nothing.

---

## 0.2 — 2026-08-24

Pre-release. **2x oversampling was withdrawn from the offer**: measured at −58.4 dB
(OD-8) and −58.5 dB (OD-9) against a −60 dB bar, and above the bar at 28 of 36 points
on the knob grid. Offering it in an unlabelled dropdown shipped a mode whose fidelity
the plugin advertised nowhere.

The 2x path was not deleted, only removed from the offer — verified rather than
asserted. Against the 0.1 binary over 240,000 samples of real playing, every
remaining factor rendered **byte for byte identical**, so the change touched the offer
and nothing else.

---

## 0.1 — 2026-08-24

First versioned snapshot: the work that already existed, frozen and given a number.

The one change made to earn it was renaming a port symbol that was still in Spanish.
A port symbol is the string a host writes into the user's saved session, so renaming
it after a stable release does not raise an error — the host fails to find the port,
loads the default, and the plugin sounds different when the project is reopened, with
no warning at all. It was done at `minorVersion 0` because that is the last window in
which it is free.
