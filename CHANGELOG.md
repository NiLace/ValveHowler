# Changelog

All notable changes to **Valve Howler**, newest first.

Versions follow the LV2 rules: the plugin's URI is the major version, so a release is
numbered `1.minor.micro`; a release has a minor number above zero, and even minor and micro
numbers.

---

## 1.6.0 — 2026-11-01

Sessions saved with 1.4.4 load with HQ off, the new default; switch HQ on for the
processing of earlier versions. Driven very hard, the clipping now follows the real op-amp
(below).

- **New HQ switch, with its button on the panel** above the Tone legend. The pedal now
  runs lighter on the CPU by default. For usual playing, the default should be enough.
  HQ is recommended for pinched harmonics, and when the signal reaching the plugin peaks
  above 0 dBFS (a booster, an overdrive or a gain plugin before the Valve Howler; the
  README's *Input level* explains the levels). Switching HQ on restores the higher-quality oversampling of earlier
  versions, which keeps pinched harmonics and very hot inputs cleaner, at the cost of
  more CPU, most of it with very hot inputs. The switch fades through the dry signal
  like a variant change, so it does not click, and the latency reported to the host does
  not change.

  The technical detail: with HQ off the top octave is very slightly softer, about 1.3
  to 2.2 dB down at 18 to 20 kHz; with HQ on the top octave is exact.
- **The op-amp is now the real 4558's.** Its output headroom follows the manufacturer's
  datasheet curves (how far the output can swing at each load and supply) instead of a
  fixed limit. At usual playing levels nothing changes; driven very hard, the clipping
  of the drive stage comes closer to the circuit's, most audibly with a booster in front.
- **The variant list closes when the host hides or moves the editor**, and gives the
  mouse and keyboard back.
- **The editor opens faster, and reopens almost at once**: the panel's images are
  decoded once per process instead of on every open.
- **Several editors of this family of pedals open at once** keep their display errors to
  themselves; one closing can no longer disturb the others.
- **The source is much smaller**: code the plugin never ran, measurement leftovers and
  stale comments are gone. The sound does not change: the output is identical sample for
  sample to the build before the cleanup.
- The version goes from 1.4.4 to 1.6.0, not 1.5.0: an odd middle number marks a
  development version under the LV2 rules.

---

## 1.4.4 — 2026-10-25

Sessions saved with 1.4.2 load unchanged, and the sound is the same.

- **Closing and reopening the editor can no longer freeze the host.** With two of these
  editors open, or next to another plugin that watches the same kind of display errors,
  closing and reopening one could leave the host stuck at full CPU on its next display
  error. The interface now installs its error guard once and never hands it back.
- **The host survives the display going away under the editor.** If the window system
  dropped the editor's connection, the host could be shut down with it. Now only the
  editor closes. Any number of editors can be open at once without their errors
  reaching the host.
- **Less CPU when driven hard.** The second stage's slew limiting is computed more
  cheaply. The output is unchanged.
- **The DC jack is drawn properly seated**, nut on its washer and plug pushed home, and
  the bundle now carries its licence file.

---

## 1.4.2 — 2026-10-18

Sessions saved with 1.4.0 load unchanged, and the sound is the same.

- **Closing the interface can no longer crash the host later.** After the editor was
  closed and the host had unloaded the interface, the host could crash the next time it
  drew a lot of text of its own. The interface now leaves nothing behind that the host
  might call.

---

## 1.4.0 — 2026-09-30

Sessions saved with 1.0.0 load unchanged.

- **The variants are renamed** OD-8 W Tone, OD-9 W Tone, OD-8 L Tone and OD-9 L Tone.
  Only the names change: a saved session picks up the variant it had.
- **A new, rendered interface.** The panel is rendered from a 3D model of the pedal on a
  pedalboard: knobs that turn, a footswitch that goes down when pressed, and a backlit
  variant display. The box takes the colour of the circuit the variant uses, and when
  the pedal is bypassed the print turns grey and the display goes dark. It stays sharp
  on high-density screens, and it fits the window if your host resizes it.
- **The interface no longer takes the host down** when the host gives it an unusual
  parent window.
- **The top octave is kept.** At 44.1 and 48 kHz the highest harmonics, around 18 to
  20 kHz, were rolled off; they now come through.
- **The reported latency is now 31 samples** (0.65 ms at 48 kHz), up from 7. Your host
  compensates for it; a project with stems bounced against 1.0.0 will not line up
  sample for sample with a new render.
- **Hot input.** Active pickups and a booster in front of the plugin are now handled
  like the real pedal.
- **Knobs glide instead of jumping.** Moving DRIVE, TONE or LEVEL no longer clicks.
  A settled knob sounds exactly as before.
- **Recovery after an extreme input.** An input far above full scale could leave the
  plugin silent until it was re-activated. It now recovers.

Version 1.2.0 was prepared but not published; its changes are the last four points
above.

---

## 1.0.0 — 2026-09-12

**First stable release.** From this version on the plugin's URI and its eight ports
(`in`, `out`, `drive`, `tone`, `level`, `latency`, `enabled`, `variant`) will not
change, so sessions saved with it keep loading.

If you used a pre-release (0.1 or 0.2), a session built on it will not reload
unchanged:

- The oversampling control is gone; the plugin always runs at 4x.
- The reported latency is 7 samples, where the pre-release reported 15.
- The engine selector, the unit seed and the unused `model` and `clipping` controls
  are gone.

The plugin took the name Valve Howler in this release.

---

## 0.2 — 2026-08-24

Pre-release. The 2x oversampling option was removed.

---

## 0.1 — 2026-08-24

Pre-release. First versioned snapshot.
