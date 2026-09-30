# Changelog

All notable changes to **Valve Howler**, newest first.

Versions follow the LV2 rules: the plugin's URI is the major version, so a release is
numbered `1.minor.micro`; a release has a minor number above zero, and even minor and micro
numbers.

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
