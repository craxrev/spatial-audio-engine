# spatial-audio-engine

Clean-room implementation of a 3D-spatial / binaural audio engine
(ambisonic-based with HRTF decode, FDN reverb, externalizer, optional
multichannel bed bus, etc.). Suitable for Web Audio AudioWorklet,
native real-time audio, or any host that can run a 128-sample block
loop at the audio sample rate.

## Contents

- **`design notes`** — the canonical, stack-agnostic build spec.
  Read this first. ~2000 lines, 17 sections. Self-contained: every
  algorithm and constant the engine needs is in here.
- **`data/`** — bundled HRTF coefficients (or design your own per §14):
  - `hrtf_decoder_native.bin` — 16 KB main ambisonic → binaural decoder
  - `hrtf_post_filter_a.bin` / `hrtf_post_filter_b.bin` — W-channel
    binauralizer pair (v0.5 default)
  - `hrtf_post_legacy_v04.bin` — optional v0.4 cross-channel
    post-coloration (only used if you implement §17)
- **`agent notes`** — agent scope and guidance.

## Getting started

```
1. Read design notes §1-§2 for the high-level shape.
2. Pick a language + audio host.
3. Implement per the spec, bottom-up (§5 → §6 → §7 → §8 → §9 → §12).
4. Cross-check against a test scene.
```

§17 (legacy v0.4 post-decoder) is optional. The default implementation
target is v0.5; §17 adds backward compatibility for v0.4-era
applications.

## Build

```
cargo build --workspace
cargo test --workspace
```

Workspace crates:

- `engine` — pure DSP library (no I/O, no plugin SDK, no host code).
- `vst` — per-track VST3 / CLAP / standalone adapter via `nih-plug`
  (wired in at M5).
- `cli` — offline WAV renderer (M16).
- `cpal-demo` — native real-time demo (post-spec consumer).

See `development notes` for the phased build plan.

## License of bundled data

The HRTF coefficient files in `data/` are extracted from a third-party
third-party engine. They are bundled here for reference and
reproducibility. If you ship a product, supply your own HRTF
coefficients (see §14 of the spec for procedure).
