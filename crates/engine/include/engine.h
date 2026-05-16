// Spatial audio engine — C ABI surface.
//
// Implementation in `crates/engine/src/c_api.rs`, built into the
// staticlib `libengine.a` when the `c-api` cargo feature is enabled:
//
//     cargo build --release -p engine --features c-api
//
// Threading: the engine handle is not safe to use concurrently from
// multiple threads. Callers must serialise process_block and the
// per-parameter setters on a single thread (typically the audio
// thread in a JUCE-style host).

#ifndef SPATIAL_AUDIO_ENGINE_H
#define SPATIAL_AUDIO_ENGINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque engine handle.
typedef struct Engine Engine;

// Allocate a new engine. Returns NULL on invalid input.
// Release via engine_destroy.
Engine* engine_new(uint32_t sample_rate, uint32_t num_sources);

// Release an engine previously obtained from engine_new.
// Safe to call on NULL.
void engine_destroy(Engine* engine);

// Block size the engine processes at, in samples. Always 128.
uint32_t engine_block_size(void);

// engine-native frame: +X forward, +Y left, +Z up, right-handed.
void engine_set_listener_position(Engine* engine, float x, float y, float z);
// Quaternion (w, x, y, z).
void engine_set_listener_rotation(Engine* engine, float w, float x, float y, float z);

void engine_set_source_position(Engine* engine, uint32_t idx, float x, float y, float z);
// Linear gain (caller does dB -> linear).
void engine_set_source_gain(Engine* engine, uint32_t idx, float gain);
void engine_set_source_active(Engine* engine, uint32_t idx, bool active);

// Source orientation quaternion (w, x, y, z) — engine-native frame.
void engine_set_source_rotation(
    Engine* engine, uint32_t idx, float w, float x, float y, float z);

// Source-only direct-path gain (linear). Reverb-send path is unaffected.
void engine_set_source_direct_path_gain(Engine* engine, uint32_t idx, float gain);

// Occlusion in [0, 1]; clamped and ramped to a per-source low-pass.
void engine_set_source_occlusion(Engine* engine, uint32_t idx, float occlusion);

// Directivity cone (§6.2). Angles in radians. Defaults {0, 2π, 1, 0}
// disable the cone.
void engine_set_source_directivity(
    Engine* engine, uint32_t idx,
    float inner_ang, float outer_ang, float outer_gain, float outer_lp);

// Per-source reverb send (linear, ramped). 0 = source doesn't feed reverb.
void engine_set_source_reverb_send(Engine* engine, uint32_t idx, float send);

// Master reverb mix (linear). 0 = dry, 1 = unity wet.
void engine_set_reverb_amount(Engine* engine, float amount);

// §9 externalizer parameters. Both 0..100; amount = 0 disables.
void engine_set_externalizer_amount(Engine* engine, float value);
void engine_set_externalizer_character(Engine* engine, float value);

// Install the main HRTF decoder from a 16,384-byte buffer matching
// data/hrtf_decoder_native.bin. Returns true on success.
bool engine_load_main_hrtf(Engine* engine, const uint8_t* bytes, size_t len);

// Process one 128-sample block.
//
// `inputs` is `num_sources * 128` f32s, source-major:
//   inputs[i*128 .. (i+1)*128] is the mono input for source i.
// `out_left` and `out_right` each receive 128 binaural f32s,
// overwritten (not accumulated).
//
// Inactive sources are ignored regardless of input contents.
// If no HRTF is loaded, both outputs are zero.
void engine_process_block(
    Engine* engine,
    const float* inputs,
    uint32_t num_sources,
    float* out_left,
    float* out_right
);

#ifdef __cplusplus
}
#endif

#endif // SPATIAL_AUDIO_ENGINE_H
