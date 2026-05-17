//! C-ABI surface for non-Rust consumers (the JUCE/C++ AU plugin, a
//! future cpal demo, etc.). Enabled by the `c-api` feature; the
//! corresponding hand-written header lives at
//! `crates/engine/include/engine.h`.
//!
//! Threading: the engine handle is not safe to use concurrently
//! from multiple threads. Callers (e.g. JUCE) must ensure that
//! `process_block` and the per-parameter setters are serialised on
//! the same thread, or guarded with a lock at the call site.

use core::ffi::c_uchar;
use core::slice;

use crate::audio_bed::BedFormat;
use crate::consts::BLOCK_SIZE;
use crate::engine::Engine;
use crate::hrtf::Hrtf;

/// Allocate a new engine. Returns `null` on invalid input.
///
/// # Safety
/// The returned pointer must be released exactly once via
/// `engine_destroy`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_new(sample_rate: u32, num_sources: u32) -> *mut Engine {
    if sample_rate == 0 {
        return core::ptr::null_mut();
    }
    let engine = Engine::new(sample_rate, num_sources as usize);
    Box::into_raw(Box::new(engine))
}

/// Release an engine previously obtained from `engine_new`.
///
/// # Safety
/// `engine` must be either null or a pointer returned by
/// `engine_new` that has not yet been destroyed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_destroy(engine: *mut Engine) {
    if !engine.is_null() {
        drop(unsafe { Box::from_raw(engine) });
    }
}

/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_listener_position(
    engine: *mut Engine,
    x: f32,
    y: f32,
    z: f32,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_listener_position(x, y, z);
    }
}

/// Quaternion is `(w, x, y, z)`.
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_listener_rotation(
    engine: *mut Engine,
    w: f32,
    x: f32,
    y: f32,
    z: f32,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_listener_rotation(w, x, y, z);
    }
}

/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_position(
    engine: *mut Engine,
    idx: u32,
    x: f32,
    y: f32,
    z: f32,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_position(idx as usize, x, y, z);
    }
}

/// `gain` is linear (caller does dB → linear).
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_gain(engine: *mut Engine, idx: u32, gain: f32) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_gain(idx as usize, gain);
    }
}

/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_active(engine: *mut Engine, idx: u32, active: bool) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_active(idx as usize, active);
    }
}

/// Source orientation quaternion `(w, x, y, z)` (engine-native frame).
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_rotation(
    engine: *mut Engine,
    idx: u32,
    w: f32,
    x: f32,
    y: f32,
    z: f32,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_rotation(idx as usize, w, x, y, z);
    }
}

/// §6.4 `direct_path_gain` (linear, multiplicative on the direct path).
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_direct_path_gain(
    engine: *mut Engine,
    idx: u32,
    gain: f32,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_direct_path_gain(idx as usize, gain);
    }
}

/// §6.3 occlusion target ∈ [0, 1]. Clamped, then smoothed over the
/// per-source occlusion ramp.
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_occlusion(
    engine: *mut Engine,
    idx: u32,
    occlusion: f32,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_occlusion(idx as usize, occlusion);
    }
}

/// §6.6 per-source reverb send (linear, ramped). 0 = no reverb send.
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_reverb_send(
    engine: *mut Engine,
    idx: u32,
    send: f32,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_reverb_send(idx as usize, send);
    }
}

/// Master reverb mix multiplier. 0 = dry, 1 = unity wet. Applied to
/// the FDN outputs before spatialisation.
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_reverb_amount(engine: *mut Engine, amount: f32) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_reverb_amount(amount);
    }
}

/// §9.1 externalizer amount (0..100). 0 = disabled.
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_externalizer_amount(engine: *mut Engine, value: f32) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_externalizer_amount(value);
    }
}

/// §9.1 externalizer character (0..100; 50 = neutral).
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_externalizer_character(engine: *mut Engine, value: f32) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_externalizer_character(value);
    }
}

/// §2.4 position_mode (0 = world, 1 = relative/head-locked).
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_position_mode(
    engine: *mut Engine,
    idx: u32,
    mode: u8,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_position_mode(idx as usize, mode);
    }
}

/// §2.5 rendering_mode (0 = spatial, 1 = stereo bypass).
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_rendering_mode(
    engine: *mut Engine,
    idx: u32,
    mode: u8,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_rendering_mode(idx as usize, mode);
    }
}

/// §6.7 input channel count (1 = mono, 2 = stereo).
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_input_channel_count(
    engine: *mut Engine,
    idx: u32,
    count: u8,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_input_channel_count(idx as usize, count);
    }
}

/// §3 4-knot distance curve. Knot gains are linear; caller must
/// pre-convert dB → linear (`10^(dB/20)`).
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn engine_set_source_distance_curve(
    engine: *mut Engine,
    idx: u32,
    a_dist: f32,
    a_gain: f32,
    b_dist: f32,
    b_gain: f32,
    c_dist: f32,
    c_gain: f32,
    d_dist: f32,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_distance_curve(
            idx as usize, a_dist, a_gain, b_dist, b_gain, c_dist, c_gain, d_dist,
        );
    }
}

/// §6.2 directivity cone. Angles in radians.
/// Defaults `{0, 2π, 1, 0}` disable the cone.
///
/// # Safety
/// `engine` must be a valid pointer from `engine_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_source_directivity(
    engine: *mut Engine,
    idx: u32,
    inner_ang: f32,
    outer_ang: f32,
    outer_gain: f32,
    outer_lp: f32,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_source_directivity(idx as usize, inner_ang, outer_ang, outer_gain, outer_lp);
    }
}

/// §2.6 audio bed format. See `audio_bed::BedFormat` for the enum
/// values (0 = NoInput / removes the bed). Returns `true` on a valid
/// format, `false` otherwise.
///
/// # Safety
/// `engine` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_audio_bed_format(
    engine: *mut Engine,
    format: u8,
) -> bool {
    let Some(e) = (unsafe { engine.as_mut() }) else { return false; };
    let Some(f) = BedFormat::from_u8(format) else { return false; };
    e.set_audio_bed_format(f);
    true
}

/// Bed master linear gain.
///
/// # Safety
/// `engine` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_audio_bed_gain(engine: *mut Engine, gain: f32) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_audio_bed_gain(gain);
    }
}

/// `headlocked = true` makes the bed move with the listener;
/// `false` (default) world-locks it.
///
/// # Safety
/// `engine` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_set_audio_bed_headlocked(
    engine: *mut Engine,
    headlocked: bool,
) {
    if let Some(e) = unsafe { engine.as_mut() } {
        e.set_audio_bed_headlocked(headlocked);
    }
}

/// §13 / §12 step 10: install the W-channel binauralizer
/// (decoder_post) from two raw blobs matching
/// `data/hrtf_post_filter_a.bin` and `_b.bin` (each
/// `W_BINAURALIZER_TAPS · 4 = 11,460 bytes`). Returns `true` on
/// success.
///
/// # Safety
/// `engine` must be valid; `filter_a` and `filter_b` must each point
/// to `filter_a_len` / `filter_b_len` readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_load_w_binauralizer(
    engine: *mut Engine,
    filter_a: *const c_uchar,
    filter_a_len: usize,
    filter_b: *const c_uchar,
    filter_b_len: usize,
) -> bool {
    let Some(e) = (unsafe { engine.as_mut() }) else { return false; };
    if filter_a.is_null() || filter_b.is_null() {
        return false;
    }
    let a = unsafe { slice::from_raw_parts(filter_a, filter_a_len) };
    let b = unsafe { slice::from_raw_parts(filter_b, filter_b_len) };
    e.load_w_binauralizer(a, b)
}

/// Install the main HRTF decoder from a 16,384-byte buffer matching
/// `data/hrtf_decoder_native.bin`. Returns `true` on success.
///
/// # Safety
/// `engine` must be valid; `bytes` must point to `len` readable
/// bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_load_main_hrtf(
    engine: *mut Engine,
    bytes: *const c_uchar,
    len: usize,
) -> bool {
    let Some(e) = (unsafe { engine.as_mut() }) else {
        return false;
    };
    if bytes.is_null() {
        return false;
    }
    let slice = unsafe { slice::from_raw_parts(bytes, len) };
    let rate = e.sample_rate;
    match Hrtf::load_from_bytes_at(slice, rate) {
        Ok(hrtf) => {
            e.load_main_hrtf(&hrtf);
            true
        }
        Err(_) => false,
    }
}

/// Block size the engine processes at, in samples. Always 128.
#[unsafe(no_mangle)]
pub extern "C" fn engine_block_size() -> u32 {
    BLOCK_SIZE as u32
}

/// Process one 128-sample block.
///
/// `inputs` is `num_sources × 2 × 128` source-major f32s. Each
/// source's 256-float slab is `[ch0_0..ch0_127, ch1_0..ch1_127]`.
/// `bed_inputs` is `n_bed_channels × 128` channel-major f32s (one
/// 128-float buffer per bed channel, in the configured format's
/// channel order); pass `null` / `0` if no bed.
/// `out_left` and `out_right` each receive 128 binaural samples
/// (overwritten, not accumulated).
///
/// # Safety
/// All pointers must be valid for the indicated lengths.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_process_block(
    engine: *mut Engine,
    inputs: *const f32,
    num_sources: u32,
    bed_inputs: *const f32,
    n_bed_channels: u32,
    out_left: *mut f32,
    out_right: *mut f32,
) {
    let Some(e) = (unsafe { engine.as_mut() }) else {
        return;
    };
    let n = num_sources as usize;
    let input_slice = if inputs.is_null() || n == 0 {
        &[][..]
    } else {
        // SAFETY: caller guarantees `inputs` is valid for `n*2*128` f32s.
        unsafe { slice::from_raw_parts(inputs as *const [[f32; BLOCK_SIZE]; 2], n) }
    };
    let nb = n_bed_channels as usize;
    let bed_slice = if bed_inputs.is_null() || nb == 0 {
        &[][..]
    } else {
        // SAFETY: caller guarantees `bed_inputs` is valid for `nb*128` f32s.
        unsafe { slice::from_raw_parts(bed_inputs as *const [f32; BLOCK_SIZE], nb) }
    };
    e.process_block(input_slice, bed_slice);
    if !out_left.is_null() {
        unsafe {
            slice::from_raw_parts_mut(out_left, BLOCK_SIZE)
                .copy_from_slice(&e.stereo_out[0]);
        }
    }
    if !out_right.is_null() {
        unsafe {
            slice::from_raw_parts_mut(out_right, BLOCK_SIZE)
                .copy_from_slice(&e.stereo_out[1]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_lifecycle() {
        unsafe {
            let e = engine_new(48000, 1);
            assert!(!e.is_null());
            engine_set_source_active(e, 0, true);
            engine_set_source_position(e, 0, 5.0, 0.0, 0.0);
            engine_set_source_gain(e, 0, 1.0);
            let inputs = vec![1.0_f32; BLOCK_SIZE * 2];
            let mut l = vec![0.0_f32; BLOCK_SIZE];
            let mut r = vec![0.0_f32; BLOCK_SIZE];
            engine_process_block(
                e,
                inputs.as_ptr(), 1,
                core::ptr::null(), 0,
                l.as_mut_ptr(), r.as_mut_ptr(),
            );
            // Without a loaded HRTF, stereo is zero.
            assert!(l.iter().all(|v| *v == 0.0));
            engine_destroy(e);
        }
    }

    #[test]
    fn null_handle_is_safe() {
        unsafe {
            engine_destroy(core::ptr::null_mut());
            engine_set_source_gain(core::ptr::null_mut(), 0, 1.0);
            engine_process_block(
                core::ptr::null_mut(),
                core::ptr::null(), 0,
                core::ptr::null(), 0,
                core::ptr::null_mut(), core::ptr::null_mut(),
            );
        }
    }

    #[test]
    fn block_size_constant() {
        assert_eq!(engine_block_size() as usize, BLOCK_SIZE);
    }
}
