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
    match Hrtf::load_from_bytes(slice) {
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
/// `inputs` is `num_sources × 128` interleaved-per-source f32s,
/// i.e. `inputs[i*128 .. (i+1)*128]` is the mono input for source
/// `i`. `out_left` and `out_right` receive 128 binaural samples
/// each (overwritten, not accumulated). Inactive sources are
/// ignored regardless of input contents.
///
/// # Safety
/// All pointers must be valid for the indicated lengths.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn engine_process_block(
    engine: *mut Engine,
    inputs: *const f32,
    num_sources: u32,
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
        // SAFETY: caller guarantees `inputs` is valid for `n*128` f32s.
        unsafe { slice::from_raw_parts(inputs as *const [f32; BLOCK_SIZE], n) }
    };
    e.process_block(input_slice);
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
            let inputs = vec![1.0_f32; BLOCK_SIZE];
            let mut l = vec![0.0_f32; BLOCK_SIZE];
            let mut r = vec![0.0_f32; BLOCK_SIZE];
            engine_process_block(e, inputs.as_ptr(), 1, l.as_mut_ptr(), r.as_mut_ptr());
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
                core::ptr::null(),
                0,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
            );
        }
    }

    #[test]
    fn block_size_constant() {
        assert_eq!(engine_block_size() as usize, BLOCK_SIZE);
    }
}
