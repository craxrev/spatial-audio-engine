//! Audio source state. M3 carried position + gain + SH-gain crossfade
//! buffers. M6 extends this with source orientation, the directivity
//! cone (§6.2), occlusion + directivity low-pass (§6.3), and
//! `direct_path_gain` (§6.4).

use core::f32::consts::TAU;

use crate::consts::{BLOCK_SIZE, NUM_AMBI, OCCLUSION_RAMP_SAMPLES};
use crate::distance::DistanceModel;
use crate::math::{Quat, Vec3};
use crate::ramp::Ramp;

#[derive(Clone, Debug)]
pub struct Source {
    pub pos: Vec3,
    pub rotation: Quat,
    /// `(object.pos − listener.pos)` rotated by `listener.quat⁻¹`.
    /// Cached so we only re-encode when it changes (§6.1).
    pub rel_pos: Vec3,
    pub pos_dirty: bool,
    pub gain: Ramp,
    /// §6.4 multiplicative gain on the direct path only (not the
    /// reverb send). Defaults to 1.
    pub direct_path_gain: f32,

    // §6.2 directivity-cone parameters (angles in radians).
    // Defaults: {0, 2π, 1, 0} = cone disabled.
    pub inner_ang: f32,
    pub outer_ang: f32,
    pub outer_gain: f32,
    pub outer_lp: f32,
    /// Per §6.5: directivity gain has its own ramp, distinct from the
    /// user-set gain ramp. Tracks `1 + t · (outer_gain − 1)`.
    pub directivity_gain: Ramp,
    /// `t · outer_lp` from §6.2, recomputed per block.
    pub directivity_lp_offset: f32,

    /// User-set occlusion target ∈ [0, 1], smoothed over
    /// `OCCLUSION_RAMP_SAMPLES`. Drives the §6.3 LP cutoff.
    pub occlusion: Ramp,

    /// §6.6 per-source reverb send. Ticks per sample like the gain
    /// and directivity ramps; the engine mixes
    /// `gain · rev_send.current · filtered` into the mono reverb input bus.
    pub rev_send: Ramp,

    /// §3 4-knot distance → gain curve. Replaces the placeholder
    /// `1/max(r, 0.1)` fallback used in M3-M8. Drives both the direct
    /// path attenuation and the reverb send distance attenuation.
    pub distance_model: DistanceModel,

    /// §2.4 `positionMode`. 0 = world (engine applies listener-inverse
    /// transform); 1 = relative (`pos` is already in listener frame,
    /// engine skips the transform — for head-locked / HUD sources).
    pub position_mode: u8,

    /// §2.5 `renderingMode`. 0 = spatial (full pipeline); 1 = stereo
    /// bypass (mix straight into stereo output, skip all spatial DSP
    /// and the reverb send).
    pub rendering_mode: u8,

    /// §6.7 input channel count. 1 = mono (only `ch0` is read);
    /// 2 = stereo (both channels SH-encoded at the same position
    /// sharing per-object state).
    pub input_channel_count: u8,

    // §6.3 1-pole low-pass state. Coefficients are recomputed only
    // when `total = clamp(occl_now + dir_lp_offset, 0, ∞)` changes
    // (either because the occlusion ramp is active or because the
    // directivity LP offset moved). Two independent states for stereo
    // input — same coefficients, separate per-channel memory so the
    // two ears of a stereo object don't bleed through the filter.
    pub lp_state: [f32; 2],
    pub lp_b0: f32,
    pub lp_a1: f32,
    pub lp_total_last: f32,

    /// Previous block's SH gains, used as the crossfade start (§6.4).
    pub sh_gains_old: [f32; NUM_AMBI],
    /// Current block's SH gains; also the steady-state value when
    /// `pos_dirty == false`.
    pub sh_gains_new: [f32; NUM_AMBI],
    pub active: bool,
}

impl Default for Source {
    fn default() -> Self {
        Self {
            pos: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            rel_pos: Vec3::ZERO,
            pos_dirty: true,
            gain: Ramp::new(0.0, BLOCK_SIZE as u32),
            direct_path_gain: 1.0,
            inner_ang: 0.0,
            outer_ang: TAU,
            outer_gain: 1.0,
            outer_lp: 0.0,
            directivity_gain: Ramp::new(1.0, BLOCK_SIZE as u32),
            directivity_lp_offset: 0.0,
            occlusion: Ramp::new(0.0, OCCLUSION_RAMP_SAMPLES),
            rev_send: Ramp::new(0.0, BLOCK_SIZE as u32),
            distance_model: DistanceModel::default(),
            position_mode: 0,
            rendering_mode: 0,
            input_channel_count: 1,
            lp_state: [0.0; 2],
            // total=0 → fc ≈ fs/2 → b0 ≈ 1, a1 ≈ 0 (transparent).
            // First block always recomputes; these are placeholders.
            lp_b0: 1.0,
            lp_a1: 0.0,
            lp_total_last: f32::NAN,
            sh_gains_old: [0.0; NUM_AMBI],
            sh_gains_new: [0.0; NUM_AMBI],
            active: false,
        }
    }
}
