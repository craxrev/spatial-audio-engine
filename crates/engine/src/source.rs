//! Audio source state. M3 carried position + gain + SH-gain crossfade
//! buffers. M6 extends this with source orientation, the directivity
//! cone (§6.2), occlusion + directivity low-pass (§6.3), and
//! `direct_path_gain` (§6.4).

use core::f32::consts::TAU;

use crate::consts::{BLOCK_SIZE, NUM_AMBI, OCCLUSION_RAMP_SAMPLES};
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

    // §6.3 1-pole low-pass state. Coefficients are recomputed only
    // when `total = clamp(occl_now + dir_lp_offset, 0, ∞)` changes
    // (either because the occlusion ramp is active or because the
    // directivity LP offset moved).
    pub lp_state: f32,
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
            lp_state: 0.0,
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
