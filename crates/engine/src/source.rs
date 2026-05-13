//! Audio source state. For M3 this carries position, an active flag,
//! a gain ramp, and the double-buffered SH gains for §6.4
//! crossfade. Occlusion, directivity, distance models, reverb send,
//! and multichannel input land in later milestones.

use crate::consts::{BLOCK_SIZE, NUM_AMBI};
use crate::math::Vec3;
use crate::ramp::Ramp;

#[derive(Clone, Debug)]
pub struct Source {
    pub pos: Vec3,
    /// `(object.pos − listener.pos)` rotated by `listener.quat⁻¹`.
    /// Cached so we only re-encode when it changes (§6.1).
    pub rel_pos: Vec3,
    pub pos_dirty: bool,
    pub gain: Ramp,
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
            rel_pos: Vec3::ZERO,
            pos_dirty: true,
            gain: Ramp::new(0.0, BLOCK_SIZE as u32),
            sh_gains_old: [0.0; NUM_AMBI],
            sh_gains_new: [0.0; NUM_AMBI],
            active: false,
        }
    }
}
