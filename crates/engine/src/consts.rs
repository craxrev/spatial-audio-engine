//! Global constants from `design notes` §4.

pub const BLOCK_SIZE: usize = 128;
pub const AMBI_ORDER: usize = 3;
pub const NUM_AMBI: usize = 16;
pub const OUTPUT_CHANNELS: usize = 2;
pub const FDN_SIZE: usize = 8;
pub const LOG2_FDN_SIZE: usize = 3;
/// One Schroeder allpass per FDN line, no cascade. Bit-verified
/// against both v0.4 and v0.5 wasms (loop bound = 8, one delay seed
/// per line). The `3` at engine+0x270 is the Hadamard butterfly
/// stage count (= log₂(FDN_SIZE)), not a per-line cascade depth.
pub const NUM_DIFFUSERS: usize = FDN_SIZE;

pub const OCCLUSION_RAMP_SAMPLES: u32 = 1000;
pub const EXTERNALIZER_RAMP_TIME: f32 = 0.05;

pub const OCCLUSION_FREQ_BASE: f32 = 1100.0;
pub const OCCLUSION_FREQ_SCALE: f32 = 20.0;
pub const OCCLUSION_FC_MAX_FACTOR: f32 = 0.425;

/// Reference-binary literal `0x410A7FAC` — ~0.34% above the math
/// identity `20/ln(10) = 8.68589`. Kept verbatim for parity with
/// native's deliberate behaviour. See §4.
pub const DB_TO_LINEAR_DIVISOR: f32 = 8.65617;

// 2 · sqrt(pi)
pub const SH_W_NORM: f32 = 3.544_907_7;

pub const DEFAULT_REVERB_LOW_HZ: f32 = 600.0;
pub const DEFAULT_REVERB_LOW_DB: f32 = -100.0;
pub const DEFAULT_REVERB_HIGH_HZ: f32 = 3500.0;
pub const DEFAULT_REVERB_HIGH_DB: f32 = -20.0;

/// Time-domain scale on Schroeder + FDN delay seeds (also feeds the
/// per-line shelf_decay budget). Bit-verified `0x3f30a3d7` overwrite
/// of the initial 1.0 at `engine_ctor:773`. Despite being labelled
/// "diffusion coefficient", it does NOT operate as the Schroeder
/// allpass `g` — that one is dynamic per line via Jot's formula.
pub const DIFFUSION_COEFFICIENT: f32 = 0.69;

/// Per-cell calibration multiplier applied at HRTF load time, per
/// §13. IEEE-754 single `0x3FCAE148 = 1.5850000381`.
pub const HRTF_LOAD_GAIN: f32 = 1.585;

/// Schroeder allpass delay-time seeds (seconds). Hardcoded in
/// `native_engine_ctor` as i32.const float bit patterns. Identical
/// between v0.4 and v0.5. Sample-count delays computed at runtime as
/// `round(DIFFUSION_COEFFICIENT * seed * fs)`.
pub const SCHROEDER_DELAY_SECONDS: [f32; NUM_DIFFUSERS] = [
    0.020346, 0.024421, 0.031604, 0.027333,
    0.022904, 0.029291, 0.013458, 0.019123,
];
/// FDN core delay-time seeds (seconds). Same provenance as above;
/// same `round(scale * seed * fs)` runtime conversion.
pub const FDN_DELAY_SECONDS: [f32; FDN_SIZE] = [
    0.153129, 0.210389, 0.127837, 0.256891,
    0.174713, 0.192303, 0.125000, 0.219991,
];

/// Per-line FDN reverb send weights (§13). Pairs 1:1 with
/// `REVERB_OUTPUT_DIRS`.
pub const REV_SEND_GAINS: [f32; FDN_SIZE] = [0.32, 0.55, 0.55, 0.41, 0.45, 0.19, 0.32, 0.63];
/// Per-line FDN reverb decode weights (§13).
pub const REV_DECODE_GAINS: [f32; FDN_SIZE] = [1.0, 1.0, 0.7, 1.0, 0.9, 0.9, 0.7, 1.0];

/// 8 reverb output directions, converted from §13's
/// `(+x = right, +y = forward, +z = up)` frame into engine-native
/// `(+X = forward, +Y = left, +Z = up)` via `(X, Y, Z) = (y, -x, z)`.
/// Used to SH-encode the FDN outputs into the ambisonic bus every block.
pub const REVERB_OUTPUT_DIRS: [[f32; 3]; FDN_SIZE] = [
    [ 0.610_319_3,  -0.616_944_43,  0.496_880_1],
    [-0.610_319_3,  -0.616_944_43, -0.496_880_1],
    [ 0.611_301_24,  0.615_971_57,  0.496_880_1],
    [-0.611_301_24,  0.615_971_57, -0.496_880_1],
    [ 0.867_768_6,  -0.009_369_127, 0.496_880_1],
    [ 0.0,          -0.867_819_2,  -0.496_880_1],
    [ 0.001_382_0,   0.867_818_1,   0.496_880_1],
    [-0.867_768_6,  -0.009_369_127,-0.496_880_1],
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ambi_count_matches_order() {
        assert_eq!(NUM_AMBI, (AMBI_ORDER + 1).pow(2));
    }

    #[test]
    fn log2_fdn_size_correct() {
        assert_eq!(1 << LOG2_FDN_SIZE, FDN_SIZE);
    }

    #[test]
    fn sh_w_norm_is_two_sqrt_pi() {
        assert!((SH_W_NORM - 2.0 * std::f32::consts::PI.sqrt()).abs() < 1e-6);
    }

    #[test]
    fn db_to_linear_divisor_is_reference_literal() {
        // Verified against both v0.4 and v0.5 wasms — the binary
        // really uses 8.65617f (0x410A7FAC), not 20/ln(10).
        assert_eq!(DB_TO_LINEAR_DIVISOR, 8.65617);
    }
}
