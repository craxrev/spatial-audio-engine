//! Global constants from `design notes` §4.

pub const BLOCK_SIZE: usize = 128;
pub const AMBI_ORDER: usize = 3;
pub const NUM_AMBI: usize = 16;
pub const OUTPUT_CHANNELS: usize = 2;
pub const FDN_SIZE: usize = 8;
pub const LOG2_FDN_SIZE: usize = 3;
pub const NUM_DIFFUSERS: usize = 10;

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

pub const DEFAULT_RT60_DB: f32 = -15.0;
pub const DEFAULT_REVERB_LOW_HZ: f32 = 600.0;
/// Native reference value (bit-verified): -100 dB at 600 Hz acts as
/// a near brick-wall HPF. Designed for an SDK where each application
/// reconfigures per scene; useless for a plugin where the user just
/// wants to hear reverb. See `PLUGIN_REVERB_LOW_DB`.
pub const DEFAULT_REVERB_LOW_DB: f32 = -100.0;
pub const DEFAULT_REVERB_HIGH_HZ: f32 = 3500.0;
pub const DEFAULT_REVERB_HIGH_DB: f32 = -20.0;
pub const DEFAULT_DIFFUSION_COEF: f32 = 0.69;

/// Plugin-friendly shelf gains. Deviates from spec so the reverb is
/// audible out of the box. See development notes "spec discrepancies".
pub const PLUGIN_REVERB_LOW_DB: f32 = -20.0;
pub const PLUGIN_REVERB_HIGH_DB: f32 = -6.0;

/// Per-cell calibration multiplier applied at HRTF load time, per
/// §13. IEEE-754 single `0x3FCAE148 = 1.5850000381`.
pub const HRTF_LOAD_GAIN: f32 = 1.585;

/// 10-stage Schroeder diffuser delays. Not bit-verified against the
/// reference (native synthesises these at runtime); see §4 and
/// development notes "spec discrepancies" — this is a faithful-sounding
/// substitute, architecturally correct, modal fingerprint differs.
pub const DIFFUSER_DELAY_SECONDS: [f32; NUM_DIFFUSERS] = [
    0.0203, 0.0244, 0.0316, 0.0273, 0.0229, 0.0293, 0.0135, 0.0191, 0.0181, 0.0257,
];
/// 8-line FDN core delays. Same caveat as the diffuser table above.
pub const FDN_DELAY_SECONDS: [f32; FDN_SIZE] = [
    0.1531, 0.2103, 0.1278, 0.2569, 0.1748, 0.1924, 0.1250, 0.2200,
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
