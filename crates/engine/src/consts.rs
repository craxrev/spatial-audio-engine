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

pub const DB_TO_LINEAR_DIVISOR: f32 = 8.65617;

// 2 · sqrt(pi)
pub const SH_W_NORM: f32 = 3.544_907_7;

pub const DEFAULT_RT60_DB: f32 = -15.0;
pub const DEFAULT_REVERB_LOW_HZ: f32 = 600.0;
pub const DEFAULT_REVERB_LOW_DB: f32 = -100.0;
pub const DEFAULT_REVERB_HIGH_HZ: f32 = 3500.0;
pub const DEFAULT_REVERB_HIGH_DB: f32 = -20.0;
pub const DEFAULT_DIFFUSION_COEF: f32 = 0.69;

/// Per-cell calibration multiplier applied at HRTF load time, per
/// §13. IEEE-754 single `0x3FCAE148 = 1.5850000381`.
pub const HRTF_LOAD_GAIN: f32 = 1.585;

pub const DIFFUSER_DELAY_SECONDS: [f32; 8] = [
    0.0203, 0.0244, 0.0316, 0.0273, 0.0229, 0.0293, 0.0135, 0.0191,
];
pub const FDN_DELAY_SECONDS: [f32; 8] = [
    0.1531, 0.2103, 0.1278, 0.2569, 0.1748, 0.1924, 0.125, 0.2200,
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
    fn db_to_linear_divisor_is_spec_literal() {
        // §4 gives 8.65617 (annotated "≈ 20 / ln(10)"). The literal
        // is ~0.34% off from the exact value (8.6859); we keep the
        // spec's number verbatim. Errors at extreme −80 dB reach ~3%
        // but the constant is used in audible ranges where divergence
        // is well under 1 dB. See note in development notes.
        assert_eq!(DB_TO_LINEAR_DIVISOR, 8.65617);
    }
}
