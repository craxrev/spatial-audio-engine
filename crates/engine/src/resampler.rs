//! §11 sample-rate handling. Kaiser-windowed sinc resampler for
//! converting authored-at-48 kHz HRIRs (and the W-binauralizer pair)
//! to the runtime engine sample rate.
//!
//! Used only at load time — never in the per-block audio path.

/// Stopband attenuation target, in dB. 80 dB → β ≈ 7.86, half-band
/// width 32 taps gives a clean transition for typical audio rates.
const STOPBAND_DB: f32 = 80.0;
const HALF_WIDTH: usize = 32;

fn kaiser_beta(stop_db: f32) -> f32 {
    if stop_db <= 21.0 {
        0.0
    } else if stop_db <= 50.0 {
        0.5842 * (stop_db - 21.0).powf(0.4) + 0.07886 * (stop_db - 21.0)
    } else {
        0.1102 * (stop_db - 8.7)
    }
}

/// Modified Bessel function `I₀(x)` via its power series.
/// Converges fast for `x ≤ ~10`; 16 terms easily reach 1e-10.
fn mod_bessel_i0(x: f32) -> f32 {
    let mut sum = 1.0_f32;
    let mut term = 1.0_f32;
    let x2_4 = x * x * 0.25;
    for n in 1..16 {
        term *= x2_4 / (n as f32 * n as f32);
        sum += term;
        if term < 1e-10 {
            break;
        }
    }
    sum
}

#[inline]
fn sinc_normalized(x: f32) -> f32 {
    if x.abs() < 1e-10 {
        1.0
    } else {
        let p = core::f32::consts::PI * x;
        p.sin() / p
    }
}

fn kaiser_window(x_norm: f32, beta: f32, inv_i0_beta: f32) -> f32 {
    let xa = x_norm.abs();
    if xa >= 1.0 {
        return 0.0;
    }
    let arg = beta * (1.0 - x_norm * x_norm).sqrt();
    mod_bessel_i0(arg) * inv_i0_beta
}

/// Resample `src` from `src_rate` to `dst_rate`, returning a fresh
/// buffer of length `round(src.len() · dst_rate / src_rate)`.
/// Identity (zero-copy clone) when rates match.
pub fn resample(src: &[f32], src_rate: u32, dst_rate: u32) -> Vec<f32> {
    if src_rate == dst_rate {
        return src.to_vec();
    }
    let ratio = dst_rate as f32 / src_rate as f32;
    // Downsampling needs an anti-alias cutoff at the lower Nyquist.
    let cutoff = ratio.min(1.0);
    let half = HALF_WIDTH as f32;
    let half_i = HALF_WIDTH as isize;
    let beta = kaiser_beta(STOPBAND_DB);
    let inv_i0_beta = 1.0 / mod_bessel_i0(beta);

    let n_dst = ((src.len() as f32 * ratio).round() as usize).max(1);
    let mut dst = Vec::with_capacity(n_dst);
    for m in 0..n_dst {
        let x = m as f32 / ratio;
        let k_center = x.floor() as isize;
        let mut acc = 0.0_f32;
        for k in (k_center - half_i + 1)..=(k_center + half_i) {
            if k < 0 {
                continue;
            }
            let ku = k as usize;
            if ku >= src.len() {
                continue;
            }
            let t = x - k as f32;
            let kernel = cutoff
                * sinc_normalized(t * cutoff)
                * kaiser_window(t / half, beta, inv_i0_beta);
            acc += src[ku] * kernel;
        }
        dst.push(acc);
    }
    dst
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_when_rates_match() {
        let src: Vec<f32> = (0..32).map(|i| i as f32 * 0.1).collect();
        let dst = resample(&src, 48000, 48000);
        assert_eq!(dst, src);
    }

    #[test]
    fn downsample_length_correct() {
        // 128 samples at 48 kHz → 117.6 at 44.1 kHz → rounds to 118.
        let src = vec![0.0_f32; 128];
        let dst = resample(&src, 48000, 44100);
        assert_eq!(dst.len(), 118);
    }

    #[test]
    fn upsample_length_correct() {
        // 128 samples at 48 kHz → 256 at 96 kHz.
        let src = vec![0.0_f32; 128];
        let dst = resample(&src, 48000, 96000);
        assert_eq!(dst.len(), 256);
    }

    #[test]
    fn impulse_resamples_to_centered_sinc() {
        // δ[N/2] at 48 kHz, resampled 48 → 96 kHz, should produce a
        // sinc-shaped response centred around the upsampled position.
        let mut src = vec![0.0_f32; 64];
        src[32] = 1.0;
        let dst = resample(&src, 48000, 96000);
        // Peak should be near 64 (= 32 * 2).
        let (peak_idx, peak_val) = dst.iter().enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .map(|(i, v)| (i, *v))
            .unwrap();
        assert!((peak_idx as i32 - 64).abs() <= 2, "peak at {peak_idx}, expected ~64");
        assert!(peak_val > 0.5, "peak value {peak_val}");
    }

    #[test]
    fn dc_energy_roughly_preserved_on_downsample() {
        // DC signal: sum should scale linearly with output length.
        let src = vec![1.0_f32; 1024];
        let dst = resample(&src, 48000, 44100);
        let src_sum: f32 = src.iter().sum();
        let dst_sum: f32 = dst.iter().sum();
        let expected_ratio = 44100.0 / 48000.0;
        let actual_ratio = dst_sum / src_sum;
        assert!(
            (actual_ratio - expected_ratio).abs() < 0.02,
            "expected ratio ~{expected_ratio}, got {actual_ratio}"
        );
    }
}
