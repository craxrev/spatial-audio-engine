//! Schroeder-Jot FDN reverb (§7).
//!
//! Signal flow:
//! ```text
//! input → 1-pole LP → 10 Schroeder allpass diffusers → 8-line FDN
//!                                                          ↓
//!                                              8 reverb outputs
//! ```
//!
//! The FDN core: each block sample, read all 8 delay-line taps, mix
//! them through an 8×8 Hadamard matrix + 1/√8 normalisation, scale
//! by per-line RT60 decay, run through a low+high shelf pair for
//! frequency-dependent damping, and write `(diffused_input + tap)`
//! back into the delay lines. Outputs are the post-decay-post-shelf
//! taps and get SH-encoded by the engine at the 8 fixed §13 directions.

use crate::biquad::Biquad;
use crate::consts::{
    BLOCK_SIZE, DEFAULT_DIFFUSION_COEF, DEFAULT_REVERB_HIGH_HZ, DEFAULT_REVERB_LOW_HZ,
    DEFAULT_RT60_DB, DIFFUSER_DELAY_SECONDS, FDN_DELAY_SECONDS, FDN_SIZE, NUM_DIFFUSERS,
    PLUGIN_REVERB_HIGH_DB, PLUGIN_REVERB_LOW_DB, REV_SEND_GAINS,
};
use crate::diffuser::Diffuser;

const HADAMARD_NORM: f32 = 0.353_553_4; // 1 / sqrt(8)
const INPUT_LP_MIN_HZ: f32 = 1400.0;
const INPUT_LP_MAX_FRAC: f32 = 0.425;
// Q for the shelf damping pair. Spec doesn't pin it down; 1/√2 gives a
// standard 6 dB/oct shelf which produces musical RT60-frequency
// behaviour without ringing.
const SHELF_Q: f32 = 0.707_106_77;

struct DelayLine {
    ring: Vec<f32>,
    head: usize,
    delay: usize,
    mask: usize,
}

impl DelayLine {
    fn new(delay_samples: usize) -> Self {
        let n = delay_samples.max(2).next_power_of_two();
        Self { ring: vec![0.0; n], head: 0, delay: delay_samples, mask: n - 1 }
    }

    #[inline]
    fn read(&self) -> f32 {
        let idx = self.head.wrapping_sub(self.delay) & self.mask;
        self.ring[idx]
    }

    #[inline]
    fn write_and_advance(&mut self, x: f32) {
        self.ring[self.head] = x;
        self.head = (self.head + 1) & self.mask;
    }

    fn reset(&mut self) {
        for v in &mut self.ring {
            *v = 0.0;
        }
        self.head = 0;
    }
}

struct FdnLine {
    delay: DelayLine,
    decay_gain: f32,
    shelf_low: Biquad,
    shelf_high: Biquad,
}

pub struct ReverbCore {
    diffusers: Vec<Diffuser>,
    fdn: Vec<FdnLine>,
    input_lp_b0: f32,
    input_lp_a1: f32,
    input_lp_state: f32,
}

impl ReverbCore {
    pub fn new(sample_rate: u32) -> Self {
        let fs = sample_rate as f32;

        // Input LP: fc = min(0.425·fs, 1400 Hz). Bilinear 1-pole.
        let fc = (INPUT_LP_MAX_FRAC * fs).min(INPUT_LP_MIN_HZ);
        let w = (core::f32::consts::PI * fc / fs).tan();
        let inv = 1.0 / (1.0 + w);
        let input_lp_b0 = w * inv;
        let input_lp_a1 = (1.0 - w) * inv;

        // 10 Schroeder allpass diffusers.
        let mut diffusers: Vec<Diffuser> = Vec::with_capacity(NUM_DIFFUSERS);
        for &sec in DIFFUSER_DELAY_SECONDS.iter() {
            let n = (sec * fs).round() as usize;
            diffusers.push(Diffuser::new(n.max(1), DEFAULT_DIFFUSION_COEF));
        }

        // 8 FDN lines: delay + per-line decay + shelf pair.
        let mut fdn: Vec<FdnLine> = Vec::with_capacity(FDN_SIZE);
        for &sec in FDN_DELAY_SECONDS.iter() {
            let n = (sec * fs).round() as usize;
            let decay_gain = compute_decay_gain(DEFAULT_RT60_DB, sec);
            fdn.push(FdnLine {
                delay: DelayLine::new(n.max(1)),
                decay_gain,
                shelf_low: Biquad::low_shelf(fs, DEFAULT_REVERB_LOW_HZ, PLUGIN_REVERB_LOW_DB, SHELF_Q),
                shelf_high: Biquad::high_shelf(fs, DEFAULT_REVERB_HIGH_HZ, PLUGIN_REVERB_HIGH_DB, SHELF_Q),
            });
        }

        Self {
            diffusers,
            fdn,
            input_lp_b0,
            input_lp_a1,
            input_lp_state: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.input_lp_state = 0.0;
        for d in &mut self.diffusers {
            d.reset();
        }
        for line in &mut self.fdn {
            line.delay.reset();
            line.shelf_low.reset();
            line.shelf_high.reset();
        }
    }

    /// Process one block: mono input → 8 spatial outputs.
    pub fn process(
        &mut self,
        input: &[f32; BLOCK_SIZE],
        outputs: &mut [[f32; BLOCK_SIZE]; FDN_SIZE],
    ) {
        let mut d_vals = [0.0_f32; FDN_SIZE];

        for i in 0..BLOCK_SIZE {
            // 1-pole input LP.
            let x = input[i];
            let y_lp = self.input_lp_b0 * x + self.input_lp_state;
            self.input_lp_state = y_lp * self.input_lp_a1 + self.input_lp_b0 * x;
            if self.input_lp_state.abs() < 1.175e-38 {
                self.input_lp_state = 0.0;
            }

            // Diffuser cascade.
            let mut s = y_lp;
            for d in &mut self.diffusers {
                s = d.process(s);
            }

            // FDN: read all taps, Hadamard, normalise, decay+shelf, writeback.
            for (j, line) in self.fdn.iter().enumerate() {
                d_vals[j] = line.delay.read();
            }
            hadamard8(&mut d_vals);
            for v in d_vals.iter_mut() {
                *v *= HADAMARD_NORM;
            }
            for (j, line) in self.fdn.iter_mut().enumerate() {
                d_vals[j] *= line.decay_gain;
                d_vals[j] = line.shelf_low.process(d_vals[j]);
                d_vals[j] = line.shelf_high.process(d_vals[j]);
                // §13: per-line input weight shapes the per-direction
                // reverb energy. Feedback (d_vals) is added unweighted.
                line.delay.write_and_advance(s * REV_SEND_GAINS[j] + d_vals[j]);
            }

            for j in 0..FDN_SIZE {
                outputs[j][i] = d_vals[j];
            }
        }
    }
}

/// `decay_gain = 10^(RT60_dB · delay_seconds · 0.05)` per §7.4. Clamp
/// the dB sum to ≤ −80 dB (i.e. clamp gain to ≥ 0 in practice).
fn compute_decay_gain(rt60_db: f32, delay_seconds: f32) -> f32 {
    let db = rt60_db * delay_seconds;
    if db < -80.0 {
        0.0
    } else {
        10.0_f32.powf(db * 0.05)
    }
}

/// In-place 8-point Hadamard (Walsh-Hadamard) transform via 3 butterfly stages.
/// Caller is responsible for the 1/√8 unitary normalisation afterward.
fn hadamard8(d: &mut [f32; 8]) {
    // stage 0 (stride 1), stage 1 (stride 2), stage 2 (stride 4).
    for stage in 0..3 {
        let stride: usize = 1 << stage;
        let step = stride * 2;
        let mut i = 0;
        while i < 8 {
            for k in 0..stride {
                let a = d[i + k];
                let b = d[i + k + stride];
                d[i + k] = a + b;
                d[i + k + stride] = a - b;
            }
            i += step;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hadamard_is_unitary() {
        // For an input with a single non-zero entry, the output should
        // have ±1 in every slot (before normalisation) so that
        // |H·x|² = 8·|x|². Apply normalisation and energy is preserved.
        let mut d = [0.0_f32; 8];
        d[3] = 1.0;
        hadamard8(&mut d);
        for v in d.iter() {
            assert!(v.abs() == 1.0, "expected ±1, got {v}");
        }
    }

    #[test]
    fn decay_gain_matches_rt60_formula() {
        // RT60_dB = -60 dB after 1s reference: gain after 1s of decay
        // should be 10^(-60/20) = 0.001.
        let g = compute_decay_gain(-60.0, 1.0);
        assert!((g - 0.001).abs() < 1e-6, "{g}");
    }

    #[test]
    fn impulse_decays_over_time() {
        let mut r = ReverbCore::new(48000);
        let mut input = [0.0_f32; BLOCK_SIZE];
        input[0] = 1.0;
        let mut outputs = [[0.0_f32; BLOCK_SIZE]; FDN_SIZE];

        r.process(&input, &mut outputs);
        input.fill(0.0);

        // Settle: let the impulse propagate through the FDN (longest
        // delay is ~12k samples ≈ 96 blocks at 48k).
        for _ in 0..100 {
            r.process(&input, &mut outputs);
        }

        // Equal-duration windows so the comparison is fair.
        fn block_energy(outputs: &[[f32; BLOCK_SIZE]; FDN_SIZE]) -> f32 {
            outputs.iter()
                .map(|ch| ch.iter().map(|v| v * v).sum::<f32>())
                .sum()
        }

        let mut e_early = 0.0_f32;
        for _ in 0..100 {
            r.process(&input, &mut outputs);
            e_early += block_energy(&outputs);
        }
        assert!(e_early > 0.0, "no reverb energy after settling");

        // Skip ahead and re-measure.
        for _ in 0..500 {
            r.process(&input, &mut outputs);
        }
        let mut e_late = 0.0_f32;
        for _ in 0..100 {
            r.process(&input, &mut outputs);
            e_late += block_energy(&outputs);
        }
        assert!(e_late < e_early, "reverb didn't decay: early={e_early} late={e_late}");
    }

    #[test]
    fn silent_input_yields_silent_output_at_rest() {
        let mut r = ReverbCore::new(48000);
        let input = [0.0_f32; BLOCK_SIZE];
        let mut outputs = [[0.0_f32; BLOCK_SIZE]; FDN_SIZE];
        r.process(&input, &mut outputs);
        for ch in outputs.iter() {
            for &v in ch.iter() {
                assert_eq!(v, 0.0);
            }
        }
    }
}
