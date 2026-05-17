//! Schroeder-Jot FDN reverb (§7).
//!
//! Signal flow:
//! ```text
//! input → 1-pole LP → 8 Schroeder allpass diffusers (1 per line)
//!                  → 8-line FDN → 8 reverb outputs
//! ```
//!
//! Per-line parameters are derived from the bit-verified seed tables
//! (`SCHROEDER_DELAY_SECONDS`, `FDN_DELAY_SECONDS`) scaled by
//! `DIFFUSION_COEFFICIENT = 0.69`. The per-line `shelf_decay_s`
//! budget (= `0.69 · (FDN_seed[i] − Schroeder_seed[i])`) drives both
//! the diffuser's allpass `g` (via Jot's `g = (1/√2)^(1/T)` formula)
//! and the shelf gains (low/high passband dB = `shelf_decay_s ×
//! shelf_dB`). Mid-band decay falls out of broadband diffusion + the
//! shelves' transition bands.

use crate::biquad::Biquad;
use crate::consts::{
    BLOCK_SIZE, DEFAULT_REVERB_HIGH_DB, DEFAULT_REVERB_HIGH_HZ, DEFAULT_REVERB_LOW_DB,
    DEFAULT_REVERB_LOW_HZ, DIFFUSION_COEFFICIENT, FDN_DELAY_SECONDS, FDN_SIZE, NUM_DIFFUSERS,
    REV_SEND_GAINS, SCHROEDER_DELAY_SECONDS,
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

        // 8 Schroeder allpasses paired 1:1 with 8 FDN lines.
        let mut diffusers: Vec<Diffuser> = Vec::with_capacity(NUM_DIFFUSERS);
        let mut fdn: Vec<FdnLine> = Vec::with_capacity(FDN_SIZE);
        for i in 0..FDN_SIZE {
            let schroeder_seed = SCHROEDER_DELAY_SECONDS[i];
            let fdn_seed       = FDN_DELAY_SECONDS[i];

            let diff_samples = (DIFFUSION_COEFFICIENT * schroeder_seed * fs).round() as usize;
            let fdn_samples  = (DIFFUSION_COEFFICIENT * fdn_seed       * fs).round() as usize;

            let shelf_decay_s = DIFFUSION_COEFFICIENT * (fdn_seed - schroeder_seed);
            let g_jot = (core::f32::consts::FRAC_1_SQRT_2).powf(1.0 / shelf_decay_s);

            let low_db_eff  = shelf_decay_s * DEFAULT_REVERB_LOW_DB;
            let high_db_eff = shelf_decay_s * DEFAULT_REVERB_HIGH_DB;

            diffusers.push(Diffuser::new(diff_samples.max(1), g_jot));
            fdn.push(FdnLine {
                delay: DelayLine::new(fdn_samples.max(1)),
                shelf_low:  Biquad::low_shelf (fs, DEFAULT_REVERB_LOW_HZ,  low_db_eff,  SHELF_Q),
                shelf_high: Biquad::high_shelf(fs, DEFAULT_REVERB_HIGH_HZ, high_db_eff, SHELF_Q),
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
    fn impulse_decays_over_time() {
        let mut r = ReverbCore::new(48000);
        let mut input = [0.0_f32; BLOCK_SIZE];
        input[0] = 1.0;
        let mut outputs = [[0.0_f32; BLOCK_SIZE]; FDN_SIZE];

        r.process(&input, &mut outputs);
        input.fill(0.0);

        // Settle: longest delay is ceil(0.69 * 0.2569 * 48k) ≈ 8508
        // samples ≈ 67 blocks at 48 kHz; double it for safety.
        for _ in 0..140 {
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
