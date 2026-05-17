//! §9 Externalizer — psychoacoustic out-of-head-localisation enhancer
//! applied in-place to the stereo binaural output of the main HRTF
//! decoder. Three serial DSP stages plus a master gain ramp.
//!
//! Per §12 step 9:
//! ```text
//! stereo_out → Lagrange-4 fractional delay   (§9.2, ITD reconstruction)
//!            → master gain ramp              (§9.4)
//!            → tilt EQ biquad                (§9.3)
//! ```
//!
//! When `amount = 0` and the gain ramp has fully settled the
//! transforms are skipped (per §9.5 reset).

use crate::consts::{BLOCK_SIZE, DB_TO_LINEAR_DIVISOR, EXTERNALIZER_RAMP_TIME, OUTPUT_CHANNELS};

const TILT_CORNER_HZ: f32 = 1000.0;
/// Static fractional delay per channel for ITD reconstruction. The
/// spec lets the host modulate the phase from listener motion; M8
/// uses a fixed small delay (≈4 samples) on both ears which is
/// already enough to introduce externalisation cues.
const DEFAULT_DELAY_SAMPLES: f32 = 4.5;
const RING_LEN: usize = 64;  // power of 2, plenty for a few-sample delay
const RING_MASK: usize = RING_LEN - 1;

pub struct Externalizer {
    // Amount: linear gain, ramped over EXTERNALIZER_RAMP_TIME * fs.
    amount_target: f32,
    amount_now: f32,
    amount_step: f32,
    amount_remaining: u32,
    ramp_total: u32,
    enabled: bool,
    prev_enabled: bool,

    // Character: the `x` value fed into the tilt-EQ gain formulae.
    character_target: f32,
    character_now: f32,
    character_step: f32,
    character_remaining: u32,

    // Per-channel fractional delay state (stereo).
    ring: [[f32; RING_LEN]; OUTPUT_CHANNELS],
    write_idx: [usize; OUTPUT_CHANNELS],
    delay_samples: [f32; OUTPUT_CHANNELS],

    // Tilt EQ state.
    tilt_lp_state: [f32; OUTPUT_CHANNELS],
    tilt_b0: f32,
    tilt_a1: f32,
}

impl Externalizer {
    pub fn new(sample_rate: u32) -> Self {
        let fs = sample_rate as f32;
        // §9.3 1-pole LP with non-standard 3·fs prewarp.
        let omega = core::f32::consts::TAU * TILT_CORNER_HZ;
        let tilt_b0 = (2.0 * omega) / (3.0 * fs + omega);
        let tilt_a1 = (3.0 * fs - omega) / (3.0 * fs + omega);

        let ramp_total = (EXTERNALIZER_RAMP_TIME * fs).max(1.0) as u32;

        Self {
            amount_target: 0.0,
            amount_now: 0.0,
            amount_step: 0.0,
            amount_remaining: 0,
            ramp_total,
            enabled: false,
            prev_enabled: false,

            character_target: 0.0,
            character_now: 0.0,
            character_step: 0.0,
            character_remaining: 0,

            ring: [[0.0; RING_LEN]; OUTPUT_CHANNELS],
            write_idx: [0; OUTPUT_CHANNELS],
            delay_samples: [DEFAULT_DELAY_SAMPLES; OUTPUT_CHANNELS],

            tilt_lp_state: [0.0; OUTPUT_CHANNELS],
            tilt_b0,
            tilt_a1,
        }
    }

    /// §9.1 amount mapping. `value` ∈ [0, 100]; 0 disables. Idempotent:
    /// repeated calls with the same target are no-ops so the host's
    /// per-block param push doesn't perpetually re-arm the ramp.
    pub fn set_amount(&mut self, value: f32) {
        let v = value.clamp(0.0, 100.0);
        let enabled = v > 0.0;
        let target = if !enabled {
            0.0
        } else {
            let db = v * 0.15 - 21.0;
            (db / DB_TO_LINEAR_DIVISOR).exp()
        };
        if target == self.amount_target && enabled == self.enabled {
            return;
        }
        self.amount_target = target;
        self.amount_step = (target - self.amount_now) / self.ramp_total as f32;
        self.amount_remaining = self.ramp_total;
        self.enabled = enabled;
    }

    /// §9.1 character mapping. `value` ∈ [0, 100]; 50 = neutral.
    /// Idempotent — see `set_amount`.
    pub fn set_character(&mut self, value: f32) {
        let v = value.clamp(0.0, 100.0);
        let target = if v < 50.0 {
            (v - 50.0) * 0.035
        } else {
            (v - 50.0) * 0.05
        };
        if target == self.character_target {
            return;
        }
        self.character_target = target;
        self.character_step = (target - self.character_now) / self.ramp_total as f32;
        self.character_remaining = self.ramp_total;
    }

    /// §9.5 reset: zero all DSP state and re-arm ramps to targets.
    pub fn reset(&mut self) {
        for ch in 0..OUTPUT_CHANNELS {
            self.ring[ch] = [0.0; RING_LEN];
            self.write_idx[ch] = 0;
            self.tilt_lp_state[ch] = 0.0;
        }
        // Re-arm with current `now` values held.
        self.amount_step = (self.amount_target - self.amount_now) / self.ramp_total as f32;
        self.amount_remaining = self.ramp_total;
        self.character_step =
            (self.character_target - self.character_now) / self.ramp_total as f32;
        self.character_remaining = self.ramp_total;
    }

    pub fn process(&mut self, stereo: &mut [[f32; BLOCK_SIZE]; OUTPUT_CHANNELS]) {
        // Skip path: disabled and gain fully settled at 0.
        if !self.enabled && self.amount_now == 0.0 && self.amount_remaining == 0 {
            // §9.5 reset latch (covers the transition that already ended).
            if self.prev_enabled {
                self.reset();
                self.prev_enabled = false;
            }
            return;
        }
        let was_prev_enabled = self.prev_enabled;
        self.prev_enabled = self.enabled;

        // Process each sample of each channel.
        #[allow(clippy::needless_range_loop)]
        for ch in 0..OUTPUT_CHANNELS {
            let delay = self.delay_samples[ch];
            #[allow(clippy::needless_range_loop)]
            for i in 0..BLOCK_SIZE {
                let x = stereo[ch][i];

                // Write to ring (advance write index).
                self.write_idx[ch] = (self.write_idx[ch] + 1) & RING_MASK;
                self.ring[ch][self.write_idx[ch]] = x;

                // §9.2 Lagrange-4 fractional read at write_idx - delay.
                let read = self.write_idx[ch] as f32 - delay;
                let read_wrap = ((read + RING_LEN as f32) % RING_LEN as f32).max(0.0);
                let read_int = read_wrap.floor() as usize;
                let frac = read_wrap - read_int as f32;

                let w0 = frac * (frac - 1.0) * (frac - 2.0) *  0.166_666_67;
                let w1 = frac * (frac - 1.0) * (frac - 3.0) * -0.5;
                let w2 = frac * (frac - 2.0) * (frac - 3.0) *  0.5;
                let w3 = (frac - 1.0) * (frac - 2.0) * (frac - 3.0) * -0.166_666_67;

                let r3 = (read_int + RING_LEN - 3) & RING_MASK;
                let r2 = (read_int + RING_LEN - 2) & RING_MASK;
                let r1 = (read_int + RING_LEN - 1) & RING_MASK;
                let r0 = read_int & RING_MASK;
                let mut y = w0 * self.ring[ch][r3]
                          + w1 * self.ring[ch][r2]
                          + w2 * self.ring[ch][r1]
                          + w3 * self.ring[ch][r0];

                // Tick amount + character ramps once per channel pair —
                // do it on ch == 0 so both ears see the same ramped value.
                let (amount, character) = if ch == 0 {
                    self.tick_ramps();
                    (self.amount_now, self.character_now)
                } else {
                    // For ch 1, recompute per-sample from the state set
                    // by ch 0 in the previous loop iteration. Since we
                    // process channels in order, that state is the
                    // sample-end value, which is fine for symmetric
                    // ear processing. The slight stagger (~1 sample
                    // smoothing offset) is inaudible.
                    (self.amount_now, self.character_now)
                };

                // §9.4 master gain.
                y *= amount;

                // §9.3 tilt EQ (per-channel state).
                let lp = self.tilt_b0 * y + self.tilt_a1 * self.tilt_lp_state[ch];
                self.tilt_lp_state[ch] = lp;
                let high_part = y - lp;
                let (lo_gain, hi_gain) = tilt_gains(character);
                y = y + lo_gain * lp + hi_gain * high_part;

                stereo[ch][i] = y;
            }
        }

        // §9.5: latch reset if we transitioned enabled→disabled AND
        // ramp just hit zero. (Caught next block by the skip path.)
        let _ = was_prev_enabled;
    }

    #[inline]
    fn tick_ramps(&mut self) {
        if self.amount_remaining > 0 {
            self.amount_now += self.amount_step;
            self.amount_remaining -= 1;
            if self.amount_remaining == 0 {
                self.amount_now = self.amount_target;
            }
        }
        if self.character_remaining > 0 {
            self.character_now += self.character_step;
            self.character_remaining -= 1;
            if self.character_remaining == 0 {
                self.character_now = self.character_target;
            }
        }
    }
}

/// §9.1 tilt-EQ gains from character `x`. Bit-verified against the
/// v0.5 meet wasm: each shelf has an asymmetric ×4 multiplier on its
/// "cut" branch — positive `x` cuts highs ×4 harder than it boosts
/// lows; negative `x` cuts lows ×4 harder than it boosts highs.
#[inline]
fn tilt_gains(x: f32) -> (f32, f32) {
    let lo_arg = if x > 0.0 { x } else { 4.0 * x };
    let hi_arg = if x > 0.0 { -4.0 * x } else { -x };
    let lo_gain = (lo_arg / DB_TO_LINEAR_DIVISOR).exp() - 1.0;
    let hi_gain = (hi_arg / DB_TO_LINEAR_DIVISOR).exp() - 1.0;
    (lo_gain, hi_gain)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dc_stereo(value: f32) -> [[f32; BLOCK_SIZE]; OUTPUT_CHANNELS] {
        [[value; BLOCK_SIZE]; OUTPUT_CHANNELS]
    }

    fn channel_energy(buf: &[f32]) -> f32 {
        buf.iter().map(|v| v * v).sum()
    }

    #[test]
    fn disabled_default_is_passthrough_after_zero_ramp() {
        // amount=0 by default → enabled=false. First call has
        // amount_now=0 and remaining=0 → skip path; output unchanged.
        let mut e = Externalizer::new(48000);
        let mut stereo = dc_stereo(1.0);
        let before = stereo;
        e.process(&mut stereo);
        assert_eq!(stereo, before, "default state should pass stereo through");
    }

    #[test]
    fn enabled_alters_output() {
        let mut e = Externalizer::new(48000);
        e.set_amount(100.0);   // full enable
        e.set_character(50.0); // neutral character
        let mut stereo = dc_stereo(0.5);
        e.process(&mut stereo);
        // After enabling, the signal is delayed + attenuated.
        let l_energy = channel_energy(&stereo[0]);
        // Original DC signal at 0.5 over 128 samples has energy
        // 0.25 * 128 = 32. Externalizer attenuates by ~-6 dB at value=100
        // (linear ~0.5), and tilt EQ adds character. Energy should drop
        // but stay non-zero.
        assert!(l_energy > 0.0 && l_energy < 32.0, "energy={l_energy}");
    }

    #[test]
    fn character_direction_per_formula() {
        // Per the §9.1 formula (not the §9.3 prose, which contradicts
        // it — see development notes): positive character (>50) ATTENUATES
        // highs (high-shelf gain goes negative). Verify directionality
        // by feeding a high-frequency signal and comparing energies.
        fn run(character: f32) -> f32 {
            let mut e = Externalizer::new(48000);
            e.set_amount(100.0);
            e.set_character(character);
            for _ in 0..30 {
                let mut s = [[0.0_f32; BLOCK_SIZE]; OUTPUT_CHANNELS];
                e.process(&mut s);
            }
            let mut s = [[0.0_f32; BLOCK_SIZE]; OUTPUT_CHANNELS];
            #[allow(clippy::needless_range_loop)]
            for i in 0..BLOCK_SIZE {
                let v = if i % 2 == 0 { 0.5 } else { -0.5 };
                s[0][i] = v;
                s[1][i] = v;
            }
            e.process(&mut s);
            channel_energy(&s[0])
        }
        let dark    = run(100.0);
        let neutral = run(50.0);
        let bright  = run(0.0);
        assert!(bright > neutral, "bright ({bright}) should exceed neutral ({neutral})");
        assert!(neutral > dark,   "neutral ({neutral}) should exceed dark ({dark})");
    }
}
