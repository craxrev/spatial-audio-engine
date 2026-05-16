//! Schroeder allpass diffuser stage (§7.2). One delay line per stage.
//!
//! ```text
//! y_delay   = ringbuf[read_idx]
//! ringbuf[read_idx] = x + g · y_delay
//! out       = y_delay − g · (x + g · y_delay)
//! read_idx  = (read_idx + 1) mod ringbuf_size
//! ```

pub struct Diffuser {
    ring: Vec<f32>,
    read_idx: usize,
    pub g: f32,
}

impl Diffuser {
    /// Allocate a diffuser with the next-power-of-2 ring buffer ≥
    /// `delay_samples`. The active delay tap is at `read_idx + delay`
    /// modulo the ring length. Stores `delay` implicitly by aligning
    /// `read_idx` with the write head (write_idx == read_idx, then
    /// advance read).
    pub fn new(delay_samples: usize, g: f32) -> Self {
        let n = delay_samples.next_power_of_two().max(2);
        Self { ring: vec![0.0; n], read_idx: 0, g }
    }

    pub fn reset(&mut self) {
        for v in &mut self.ring {
            *v = 0.0;
        }
        self.read_idx = 0;
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let y_delay = self.ring[self.read_idx];
        let store = x + self.g * y_delay;
        self.ring[self.read_idx] = store;
        let out = y_delay - self.g * store;
        self.read_idx += 1;
        if self.read_idx == self.ring.len() {
            self.read_idx = 0;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn allpass_preserves_long_term_energy() {
        // For an allpass filter, output energy ≈ input energy over a
        // sufficiently long stationary signal (transient excluded).
        let mut d = Diffuser::new(64, 0.69);
        let mut ein = 0.0_f32;
        let mut eout = 0.0_f32;
        // Run a deterministic noise-ish signal for a few thousand samples.
        let mut x = 0.123_f32;
        for i in 0..8192 {
            x = (x * 1.000_321 + 0.000_456).fract();
            let s = x * 2.0 - 1.0;
            let y = d.process(s);
            // Skip the first ring-buffer fill for steady-state.
            if i >= 256 {
                ein += s * s;
                eout += y * y;
            }
        }
        let ratio = eout / ein;
        assert!(
            (ratio - 1.0).abs() < 0.05,
            "allpass energy ratio should be ~1, got {ratio}"
        );
    }
}
