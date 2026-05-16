//! Generic biquad in Direct-Form-II-Transposed (numerator b0/b1/b2,
//! denominator a1/a2 with a0 normalised). Used by the FDN shelf
//! damping pair (§7.3) and, later, the externalizer tilt EQ (§9.3).
//!
//! Coefficient form: y[n] = b0·x[n] + s1 ; s1' = b1·x[n] − a1·y[n] + s2 ;
//! s2' = b2·x[n] − a2·y[n].

#[derive(Clone, Copy, Debug, Default)]
pub struct Biquad {
    pub b0: f32,
    pub b1: f32,
    pub b2: f32,
    pub a1: f32,
    pub a2: f32,
    pub s1: f32,
    pub s2: f32,
}

impl Biquad {
    pub const IDENTITY: Self =
        Self { b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0, s1: 0.0, s2: 0.0 };

    /// Audio-EQ-cookbook low-shelf biquad. `fc` = corner frequency,
    /// `gain_db` = gain at low-frequency asymptote (relative to 0 dB at
    /// high frequencies), `q` = shelf slope (1/√2 = mild, larger = steeper).
    pub fn low_shelf(fs: f32, fc: f32, gain_db: f32, q: f32) -> Self {
        let a = 10.0_f32.powf(gain_db / 40.0);
        let w0 = core::f32::consts::TAU * fc / fs;
        let cw = w0.cos();
        let sw = w0.sin();
        let alpha = sw / (2.0 * q);
        let sqrt_a = a.sqrt();
        let two_sqrt_a_alpha = 2.0 * sqrt_a * alpha;

        let b0 = a * ((a + 1.0) - (a - 1.0) * cw + two_sqrt_a_alpha);
        let b1 = 2.0 * a * ((a - 1.0) - (a + 1.0) * cw);
        let b2 = a * ((a + 1.0) - (a - 1.0) * cw - two_sqrt_a_alpha);
        let a0 = (a + 1.0) + (a - 1.0) * cw + two_sqrt_a_alpha;
        let a1 = -2.0 * ((a - 1.0) + (a + 1.0) * cw);
        let a2 = (a + 1.0) + (a - 1.0) * cw - two_sqrt_a_alpha;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            s1: 0.0,
            s2: 0.0,
        }
    }

    /// Audio-EQ-cookbook high-shelf biquad. `gain_db` = gain at
    /// high-frequency asymptote (relative to 0 dB at low frequencies).
    pub fn high_shelf(fs: f32, fc: f32, gain_db: f32, q: f32) -> Self {
        let a = 10.0_f32.powf(gain_db / 40.0);
        let w0 = core::f32::consts::TAU * fc / fs;
        let cw = w0.cos();
        let sw = w0.sin();
        let alpha = sw / (2.0 * q);
        let sqrt_a = a.sqrt();
        let two_sqrt_a_alpha = 2.0 * sqrt_a * alpha;

        let b0 = a * ((a + 1.0) + (a - 1.0) * cw + two_sqrt_a_alpha);
        let b1 = -2.0 * a * ((a - 1.0) + (a + 1.0) * cw);
        let b2 = a * ((a + 1.0) + (a - 1.0) * cw - two_sqrt_a_alpha);
        let a0 = (a + 1.0) - (a - 1.0) * cw + two_sqrt_a_alpha;
        let a1 = 2.0 * ((a - 1.0) - (a + 1.0) * cw);
        let a2 = (a + 1.0) - (a - 1.0) * cw - two_sqrt_a_alpha;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            s1: 0.0,
            s2: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.s1;
        self.s1 = self.b1 * x - self.a1 * y + self.s2;
        self.s2 = self.b2 * x - self.a2 * y;
        if self.s1.abs() < 1.175e-38 {
            self.s1 = 0.0;
        }
        if self.s2.abs() < 1.175e-38 {
            self.s2 = 0.0;
        }
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_passes_signal_through() {
        let mut b = Biquad::IDENTITY;
        for x in [-1.0_f32, 0.0, 0.5, 1.0, -0.3] {
            assert_eq!(b.process(x), x);
        }
    }

    #[test]
    fn low_shelf_with_zero_gain_is_near_identity() {
        let mut b = Biquad::low_shelf(48000.0, 1000.0, 0.0, 1.0);
        let mut peak_err = 0.0_f32;
        // White-ish signal: alternating impulse train.
        let mut x = 0.5_f32;
        for _ in 0..256 {
            let y = b.process(x);
            peak_err = peak_err.max((y - x).abs());
            x = -x;
        }
        assert!(peak_err < 1e-3, "peak_err={peak_err}");
    }

    #[test]
    fn high_shelf_with_zero_gain_is_near_identity() {
        let mut b = Biquad::high_shelf(48000.0, 4000.0, 0.0, 1.0);
        let mut peak_err = 0.0_f32;
        let mut x = 0.5_f32;
        for _ in 0..256 {
            let y = b.process(x);
            peak_err = peak_err.max((y - x).abs());
            x = -x;
        }
        assert!(peak_err < 1e-3, "peak_err={peak_err}");
    }
}
