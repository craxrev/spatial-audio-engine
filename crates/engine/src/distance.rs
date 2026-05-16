//! 4-knot piecewise distance → gain function (§3). Knots A, B, C
//! at increasing distances with linear gains; D is the silence
//! anchor (gain implicitly 0).
//!
//! Gains are stored linear; callers using dB convert via
//! `10^(dB/20)` before constructing the model.

#[derive(Clone, Copy, Debug)]
pub struct DistanceModel {
    pub a_dist: f32,
    pub a_gain: f32,
    pub b_dist: f32,
    pub b_gain: f32,
    pub c_dist: f32,
    pub c_gain: f32,
    pub d_dist: f32,
}

impl Default for DistanceModel {
    /// Bit-verified spec default: (1 m, 0 dB), (12 m, −20 dB),
    /// (60 m, −60 dB), (100 m → silence).
    fn default() -> Self {
        Self {
            a_dist: 1.0,
            a_gain: 1.0,    // 0 dB
            b_dist: 12.0,
            b_gain: 0.1,    // -20 dB
            c_dist: 60.0,
            c_gain: 0.001,  // -60 dB
            d_dist: 100.0,
        }
    }
}

impl DistanceModel {
    pub fn gain_at(&self, r: f32) -> f32 {
        if r <= self.a_dist {
            self.a_gain
        } else if r <= self.b_dist {
            lerp(
                self.a_gain,
                self.b_gain,
                (r - self.a_dist) / (self.b_dist - self.a_dist),
            )
        } else if r <= self.c_dist {
            lerp(
                self.b_gain,
                self.c_gain,
                (r - self.b_dist) / (self.c_dist - self.b_dist),
            )
        } else if r <= self.d_dist {
            lerp(
                self.c_gain,
                0.0,
                (r - self.c_dist) / (self.d_dist - self.c_dist),
            )
        } else {
            0.0
        }
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + t * (b - a)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_model() -> DistanceModel {
        DistanceModel {
            a_dist: 1.0,
            a_gain: 1.0,
            b_dist: 10.0,
            b_gain: 0.1,
            c_dist: 100.0,
            c_gain: 0.01,
            d_dist: 200.0,
        }
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-6, "got {a}, want {b}");
    }

    #[test]
    fn flat_below_a_dist() {
        let m = sample_model();
        approx(m.gain_at(0.0), 1.0);
        approx(m.gain_at(0.5), 1.0);
        approx(m.gain_at(1.0), 1.0);
    }

    #[test]
    fn lerp_inside_each_segment() {
        let m = sample_model();
        approx(m.gain_at(5.5), 0.55); // mid of [1, 10]
        approx(m.gain_at(10.0), 0.1);
        approx(m.gain_at(55.0), 0.055); // mid of [10, 100]
        approx(m.gain_at(100.0), 0.01);
        approx(m.gain_at(150.0), 0.005); // mid of [100, 200]
    }

    #[test]
    fn zero_at_or_past_d() {
        let m = sample_model();
        approx(m.gain_at(200.0), 0.0);
        approx(m.gain_at(500.0), 0.0);
    }

    #[test]
    fn continuous_at_knots() {
        let m = sample_model();
        let eps = 1e-3_f32;
        for &k in &[m.a_dist, m.b_dist, m.c_dist, m.d_dist] {
            let lo = m.gain_at(k - eps);
            let hi = m.gain_at(k + eps);
            assert!(
                (lo - hi).abs() < 1e-3,
                "discontinuity at r={k}: lo={lo}, hi={hi}"
            );
        }
    }
}
