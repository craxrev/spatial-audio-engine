//! Third-order real spherical harmonics, ACN ordering, N3D
//! normalization. Closed-form per §5. The encoder takes a unit
//! vector in engine-native frame and produces 16 basis values.
//!
//! The §5 per-source wrapper (distance attenuation, listener
//! transform, `SH_W_NORM` scaling) lives in the per-source DSP path
//! (M3) — this module is just the basis itself.

// Constants are written at the spec's 8-digit precision; f32
// rounds them but the source stays verifiable against §5.
#![allow(clippy::excessive_precision)]

use crate::consts::NUM_AMBI;
use crate::math::Vec3;

// Degree-0
const C0: f32 = 0.282_094_79;
// Degree-1
const C1: f32 = 0.488_602_51;
// Degree-2: m = ±2, ±1 (excluding 0 and +2)
const C2A: f32 = 1.092_548_43;
// Degree-2: m = 0
const C2B: f32 = 0.315_391_57;
// Degree-2: m = +2
const C2C: f32 = 0.546_274_21;
// Degree-3: m = ±3
const C3A: f32 = 0.590_043_58;
// Degree-3: m = −2
const C3B: f32 = 2.890_611_44;
// Degree-3: m = ±1
const C3C: f32 = 0.457_045_8;
// Degree-3: m = 0
const C3D: f32 = 0.373_176_33;
// Degree-3: m = +2
const C3E: f32 = 1.445_305_72;

/// Evaluate the 16 ACN/N3D real SH basis values for a unit vector.
pub fn sh_basis_n3d(unit: Vec3) -> [f32; NUM_AMBI] {
    let mut out = [0.0; NUM_AMBI];
    sh_basis_n3d_into(unit, &mut out);
    out
}

/// In-place form to skip the array copy in hot paths.
pub fn sh_basis_n3d_into(unit: Vec3, out: &mut [f32; NUM_AMBI]) {
    let Vec3 { x, y, z } = unit;
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let yz = y * z;
    let xz = x * z;
    let xyz = xy * z;
    let five_zz_minus_1 = 5.0 * zz - 1.0;

    out[0] = C0;
    out[1] = C1 * y;
    out[2] = C1 * z;
    out[3] = C1 * x;
    out[4] = C2A * xy;
    out[5] = C2A * yz;
    out[6] = C2B * (3.0 * zz - 1.0);
    out[7] = C2A * xz;
    out[8] = C2C * (xx - yy);
    out[9] = C3A * (3.0 * xx - yy) * y;
    out[10] = C3B * xyz;
    out[11] = C3C * y * five_zz_minus_1;
    out[12] = C3D * z * (5.0 * zz - 3.0);
    out[13] = C3C * x * five_zz_minus_1;
    out[14] = C3E * (xx - yy) * z;
    out[15] = C3A * (xx - 3.0 * yy) * x;
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f32 = 1e-5;

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < TOL, "got {a}, want {b}");
    }

    #[test]
    fn dc_term_is_constant() {
        for unit in [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(-0.577_350_3, 0.577_350_3, 0.577_350_3),
        ] {
            let y = sh_basis_n3d(unit);
            approx(y[0], 0.282_094_79);
        }
    }

    #[test]
    fn basis_values_at_plus_x() {
        // +X = native forward.
        let y = sh_basis_n3d(Vec3::new(1.0, 0.0, 0.0));
        approx(y[0], 0.282_094_79);
        approx(y[1], 0.0);
        approx(y[2], 0.0);
        approx(y[3], 0.488_602_51);
        approx(y[4], 0.0);
        approx(y[5], 0.0);
        approx(y[6], -0.315_391_57);
        approx(y[7], 0.0);
        approx(y[8], 0.546_274_21);
        approx(y[9], 0.0);
        approx(y[10], 0.0);
        approx(y[11], 0.0);
        approx(y[12], 0.0);
        approx(y[13], -0.457_045_8);
        approx(y[14], 0.0);
        approx(y[15], 0.590_043_58);
    }

    #[test]
    fn basis_values_at_plus_y() {
        // +Y = native left.
        let y = sh_basis_n3d(Vec3::new(0.0, 1.0, 0.0));
        approx(y[0], 0.282_094_79);
        approx(y[1], 0.488_602_51);
        approx(y[2], 0.0);
        approx(y[3], 0.0);
        approx(y[4], 0.0);
        approx(y[5], 0.0);
        approx(y[6], -0.315_391_57);
        approx(y[7], 0.0);
        approx(y[8], -0.546_274_21);
        approx(y[9], -0.590_043_58);
        approx(y[10], 0.0);
        approx(y[11], -0.457_045_8);
        approx(y[12], 0.0);
        approx(y[13], 0.0);
        approx(y[14], 0.0);
        approx(y[15], 0.0);
    }

    #[test]
    fn basis_values_at_plus_z() {
        // +Z = native up.
        let y = sh_basis_n3d(Vec3::new(0.0, 0.0, 1.0));
        approx(y[0], 0.282_094_79);
        approx(y[1], 0.0);
        approx(y[2], 0.488_602_51);
        approx(y[3], 0.0);
        approx(y[4], 0.0);
        approx(y[5], 0.0);
        approx(y[6], 0.630_783_14);
        approx(y[7], 0.0);
        approx(y[8], 0.0);
        approx(y[9], 0.0);
        approx(y[10], 0.0);
        approx(y[11], 0.0);
        approx(y[12], 0.746_352_66);
        approx(y[13], 0.0);
        approx(y[14], 0.0);
        approx(y[15], 0.0);
    }

    #[test]
    fn sign_flip_under_axis_negation() {
        // Degree-1 components flip sign with their axis.
        let p = sh_basis_n3d(Vec3::new(1.0, 0.0, 0.0));
        let n = sh_basis_n3d(Vec3::new(-1.0, 0.0, 0.0));
        approx(p[3], -n[3]);
        // Degree-2 m=0 (zonal): symmetric under x-negation.
        approx(p[6], n[6]);
        // Degree-3 m=+3: odd in x.
        approx(p[15], -n[15]);
    }

    #[test]
    fn in_place_matches_returned() {
        let unit = Vec3::new(0.3, 0.4, 0.866_025_4).normalize();
        let a = sh_basis_n3d(unit);
        let mut b = [0.0; NUM_AMBI];
        sh_basis_n3d_into(unit, &mut b);
        for i in 0..NUM_AMBI {
            approx(a[i], b[i]);
        }
    }
}
