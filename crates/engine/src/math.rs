//! `Vec3` and `Quat` primitives. engine-native frame is +X forward,
//! +Y left, +Z up, right-handed. Quaternions are `(w, x, y, z)` with
//! `w` scalar.

use core::ops::{Add, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn normalize(self) -> Self {
        let l = self.length();
        if l > 0.0 {
            Self::new(self.x / l, self.y / l, self.z / l)
        } else {
            Self::ZERO
        }
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub const IDENTITY: Self = Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    pub fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Active rotation: returns `q · v · q*`. To apply the passive
    /// `q⁻¹·v·q` from §6.1 (world → listener frame), call
    /// `q.conjugate().rotate(v)` on a unit quaternion.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let xyz = Vec3::new(self.x, self.y, self.z);
        let d = xyz.dot(v);
        let c = xyz.cross(v);
        let s = self.w * self.w - xyz.dot(xyz);
        Vec3::new(
            s * v.x + 2.0 * d * xyz.x + 2.0 * self.w * c.x,
            s * v.y + 2.0 * d * xyz.y + 2.0 * self.w * c.y,
            s * v.z + 2.0 * d * xyz.z + 2.0 * self.w * c.z,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_4;

    fn approx_vec(a: Vec3, b: Vec3, tol: f32) -> bool {
        (a.x - b.x).abs() < tol && (a.y - b.y).abs() < tol && (a.z - b.z).abs() < tol
    }

    #[test]
    fn vec3_dot_cross_canonical() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(x.dot(y), 0.0);
        assert_eq!(x.cross(y), Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(y.cross(x), Vec3::new(0.0, 0.0, -1.0));
    }

    #[test]
    fn vec3_length_and_normalize() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        assert_eq!(v.length(), 5.0);
        let n = v.normalize();
        assert!((n.length() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn vec3_normalize_zero() {
        assert_eq!(Vec3::ZERO.normalize(), Vec3::ZERO);
    }

    #[test]
    fn quat_identity_is_noop() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(Quat::IDENTITY.rotate(v), v);
    }

    #[test]
    fn quat_90_about_z_maps_x_to_y() {
        // Active rotation by +90° about Z: (1,0,0) → (0,1,0).
        let q = Quat::new(FRAC_PI_4.cos(), 0.0, 0.0, FRAC_PI_4.sin());
        let r = q.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!(approx_vec(r, Vec3::new(0.0, 1.0, 0.0), 1e-6));
    }

    #[test]
    fn quat_90_about_x_maps_y_to_z() {
        let q = Quat::new(FRAC_PI_4.cos(), FRAC_PI_4.sin(), 0.0, 0.0);
        let r = q.rotate(Vec3::new(0.0, 1.0, 0.0));
        assert!(approx_vec(r, Vec3::new(0.0, 0.0, 1.0), 1e-6));
    }

    #[test]
    fn quat_conjugate_undoes_rotation() {
        let q = Quat::new(FRAC_PI_4.cos(), 0.1, 0.2, FRAC_PI_4.sin());
        // Normalize for safety.
        let n = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
        let q = Quat::new(q.w / n, q.x / n, q.y / n, q.z / n);
        let v = Vec3::new(1.0, 2.0, 3.0);
        let r = q.conjugate().rotate(q.rotate(v));
        assert!(approx_vec(r, v, 1e-5));
    }

    #[test]
    fn quat_passive_rotation_via_conjugate() {
        // Passive q⁻¹·v·q: rotating frame by +90° about Z is equivalent
        // to rotating the vector by −90°. So (1,0,0) → (0,−1,0).
        let q = Quat::new(FRAC_PI_4.cos(), 0.0, 0.0, FRAC_PI_4.sin());
        let r = q.conjugate().rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!(approx_vec(r, Vec3::new(0.0, -1.0, 0.0), 1e-6));
    }
}
