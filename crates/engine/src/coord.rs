//! Coordinate-system conversion at the public API boundary (§6.10).
//! engine-native frame is +X forward, +Y left, +Z up, right-handed.
//! All engine-internal state is stored in engine-native form.

use crate::math::{Quat, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoordSystem {
    Native,
    OpenGL,
    Unity,
}

pub fn position_to_native(sys: CoordSystem, p: Vec3) -> Vec3 {
    match sys {
        CoordSystem::Native => p,
        CoordSystem::OpenGL => Vec3::new(-p.z, -p.x, p.y),
        CoordSystem::Unity => Vec3::new(p.z, -p.x, p.y),
    }
}

pub fn quaternion_to_native(sys: CoordSystem, q: Quat) -> Quat {
    match sys {
        CoordSystem::Native => q,
        CoordSystem::OpenGL => Quat::new(q.w, -q.z, -q.x, q.y),
        CoordSystem::Unity => Quat::new(-q.w, q.z, -q.x, q.y),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_is_identity() {
        let p = Vec3::new(1.0, 2.0, 3.0);
        let q = Quat::new(0.5, 0.5, 0.5, 0.5);
        assert_eq!(position_to_native(CoordSystem::Native, p), p);
        assert_eq!(quaternion_to_native(CoordSystem::Native, q), q);
    }

    #[test]
    fn opengl_axes_map_to_native() {
        // openGL: +X right, +Y up, −Z forward (RH)
        // native: +X forward, +Y left, +Z up (RH)
        assert_eq!(
            position_to_native(CoordSystem::OpenGL, Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.0, -1.0, 0.0),
            "openGL right → native −Y (right)"
        );
        assert_eq!(
            position_to_native(CoordSystem::OpenGL, Vec3::new(0.0, 1.0, 0.0)),
            Vec3::new(0.0, 0.0, 1.0),
            "openGL up → native +Z"
        );
        assert_eq!(
            position_to_native(CoordSystem::OpenGL, Vec3::new(0.0, 0.0, -1.0)),
            Vec3::new(1.0, 0.0, 0.0),
            "openGL forward (−Z) → native +X"
        );
    }

    #[test]
    fn unity_axes_map_to_native() {
        // unity: +X right, +Y up, +Z forward (LH)
        assert_eq!(
            position_to_native(CoordSystem::Unity, Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.0, -1.0, 0.0),
            "unity right → native −Y (right)"
        );
        assert_eq!(
            position_to_native(CoordSystem::Unity, Vec3::new(0.0, 1.0, 0.0)),
            Vec3::new(0.0, 0.0, 1.0),
            "unity up → native +Z"
        );
        assert_eq!(
            position_to_native(CoordSystem::Unity, Vec3::new(0.0, 0.0, 1.0)),
            Vec3::new(1.0, 0.0, 0.0),
            "unity forward (+Z) → native +X"
        );
    }

    #[test]
    fn opengl_quaternion_permutes_components() {
        let q = Quat::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            quaternion_to_native(CoordSystem::OpenGL, q),
            Quat::new(1.0, -4.0, -2.0, 3.0)
        );
    }

    #[test]
    fn unity_quaternion_flips_w_and_permutes() {
        let q = Quat::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            quaternion_to_native(CoordSystem::Unity, q),
            Quat::new(-1.0, 4.0, -2.0, 3.0)
        );
    }
}
