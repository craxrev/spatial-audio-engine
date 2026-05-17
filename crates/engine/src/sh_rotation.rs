//! Real-SH (ACN/N3D) rotation matrices for orders 0..=3 derived from
//! a 3D rotation quaternion. Used by §12 step 6 ambisonic-bed branch
//! to keep a world-locked bed in the listener-relative frame when the
//! listener rotates.
//!
//! Implementation strategy: **sample-and-fit**. Pre-sample 24
//! Fibonacci-sphere directions and compute their SH basis (16
//! channels per direction). For a given rotation, compute the SH
//! basis at the rotated sample directions and solve
//! `M · Y(d_k) = Y(R · d_k)` in the least-squares sense for `M`.
//!
//! For sufficient (and well-distributed) samples this is numerically
//! stable up to order 3. The fit step happens once per
//! listener-rotation change, then `M` is reused per audio block.

use crate::consts::NUM_AMBI;
use crate::math::{Quat, Vec3};
use crate::sh::sh_basis_n3d_into;

const N_SAMPLES: usize = 24;

/// Pre-baked: the 24 sample directions, the N×16 SH-basis matrix at
/// those directions (`A`), and the 16×16 inverse "fit" matrix
/// `(AᵀA)⁻¹ Aᵀ`. With these, a new rotation matrix `M` is
/// `B · (AᵀA)⁻¹ Aᵀ` where `B[k][i] = Y_i(R · d_k)`.
struct Samples {
    dirs: [Vec3; N_SAMPLES],
    /// Pseudoinverse-style fit operator stored row-major:
    /// `fit[i * N_SAMPLES + k] = ((AᵀA)⁻¹ Aᵀ)[i, k]`.
    fit: [f32; NUM_AMBI * N_SAMPLES],
}

fn build_samples() -> Samples {
    let dirs = fibonacci_sphere();

    // A: N×16 matrix, row-major: A[k * 16 + i] = Y_i(d_k).
    let mut a = [0.0_f32; N_SAMPLES * NUM_AMBI];
    for (k, dir) in dirs.iter().enumerate() {
        let mut row = [0.0_f32; NUM_AMBI];
        sh_basis_n3d_into(*dir, &mut row);
        a[k * NUM_AMBI..(k + 1) * NUM_AMBI].copy_from_slice(&row);
    }

    // AᵀA: 16×16 matrix.
    let mut ata = [[0.0_f32; NUM_AMBI]; NUM_AMBI];
    #[allow(clippy::needless_range_loop)]
    for i in 0..NUM_AMBI {
        #[allow(clippy::needless_range_loop)]
        for j in 0..NUM_AMBI {
            let mut s = 0.0_f32;
            #[allow(clippy::needless_range_loop)]
            for k in 0..N_SAMPLES {
                s += a[k * NUM_AMBI + i] * a[k * NUM_AMBI + j];
            }
            ata[i][j] = s;
        }
    }
    let ata_inv = invert_16x16(&ata).expect("AᵀA invertible for Fibonacci samples");

    // fit = (AᵀA)⁻¹ Aᵀ — a 16×N matrix.
    let mut fit = [0.0_f32; NUM_AMBI * N_SAMPLES];
    #[allow(clippy::needless_range_loop)]
    for i in 0..NUM_AMBI {
        #[allow(clippy::needless_range_loop)]
        for k in 0..N_SAMPLES {
            let mut s = 0.0_f32;
            #[allow(clippy::needless_range_loop)]
            for j in 0..NUM_AMBI {
                s += ata_inv[i][j] * a[k * NUM_AMBI + j];
            }
            fit[i * N_SAMPLES + k] = s;
        }
    }

    let mut out_dirs = [Vec3::ZERO; N_SAMPLES];
    out_dirs.copy_from_slice(&dirs);
    Samples { dirs: out_dirs, fit }
}

fn fibonacci_sphere() -> Vec<Vec3> {
    use core::f32::consts::PI;
    let phi = (1.0_f32 + 5.0_f32.sqrt()) * 0.5;
    let n = N_SAMPLES as f32;
    (0..N_SAMPLES)
        .map(|i| {
            let y = 1.0 - (i as f32 + 0.5) * 2.0 / n;
            let r = (1.0 - y * y).sqrt();
            let theta = 2.0 * PI * (i as f32) / phi;
            Vec3::new(r * theta.cos(), r * theta.sin(), y)
        })
        .collect()
}

/// Gauss-Jordan inversion of a 16×16 matrix. Returns `None` if singular.
fn invert_16x16(m: &[[f32; NUM_AMBI]; NUM_AMBI]) -> Option<[[f32; NUM_AMBI]; NUM_AMBI]> {
    let n = NUM_AMBI;
    let mut aug = [[0.0_f32; 2 * NUM_AMBI]; NUM_AMBI];
    #[allow(clippy::needless_range_loop)]
    for i in 0..n {
        #[allow(clippy::needless_range_loop)]
        for j in 0..n {
            aug[i][j] = m[i][j];
        }
        aug[i][n + i] = 1.0;
    }
    // Forward elim with partial pivoting.
    #[allow(clippy::needless_range_loop)]
    for col in 0..n {
        let mut piv = col;
        for r in (col + 1)..n {
            if aug[r][col].abs() > aug[piv][col].abs() {
                piv = r;
            }
        }
        if aug[piv][col].abs() < 1e-12 {
            return None;
        }
        if piv != col {
            aug.swap(col, piv);
        }
        let inv = 1.0 / aug[col][col];
        #[allow(clippy::needless_range_loop)]
        for j in 0..(2 * n) {
            aug[col][j] *= inv;
        }
        #[allow(clippy::needless_range_loop)]
        for r in 0..n {
            if r == col {
                continue;
            }
            let factor = aug[r][col];
            if factor == 0.0 {
                continue;
            }
            #[allow(clippy::needless_range_loop)]
            for j in 0..(2 * n) {
                aug[r][j] -= factor * aug[col][j];
            }
        }
    }
    let mut out = [[0.0_f32; NUM_AMBI]; NUM_AMBI];
    #[allow(clippy::needless_range_loop)]
    for i in 0..n {
        #[allow(clippy::needless_range_loop)]
        for j in 0..n {
            out[i][j] = aug[i][n + j];
        }
    }
    Some(out)
}

thread_local! {
    static SAMPLES: Samples = build_samples();
}

/// Full 16×16 SH rotation matrix corresponding to `quat`.
#[derive(Clone, Debug)]
pub struct ShRotation {
    /// Row-major 16×16 matrix. Note this is block-diagonal in
    /// principle (orders don't mix); off-block entries are noise from
    /// the least-squares fit (≤ ~1e-5 in practice).
    pub m: [f32; NUM_AMBI * NUM_AMBI],
}

impl ShRotation {
    pub fn identity() -> Self {
        let mut m = [0.0_f32; NUM_AMBI * NUM_AMBI];
        #[allow(clippy::needless_range_loop)]
        for i in 0..NUM_AMBI {
            m[i * NUM_AMBI + i] = 1.0;
        }
        Self { m }
    }

    pub fn from_quat(quat: Quat) -> Self {
        SAMPLES.with(|s| {
            // B[k][i] = Y_i(quat · d_k).
            let mut b = [0.0_f32; N_SAMPLES * NUM_AMBI];
            let mut row = [0.0_f32; NUM_AMBI];
            #[allow(clippy::needless_range_loop)]
            for k in 0..N_SAMPLES {
                let rotated = quat.rotate(s.dirs[k]);
                sh_basis_n3d_into(rotated, &mut row);
                b[k * NUM_AMBI..(k + 1) * NUM_AMBI].copy_from_slice(&row);
            }
            // M = Bᵀ · fitᵀ... let me re-derive:
            // For each direction d_k: M · Y(d_k) = Y(R·d_k).
            // Stack: M · Aᵀ = Bᵀ → M = Bᵀ · (Aᵀ)⁺ = Bᵀ · A · (AᵀA)⁻¹
            // We stored `fit = (AᵀA)⁻¹ Aᵀ` (16×N), so `fitᵀ = A · (AᵀA)⁻¹`
            // wait that's only true if `(AᵀA)⁻¹` is symmetric — it is.
            // So M[i][j] = Σ_k Bᵀ[i][k] · fitᵀ[k][j]
            //           = Σ_k B[k][i] · fit[j][k].
            let mut m = [0.0_f32; NUM_AMBI * NUM_AMBI];
            #[allow(clippy::needless_range_loop)]
            for i in 0..NUM_AMBI {
                #[allow(clippy::needless_range_loop)]
                for j in 0..NUM_AMBI {
                    let mut acc = 0.0_f32;
                    #[allow(clippy::needless_range_loop)]
                    for k in 0..N_SAMPLES {
                        acc += b[k * NUM_AMBI + i] * s.fit[j * N_SAMPLES + k];
                    }
                    m[i * NUM_AMBI + j] = acc;
                }
            }
            Self { m }
        })
    }

    /// Apply this rotation to the 16-channel SH coefficient vector
    /// `coefs`, writing the result into `out`. ACN ordering.
    pub fn apply(&self, coefs: &[f32], out: &mut [f32]) {
        debug_assert_eq!(coefs.len(), NUM_AMBI);
        debug_assert_eq!(out.len(), NUM_AMBI);
        #[allow(clippy::needless_range_loop)]
        for i in 0..NUM_AMBI {
            let mut acc = 0.0_f32;
            #[allow(clippy::needless_range_loop)]
            for j in 0..NUM_AMBI {
                acc += self.m[i * NUM_AMBI + j] * coefs[j];
            }
            out[i] = acc;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_vec(a: &[f32], b: &[f32], tol: f32) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < tol)
    }

    #[test]
    fn identity_quat_yields_identity_rotation() {
        let rot = ShRotation::from_quat(Quat::IDENTITY);
        #[allow(clippy::needless_range_loop)]
        for i in 0..NUM_AMBI {
            #[allow(clippy::needless_range_loop)]
            for j in 0..NUM_AMBI {
                let want = if i == j { 1.0 } else { 0.0 };
                let got = rot.m[i * NUM_AMBI + j];
                assert!(
                    (got - want).abs() < 1e-3,
                    "({i},{j}): got {got}, want {want}"
                );
            }
        }
    }

    #[test]
    fn rotation_matches_sh_basis_at_rotated_direction() {
        let rotations = [
            // 90° about Z
            Quat::new(core::f32::consts::FRAC_PI_4.cos(), 0.0, 0.0, core::f32::consts::FRAC_PI_4.sin()),
            // 90° about X
            Quat::new(core::f32::consts::FRAC_PI_4.cos(), core::f32::consts::FRAC_PI_4.sin(), 0.0, 0.0),
            // 90° about Y
            Quat::new(core::f32::consts::FRAC_PI_4.cos(), 0.0, core::f32::consts::FRAC_PI_4.sin(), 0.0),
            // Arbitrary
            {
                let h = 0.3_f32;
                let s = h.sin();
                let c = h.cos();
                let n = (1.0_f32 / 3.0).sqrt();
                Quat::new(c, s * n, s * n, s * n)
            },
        ];
        let directions = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.6, 0.6, 0.5291503).normalize(),
            Vec3::new(-0.5, 0.5, core::f32::consts::FRAC_1_SQRT_2).normalize(),
        ];

        for &quat in &rotations {
            let rot = ShRotation::from_quat(quat);
            for &dir in &directions {
                let rotated_dir = quat.rotate(dir);
                let mut y_rotated = [0.0_f32; NUM_AMBI];
                sh_basis_n3d_into(rotated_dir, &mut y_rotated);

                let mut y_orig = [0.0_f32; NUM_AMBI];
                sh_basis_n3d_into(dir, &mut y_orig);
                let mut y_via_matrix = [0.0_f32; NUM_AMBI];
                rot.apply(&y_orig, &mut y_via_matrix);

                assert!(
                    approx_vec(&y_rotated, &y_via_matrix, 1e-3),
                    "rotation mismatch for quat {quat:?}, dir {dir:?}:\n  direct = {y_rotated:?}\n  via M  = {y_via_matrix:?}"
                );
            }
        }
    }

    #[test]
    fn block_diagonal_structure() {
        // For a 90° Z rotation, the matrix should have non-zero entries
        // only within each order's block (off-block ≤ noise from fit).
        let q = Quat::new(core::f32::consts::FRAC_PI_4.cos(), 0.0, 0.0, core::f32::consts::FRAC_PI_4.sin());
        let rot = ShRotation::from_quat(q);
        // Order boundaries: 0..1, 1..4, 4..9, 9..16.
        let order_of = |i: usize| -> usize {
            if i < 1 { 0 } else if i < 4 { 1 } else if i < 9 { 2 } else { 3 }
        };
        #[allow(clippy::needless_range_loop)]
        for i in 0..NUM_AMBI {
            #[allow(clippy::needless_range_loop)]
            for j in 0..NUM_AMBI {
                if order_of(i) != order_of(j) {
                    let v = rot.m[i * NUM_AMBI + j];
                    assert!(v.abs() < 5e-3, "off-block ({i},{j}) = {v}");
                }
            }
        }
    }
}
