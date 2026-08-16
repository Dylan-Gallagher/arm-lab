//! Geometric Jacobian, from scratch.
//!
//! For a revolute joint `i` with world-frame axis `z_i` through anchor `a_i`,
//! the columns of the basic geometric Jacobian are
//!
//! ```text
//! J_v[:, i] = z_i × (p_ee − a_i)      (linear part)
//! J_w[:, i] = z_i                      (angular part, world frame)
//! ```
//!
//! The matrix is `6 × n` with rows `[0..3) = linear`, `[3..6) = angular`.
//! The tests verify it against numerical differentiation of the FK.

use nalgebra::DMatrix;

use crate::chain::Chain;
use crate::kinematics::fk_full;

/// Geometric Jacobian of the end-effector frame at configuration `q`.
pub fn jacobian(chain: &Chain, q: &[f64]) -> DMatrix<f64> {
    let (poses, ee) = fk_full(chain, q);
    let n = chain.dof();
    let mut j = DMatrix::<f64>::zeros(6, n);
    let mut col = 0;
    for pose in &poses {
        let Some((anchor, axis)) = pose.joint_anchor_axis else {
            continue;
        };
        let r = ee.translation.vector - anchor.coords;
        j[(0, col)] = axis.y * r.z - axis.z * r.y;
        j[(1, col)] = axis.z * r.x - axis.x * r.z;
        j[(2, col)] = axis.x * r.y - axis.y * r.x;
        j[(3, col)] = axis.x;
        j[(4, col)] = axis.y;
        j[(5, col)] = axis.z;
        col += 1;
    }
    debug_assert_eq!(col, n);
    j
}
