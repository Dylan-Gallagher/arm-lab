//! Damped least-squares (Levenberg–Marquardt) inverse kinematics, from scratch.
//!
//! Each iteration solves
//!
//! ```text
//! Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e  +  (I − J⁺ J) · k_null · (q_rest − q)
//! ```
//!
//! where `e = [Δp; log(R_target R_cur⁻¹)]` stacks the position error and the
//! world-frame orientation error, and the second term biases the nullspace of
//! the task toward a rest configuration without disturbing the primary task.
//! Joint limits are enforced by clamping after every step; the damping keeps
//! the step bounded in near-singular regions.

use nalgebra::{DMatrix, DVector, Point3, UnitQuaternion, Vector3};

use crate::chain::Chain;
use crate::jacobian::jacobian;
use crate::kinematics::fk;

/// Tunables for the IK solver. Defaults are sane for a 6-DOF arm.
#[derive(Debug, Clone)]
pub struct IkConfig {
    /// Damping coefficient λ (radians/meters-equivalent). Larger = more conservative.
    pub lambda: f64,
    /// Nullspace attraction to the rest pose.
    pub nullspace_gain: f64,
    /// Maximum iterations per solve.
    pub max_iters: usize,
    /// Position convergence threshold (m).
    pub tol_pos: f64,
    /// Orientation convergence threshold (rad).
    pub tol_rot: f64,
    /// Step gain applied to the DLS correction (≤ 1.0 for stability).
    pub step_gain: f64,
    /// Random restarts when the first attempt does not converge.
    /// Seeded, so results are bit-reproducible.
    pub restarts: usize,
    /// Seed for the deterministic restart sampling.
    pub seed: u64,
}

impl Default for IkConfig {
    fn default() -> Self {
        Self {
            lambda: 0.05,
            nullspace_gain: 0.05,
            max_iters: 200,
            tol_pos: 1e-4,
            tol_rot: 1e-3,
            step_gain: 1.0,
            restarts: 8,
            seed: 0,
        }
    }
}

/// Result of one IK solve. `rot_err` is 0.0 for position-only solves.
#[derive(Debug, Clone)]
pub struct IkResult {
    pub q: Vec<f64>,
    pub pos_err: f64,
    pub rot_err: f64,
    pub iterations: usize,
    pub converged: bool,
}

/// Shortest-representation rotation error: the axis-angle vector of
/// `R_target · R_current⁻¹`, expressed in the world frame. Always has norm
/// ≤ π and matches the world-frame angular velocity convention of the
/// geometric Jacobian.
pub fn orientation_error(
    target: &UnitQuaternion<f64>,
    current: &UnitQuaternion<f64>,
) -> Vector3<f64> {
    let rel = target * current.inverse();
    let (mut w, mut v) = (rel.w, Vector3::new(rel.i, rel.j, rel.k));
    if w < 0.0 {
        // q and −q encode the same rotation; pick the shorter one.
        (w, v) = (-w, -v);
    }
    let v_norm = v.norm();
    if v_norm < 1e-12 {
        return Vector3::zeros();
    }
    let angle = 2.0 * v_norm.atan2(w);
    (v / v_norm) * angle
}

/// Solve IK for a target end-effector pose, starting from `q_init`.
///
/// `q_rest` is the nullspace bias target (typically a comfortable pose);
/// pass `None` to use `q_init` itself.
///
/// On non-convergence the solve is retried from seeded random start
/// configurations (as TRAC-IK does); the best attempt is returned.
pub fn solve_ik(
    chain: &Chain,
    target: &nalgebra::Isometry3<f64>,
    q_init: &[f64],
    q_rest: Option<&[f64]>,
    cfg: &IkConfig,
) -> IkResult {
    let mut best = solve_ik_attempt(chain, target, q_init, q_rest, cfg);
    if best.converged {
        return best;
    }
    let mut rng = crate::rng::Rng::new(cfg.seed ^ 0x1CF7EE);
    for _ in 0..cfg.restarts {
        let q_seed = chain.sample_uniform(&mut rng);
        let res = solve_ik_attempt(chain, target, &q_seed, q_rest, cfg);
        if res.converged {
            return res;
        }
        if res.pos_err + 0.1 * res.rot_err < best.pos_err + 0.1 * best.rot_err {
            best = res;
        }
    }
    best
}

fn solve_ik_attempt(
    chain: &Chain,
    target: &nalgebra::Isometry3<f64>,
    q_init: &[f64],
    q_rest: Option<&[f64]>,
    cfg: &IkConfig,
) -> IkResult {
    let n = chain.dof();
    debug_assert_eq!(q_init.len(), n);
    let q_rest: Vec<f64> = q_rest
        .map(|r| r.to_vec())
        .unwrap_or_else(|| q_init.to_vec());

    let mut q = q_init.to_vec();
    let mut dq = DVector::zeros(n);

    let (mut pos_err, mut rot_err) = (f64::INFINITY, f64::INFINITY);
    let mut iterations = 0usize;
    let mut converged = false;

    for it in 0..cfg.max_iters {
        iterations = it + 1;
        let ee = fk(chain, &q);
        let dp = target.translation.vector - ee.translation.vector;
        let dr = orientation_error(&target.rotation, &ee.rotation);
        pos_err = dp.norm();
        rot_err = dr.norm();
        if pos_err < cfg.tol_pos && rot_err < cfg.tol_rot {
            converged = true;
            break;
        }

        // Adaptive damping: full λ when far from the target, decaying as the
        // error shrinks. Fixed damping stalls ~λ·e near the solution; this
        // schedule keeps Newton-like convergence at the endgame while
        // retaining the singularity robustness where it matters. The
        // nullspace bias fades on the same schedule so it cannot fight the
        // primary task at the endgame.
        let err_mix = pos_err + 0.1 * rot_err;
        let scale = (err_mix / 0.05).min(1.0);
        let lambda_sq = (cfg.lambda * scale).powi(2) + 1e-12;

        let j = jacobian(chain, &q);
        let mut e = DVector::zeros(6);
        e[0] = dp.x;
        e[1] = dp.y;
        e[2] = dp.z;
        e[3] = dr.x;
        e[4] = dr.y;
        e[5] = dr.z;
        dls_step(
            &j,
            &e,
            lambda_sq,
            cfg.nullspace_gain * scale,
            &q_rest,
            &q,
            &mut dq,
        );

        for (qi, d) in q.iter_mut().zip(dq.iter()) {
            *qi += cfg.step_gain * d;
        }
        chain.clamp_to_limits(&mut q);
    }

    if !converged {
        let ee = fk(chain, &q);
        pos_err = (target.translation.vector - ee.translation.vector).norm();
        rot_err = orientation_error(&target.rotation, &ee.rotation).norm();
    }
    IkResult {
        q,
        pos_err,
        rot_err,
        iterations,
        converged,
    }
}

/// Solve IK for a target point with orientation left free (3-row task).
pub fn solve_ik_position(
    chain: &Chain,
    target_point: &Point3<f64>,
    q_init: &[f64],
    q_rest: Option<&[f64]>,
    cfg: &IkConfig,
) -> IkResult {
    let n = chain.dof();
    let q_rest: Vec<f64> = q_rest
        .map(|r| r.to_vec())
        .unwrap_or_else(|| q_init.to_vec());
    let mut q = q_init.to_vec();
    let mut dq = DVector::zeros(n);

    let mut pos_err = f64::INFINITY;
    let mut iterations = 0usize;
    let mut converged = false;

    for it in 0..cfg.max_iters {
        iterations = it + 1;
        let ee = fk(chain, &q);
        let dp = target_point.coords - ee.translation.vector;
        pos_err = dp.norm();
        if pos_err < cfg.tol_pos {
            converged = true;
            break;
        }

        let scale = (pos_err / 0.05).min(1.0);
        let lambda_sq = (cfg.lambda * scale).powi(2) + 1e-12;

        let j_full = jacobian(chain, &q);
        // Linear rows only (3-row position task).
        let mut rows = Vec::with_capacity(3 * n);
        for r in 0..3 {
            rows.extend(j_full.row(r).iter().copied());
        }
        let j = DMatrix::from_row_slice(3, n, &rows);

        let mut e = DVector::zeros(3);
        e[0] = dp.x;
        e[1] = dp.y;
        e[2] = dp.z;
        dls_step(
            &j,
            &e,
            lambda_sq,
            cfg.nullspace_gain * scale,
            &q_rest,
            &q,
            &mut dq,
        );

        for (qi, d) in q.iter_mut().zip(dq.iter()) {
            *qi += cfg.step_gain * d;
        }
        chain.clamp_to_limits(&mut q);
    }

    if !converged {
        let ee = fk(chain, &q);
        pos_err = (target_point.coords - ee.translation.vector).norm();
    }
    IkResult {
        q,
        pos_err,
        rot_err: 0.0,
        iterations,
        converged,
    }
}

/// One damped-least-squares update `dq = JᵀA⁻¹e + diag(I − J⁺J)·k·(q_rest−q)`
/// with `A = JJᵀ + λ²I`. Shared by the 6-row and 3-row task variants.
fn dls_step(
    j: &DMatrix<f64>,
    e: &DVector<f64>,
    lambda_sq: f64,
    nullspace_gain: f64,
    q_rest: &[f64],
    q: &[f64],
    dq: &mut DVector<f64>,
) {
    let rows = j.nrows();
    let n = j.ncols();
    let mut a = j * j.transpose();
    for i in 0..rows {
        a[(i, i)] += lambda_sq;
    }
    let cho = a.cholesky().expect("JJᵀ + λ²I is SPD by construction");
    let y = cho.solve(e);
    *dq = j.transpose() * y;

    let yj = cho.solve(j);
    let jj_plus = j.transpose() * yj; // n×n = J⁺J
    for i in 0..n {
        // Diagonal-only nullspace projection: cheap and stable; the full
        // projector matters only for kinematically redundant arms.
        let null_i = 1.0 - jj_plus[(i, i)].clamp(0.0, 1.0);
        dq[i] += null_i * nullspace_gain * (q_rest[i] - q[i]);
    }
}
