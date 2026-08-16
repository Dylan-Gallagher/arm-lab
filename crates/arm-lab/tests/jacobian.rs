//! Geometric Jacobian vs numerical differentiation of the FK — the standard
//! correctness gate for any hand-written Jacobian.

mod common;

use arm_lab::jacobian::jacobian;
use arm_lab::kinematics::fk;
use common::{Rng, random_q, ur5e_chain};
use nalgebra::Vector3;

fn numerical_jacobian(chain: &arm_lab::Chain, q: &[f64], h: f64) -> nalgebra::DMatrix<f64> {
    let n = q.len();
    let mut jn = nalgebra::DMatrix::<f64>::zeros(6, n);
    for i in 0..n {
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[i] += h;
        qm[i] -= h;
        let tp = fk(chain, &qp);
        let tm = fk(chain, &qm);
        let dp = (tp.translation.vector - tm.translation.vector) / (2.0 * h);
        // Angular: log of the relative rotation, world frame.
        let rel = tp.rotation * tm.rotation.inverse();
        let (mut w, mut v) = (rel.w, Vector3::new(rel.i, rel.j, rel.k));
        if w < 0.0 {
            w = -w;
            v = -v;
        }
        let vn = v.norm();
        let dr: Vector3<f64> = if vn < 1e-14 {
            Vector3::zeros()
        } else {
            (v / vn) * (2.0 * vn.atan2(w)) / (2.0 * h)
        };
        jn[(0, i)] = dp.x;
        jn[(1, i)] = dp.y;
        jn[(2, i)] = dp.z;
        jn[(3, i)] = dr.x;
        jn[(4, i)] = dr.y;
        jn[(5, i)] = dr.z;
    }
    jn
}

#[test]
fn jacobian_matches_numerical() {
    let chain = ur5e_chain();
    let mut rng = Rng::new(0x3AC0B);
    let mut worst = 0.0f64;

    for _ in 0..30 {
        let q = random_q(&chain, &mut rng);
        let j = jacobian(&chain, &q);
        let jn = numerical_jacobian(&chain, &q, 1e-6);
        for r in 0..6 {
            for c in 0..chain.dof() {
                let d = (j[(r, c)] - jn[(r, c)]).abs();
                worst = worst.max(d);
            }
        }
    }

    println!("jacobian check: worst abs diff vs numerical = {worst:.3e}");
    assert!(
        worst < 1e-5,
        "geometric Jacobian disagrees with numerical: {worst:e}"
    );
}

#[test]
fn jacobian_singular_at_straight_arm() {
    // At q = 0 the UR5e is near a wrist singularity; J should lose rank
    // (smallest singular value well below the largest).
    let chain = ur5e_chain();
    let j = jacobian(&chain, &chain.zero());
    let svd = j.svd(true, true);
    let s = svd.singular_values.clone();
    let (smin, smax) = (s.min(), s.max());
    println!(
        "q=0 singular values: min={smin:.4}, max={smax:.4}, cond={:.0}",
        smax / smin
    );
    assert!(
        smin < 0.2 * smax,
        "expected rank deficiency at the wrist singularity"
    );
}
