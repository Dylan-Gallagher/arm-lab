//! IK convergence statistics: random reachable targets (sampled as FK of
//! random valid joint configurations), solved from a fixed start pose.
//! This is the table that goes into the writeup.

mod common;

use arm_lab::ik::{IkConfig, solve_ik};
use arm_lab::kinematics::fk;
use common::{Rng, random_q, ur5e_chain};
use std::f64::consts::FRAC_PI_2;

#[test]
fn ik_converges_on_random_reachable_targets() {
    let chain = ur5e_chain();
    let cfg = IkConfig::default();

    // Comfortable rest pose (the MJCF "home" keyframe).
    let q_rest = [
        -FRAC_PI_2, -FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2, -FRAC_PI_2, 0.0,
    ];
    let q_start = q_rest.to_vec();

    const N: usize = 1000;
    let mut rng = Rng::new(0x1C2026);

    let (mut solved, mut iters_total, mut pos_worst) = (0usize, 0usize, 0.0f64);
    let mut rot_worst = 0.0f64;
    let mut failures = Vec::new();

    for i in 0..N {
        let q_target = random_q(&chain, &mut rng);
        let target = fk(&chain, &q_target);
        let res = solve_ik(&chain, &target, &q_start, Some(&q_rest), &cfg);
        iters_total += res.iterations;
        pos_worst = pos_worst.max(res.pos_err);
        rot_worst = rot_worst.max(res.rot_err);

        // Limits respected, always.
        for (qi, lim) in res.q.iter().zip(chain.joint_limits()) {
            if let Some((lo, hi)) = lim {
                assert!(
                    *qi >= lo - 1e-12 && *qi <= hi + 1e-12,
                    "joint limit violated: {qi} not in [{lo},{hi}]"
                );
            }
        }

        if res.converged && res.pos_err < 1e-3 && res.rot_err < 1e-2 {
            solved += 1;
        } else if failures.len() < 5 {
            failures.push((i, res.pos_err, res.rot_err, res.iterations));
        }
    }

    let rate = solved as f64 / N as f64 * 100.0;
    println!(
        "IK stats over {N} random reachable targets (6-DOF task, start = home):\n\
         success rate:    {rate:.1}%\n\
         mean iterations: {:.1}\n\
         worst pos err:   {pos_worst:.3e} m\n\
         worst rot err:   {rot_worst:.3e} rad",
        iters_total as f64 / N as f64
    );
    for (i, p, r, it) in &failures {
        println!("  failure: case {i}: pos {p:.3e} m, rot {r:.3e} rad, {it} iters");
    }

    assert!(
        rate >= 95.0,
        "IK success rate {rate:.1}% below the 95% exit criterion"
    );
}

#[test]
fn ik_position_only_moves_to_point() {
    use arm_lab::ik::solve_ik_position;
    use nalgebra::Point3;

    let chain = ur5e_chain();
    let cfg = IkConfig::default();
    let q_start = [
        -FRAC_PI_2, -FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2, -FRAC_PI_2, 0.0,
    ];

    let ee_home = fk(&chain, &q_start).translation.vector;
    let target = Point3::new(ee_home.x + 0.15, ee_home.y - 0.10, ee_home.z + 0.08);

    let res = solve_ik_position(&chain, &target, &q_start, Some(&q_start), &cfg);
    assert!(
        res.converged,
        "position IK failed to converge: {}",
        res.pos_err
    );
    assert!(
        res.pos_err < 1e-4,
        "position IK error too large: {}",
        res.pos_err
    );
}
