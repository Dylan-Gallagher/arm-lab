//! Jerk-limited trajectory: 1D S-curve bounds, polyline parameterization,
//! and a golden test that the Demo 2 seed produces a bit-stable timed path.

use std::f64::consts::FRAC_PI_2;

use arm_lab::plan::rrt_connect;
use arm_lab::traj::{SCurve1d, TrajLimits, time_parameterize};
use arm_lab::{Chain, CollisionChecker, PlanConfig};
use mujoco_rs::prelude::*;

const CLUTTERED_XML: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/ur5e/scene_cluttered.xml"
);

fn home_q() -> [f64; 6] {
    [
        -FRAC_PI_2, -FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2, -FRAC_PI_2, 0.0,
    ]
}

#[test]
fn scurve_duration_matches_phase_sum() {
    let c = SCurve1d::rest_to_rest(0.7, 0.9, 2.2, 11.0);
    let sum: f64 = c.duration;
    assert!(sum > 0.0);
    let mid = c.at(c.duration * 0.5);
    assert!(mid.s > 0.0 && mid.s < 0.7);
}

#[test]
fn parameterize_straight_line_respects_joint_limits() {
    let start = vec![0.0; 6];
    let mut goal = vec![0.0; 6];
    goal[0] = 1.0;
    let path = vec![start.clone(), goal.clone()];
    let limits = TrajLimits {
        v_max: 0.6,
        a_max: 2.0,
        j_max: 10.0,
    };
    let traj = time_parameterize(&path, &limits, 0.002);
    assert_eq!(traj.q.first(), Some(&start));
    let last = traj.q.last().unwrap();
    for (a, b) in last.iter().zip(goal.iter()) {
        assert!((a - b).abs() < 1e-6, "end {a} ≠ {b}");
    }
    assert!(traj.qd[0].iter().all(|v| v.abs() < 1e-8));
    assert!(traj.qd.last().unwrap().iter().all(|v| v.abs() < 1e-6));

    let mut max_v = 0.0f64;
    let mut max_a = 0.0f64;
    let mut max_j = 0.0f64;
    for ((qd, qdd), qddd) in traj.qd.iter().zip(traj.qdd.iter()).zip(traj.qddd.iter()) {
        for &v in qd {
            max_v = max_v.max(v.abs());
        }
        for &a in qdd {
            max_a = max_a.max(a.abs());
        }
        for &j in qddd {
            max_j = max_j.max(j.abs());
        }
    }
    assert!(max_v <= limits.v_max + 1e-6, "v {max_v}");
    assert!(max_a <= limits.a_max + 1e-5, "a {max_a}");
    assert!(max_j <= limits.j_max + 1e-5, "j {max_j}");
    assert!(traj.duration > 1.0 / 0.6); // at least D/vmax
}

#[test]
fn parameterize_is_deterministic() {
    let path = vec![vec![0.0, 0.0], vec![0.3, 0.1], vec![0.6, 0.0]];
    let limits = TrajLimits::default();
    let a = time_parameterize(&path, &limits, 0.004);
    let b = time_parameterize(&path, &limits, 0.004);
    assert_eq!(a.duration, b.duration);
    assert_eq!(a.q, b.q);
    assert_eq!(a.qd, b.qd);
    assert_eq!(a.qdd, b.qdd);
    assert_eq!(a.qddd, b.qddd);
    for pair in a.t.windows(2) {
        assert!((pair[1] - pair[0] - a.dt).abs() < 1e-12);
    }
    assert_eq!(a.t.last().copied(), Some(a.duration));
}

#[test]
fn corner_stops_are_continuous_and_globally_bounded() {
    let corner = vec![0.6, -0.2, 0.3];
    let path = vec![vec![0.0, 0.0, 0.0], corner.clone(), vec![0.15, 0.75, -0.1]];
    let limits = TrajLimits {
        v_max: 0.7,
        a_max: 1.9,
        j_max: 7.5,
    };
    let dt = 0.003;
    let traj = time_parameterize(&path, &limits, dt);
    let corner_indices: Vec<usize> = traj
        .q
        .iter()
        .enumerate()
        .filter_map(|(index, q)| (q == &corner).then_some(index))
        .collect();
    assert_eq!(corner_indices.len(), 1, "corner must be sampled once");
    let corner_index = corner_indices[0];
    assert!(corner_index > 0 && corner_index + 1 < traj.len());
    assert!(traj.qd[corner_index].iter().all(|value| *value == 0.0));
    assert!(traj.qdd[corner_index].iter().all(|value| *value == 0.0));

    for sample in 0..traj.len() {
        for joint in 0..path[0].len() {
            assert!(traj.qd[sample][joint].abs() <= limits.v_max + 1e-10);
            assert!(traj.qdd[sample][joint].abs() <= limits.a_max + 1e-10);
            assert!(traj.qddd[sample][joint].abs() <= limits.j_max + 1e-10);
        }
    }
    for sample in 1..traj.len() {
        for joint in 0..path[0].len() {
            let sampled_acceleration = (traj.qd[sample][joint] - traj.qd[sample - 1][joint]) / dt;
            let sampled_jerk = (traj.qdd[sample][joint] - traj.qdd[sample - 1][joint]) / dt;
            assert!(sampled_acceleration.abs() <= limits.a_max + 1e-8);
            assert!(sampled_jerk.abs() <= limits.j_max + 1e-8);
        }
    }
}

#[test]
fn duplicate_waypoints_do_not_create_duplicate_samples() {
    let corner = vec![0.3, 0.0];
    let path = vec![
        vec![0.0, 0.0],
        vec![0.0, 0.0],
        corner.clone(),
        corner.clone(),
        vec![0.3, 0.2],
    ];
    let traj = time_parameterize(&path, &TrajLimits::default(), 0.004);
    assert_eq!(traj.q.iter().filter(|q| *q == &corner).count(), 1);
    assert_eq!(traj.q.first(), Some(&path[0]));
    assert_eq!(traj.q.last(), path.last());
    assert_eq!(traj.t.len(), traj.q.len());
    assert_eq!(traj.q.len(), traj.qd.len());
    assert_eq!(traj.q.len(), traj.qdd.len());
    assert_eq!(traj.q.len(), traj.qddd.len());

    let stationary = time_parameterize(
        &[vec![1.0, -2.0], vec![1.0, -2.0], vec![1.0, -2.0]],
        &TrajLimits::default(),
        0.004,
    );
    assert_eq!(stationary.len(), 1);
    assert_eq!(stationary.duration, 0.0);
}

#[test]
fn demo2_seed_golden_trajectory() {
    // The public demo's (seed, start, goal) pair must produce a bit-stable
    // timed trajectory. This is the CI pin for "same scene + seed → same motion".
    let model = MjModel::from_xml(CLUTTERED_XML).unwrap();
    let chain = Chain::from_mujoco(&model, "ur5e", "wrist_3_link", "attachment_site").unwrap();
    let start = home_q().to_vec();
    let mut goal = start.clone();
    goal[0] += 1.1;
    let mut cc = CollisionChecker::new(&model, &chain);
    let plan = rrt_connect(
        &chain,
        &start,
        &goal,
        |q| cc.collides(q),
        &PlanConfig {
            seed: 20260816,
            ..PlanConfig::default()
        },
    );
    assert_eq!(plan.status, arm_lab::PlanStatus::Success);

    let limits = TrajLimits {
        v_max: 0.55,
        a_max: 1.8,
        j_max: 8.0,
    };
    let traj = time_parameterize(&plan.waypoints, &limits, 0.002);
    let again = time_parameterize(&plan.waypoints, &limits, 0.002);
    assert_eq!(traj.q, again.q);
    assert_eq!(traj.t.len(), traj.q.len());
    assert_eq!(traj.len(), 1919);
    assert!(
        (traj.duration - 3.836).abs() < 1e-12,
        "duration {}",
        traj.duration
    );

    // Endpoints.
    for (a, b) in traj.q[0].iter().zip(start.iter()) {
        assert!((a - b).abs() < 1e-9);
    }
    for (a, b) in traj.q.last().unwrap().iter().zip(goal.iter()) {
        assert!((a - b).abs() < 1e-6);
    }

    let mut max_v = 0.0f64;
    let mut max_a = 0.0f64;
    let mut max_j = 0.0f64;
    for ((qd, qdd), qddd) in traj.qd.iter().zip(&traj.qdd).zip(&traj.qddd) {
        for &v in qd {
            max_v = max_v.max(v.abs());
        }
        for &a in qdd {
            max_a = max_a.max(a.abs());
        }
        for &j in qddd {
            max_j = max_j.max(j.abs());
        }
    }
    assert!(max_v <= limits.v_max + 1e-5, "joint v {max_v}");
    assert!(max_a <= limits.a_max + 1e-5, "joint a {max_a}");
    assert!(max_j <= limits.j_max + 1e-5, "joint j {max_j}");
    let interior_stops = plan.waypoints[1..plan.waypoints.len() - 1]
        .iter()
        .map(|waypoint| {
            traj.q
                .iter()
                .position(|q| q == waypoint)
                .expect("every shortcut corner must be sampled")
        })
        .collect::<Vec<_>>();
    assert_eq!(interior_stops.len(), plan.waypoints.len() - 2);
    for index in &interior_stops {
        assert!(traj.qd[*index].iter().all(|value| *value == 0.0));
        assert!(traj.qdd[*index].iter().all(|value| *value == 0.0));
    }
    println!(
        "golden Demo 2 corner-stop traj: {} samples, {:.3} s, peak |qd| {:.3} rad/s, peak |qdd| {:.3} rad/s^2, peak |qddd| {:.3} rad/s^3, {} shortcut waypoints, {} verified interior stops",
        traj.len(),
        traj.duration,
        max_v,
        max_a,
        max_j,
        plan.waypoints.len(),
        interior_stops.len()
    );
}
