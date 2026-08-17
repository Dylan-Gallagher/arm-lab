//! RRT-Connect: synthetic joint-space tests (no MuJoCo) plus a UR5e scene
//! with a pillar that the straight-line interpolant cannot clear.

mod common;

use std::f64::consts::FRAC_PI_2;

use arm_lab::plan::{PlanConfig, PlanStatus, densify, edge_free, path_length, rrt_connect};
use arm_lab::{Chain, CollisionChecker};
use common::ur5e_chain;
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

fn box_collides(q: &[f64]) -> bool {
    // A wall in joint space: q0 ∈ (0.35, 0.65) AND |q1| < 0.20.
    // start = 0, goal = 1 on q0 with q1 = 0 hits it; going around via q1
    // > 0.20 is free. The other joints are unused.
    q[0] > 0.35 && q[0] < 0.65 && q[1].abs() < 0.20
}

#[test]
fn rrt_connect_goes_around_joint_space_box() {
    let chain = ur5e_chain();
    let start = vec![0.0; chain.dof()];
    let mut goal = vec![0.0; chain.dof()];
    goal[0] = 1.0;

    let mut scratch = vec![0.0; chain.dof()];
    assert!(
        !edge_free(&start, &goal, 0.05, &mut box_collides, &mut scratch),
        "the fixture is wrong: the straight line should hit the box"
    );

    let cfg = PlanConfig {
        seed: 7,
        densify: true,
        ..PlanConfig::default()
    };
    let res = rrt_connect(&chain, &start, &goal, box_collides, &cfg);
    assert_eq!(res.status, PlanStatus::Success, "planner failed to connect");
    assert!(
        (res.path.first().unwrap()[0] - start[0]).abs() < 1e-12
            && (res.path.last().unwrap()[0] - goal[0]).abs() < 1e-12,
        "path must start at start and end at goal"
    );

    let mut scratch = vec![0.0; chain.dof()];
    for q in &res.path {
        assert!(!box_collides(q), "densified path has a colliding waypoint");
    }
    for w in res.path.windows(2) {
        assert!(
            edge_free(
                &w[0],
                &w[1],
                cfg.resolution,
                &mut box_collides,
                &mut scratch
            ),
            "densified path has a colliding edge"
        );
    }
    assert!(
        res.waypoints.len() < res.nodes,
        "shortcut should leave fewer waypoints than RRT nodes ({} waypoints, {} nodes)",
        res.waypoints.len(),
        res.nodes
    );
    println!(
        "joint-space box: {} iters, {} nodes, {} waypoints (removed {}), cost {:.3} rad, {:.1} ms",
        res.iterations,
        res.nodes,
        res.waypoints.len(),
        res.shortcut_removed,
        res.cost,
        res.elapsed_s * 1e3
    );
}

#[test]
fn rrt_connect_is_deterministic() {
    let chain = ur5e_chain();
    let start = vec![0.0; chain.dof()];
    let mut goal = vec![0.0; chain.dof()];
    goal[0] = 1.0;
    let cfg = PlanConfig {
        seed: 99,
        densify: false,
        ..PlanConfig::default()
    };
    let a = rrt_connect(&chain, &start, &goal, box_collides, &cfg);
    let b = rrt_connect(&chain, &start, &goal, box_collides, &cfg);
    assert_eq!(a.status, PlanStatus::Success);
    assert_eq!(a.waypoints, b.waypoints);
    assert_eq!(a.nodes, b.nodes);
    assert_eq!(a.iterations, b.iterations);
}

#[test]
fn rrt_connect_direct_when_open() {
    let chain = ur5e_chain();
    let start = vec![0.0; chain.dof()];
    let mut goal = vec![0.0; chain.dof()];
    goal[0] = 0.2; // well clear of the box at 0.35
    let cfg = PlanConfig::default();
    let res = rrt_connect(&chain, &start, &goal, box_collides, &cfg);
    assert_eq!(res.status, PlanStatus::Success);
    assert_eq!(res.iterations, 0, "open line should skip RRT");
    assert_eq!(res.waypoints.len(), 2);
}

#[test]
fn densify_preserves_ends_and_spacing() {
    let path = [vec![0.0, 0.0, 0.0], vec![1.0, 0.0, 0.0]];
    let d = densify(&path, 0.2);
    assert_eq!(d[0], path[0]);
    assert_eq!(d[d.len() - 1], path[1]);
    for w in d.windows(2) {
        assert!(path_length(w) <= 0.2 + 1e-12);
    }
}

fn cluttered_model() -> MjModel {
    MjModel::from_xml(CLUTTERED_XML).expect("failed to load cluttered UR5e scene")
}

fn cluttered_chain(model: &MjModel) -> Chain {
    Chain::from_mujoco(model, "ur5e", "wrist_3_link", "attachment_site")
        .expect("failed to extract UR5e chain")
}

#[test]
fn home_is_collision_free_in_cluttered_scene() {
    let model = cluttered_model();
    let chain = cluttered_chain(&model);
    let mut cc = CollisionChecker::new(&model, &chain);
    assert!(
        cc.robot_geom_count() > 0,
        "checker found no robot collision geoms"
    );
    assert!(
        !cc.collides(&home_q()),
        "home configuration collides with the pillar — move the pillar"
    );
}

#[test]
fn pillar_is_detected() {
    let model = cluttered_model();
    let chain = cluttered_chain(&model);
    let mut cc = CollisionChecker::new(&model, &chain);
    let start = home_q();
    let mut goal = start;
    goal[0] += 1.1;
    let mut scratch = vec![0.0; chain.dof()];
    let mut collides = |q: &[f64]| cc.collides(q);
    let hits = !edge_free(&start, &goal, 0.04, &mut collides, &mut scratch);
    assert!(
        hits,
        "straight-line pan of 1.1 rad from home does not hit the pillar — restage the scene"
    );
    assert!(
        !cc.collides(&goal),
        "panned goal itself is in collision — shrink/move the pillar"
    );
}

#[test]
fn collision_audit_matches_boolean_and_identifies_pillar() {
    let model = cluttered_model();
    let chain = cluttered_chain(&model);
    let start = home_q();
    let mut q = start;
    let mut checker = CollisionChecker::new(&model, &chain);
    checker.contact_threshold = 0.0;
    let mut first_contacts = None;
    for step in 0..=100 {
        q[0] = start[0] + 1.1 * step as f64 / 100.0;
        let contacts = checker.robot_contacts(&q);
        if !contacts.is_empty() {
            assert!(checker.collides(&q));
            first_contacts = Some(contacts);
            break;
        }
    }
    let contacts = first_contacts.expect("pan sweep never contacted pillar");
    assert!(contacts.iter().all(|contact| {
        contact.distance_m < 0.0
            && !contact.geom1_name.is_empty()
            && !contact.geom2_name.is_empty()
            && !contact.body1_name.is_empty()
            && !contact.body2_name.is_empty()
    }));
    assert!(
        contacts
            .iter()
            .any(|contact| { contact.geom1_name == "pillar" || contact.geom2_name == "pillar" })
    );
}

#[test]
fn rrt_dodges_ur5e_pillar() {
    let model = cluttered_model();
    let chain = cluttered_chain(&model);
    let mut cc = CollisionChecker::new(&model, &chain);
    let start = home_q().to_vec();
    let mut goal = start.clone();
    goal[0] += 1.1;

    let cfg = PlanConfig {
        seed: 20260816,
        ..PlanConfig::default()
    };
    let res = rrt_connect(&chain, &start, &goal, |q| cc.collides(q), &cfg);
    assert_eq!(
        res.status,
        PlanStatus::Success,
        "failed to dodge the pillar ({:?}, {:.1} ms, {} nodes)",
        res.status,
        res.elapsed_s * 1e3,
        res.nodes
    );

    let mut cc = CollisionChecker::new(&model, &chain);
    for q in &res.path {
        assert!(!cc.collides(q), "planned path collides");
    }
    let mut scratch = vec![0.0; chain.dof()];
    let mut collides = |q: &[f64]| cc.collides(q);
    for w in res.path.windows(2) {
        assert!(edge_free(
            &w[0],
            &w[1],
            cfg.resolution,
            &mut collides,
            &mut scratch
        ));
    }
    println!(
        "UR5e pillar dodge: {} iters, {} nodes, {} waypoints (removed {}), cost {:.3} rad, {:.1} ms",
        res.iterations,
        res.nodes,
        res.waypoints.len(),
        res.shortcut_removed,
        res.cost,
        res.elapsed_s * 1e3
    );
}

#[test]
fn rrt_cluttered_median_under_one_second() {
    let model = cluttered_model();
    let chain = cluttered_chain(&model);
    let start = home_q().to_vec();
    let mut goal = start.clone();
    goal[0] += 1.1;

    const N: usize = 11;
    let mut times = Vec::with_capacity(N);
    let mut costs = Vec::with_capacity(N);
    for k in 0..N {
        let mut cc = CollisionChecker::new(&model, &chain);
        let cfg = PlanConfig {
            seed: 1000 + k as u64,
            ..PlanConfig::default()
        };
        let res = rrt_connect(&chain, &start, &goal, |q| cc.collides(q), &cfg);
        assert_eq!(res.status, PlanStatus::Success, "trial {k} failed");
        times.push(res.elapsed_s);
        costs.push(res.cost);
    }
    times.sort_by(|a, b| a.total_cmp(b));
    costs.sort_by(|a, b| a.total_cmp(b));
    let median_t = times[N / 2];
    let median_c = costs[N / 2];
    println!(
        "cluttered UR5e, {N} seeds: median {:.1} ms (min {:.1}, max {:.1}); median cost {:.3} rad",
        median_t * 1e3,
        times[0] * 1e3,
        times[N - 1] * 1e3,
        median_c
    );
    assert!(
        median_t < 1.0,
        "median planning time {median_t:.3}s exceeds the 1s exit criterion"
    );
}

/// Sanity that the empty scene's home still agrees with the original model
/// loader used by the kinematics tests (chain extraction is scene-independent
/// as long as the robot XML is the same).
#[test]
fn cluttered_chain_matches_bare_ur5e() {
    let bare = ur5e_chain();
    let model = cluttered_model();
    let cluttered = cluttered_chain(&model);
    assert_eq!(bare.dof(), cluttered.dof());
    assert_eq!(bare.joint_names(), cluttered.joint_names());
}
