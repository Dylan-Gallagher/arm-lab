//! Pick-and-place scene: IK reachability, blocked carry interpolant,
//! RRT-Connect around the pillar.

use arm_lab::ik::{IkConfig, solve_ik};
use arm_lab::kinematics::fk;
use arm_lab::plan::{edge_free, rrt_connect};
use arm_lab::{Chain, CollisionChecker, PlanConfig, PlanStatus};
use mujoco_rs::prelude::*;
use nalgebra::{Isometry3, Translation3};

const SCENE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/ur5e/scene_pickplace.xml"
);

const PICK_APPROACH: [f64; 3] = [-0.24, 0.58, 0.52];
const PLACE_APPROACH: [f64; 3] = [0.22, 0.58, 0.52];
const PICK: [f64; 3] = [-0.24, 0.58, 0.42];
const PLACE: [f64; 3] = [0.22, 0.58, 0.42];
const SEED: u64 = 20260816;
// The cube is absent from planner geometry; this only rejects robot penetration.
const CARRY_CONTACT_THRESHOLD: f64 = 0.0;

fn load() -> (MjModel, Chain, Vec<f64>) {
    let model = MjModel::from_xml(SCENE).unwrap();
    let chain = Chain::from_mujoco(&model, "ur5e", "wrist_3_link", "attachment_site").unwrap();
    let q_home = {
        let mut data = MjData::new(&model);
        let home_key = model.name_to_id(MjtObj::mjOBJ_KEY, "home").unwrap();
        data.reset_keyframe(home_key).unwrap();
        data.forward();
        chain
            .qpos_addresses()
            .iter()
            .map(|&adr| data.qpos()[adr])
            .collect()
    };
    (model, chain, q_home)
}

fn ik_at(
    chain: &Chain,
    xyz: [f64; 3],
    q_home: &[f64],
    rot: nalgebra::UnitQuaternion<f64>,
) -> Vec<f64> {
    let target = Isometry3::from_parts(Translation3::new(xyz[0], xyz[1], xyz[2]), rot);
    let ik = solve_ik(
        chain,
        &target,
        q_home,
        Some(q_home),
        &IkConfig {
            seed: SEED,
            ..IkConfig::default()
        },
    );
    assert!(ik.converged, "IK stalled at {xyz:?}");
    ik.q
}

#[test]
fn home_is_free_and_poses_are_reachable() {
    let (model, chain, q_home) = load();
    let mut cc = CollisionChecker::new(&model, &chain);
    assert!(!cc.collides(&q_home));
    let rot = fk(&chain, &q_home).rotation;
    for xyz in [PICK_APPROACH, PICK, PLACE_APPROACH, PLACE] {
        let q = ik_at(&chain, xyz, &q_home, rot);
        assert!(!cc.collides(&q), "collision at {xyz:?}");
    }
}

#[test]
fn cube_is_visual_only() {
    let (model, _, _) = load();
    let cube_geom = model.name_to_id(MjtObj::mjOBJ_GEOM, "cube").unwrap();
    assert_eq!(model.geom_contype()[cube_geom], 0);
    assert_eq!(model.geom_conaffinity()[cube_geom], 0);
}

#[test]
fn carry_straight_line_hits_pillar_rrt_succeeds() {
    let (model, chain, q_home) = load();
    let mut cc = CollisionChecker::new(&model, &chain);
    cc.contact_threshold = CARRY_CONTACT_THRESHOLD;
    let rot = fk(&chain, &q_home).rotation;
    let q_pick = ik_at(&chain, PICK_APPROACH, &q_home, rot);
    let q_place = ik_at(&chain, PLACE_APPROACH, &q_home, rot);

    let mut scratch = vec![0.0; chain.dof()];
    let mut collides = |q: &[f64]| cc.collides(q);
    assert!(
        !edge_free(&q_pick, &q_place, 0.05, &mut collides, &mut scratch),
        "fixture broken: carry interpolant does not hit the pillar"
    );

    let plan = rrt_connect(
        &chain,
        &q_pick,
        &q_place,
        |q| cc.collides(q),
        &PlanConfig {
            seed: SEED,
            ..PlanConfig::default()
        },
    );
    assert_eq!(plan.status, PlanStatus::Success);
    assert!(plan.elapsed_s < 1.0, "carry plan {} s", plan.elapsed_s);
    assert!(
        plan.waypoints.len() >= 3,
        "expected a dodge, got {} waypoints",
        plan.waypoints.len()
    );
}
