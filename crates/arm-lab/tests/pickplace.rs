//! Pick-and-place scene: IK reachability, blocked carry interpolant,
//! RRT-Connect around the pillar.

use std::collections::HashSet;

use arm_lab::ik::{IkConfig, solve_ik};
use arm_lab::kinematics::fk;
use arm_lab::plan::{densify, edge_free, rrt_connect};
use arm_lab::{
    AttachedBoxCollisionChecker, AttachedBoxError, AttachedBoxSpec, Chain, CollisionChecker,
    PlanConfig, PlanStatus,
};
use mujoco_rs::prelude::*;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

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
const PAYLOAD_CLEARANCE_M: f64 = 0.005;
const PAYLOAD_ENVIRONMENT: [&str; 3] = ["floor", "table", "pillar"];

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

fn payload_spec() -> AttachedBoxSpec {
    AttachedBoxSpec::new(
        "cube",
        Isometry3::translation(0.0, 0.0, 0.035),
        PAYLOAD_ENVIRONMENT,
        PAYLOAD_CLEARANCE_M,
    )
}

fn attached_checker<'a>(
    model: &'a MjModel,
    chain: &Chain,
) -> AttachedBoxCollisionChecker<&'a MjModel> {
    let mut checker = AttachedBoxCollisionChecker::new(model, chain, payload_spec()).unwrap();
    checker.set_robot_contact_threshold(CARRY_CONTACT_THRESHOLD);
    checker
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

#[test]
fn attached_cube_endpoints_are_free_and_pair_scope_excludes_robot() {
    let (model, chain, q_home) = load();
    let rot = fk(&chain, &q_home).rotation;
    let q_pick = ik_at(&chain, PICK_APPROACH, &q_home, rot);
    let q_place = ik_at(&chain, PLACE_APPROACH, &q_home, rot);
    let mut checker = attached_checker(&model, &chain);

    assert_eq!(checker.robot_contact_threshold(), 0.0);
    assert_eq!(checker.payload_clearance_m(), PAYLOAD_CLEARANCE_M);
    assert_eq!(checker.proxy_geom_name(), "cube");
    assert_eq!(
        checker.environment_geom_names().collect::<Vec<_>>(),
        PAYLOAD_ENVIRONMENT
    );
    for (label, q) in [("pick", &q_pick), ("place", &q_place)] {
        assert!(!checker.robot_collides(q), "{label} robot collision");
        let distances = checker.payload_distances(q);
        assert_eq!(distances.len(), PAYLOAD_ENVIRONMENT.len());
        assert!(
            distances
                .iter()
                .all(|distance| !distance.violates_clearance()),
            "{label} payload violation: {distances:?}"
        );
    }

    let queried: HashSet<_> = checker
        .environment_geom_names()
        .map(|name| model.name_to_id(MjtObj::mjOBJ_GEOM, name).unwrap())
        .collect();
    let chain_bodies: HashSet<_> = chain.links().iter().map(|link| link.body_id).collect();
    for (geom_id, &body_id) in model.geom_bodyid().iter().enumerate() {
        if body_id != 0 && chain_bodies.contains(&(body_id as usize)) {
            assert!(
                !queried.contains(&geom_id),
                "robot geom {geom_id} unexpectedly entered payload pair scope"
            );
        }
    }
    for excluded in ["cube", "place_pad"] {
        let geom_id = model.name_to_id(MjtObj::mjOBJ_GEOM, excluded).unwrap();
        assert!(
            !queried.contains(&geom_id),
            "queried excluded geom {excluded}"
        );
    }
}

#[test]
fn attached_cube_blocks_straight_carry_and_planned_path_is_sampled_clear() {
    let (model, chain, q_home) = load();
    let rot = fk(&chain, &q_home).rotation;
    let q_pick = ik_at(&chain, PICK_APPROACH, &q_home, rot);
    let q_place = ik_at(&chain, PLACE_APPROACH, &q_home, rot);
    let cfg = PlanConfig {
        seed: SEED,
        ..PlanConfig::default()
    };
    assert_eq!(cfg.resolution, 0.05);

    let mut scratch = vec![0.0; chain.dof()];
    let mut robot_checker = attached_checker(&model, &chain);
    let robot_blocks_straight = !edge_free(
        &q_pick,
        &q_place,
        cfg.resolution,
        &mut |q| robot_checker.robot_collides(q),
        &mut scratch,
    );
    let mut payload_checker = attached_checker(&model, &chain);
    let payload_blocks_straight = !edge_free(
        &q_pick,
        &q_place,
        cfg.resolution,
        &mut |q| payload_checker.payload_collides(q),
        &mut scratch,
    );
    let mut combined_checker = attached_checker(&model, &chain);
    assert!(
        !edge_free(
            &q_pick,
            &q_place,
            cfg.resolution,
            &mut |q| combined_checker.collides(q),
            &mut scratch,
        ),
        "fixture broken: combined straight carry is free"
    );
    eprintln!(
        "straight carry blockers: robot={robot_blocks_straight}, payload={payload_blocks_straight}"
    );

    let straight_samples = densify(&[q_pick.clone(), q_place.clone()], cfg.resolution);
    let mut straight_audit = attached_checker(&model, &chain);
    let mut straight_robot_collision_samples = 0usize;
    let mut straight_payload_violation_samples = 0usize;
    let mut straight_min_payload_distance_m = f64::INFINITY;
    let mut straight_min_payload_pair = String::new();
    for q in &straight_samples {
        straight_robot_collision_samples += usize::from(straight_audit.robot_collides(q));
        let distances = straight_audit.payload_distances(q);
        straight_payload_violation_samples += usize::from(
            distances
                .iter()
                .any(|distance| distance.violates_clearance()),
        );
        for distance in distances {
            if distance.distance_m < straight_min_payload_distance_m {
                straight_min_payload_distance_m = distance.distance_m;
                straight_min_payload_pair = distance.identity();
            }
        }
    }

    let mut planner_a = attached_checker(&model, &chain);
    let plan_a = rrt_connect(&chain, &q_pick, &q_place, |q| planner_a.collides(q), &cfg);
    assert_eq!(plan_a.status, PlanStatus::Success);
    assert!(plan_a.waypoints.len() >= 3, "expected a carry dodge");

    let mut audit = attached_checker(&model, &chain);
    let mut planned_min_payload_distance_m = f64::INFINITY;
    let mut planned_min_payload_pair = String::new();
    for (sample, q) in plan_a.path.iter().enumerate() {
        assert!(
            !audit.robot_collides(q),
            "robot collision at path sample {sample}"
        );
        let distances = audit.payload_distances(q);
        assert!(
            distances
                .iter()
                .all(|distance| !distance.violates_clearance()),
            "payload violation at path sample {sample}: {distances:?}"
        );
        for distance in distances {
            if distance.distance_m < planned_min_payload_distance_m {
                planned_min_payload_distance_m = distance.distance_m;
                planned_min_payload_pair = distance.identity();
            }
        }
    }

    eprintln!(
        "attached carry audit: straight_samples={}, straight_robot_collision_samples={}, \
         straight_payload_violation_samples={}, straight_min_payload_distance_m={:.9}, \
         straight_min_payload_pair={}, \
         planned_waypoints={}, planned_samples={}, planned_min_payload_distance_m={:.9}, \
         planned_min_payload_pair={}, planned_cost_rad={:.9}",
        straight_samples.len(),
        straight_robot_collision_samples,
        straight_payload_violation_samples,
        straight_min_payload_distance_m,
        straight_min_payload_pair,
        plan_a.waypoints.len(),
        plan_a.path.len(),
        planned_min_payload_distance_m,
        planned_min_payload_pair,
        plan_a.cost,
    );

    let mut planner_b = attached_checker(&model, &chain);
    let plan_b = rrt_connect(&chain, &q_pick, &q_place, |q| planner_b.collides(q), &cfg);
    assert_eq!(plan_b.status, PlanStatus::Success);
    assert_eq!(plan_a.waypoints, plan_b.waypoints);
    assert_eq!(plan_a.path, plan_b.path);
}

#[test]
fn attached_box_spec_rejects_invalid_pair_scopes() {
    let (model, chain, _) = load();
    let pose = Isometry3::translation(0.0, 0.0, 0.035);
    let make = |proxy: &str, environment: &[&str], clearance: f64| {
        AttachedBoxCollisionChecker::new(
            &model,
            &chain,
            AttachedBoxSpec::new(proxy, pose, environment.iter().copied(), clearance),
        )
    };

    assert!(matches!(
        make("cube", &[], PAYLOAD_CLEARANCE_M),
        Err(AttachedBoxError::EmptyEnvironmentSet)
    ));
    assert!(matches!(
        make("missing", &PAYLOAD_ENVIRONMENT, PAYLOAD_CLEARANCE_M),
        Err(AttachedBoxError::UnknownProxyGeom(_))
    ));
    assert!(matches!(
        make("floor", &["table"], PAYLOAD_CLEARANCE_M),
        Err(AttachedBoxError::ProxyIsNotBox { .. })
    ));
    assert!(matches!(
        make("table", &["pillar"], PAYLOAD_CLEARANCE_M),
        Err(AttachedBoxError::ProxyBodyIsNotMocap { .. })
    ));
    assert!(matches!(
        make("cube", &["missing"], PAYLOAD_CLEARANCE_M),
        Err(AttachedBoxError::UnknownEnvironmentGeom(_))
    ));
    assert!(matches!(
        make("cube", &["floor", "floor"], PAYLOAD_CLEARANCE_M),
        Err(AttachedBoxError::DuplicateEnvironmentGeom(_))
    ));
    assert!(matches!(
        make("cube", &["cube"], PAYLOAD_CLEARANCE_M),
        Err(AttachedBoxError::ProxyInEnvironmentSet(_))
    ));
    assert!(matches!(
        make("cube", &["place_pad"], PAYLOAD_CLEARANCE_M),
        Err(AttachedBoxError::EnvironmentGeomIsNotContactEnabled(_))
    ));
    assert!(matches!(
        make("cube", &["floor"], -0.001),
        Err(AttachedBoxError::InvalidClearance(_))
    ));
    assert!(matches!(
        make("cube", &["floor"], f64::NAN),
        Err(AttachedBoxError::InvalidClearance(_))
    ));

    let mut non_finite_pose = pose;
    non_finite_pose.translation.vector.x = f64::NAN;
    assert!(matches!(
        AttachedBoxCollisionChecker::new(
            &model,
            &chain,
            AttachedBoxSpec::new(
                "cube",
                non_finite_pose,
                PAYLOAD_ENVIRONMENT,
                PAYLOAD_CLEARANCE_M,
            ),
        ),
        Err(AttachedBoxError::NonFiniteProxyTransform)
    ));
}

#[test]
fn attached_box_spec_rejects_robot_geom_in_environment_scope() {
    let xml = r#"
        <mujoco model="attached-box-validation">
          <worldbody>
            <body name="base">
              <body name="tip">
                <joint name="hinge" type="hinge" axis="0 0 1"/>
                <geom name="robot_collision" type="capsule" size="0.03 0.10"/>
                <site name="attachment_site" pos="0 0 0.1"/>
              </body>
            </body>
            <body name="proxy_body" mocap="true">
              <geom name="proxy" type="box" size="0.02 0.02 0.02"
                    contype="0" conaffinity="0"/>
            </body>
          </worldbody>
        </mujoco>
    "#;
    let model = MjModel::from_xml_string(xml).unwrap();
    let chain = Chain::from_mujoco(&model, "mini", "tip", "attachment_site").unwrap();
    let result = AttachedBoxCollisionChecker::new(
        &model,
        &chain,
        AttachedBoxSpec::new(
            "proxy",
            Isometry3::identity(),
            ["robot_collision"],
            PAYLOAD_CLEARANCE_M,
        ),
    );
    assert!(matches!(
        result,
        Err(AttachedBoxError::EnvironmentGeomBelongsToRobot(_))
    ));
}

#[test]
fn attached_proxy_world_pose_matches_fk_with_nonzero_geom_local_pose() {
    let xml = r#"
        <mujoco model="attached-box-transform">
          <worldbody>
            <geom name="obstacle" type="box" pos="2 0 0" size="0.1 0.1 0.1"/>
            <body name="base" pos="0.10 -0.20 0.30" quat="0.9807853 0 0 0.1950903">
              <body name="tip" pos="0.20 0.05 0.10">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom name="robot_collision" type="capsule" size="0.03 0.10"/>
                <site name="attachment_site" pos="0.04 -0.02 0.12"
                      quat="0.9659258 0.2588190 0 0"/>
              </body>
            </body>
            <body name="proxy_body" mocap="true" pos="-0.3 0.4 0.2">
              <geom name="proxy" type="box" size="0.02 0.03 0.04"
                    pos="0.011 -0.017 0.023"
                    quat="0.9238795 0 0.3826834 0"
                    contype="0" conaffinity="0"/>
            </body>
          </worldbody>
        </mujoco>
    "#;
    let model = MjModel::from_xml_string(xml).unwrap();
    let chain = Chain::from_mujoco(&model, "mini", "tip", "attachment_site").unwrap();
    let proxy_in_ee = Isometry3::from_parts(
        Translation3::new(0.031, -0.019, 0.047),
        UnitQuaternion::from_euler_angles(0.17, -0.23, 0.31),
    );
    let mut checker = AttachedBoxCollisionChecker::new(
        &model,
        &chain,
        AttachedBoxSpec::new("proxy", proxy_in_ee, ["obstacle"], PAYLOAD_CLEARANCE_M),
    )
    .unwrap();
    let q = [0.37];
    let expected = fk(&chain, &q) * proxy_in_ee;
    let actual = checker.proxy_world_pose(&q);

    let translation_error = (expected.translation.vector - actual.translation.vector).norm();
    let rotation_error = expected.rotation.angle_to(&actual.rotation);
    assert!(
        translation_error < 1e-10,
        "proxy translation error {translation_error:.3e} m"
    );
    assert!(
        rotation_error < 1e-10,
        "proxy rotation error {rotation_error:.3e} rad"
    );
}

#[test]
fn intended_robot_proxy_contact_is_excluded_from_combined_predicate() {
    let xml = r#"
        <mujoco model="attached-box-allowed-contact">
          <worldbody>
            <geom name="obstacle" type="box" pos="2 0 0" size="0.1 0.1 0.1"/>
            <body name="base">
              <body name="tip">
                <joint name="hinge" type="hinge" axis="0 0 1"/>
                <geom name="robot_collision" type="sphere" size="0.10"/>
                <site name="attachment_site"/>
              </body>
            </body>
            <body name="proxy_body" mocap="true">
              <geom name="proxy" type="box" size="0.05 0.05 0.05"/>
            </body>
          </worldbody>
        </mujoco>
    "#;
    let model = MjModel::from_xml_string(xml).unwrap();
    let chain = Chain::from_mujoco(&model, "mini", "tip", "attachment_site").unwrap();
    let q = [0.0];

    let mut unscoped_robot = CollisionChecker::new(&model, &chain);
    unscoped_robot.contact_threshold = 0.0;
    assert!(
        unscoped_robot.collides(&q),
        "fixture broken: contact-enabled proxy does not overlap robot"
    );

    let mut checker = AttachedBoxCollisionChecker::new(
        &model,
        &chain,
        AttachedBoxSpec::new(
            "proxy",
            Isometry3::identity(),
            ["obstacle"],
            PAYLOAD_CLEARANCE_M,
        ),
    )
    .unwrap();
    checker.set_robot_contact_threshold(0.0);
    assert!(!checker.robot_collides(&q));
    assert!(!checker.payload_collides(&q));
    assert!(!checker.collides(&q));
}
