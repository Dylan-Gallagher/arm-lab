//! Demo 3 — pick-and-place around a pillar, from scratch.
//!
//! IK solves grasp poses, RRT-Connect carries the cube around a pillar that
//! blocks the joint-space interpolant, a pair-scoped attached-box proxy checks
//! the carried cube against the environment, a jerk-bounded scalar S-curve
//! times every segment, and a mocap weld stands in for a gripper (scripted
//! attach, not contact-rich grasping).
//!
//! ```text
//! cargo run --release -p arm-lab-demo --bin demo3
//! cargo run --release -p arm-lab-demo --bin demo3 -- --render
//! ```

use std::path::Path;

use arm_lab::ik::{IkConfig, solve_ik};
use arm_lab::kinematics::fk;
use arm_lab::plan::rrt_connect;
use arm_lab::traj::{TrajLimits, time_parameterize};
use arm_lab::{
    AttachedBoxCollisionChecker, AttachedBoxSpec, Chain, CollisionChecker, PlanConfig, PlanStatus,
};
use arm_lab_demo::{
    GIF_FPS, RENDER_EVERY, RENDER_H, RENDER_W, capture_frame, encode_gif, gravity_compensate,
    init_recording, log_transform, parse_args, read_q, set_ctrl, traj_step,
};
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

const SCENE_XML: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/ur5e/scene_pickplace.xml"
);

const ACTUATORS: [&str; 6] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
];

/// Cube center in the EE frame: along site Z (already −world Z at home).
const CUBE_IN_EE: Vector3<f64> = Vector3::new(0.0, 0.0, 0.035);

const PICK: [f64; 3] = [-0.24, 0.58, 0.42];
const PICK_APPROACH: [f64; 3] = [-0.24, 0.58, 0.52];
const PLACE: [f64; 3] = [0.22, 0.58, 0.42];
const PLACE_APPROACH: [f64; 3] = [0.22, 0.58, 0.52];
const CUBE_REST_PICK: [f64; 3] = [-0.24, 0.58, 0.385];
const CUBE_REST_PLACE: [f64; 3] = [0.22, 0.58, 0.385];

const KV_OVER_KP: f64 = 0.2;
const LOG_EVERY: usize = 2;
const SETTLE_STEPS: usize = 80;
/// Carry planning rejects sampled robot penetration.
const CARRY_CONTACT_THRESHOLD: f64 = 0.0;
/// Pair-scoped positive planning buffer for the attached cube only.
const PAYLOAD_CLEARANCE_M: f64 = 0.005;
const PAYLOAD_ENVIRONMENT: [&str; 3] = ["floor", "table", "pillar"];
const SEED: u64 = 20260816;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (do_render, mode, rrd_path_override) = parse_args(&args);

    let model = MjModel::from_xml(SCENE_XML).expect("failed to load pick-and-place scene");
    let mut data = MjData::new(&model);
    let dt = model.opt().timestep;
    let chain = Chain::from_mujoco(&model, "ur5e", "wrist_3_link", "attachment_site")
        .expect("failed to extract chain");

    let home_key = model
        .name_to_id(MjtObj::mjOBJ_KEY, "home")
        .expect("home keyframe");
    data.reset_keyframe(home_key).expect("reset to home");
    data.forward();

    let q_home = read_q(&data, &chain);
    let dof_adrs = chain.dof_addresses();
    let ee_home = fk(&chain, &q_home);
    let rot = ee_home.rotation;

    let ik_cfg = IkConfig {
        seed: SEED,
        ..IkConfig::default()
    };
    let mut cc = CollisionChecker::new(&model, &chain);
    assert!(!cc.collides(&q_home), "home is in collision");
    let mut carry_cc = AttachedBoxCollisionChecker::new(
        &model,
        &chain,
        AttachedBoxSpec::new(
            "cube",
            Isometry3::translation(CUBE_IN_EE.x, CUBE_IN_EE.y, CUBE_IN_EE.z),
            PAYLOAD_ENVIRONMENT,
            PAYLOAD_CLEARANCE_M,
        ),
    )
    .expect("valid attached cube collision proxy");
    carry_cc.set_robot_contact_threshold(CARRY_CONTACT_THRESHOLD);

    let q_pick_app = solve_named(
        "pick_approach",
        &chain,
        &PICK_APPROACH,
        rot,
        &q_home,
        &ik_cfg,
        &mut cc,
    );
    let q_pick = solve_named("pick", &chain, &PICK, rot, &q_pick_app, &ik_cfg, &mut cc);
    let q_place_app = solve_named(
        "place_approach",
        &chain,
        &PLACE_APPROACH,
        rot,
        &q_home,
        &ik_cfg,
        &mut cc,
    );
    let q_place = solve_named("place", &chain, &PLACE, rot, &q_place_app, &ik_cfg, &mut cc);

    let rec = init_recording(
        "arm-lab demo3",
        &mode,
        "demo_output/demo3.rrd",
        rrd_path_override,
    );
    rec.log(
        "world/description",
        &rerun::TextLog::new(
            "UR5e · Demo 3 · pick-and-place around a pillar, from scratch, no ROS/MoveIt",
        ),
    )
    .ok();

    let mut renderer = None;
    let frame_dir = Path::new("demo_output/frames3");
    let mut frame_idx: usize = 0;
    if do_render {
        std::fs::create_dir_all(frame_dir).expect("create frame dir");
        for old in std::fs::read_dir(frame_dir).unwrap().flatten() {
            std::fs::remove_file(old.path()).ok();
        }
        let mut cam = MjvCamera::new_free(&model);
        cam.lookat = [0.00, 0.52, 0.28];
        cam.azimuth = 138.0;
        cam.elevation = -18.0;
        cam.distance = 2.15;
        let r = MjRenderer::builder()
            .width(RENDER_W)
            .height(RENDER_H)
            .camera(cam)
            .build(&model)
            .expect("failed to init EGL offscreen renderer");
        renderer = Some(r);
        println!("[demo3] offscreen render: {RENDER_W}x{RENDER_H} @ {GIF_FPS} fps");
    }

    let link_paths: Vec<String> = chain
        .links()
        .iter()
        .map(|l| format!("world/arm/{}", l.name))
        .collect();

    let limits = TrajLimits {
        v_max: 0.70,
        a_max: 2.2,
        j_max: 10.0,
    };
    let plan_cfg = PlanConfig {
        seed: SEED,
        ..PlanConfig::default()
    };

    let mut ctx = ExecCtx {
        data: &mut data,
        chain: &chain,
        rec: &rec,
        renderer: &mut renderer,
        frame_dir,
        frame_idx: &mut frame_idx,
        link_paths: &link_paths,
        dof_adrs: &dof_adrs,
        dt,
        limits,
        grasped: false,
        cube_park: CUBE_REST_PICK,
        step_idx: 0,
        worst_track: 0.0,
        do_render,
    };

    set_cube(ctx.data, CUBE_REST_PICK, rot);
    set_ctrl(ctx.data, &ACTUATORS, &q_home);
    for _ in 0..SETTLE_STEPS {
        gravity_compensate(ctx.data, &dof_adrs);
        ctx.data.step();
    }

    let mut q = q_home.clone();
    q = go(
        "home → pick approach",
        &mut ctx,
        &mut cc,
        &plan_cfg,
        &q,
        &q_pick_app,
        1e-3,
    );
    q = go(
        "descend to pick",
        &mut ctx,
        &mut cc,
        &plan_cfg,
        &q,
        &q_pick,
        1e-3,
    );
    ctx.grasped = true;
    rec.log("events", &rerun::TextLog::new("grasp")).ok();
    q = go(
        "retract",
        &mut ctx,
        &mut cc,
        &plan_cfg,
        &q,
        &q_pick_app,
        1e-3,
    );
    q = go_with_collision(
        "carry around pillar",
        &mut ctx,
        &plan_cfg,
        &q,
        &q_place_app,
        &mut |candidate| carry_cc.collides(candidate),
    );
    q = go(
        "descend to place",
        &mut ctx,
        &mut cc,
        &plan_cfg,
        &q,
        &q_place,
        1e-3,
    );
    ctx.grasped = false;
    ctx.cube_park = CUBE_REST_PLACE;
    set_cube(ctx.data, CUBE_REST_PLACE, rot);
    rec.log("events", &rerun::TextLog::new("release")).ok();
    q = go(
        "retract from place",
        &mut ctx,
        &mut cc,
        &plan_cfg,
        &q,
        &q_place_app,
        1e-3,
    );

    set_ctrl(ctx.data, &ACTUATORS, &q);
    for s in 0..SETTLE_STEPS {
        set_cube(ctx.data, CUBE_REST_PLACE, rot);
        gravity_compensate(ctx.data, &dof_adrs);
        ctx.data.step();
        if ctx.do_render && s.is_multiple_of(RENDER_EVERY) {
            capture_frame(ctx.renderer, ctx.data, ctx.frame_dir, ctx.frame_idx);
        }
    }

    println!(
        "[demo3] done · worst joint-track {:.4} rad",
        ctx.worst_track
    );
    if do_render {
        encode_gif(
            frame_dir,
            *ctx.frame_idx,
            Path::new("demo_output/demo3.gif"),
        );
    }
}

struct ExecCtx<'a, 'm> {
    data: &'a mut MjData<&'m MjModel>,
    chain: &'a Chain,
    rec: &'a rerun::RecordingStream,
    renderer: &'a mut Option<MjRenderer>,
    frame_dir: &'a Path,
    frame_idx: &'a mut usize,
    link_paths: &'a [String],
    dof_adrs: &'a [usize],
    dt: f64,
    limits: TrajLimits,
    grasped: bool,
    cube_park: [f64; 3],
    step_idx: usize,
    worst_track: f64,
    do_render: bool,
}

fn go(
    name: &str,
    ctx: &mut ExecCtx<'_, '_>,
    cc: &mut CollisionChecker<&MjModel>,
    plan_cfg: &PlanConfig,
    q_from: &[f64],
    q_to: &[f64],
    contact_threshold: f64,
) -> Vec<f64> {
    cc.contact_threshold = contact_threshold;
    let mut collides = |q: &[f64]| cc.collides(q);
    go_with_collision(name, ctx, plan_cfg, q_from, q_to, &mut collides)
}

fn go_with_collision(
    name: &str,
    ctx: &mut ExecCtx<'_, '_>,
    plan_cfg: &PlanConfig,
    q_from: &[f64],
    q_to: &[f64],
    collides: &mut impl FnMut(&[f64]) -> bool,
) -> Vec<f64> {
    let t0 = std::time::Instant::now();
    let plan = rrt_connect(ctx.chain, q_from, q_to, collides, plan_cfg);
    let plan_ms = t0.elapsed().as_secs_f64() * 1e3;
    assert_eq!(
        plan.status,
        PlanStatus::Success,
        "{name}: planner failed ({:?})",
        plan.status
    );
    let traj = time_parameterize(&plan.waypoints, &ctx.limits, ctx.dt);
    println!(
        "[demo3] {name}: plan {plan_ms:.1} ms · {} waypoints · S-curve {:.2} s",
        plan.waypoints.len(),
        traj.duration
    );
    ctx.rec
        .log("events", &rerun::TextLog::new(name.to_string()))
        .ok();

    let grasped = ctx.grasped;
    let cube_park = ctx.cube_park;
    for (i, q_des) in traj.q.iter().enumerate() {
        let qd_des = &traj.qd[i];
        let chain = ctx.chain;
        let err = traj_step(
            ctx.data,
            chain,
            &ACTUATORS,
            q_des,
            qd_des,
            KV_OVER_KP,
            ctx.dof_adrs,
            |data| {
                if grasped {
                    weld_cube(data, chain, q_des);
                } else {
                    set_cube(data, cube_park, fk(chain, q_des).rotation);
                }
            },
        );
        ctx.worst_track = ctx.worst_track.max(err);

        if ctx.do_render && i.is_multiple_of(RENDER_EVERY) {
            capture_frame(ctx.renderer, ctx.data, ctx.frame_dir, ctx.frame_idx);
        }
        if !i.is_multiple_of(LOG_EVERY) {
            continue;
        }
        let q_meas = read_q(ctx.data, ctx.chain);
        let (poses, ee) = arm_lab::kinematics::fk_full(ctx.chain, &q_meas);
        ctx.rec.set_time_sequence("step", ctx.step_idx as i64);
        for (pose, path) in poses.iter().zip(ctx.link_paths.iter()) {
            log_transform(ctx.rec, path, &pose.world);
        }
        log_transform(ctx.rec, "world/ee", &ee);
        ctx.rec
            .log("plot/joint_track_err_rad", &rerun::Scalars::single(err))
            .ok();
        ctx.step_idx += 1;
    }
    traj.q.last().cloned().unwrap_or_else(|| q_to.to_vec())
}

fn solve_named(
    name: &str,
    chain: &Chain,
    xyz: &[f64; 3],
    rot: UnitQuaternion<f64>,
    q_init: &[f64],
    cfg: &IkConfig,
    cc: &mut CollisionChecker<&MjModel>,
) -> Vec<f64> {
    let target = Isometry3::from_parts(Translation3::new(xyz[0], xyz[1], xyz[2]), rot);
    let ik = solve_ik(chain, &target, q_init, Some(q_init), cfg);
    assert!(
        ik.converged,
        "{name} IK failed: pos {:.3e} rot {:.3e}",
        ik.pos_err, ik.rot_err
    );
    assert!(!cc.collides(&ik.q), "{name} IK solution is in collision");
    println!(
        "[demo3] IK {name}: pos_err {:.2e} rot_err {:.2e} iters {}",
        ik.pos_err, ik.rot_err, ik.iterations
    );
    ik.q
}

fn weld_cube(data: &mut MjData<&MjModel>, chain: &Chain, q: &[f64]) {
    let ee = fk(chain, q);
    let p = ee.translation.vector + ee.rotation * CUBE_IN_EE;
    set_cube(data, [p.x, p.y, p.z], ee.rotation);
}

fn set_cube(data: &mut MjData<&MjModel>, pos: [f64; 3], rot: UnitQuaternion<f64>) {
    data.mocap_pos_mut()[0] = pos;
    data.mocap_quat_mut()[0] = [rot.w, rot.i, rot.j, rot.k];
}
