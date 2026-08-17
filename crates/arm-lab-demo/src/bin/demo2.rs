//! Demo 2 — "the arm that goes around".
//!
//! Joint-space RRT-Connect plans a collision-free path for the UR5e around a
//! pillar that blocks the straight-line interpolant from home to a panned
//! goal. The shortcut edges are timed with rest-to-rest, jerk-bounded S-curves
//! and tracked with a gravity-compensated PD servo (velocity feedforward
//! through the Menagerie position actuators).
//!
//! ```text
//! cargo run --release -p arm-lab-demo --bin demo2
//! cargo run --release -p arm-lab-demo --bin demo2 -- --render
//! ```

use std::path::Path;

use arm_lab::kinematics::fk;
use arm_lab::plan::{PlanStatus, edge_free, rrt_connect};
use arm_lab::traj::{TrajLimits, time_parameterize};
use arm_lab::{Chain, CollisionChecker, PlanConfig};
use arm_lab_demo::{
    GIF_FPS, RENDER_EVERY, RENDER_H, RENDER_W, capture_frame, encode_gif, gravity_compensate,
    init_recording, log_transform, parse_args, read_q, set_ctrl, set_ctrl_pd,
};
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;

const SCENE_XML: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/ur5e/scene_cluttered.xml"
);

const ACTUATORS: [&str; 6] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
];

const KV_OVER_KP: f64 = 0.2; // Menagerie size3 and size1 both have kv/kp = 0.2
const LOG_EVERY: usize = 2;
const SETTLE_STEPS: usize = 150;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (do_render, mode, rrd_path_override) = parse_args(&args);

    let model = MjModel::from_xml(SCENE_XML).expect("failed to load cluttered UR5e scene");
    let mut data = MjData::new(&model);
    let dt = model.opt().timestep;
    let chain = Chain::from_mujoco(&model, "ur5e", "wrist_3_link", "attachment_site")
        .expect("failed to extract chain");

    let home_key = model
        .name_to_id(MjtObj::mjOBJ_KEY, "home")
        .expect("home keyframe");
    data.reset_keyframe(home_key).expect("reset to home");
    data.forward();

    let q_start = read_q(&data, &chain);
    let mut q_goal = q_start.clone();
    q_goal[0] += 1.1; // shoulder_pan: the straight interpolant hits the pillar
    let dof_adrs = chain.dof_addresses();

    // ---- Plan --------------------------------------------------------------
    let mut cc = CollisionChecker::new(&model, &chain);
    let mut scratch = vec![0.0; chain.dof()];
    let mut collides = |q: &[f64]| cc.collides(q);
    let blocked = !edge_free(&q_start, &q_goal, 0.05, &mut collides, &mut scratch);
    assert!(
        blocked,
        "demo2 fixture broken: home→goal straight line does not hit the pillar"
    );

    let cfg = PlanConfig {
        seed: 20260816,
        ..PlanConfig::default()
    };
    let mut cc = CollisionChecker::new(&model, &chain);
    let t0 = std::time::Instant::now();
    let plan = rrt_connect(&chain, &q_start, &q_goal, |q| cc.collides(q), &cfg);
    let plan_ms = t0.elapsed().as_secs_f64() * 1e3;
    assert_eq!(
        plan.status,
        PlanStatus::Success,
        "RRT-Connect failed to dodge the pillar"
    );
    println!(
        "[demo2] planned in {plan_ms:.1} ms · {} iters · {} nodes · {} waypoints (shortcut removed {}) · cost {:.3} rad · {} densified",
        plan.iterations,
        plan.nodes,
        plan.waypoints.len(),
        plan.shortcut_removed,
        plan.cost,
        plan.path.len()
    );

    let limits = TrajLimits {
        v_max: 0.55,
        a_max: 1.8,
        j_max: 8.0,
    };
    let traj = time_parameterize(&plan.waypoints, &limits, dt);
    println!(
        "[demo2] S-curve: {:.2} s · {} samples @ dt={dt} · limits v≤{} a≤{} j≤{}",
        traj.duration,
        traj.len(),
        limits.v_max,
        limits.a_max,
        limits.j_max
    );

    // ---- Rerun + renderer --------------------------------------------------
    let rec = init_recording(
        "arm-lab demo2",
        &mode,
        "demo_output/demo2.rrd",
        rrd_path_override,
    );
    rec.log(
        "world/description",
        &rerun::TextLog::new(
            "UR5e · Demo 2 · RRT-Connect around a pillar, from scratch, no ROS/MoveIt",
        ),
    )
    .ok();
    rec.log(
        "events",
        &rerun::TextLog::new(format!(
            "plan: {plan_ms:.1} ms, {} waypoints after shortcut, cost {:.3} rad",
            plan.waypoints.len(),
            plan.cost
        )),
    )
    .ok();

    let planned_ee: Vec<[f32; 3]> = plan
        .path
        .iter()
        .map(|q| {
            let p = fk(&chain, q).translation.vector;
            [p.x as f32, p.y as f32, p.z as f32]
        })
        .collect();
    rec.log(
        "world/planned_ee",
        &rerun::LineStrips3D::new([planned_ee.clone()]),
    )
    .ok();
    rec.log(
        "world/planned_ee/samples",
        &rerun::Points3D::new(planned_ee),
    )
    .ok();

    let mut renderer = None;
    let frame_dir = Path::new("demo_output/frames2");
    let mut frame_idx: usize = 0;
    if do_render {
        std::fs::create_dir_all(frame_dir).expect("create frame dir");
        for old in std::fs::read_dir(frame_dir).unwrap().flatten() {
            std::fs::remove_file(old.path()).ok();
        }
        let mut cam = MjvCamera::new_free(&model);
        // Look at the pillar / swept workspace (home EE at ~(-0.13, 0.49, 0.49)).
        cam.lookat = [-0.30, 0.28, 0.28];
        cam.azimuth = 145.0;
        cam.elevation = -16.0;
        cam.distance = 2.2;
        let r = MjRenderer::builder()
            .width(RENDER_W)
            .height(RENDER_H)
            .camera(cam)
            .build(&model)
            .expect("failed to init EGL offscreen renderer");
        renderer = Some(r);
        println!("[demo2] offscreen render: {RENDER_W}x{RENDER_H} @ {GIF_FPS} fps");
    }

    let link_paths: Vec<String> = chain
        .links()
        .iter()
        .map(|l| format!("world/arm/{}", l.name))
        .collect();

    // ---- Execute -----------------------------------------------------------
    set_ctrl(&mut data, &ACTUATORS, &q_start);
    for _ in 0..SETTLE_STEPS {
        gravity_compensate(&mut data, &dof_adrs);
        data.step();
    }

    let mut step_idx: usize = 0;
    let mut worst_track = 0.0f64;
    let mut peak_qd = 0.0f64;

    for (i, q_des) in traj.q.iter().enumerate() {
        let qd_des = &traj.qd[i];
        set_ctrl_pd(&mut data, &ACTUATORS, q_des, qd_des, KV_OVER_KP);
        gravity_compensate(&mut data, &dof_adrs);
        data.step();

        if renderer.is_some() && i.is_multiple_of(RENDER_EVERY) {
            capture_frame(&mut renderer, &mut data, frame_dir, &mut frame_idx);
        }
        if !i.is_multiple_of(LOG_EVERY) {
            continue;
        }
        let q_meas = read_q(&data, &chain);
        let (poses, ee) = arm_lab::kinematics::fk_full(&chain, &q_meas);
        rec.set_time_sequence("step", step_idx as i64);
        for (pose, path) in poses.iter().zip(link_paths.iter()) {
            log_transform(&rec, path, &pose.world);
        }
        log_transform(&rec, "world/ee", &ee);
        rec.log(
            "world/ee/point",
            &rerun::Points3D::new([[ee.translation.x, ee.translation.y, ee.translation.z]]),
        )
        .ok();

        let track_err: f64 = q_meas
            .iter()
            .zip(q_des.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        worst_track = worst_track.max(track_err);
        let speed: f64 = qd_des.iter().map(|v| v * v).sum::<f64>().sqrt();
        peak_qd = peak_qd.max(speed);
        rec.log(
            "plot/joint_track_err_rad",
            &rerun::Scalars::single(track_err),
        )
        .ok();
        rec.log("plot/qd_norm_rad_s", &rerun::Scalars::single(speed))
            .ok();
        rec.log(
            "plot/ee_height_m",
            &rerun::Scalars::single(ee.translation.z),
        )
        .ok();
        rec.log("plot/ee_x_m", &rerun::Scalars::single(ee.translation.x))
            .ok();
        rec.log("plot/ee_y_m", &rerun::Scalars::single(ee.translation.y))
            .ok();
        step_idx += 1;
    }

    // Hold the goal so the GIF ends on a still frame.
    set_ctrl(&mut data, &ACTUATORS, &q_goal);
    for s in 0..SETTLE_STEPS {
        gravity_compensate(&mut data, &dof_adrs);
        data.step();
        if renderer.is_some() && s.is_multiple_of(RENDER_EVERY) {
            capture_frame(&mut renderer, &mut data, frame_dir, &mut frame_idx);
        }
    }

    let q_final = read_q(&data, &chain);
    let goal_err: f64 = q_final
        .iter()
        .zip(q_goal.iter())
        .map(|(a, b)| (a - b).powi(2))
        .sum::<f64>()
        .sqrt();
    println!(
        "[demo2] executed {} samples ({:.2} s) · peak ‖qd‖ {peak_qd:.3} rad/s · worst joint-track {worst_track:.4} rad · final goal err {goal_err:.4} rad",
        traj.len(),
        traj.duration
    );

    if do_render {
        encode_gif(frame_dir, frame_idx, Path::new("demo_output/demo2.gif"));
    }
}
