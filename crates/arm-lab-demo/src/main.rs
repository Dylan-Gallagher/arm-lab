//! Demo 1 — "the arm that moves itself".
//!
//! A static UR5e in MuJoCo steps its end effector through a sequence of
//! IK-solved target poses. Everything interesting happens in this binary:
//!
//! 1. the kinematic chain is extracted from the compiled MuJoCo model
//!    (offsets, axes, limits, EE site) — no hand-entered DH parameters;
//! 2. targets are solved with the in-repo damped-least-squares IK
//!    (position + orientation, joint limits, seeded restarts);
//! 3. the solved joint configuration is commanded to MuJoCo's position
//!    actuators, and the physics moves the real arm;
//! 4. Rerun records link transforms, the EE pose, targets, and live error
//!    plots (command tracking + Cartesian error).
//!
//! Run with `cargo run --release -p arm-lab-demo`, then open the recording:
//! `rerun demo_output/demo1.rrd` (or pass `--spawn` to launch the viewer
//! directly, or `--connect` to stream into a running one started with `rerun`).
//! Add `--render` to also capture the run offscreen (MuJoCo EGL) and encode
//! `demo_output/demo1.gif` with ffmpeg — no window needed.

use std::path::{Path, PathBuf};

use arm_lab::Chain;
use arm_lab::ik::{IkConfig, solve_ik};
use arm_lab::kinematics::fk;
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;
use nalgebra::{Isometry3, Vector3};
use rerun::{RecordingStream, RecordingStreamBuilder};

const SCENE_XML: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/ur5e/scene.xml");

const RENDER_W: u32 = 960;
const RENDER_H: u32 = 540;
/// Sim steps between captured frames (MuJoCo runs 0.5 kHz; 25 steps = 20 fps).
const RENDER_EVERY: usize = 25;
const GIF_FPS: u32 = 20;

/// Waypoints, relative to the home end-effector position, visited in order
/// (and back to start). Orientation is held at the home EE orientation.
const TARGET_OFFSETS: &[(f64, f64, f64)] = &[
    (0.00, 0.00, 0.00), // home
    (0.12, 0.00, 0.05),
    (0.12, -0.18, 0.05),
    (-0.08, -0.18, 0.22),
    (-0.08, 0.10, 0.15),
    (0.00, 0.00, 0.00), // return home
];

const SETTLE_SECONDS: f64 = 1.6;
const LOG_EVERY: usize = 2;

fn main() {
    // ---- CLI ------------------------------------------------------------
    let args: Vec<String> = std::env::args().skip(1).collect();
    let do_render = args.iter().any(|a| a == "--render");
    let mode = args
        .iter()
        .find(|a| a.as_str() == "--spawn" || a.as_str() == "--connect")
        .cloned()
        .unwrap_or_default();
    let rrd_path_override = args.iter().find(|a| !a.starts_with("--")).cloned();

    // ---- Sim + chain ------------------------------------------------------
    let model = MjModel::from_xml(SCENE_XML).expect("failed to load UR5e scene");
    let mut data = MjData::new(&model);
    let dt = model.opt().timestep;
    let chain = Chain::from_mujoco(&model, "ur5e", "wrist_3_link", "attachment_site")
        .expect("failed to extract chain");
    assert_eq!(chain.dof(), 6, "UR5e must be 6-DOF");

    let home_key = model
        .name_to_id(MjtObj::mjOBJ_KEY, "home")
        .expect("home keyframe");
    data.reset_keyframe(home_key).expect("reset to home");
    data.forward();

    let q_home = read_q(&data, &chain);
    let dof_adrs = chain.dof_addresses();
    let ee_home = fk(&chain, &q_home);
    println!(
        "[demo1] chain: {} links, {} DOF; home EE at {:.3}",
        chain.links().len(),
        chain.dof(),
        ee_home.translation.vector
    );

    // ---- Rerun sink --------------------------------------------------------
    let rec = init_recording(&mode, rrd_path_override);

    // ---- Offscreen renderer (optional) -------------------------------------
    let mut renderer = None;
    let frame_dir = Path::new("demo_output/frames");
    let mut frame_idx: usize = 0;
    if do_render {
        std::fs::create_dir_all(frame_dir).expect("create frame dir");
        for old in std::fs::read_dir(frame_dir).unwrap().flatten() {
            std::fs::remove_file(old.path()).ok();
        }
        let mut cam = MjvCamera::new_free(&model);
        cam.lookat = [-0.10, 0.15, 0.25];
        cam.azimuth = 132.0;
        cam.elevation = -18.0;
        cam.distance = 2.1;
        let r = MjRenderer::builder()
            .width(RENDER_W)
            .height(RENDER_H)
            .camera(cam)
            .build(&model)
            .expect("failed to init EGL offscreen renderer");
        renderer = Some(r);
        println!("[demo1] offscreen render: {RENDER_W}x{RENDER_H} @ {GIF_FPS} fps");
    }

    // Static scene context.
    rec.log(
        "world/targets/description",
        &rerun::TextLog::new(
            "UR5e · Demo 1 · FK+IK from scratch, commanded into MuJoCo position actuators",
        ),
    )
    .ok();

    let link_paths: Vec<String> = chain
        .links()
        .iter()
        .map(|l| format!("world/arm/{}", l.name))
        .collect();
    let joint_names: Vec<String> = chain.joint_names().into_iter().map(str::to_owned).collect();
    let actuator_names = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    ];

    // ---- Target sequence ---------------------------------------------------
    let mut step_idx: usize = 0;
    let mut report_rows: Vec<String> = Vec::new();
    let mut q_current = q_home.clone();
    let mut first = true;

    for (ti, off) in TARGET_OFFSETS.iter().enumerate() {
        let target = Isometry3::from_parts(
            (ee_home.translation.vector + Vector3::new(off.0, off.1, off.2)).into(),
            ee_home.rotation,
        );

        // Solve IK from the current configuration (continuity), home-biased.
        let t0 = std::time::Instant::now();
        let ik = solve_ik(
            &chain,
            &target,
            &q_current,
            Some(&q_home),
            &IkConfig::default(),
        );
        let solve_ms = t0.elapsed().as_secs_f64() * 1e3;
        q_current = ik.q.clone();

        log_transform(&rec, "world/target", &target);
        rec.log(
            "world/target/point",
            &rerun::Points3D::new([[
                target.translation.x,
                target.translation.y,
                target.translation.z,
            ]]),
        )
        .ok();
        rec.log(
            "events",
            &rerun::TextLog::new(format!(
                "target {ti}: IK {} in {solve_ms:.1} ms (pos err {:.2e} m, rot err {:.2e} rad)",
                if ik.converged { "converged" } else { "STALLED" },
                ik.pos_err,
                ik.rot_err
            )),
        )
        .ok();

        // Command and servo: hold the solved joints; the physics does the rest.
        for (act, &qi) in actuator_names.iter().zip(ik.q.iter()) {
            data.actuator(act).unwrap().view_mut(&mut data).ctrl[0] = qi;
        }
        if first {
            // Let the initial keyframe state settle before logging motion.
            for _ in 0..100 {
                gravity_compensate(&mut data, &dof_adrs);
                data.step();
            }
            first = false;
        }

        let steps = (SETTLE_SECONDS / dt) as usize;
        let (mut worst_track, mut final_pos_err) = (0.0f64, 0.0f64);
        for s in 0..steps {
            gravity_compensate(&mut data, &dof_adrs);
            data.step();
            if renderer.is_some() && s % RENDER_EVERY == 0 {
                capture_frame(&mut renderer, &mut data, frame_dir, &mut frame_idx);
            }
            if s % LOG_EVERY != 0 {
                continue;
            }
            let q_meas = read_q(&data, &chain);
            let (poses, ee) = arm_lab::kinematics::fk_full(&chain, &q_meas);

            // Stamp this frame BEFORE logging anything that belongs to it.
            rec.set_time_sequence("step", step_idx as i64);

            for (pose, path) in poses.iter().zip(link_paths.iter()) {
                log_transform(&rec, path, &pose.world);
            }
            log_transform(&rec, "world/ee", &ee);

            let track_err = {
                let d: f64 = q_meas
                    .iter()
                    .zip(ik.q.iter())
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt();
                d
            };
            let pos_err = (target.translation.vector - ee.translation.vector).norm();
            worst_track = worst_track.max(track_err);
            final_pos_err = pos_err;

            // Plots: one scalar per path; each joint gets its own series.
            rec.log("plot/cartesian_err_m", &rerun::Scalars::single(pos_err))
                .ok();
            rec.log(
                "plot/joint_track_err_rad",
                &rerun::Scalars::single(track_err),
            )
            .ok();
            rec.log(
                "plot/ee_height_m",
                &rerun::Scalars::single(ee.translation.z),
            )
            .ok();
            for ((name, &qi), &qc) in joint_names.iter().zip(q_meas.iter()).zip(ik.q.iter()) {
                rec.log(format!("plot/q/{name}"), &rerun::Scalars::single(qi))
                    .ok();
                rec.log(format!("plot/qcmd/{name}"), &rerun::Scalars::single(qc))
                    .ok();
            }
            step_idx += 1;
        }

        report_rows.push(format!(
            "  target {ti}: IK {:>9} {:>5.1} ms | worst joint-track {worst_track:.4} rad | final Cartesian err {final_pos_err:.4} m",
            if ik.converged { "converged" } else { "stalled" },
            solve_ms
        ));
    }

    println!("\n[demo1] target report:");
    for row in &report_rows {
        println!("{row}");
    }
    println!("\n[demo1] done: {} log frames. Recording saved.", step_idx);

    // ---- GIF assembly ------------------------------------------------------
    if do_render {
        encode_gif(frame_dir, frame_idx);
    }
}

fn read_q(data: &MjData<&MjModel>, chain: &Chain) -> Vec<f64> {
    chain
        .qpos_addresses()
        .iter()
        .map(|&adr| data.qpos()[adr])
        .collect()
}

/// Exact model-based gravity/Coriolis feedforward on the chain joints.
///
/// The Menagerie position actuators (kp=2000, kv=400) hold a steady-state
/// error of τ_gravity/kp under load — several millimeters at the tool.
/// Writing `qfrc_bias` (MuJoCo's C(q,v) term, gravity included) into
/// `qfrc_applied` cancels it, exactly as a real controller's gravity
/// compensation would, leaving the servo to track dynamics only.
/// `qfrc_bias` is read from the current state and applied for the coming
/// step, where the same bias term is recomputed from that same state.
fn gravity_compensate(data: &mut MjData<&MjModel>, dof_adrs: &[usize]) {
    for &d in dof_adrs {
        data.qfrc_applied_mut()[d] = data.qfrc_bias()[d];
    }
}

fn init_recording(mode: &str, rrd_path_override: Option<String>) -> RecordingStream {
    let builder = RecordingStreamBuilder::new("arm-lab demo1");
    match mode {
        "--spawn" => builder.spawn().expect("failed to spawn rerun viewer"),
        "--connect" => builder
            .connect_grpc()
            .expect("failed to connect to rerun viewer"),
        _ => {
            let path = rrd_path_override.unwrap_or_else(|| "demo_output/demo1.rrd".to_string());
            let path = PathBuf::from(path);
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir).ok();
            }
            builder.save(&path).expect("failed to save recording")
        }
    }
}

/// Offscreen-render the current simulation state and save one PNG frame.
fn capture_frame(
    renderer: &mut Option<MjRenderer>,
    data: &mut MjData<&MjModel>,
    frame_dir: &Path,
    frame_idx: &mut usize,
) {
    let r = renderer.as_mut().expect("renderer");
    r.sync_data(data).expect("sync scene");
    r.render().expect("render frame");
    let path = frame_dir.join(format!("f{:05}.png", *frame_idx));
    r.save_rgb(&path).expect("save frame");
    *frame_idx += 1;
}

/// Two-pass palette GIF via ffmpeg (system dependency).
fn encode_gif(frame_dir: &Path, frame_count: usize) {
    if frame_count == 0 {
        eprintln!("[demo1] no frames captured; skipping GIF");
        return;
    }
    let palette = frame_dir.join("palette.png");
    let out = Path::new("demo_output/demo1.gif");
    if let Some(dir) = out.parent() {
        std::fs::create_dir_all(dir).ok();
    }
    let frames_glob = frame_dir.join("f%05d.png");

    let status = std::process::Command::new("ffmpeg")
        .args(["-y", "-loglevel", "error"])
        .arg("-framerate")
        .arg(GIF_FPS.to_string())
        .arg("-i")
        .arg(&frames_glob)
        .args(["-vf", "palettegen=stats_mode=diff"])
        .arg(&palette)
        .status()
        .expect("failed to spawn ffmpeg (is it installed?)");
    assert!(status.success(), "palettegen pass failed");

    let status = std::process::Command::new("ffmpeg")
        .args(["-y", "-loglevel", "error"])
        .arg("-framerate")
        .arg(GIF_FPS.to_string())
        .arg("-i")
        .arg(&frames_glob)
        .arg("-i")
        .arg(&palette)
        .args([
            "-lavfi",
            "paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
        ])
        .arg(out)
        .status()
        .expect("failed to spawn ffmpeg");
    assert!(status.success(), "paletteuse pass failed");

    let size = std::fs::metadata(out).map(|m| m.len()).unwrap_or(0);
    println!(
        "[demo1] GIF written: {} ({} frames, {:.1} MB)",
        out.display(),
        frame_count,
        size as f64 / 1e6
    );
}

fn log_transform(rec: &RecordingStream, path: &str, iso: &Isometry3<f64>) {
    let t = iso.translation;
    let q = iso.rotation;
    // Rerun quaternions are [x, y, z, w] (nalgebra's q.i/q.j/q.k are x/y/z).
    rec.log(
        path,
        &rerun::Transform3D::from_translation_rotation(
            [t.x, t.y, t.z],
            rerun::datatypes::Quaternion([q.i as f32, q.j as f32, q.k as f32, q.w as f32]),
        ),
    )
    .ok();
}
