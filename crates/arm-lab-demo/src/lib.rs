//! Shared demo helpers: gravity compensation, Rerun logging, offscreen GIF.

use std::path::{Path, PathBuf};

use arm_lab::Chain;
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;
use nalgebra::Isometry3;
use rerun::{RecordingStream, RecordingStreamBuilder};

pub const RENDER_W: u32 = 960;
pub const RENDER_H: u32 = 540;
/// Sim steps between captured frames (MuJoCo runs 0.5 kHz; 25 steps = 20 fps).
pub const RENDER_EVERY: usize = 25;
pub const GIF_FPS: u32 = 20;

pub fn read_q(data: &MjData<&MjModel>, chain: &Chain) -> Vec<f64> {
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
/// Writing `qfrc_bias` into `qfrc_applied` cancels it.
pub fn gravity_compensate(data: &mut MjData<&MjModel>, dof_adrs: &[usize]) {
    for &d in dof_adrs {
        data.qfrc_applied_mut()[d] = data.qfrc_bias()[d];
    }
}

pub fn set_ctrl(data: &mut MjData<&MjModel>, names: &[&str], q: &[f64]) {
    for (act, &qi) in names.iter().zip(q.iter()) {
        data.actuator(act).unwrap().view_mut(data).ctrl[0] = qi;
    }
}

pub fn init_recording(
    app: &'static str,
    mode: &str,
    default_rrd: &str,
    rrd_path_override: Option<String>,
) -> RecordingStream {
    let builder = RecordingStreamBuilder::new(app);
    match mode {
        "--spawn" => builder.spawn().expect("failed to spawn rerun viewer"),
        "--connect" => builder
            .connect_grpc()
            .expect("failed to connect to rerun viewer"),
        _ => {
            let path = rrd_path_override.unwrap_or_else(|| default_rrd.to_string());
            let path = PathBuf::from(path);
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir).ok();
            }
            builder.save(&path).expect("failed to save recording")
        }
    }
}

pub fn capture_frame(
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

pub fn encode_gif(frame_dir: &Path, frame_count: usize, out: &Path) {
    if frame_count == 0 {
        eprintln!("no frames captured; skipping GIF");
        return;
    }
    let palette = frame_dir.join("palette.png");
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
        "GIF written: {} ({} frames, {:.1} MB)",
        out.display(),
        frame_count,
        size as f64 / 1e6
    );
}

pub fn log_transform(rec: &RecordingStream, path: &str, iso: &Isometry3<f64>) {
    let t = iso.translation;
    let q = iso.rotation;
    rec.log(
        path,
        &rerun::Transform3D::from_translation_rotation(
            [t.x, t.y, t.z],
            rerun::datatypes::Quaternion([q.i as f32, q.j as f32, q.k as f32, q.w as f32]),
        ),
    )
    .ok();
}

pub fn parse_args(args: &[String]) -> (bool, String, Option<String>) {
    let do_render = args.iter().any(|a| a == "--render");
    let mode = args
        .iter()
        .find(|a| a.as_str() == "--spawn" || a.as_str() == "--connect")
        .cloned()
        .unwrap_or_default();
    let rrd_path_override = args.iter().find(|a| !a.starts_with("--")).cloned();
    (do_render, mode, rrd_path_override)
}
