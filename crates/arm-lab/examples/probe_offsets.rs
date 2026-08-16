//! Dev probe: check which demo-1 waypoint offsets are IK-reachable with the
//! home orientation held, and print convergence stats. Not part of the demo.
//!
//! `cargo run --release -p arm-lab --example probe_offsets`

use arm_lab::Chain;
use arm_lab::ik::{IkConfig, solve_ik};
use arm_lab::kinematics::fk;
use mujoco_rs::prelude::*;

const SCENE_XML: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/ur5e/scene.xml");

fn main() {
    let model = MjModel::from_xml(SCENE_XML).expect("load scene");
    let mut data = MjData::new(&model);
    let chain =
        Chain::from_mujoco(&model, "ur5e", "wrist_3_link", "attachment_site").expect("chain");

    let home_key = model.name_to_id(MjtObj::mjOBJ_KEY, "home").expect("key");
    data.reset_keyframe(home_key).unwrap();
    data.forward();

    let q_home: Vec<f64> = chain
        .qpos_addresses()
        .iter()
        .map(|&adr| data.qpos()[adr])
        .collect();
    let ee_home = fk(&chain, &q_home);
    println!("home EE: {:.4}", ee_home.translation);

    let candidates: &[(f64, f64, f64)] = &[
        (0.00, 0.00, 0.00),
        (0.12, 0.00, 0.05),
        (0.12, -0.18, 0.05),
        (-0.08, -0.18, 0.22),
        (-0.08, 0.10, 0.22),
        (-0.08, 0.10, 0.15),
        (-0.04, 0.08, 0.18),
        (0.06, 0.12, 0.15),
        (0.00, 0.14, 0.10),
    ];

    let cfg = IkConfig::default();
    let mut q_cur = q_home.clone();
    for &off in candidates {
        let target = nalgebra::Isometry3::from_parts(
            (ee_home.translation.vector + nalgebra::Vector3::new(off.0, off.1, off.2)).into(),
            ee_home.rotation,
        );
        let t0 = std::time::Instant::now();
        let ik = solve_ik(&chain, &target, &q_cur, Some(&q_home), &cfg);
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        println!(
            "offset ({:+.2},{:+.2},{:+.2}): {:>9} pos_err {:.2e} rot_err {:.2e} iters {:>3} restarts_used {:>2} {ms:6.1} ms",
            off.0,
            off.1,
            off.2,
            if ik.converged { "converged" } else { "STALLED" },
            ik.pos_err,
            ik.rot_err,
            ik.iterations,
            0,
        );
        q_cur = ik.q.clone();
    }
}
