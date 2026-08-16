//! Probe IK + collision for candidate pick/place poses on `scene_pickplace.xml`.
//!
//! `cargo run --release -p arm-lab --example probe_pickplace`

use arm_lab::ik::{IkConfig, solve_ik};
use arm_lab::kinematics::fk;
use arm_lab::{Chain, CollisionChecker};
use mujoco_rs::prelude::*;
use nalgebra::{Isometry3, Translation3};

const SCENE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/ur5e/scene_pickplace.xml"
);

fn main() {
    let model = MjModel::from_xml(SCENE).unwrap();
    let mut data = MjData::new(&model);
    let chain = Chain::from_mujoco(&model, "ur5e", "wrist_3_link", "attachment_site").unwrap();
    let home_key = model.name_to_id(MjtObj::mjOBJ_KEY, "home").unwrap();
    data.reset_keyframe(home_key).unwrap();
    data.forward();

    let q_home: Vec<f64> = chain
        .qpos_addresses()
        .iter()
        .map(|&adr| data.qpos()[adr])
        .collect();
    let ee = fk(&chain, &q_home);
    let r = ee.rotation.to_rotation_matrix();
    let z = r.matrix().column(2);
    println!(
        "home EE pos {:.4}  z-axis ({:.3},{:.3},{:.3})",
        ee.translation.vector, z.x, z.y, z.z
    );

    let mut cc = CollisionChecker::new(&model, &chain);
    println!(
        "home collides={}  robot geoms={}",
        cc.collides(&q_home),
        cc.robot_geom_count()
    );
    for c in cc.data().contact() {
        if c.dist >= 1e-3 {
            continue;
        }
        let n1 = model
            .id_to_name(MjtObj::mjOBJ_GEOM, c.geom[0] as usize)
            .unwrap_or("?");
        let n2 = model
            .id_to_name(MjtObj::mjOBJ_GEOM, c.geom[1] as usize)
            .unwrap_or("?");
        println!("  contact {n1} vs {n2} dist={:.4}", c.dist);
    }

    let cfg = IkConfig {
        seed: 20260816,
        ..IkConfig::default()
    };

    let candidates: &[(&str, f64, f64, f64)] = &[
        ("pick_approach", -0.24, 0.58, 0.52),
        ("pick", -0.24, 0.58, 0.42),
        ("place_approach", 0.22, 0.58, 0.52),
        ("place", 0.22, 0.58, 0.42),
        ("carry_over", 0.00, 0.58, 0.85),
    ];

    let mut qs = Vec::new();
    for &(name, x, y, zpos) in candidates {
        let target = Isometry3::from_parts(Translation3::new(x, y, zpos), ee.rotation);
        let ik = solve_ik(&chain, &target, &q_home, Some(&q_home), &cfg);
        let hit = ik.converged && cc.collides(&ik.q);
        println!(
            "{name:<16} conv={} pos {:.2e} rot {:.2e} iters {:>3} collides={hit}",
            ik.converged, ik.pos_err, ik.rot_err, ik.iterations
        );
        if ik.converged && cc.collides(&ik.q) {
            for c in cc.data().contact() {
                if c.dist >= 1e-3 {
                    continue;
                }
                let n1 = model
                    .id_to_name(MjtObj::mjOBJ_GEOM, c.geom[0] as usize)
                    .unwrap_or("?");
                let n2 = model
                    .id_to_name(MjtObj::mjOBJ_GEOM, c.geom[1] as usize)
                    .unwrap_or("?");
                println!("    {n1} vs {n2} dist={:.4}", c.dist);
            }
        }
        if ik.converged {
            qs.push((name, ik.q));
        }
    }

    if qs.len() >= 4 {
        let pick_a = &qs.iter().find(|(n, _)| *n == "pick_approach").unwrap().1;
        let place_a = &qs.iter().find(|(n, _)| *n == "place_approach").unwrap().1;
        let mut scratch = vec![0.0; chain.dof()];
        let mut collides = |q: &[f64]| cc.collides(q);
        let open = arm_lab::plan::edge_free(pick_a, place_a, 0.05, &mut collides, &mut scratch);
        println!("pick_approach → place_approach straight edge free={open}");
    }
}
