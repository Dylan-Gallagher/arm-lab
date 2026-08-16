//! Print EE positions and collision flags along a shoulder-pan sweep from home.
//! `cargo run --release -p arm-lab --example probe_collision`

use std::f64::consts::FRAC_PI_2;

use arm_lab::kinematics::fk;
use arm_lab::{Chain, CollisionChecker};
use mujoco_rs::prelude::*;

const SCENE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/ur5e/scene_cluttered.xml"
);

fn main() {
    let model = MjModel::from_xml(SCENE).unwrap();
    let chain = Chain::from_mujoco(&model, "ur5e", "wrist_3_link", "attachment_site").unwrap();
    let mut cc = CollisionChecker::new(&model, &chain);
    println!("robot collision geoms: {}", cc.robot_geom_count());

    let mut q = vec![
        -FRAC_PI_2, -FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2, -FRAC_PI_2, 0.0,
    ];
    println!("home collides: {}", cc.collides(&q));
    for k in 0..=20 {
        q[0] = -FRAC_PI_2 + 1.4 * (k as f64 / 20.0);
        let ee = fk(&chain, &q).translation.vector;
        let hit = cc.collides(&q);
        println!(
            "k={k:02} pan={:+.3} EE=({:+.3},{:+.3},{:+.3}) collides={hit}",
            q[0], ee.x, ee.y, ee.z
        );
    }
}
