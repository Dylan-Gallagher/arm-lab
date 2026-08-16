//! FK correctness gate #2: our FK vs the independent `k` crate (openrr),
//! driven through a URDF exported from the extracted chain. Two independent
//! implementations, same robot, must agree.

mod common;

use arm_lab::kinematics::fk;
use common::{Rng, random_q, ur5e_chain};
use nalgebra::UnitQuaternion;

fn k_serial_chain() -> k::SerialChain<f64> {
    let chain = ur5e_chain();
    let urdf_robot =
        urdf_rs::read_from_string(&chain.to_urdf_string()).expect("generated URDF failed to parse");
    let k_chain = k::Chain::from(&urdf_robot);
    let end = k_chain
        .find_link("ee_tool")
        .expect("tool link present in generated URDF");
    k::SerialChain::from_end(end)
}

#[test]
fn fk_matches_k_crate() {
    let chain = ur5e_chain();
    let serial = k_serial_chain();

    assert_eq!(serial.dof(), chain.dof(), "k chain DOF mismatch");

    let mut rng = Rng::new(0x5EED_0002);
    let mut worst_pos = 0.0f64;
    let mut worst_rot = 0.0f64;

    for _ in 0..50 {
        let q = random_q(&chain, &mut rng);
        let ours = fk(&chain, &q);

        serial
            .set_joint_positions(&q)
            .expect("k rejected joint positions");
        serial.update_transforms();
        let theirs = serial.end_transform();

        // k ships its own nalgebra version; cross the boundary by coordinates.
        let tp = theirs.translation.vector;
        let tq = theirs.rotation;
        let d = (ours.translation.vector - nalgebra::Vector3::new(tp.x, tp.y, tp.z)).norm();
        let rel = ours.rotation
            * UnitQuaternion::new_normalize(nalgebra::Quaternion::new(tq.w, tq.i, tq.j, tq.k))
                .inverse();
        let r = 2.0 * rel.imag().norm().atan2(rel.w.abs());
        worst_pos = worst_pos.max(d);
        worst_rot = worst_rot.max(r);
    }

    println!("k cross-check: worst |Δp| = {worst_pos:.3e} m, worst Δθ = {worst_rot:.3e} rad");
    assert!(
        worst_pos < 1e-9,
        "position disagreement with k: {worst_pos:e} m"
    );
    assert!(
        worst_rot < 1e-9,
        "orientation disagreement with k: {worst_rot:e} rad"
    );
}

#[test]
fn generated_urdf_joint_limits_round_trip() {
    let chain = ur5e_chain();
    let serial = k_serial_chain();

    let ours: Vec<(f64, f64)> = chain
        .joint_limits()
        .into_iter()
        .map(|l| l.expect("UR5e joints are all limited"))
        .collect();
    let theirs: Vec<Option<(f64, f64)>> = serial
        .iter_joints()
        .map(|j| j.limits.as_ref().map(|r| (r.min, r.max)))
        .collect();

    // k serializes root-first like us; compare actuated joints only.
    let mut i_ours = 0;
    for (i, t) in theirs.iter().enumerate() {
        if t.is_none() {
            continue; // fixed joint
        }
        let (lo_o, hi_o) = ours[i_ours];
        let (lo_t, hi_t) = t.unwrap();
        assert!(
            (lo_o - lo_t).abs() < 1e-9 && (hi_o - hi_t).abs() < 1e-9,
            "joint {i} limits disagree: ours {lo_o}..{hi_o}, k {lo_t}..{hi_t}"
        );
        i_ours += 1;
    }
    assert_eq!(i_ours, ours.len(), "actuated joint count mismatch");
}
