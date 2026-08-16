//! FK correctness gate #1: our from-scratch FK must agree with MuJoCo's own
//! computed body and site poses, on random configurations, to float tolerance.

mod common;

use arm_lab::kinematics::fk_full;
use common::{Rng, random_q, set_qpos, ur5e_chain};
use nalgebra::{Rotation3, UnitQuaternion};

/// Rotation angle between two rotations, robust at identity (atan2 form,
/// unlike `Rotation3::angle()` which NaNs at exactly 0).
fn angle_between(a: &Rotation3<f64>, b: &Rotation3<f64>) -> f64 {
    let rel = UnitQuaternion::from_rotation_matrix(&(a * b.transpose()));
    let (mut w, v) = (rel.w, rel.imag().norm());
    if w < 0.0 {
        w = -w;
    }
    2.0 * v.atan2(w)
}

fn mat_from_row_major(m: &[f64]) -> Rotation3<f64> {
    debug_assert_eq!(m.len(), 9);
    Rotation3::from_matrix_unchecked(nalgebra::Matrix3::new(
        m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8],
    ))
}

#[test]
fn fk_matches_mujoco_bodies_and_site() {
    let chain = ur5e_chain();
    let model = common::ur5e_model();
    let mut data = mujoco_rs::prelude::MjData::new(&model);
    let mut rng = Rng::new(0xC0FFEE);

    for case in 0..50 {
        let q = random_q(&chain, &mut rng);
        set_qpos(&mut data, &chain, &q);
        data.forward();

        let (poses, ee) = fk_full(&chain, &q);

        // Every link frame vs MuJoCo xpos/xmat.
        for pose in &poses {
            let view = data.body(&pose.name).unwrap().view(&data);
            let mj_t = nalgebra::Vector3::new(view.xpos[0], view.xpos[1], view.xpos[2]);
            let mj_r = mat_from_row_major(&view.xmat);
            let d = (pose.world.translation.vector - mj_t).norm();
            let r_err = angle_between(&pose.world.rotation.to_rotation_matrix(), &mj_r);
            assert!(
                d < 1e-9 && r_err < 1e-9,
                "case {case}: link {} disagrees with MuJoCo: |Δp|={d:e}, Δθ={r_err:e}",
                pose.name
            );
        }

        // End-effector site vs MuJoCo site xpos/xmat.
        let site = data.site("attachment_site").unwrap().view(&data);
        let site_t = nalgebra::Vector3::new(site.xpos[0], site.xpos[1], site.xpos[2]);
        let site_r = mat_from_row_major(&site.xmat);
        let d = (ee.translation.vector - site_t).norm();
        let r_err = angle_between(&ee.rotation.to_rotation_matrix(), &site_r);
        assert!(
            d < 1e-9 && r_err < 1e-9,
            "case {case}: EE site disagrees with MuJoCo: |Δp|={d:e}, Δθ={r_err:e}"
        );
    }
}

#[test]
fn home_keyframe_matches() {
    // The MJCF keyframe "home" — sanity on one known configuration.
    let chain = ur5e_chain();
    let model = common::ur5e_model();
    let mut data = mujoco_rs::prelude::MjData::new(&model);
    let home_id = model
        .name_to_id(mujoco_rs::prelude::MjtObj::mjOBJ_KEY, "home")
        .unwrap();
    data.reset_keyframe(home_id).expect("home keyframe exists");
    data.forward();

    let q: Vec<f64> = chain
        .qpos_addresses()
        .iter()
        .map(|&adr| data.qpos()[adr])
        .collect();
    let (_, ee) = fk_full(&chain, &q);
    let site = data.site("attachment_site").unwrap().view(&data);
    let site_t = nalgebra::Vector3::new(site.xpos[0], site.xpos[1], site.xpos[2]);
    let d = (ee.translation.vector - site_t).norm();
    assert!(d < 1e-9, "home keyframe EE mismatch: {d:e} m");

    println!(
        "UR5e home EE position: {:.4} m (matches MuJoCo to {d:e})",
        ee.translation.vector
    );
}
