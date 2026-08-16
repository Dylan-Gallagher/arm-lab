//! Forward kinematics, from scratch: chain transforms via `nalgebra`.
//!
//! For each link the composition is, exactly mirroring MuJoCo's own
//! kinematics for single-hinge bodies:
//!
//! ```text
//! T_i = T_{i-1} · Iso(body_pos, body_quat) · Iso(anchor) · Rot(axis, q_i) · Iso(-anchor)
//! ```
//!
//! and the end-effector pose is `T_last · tool`. The tests verify agreement
//! with MuJoCo's computed `xpos`/`xmat` to floating-point tolerance, and with
//! the independent `k` crate's FK.

use nalgebra::{Isometry3, Translation3, UnitQuaternion};

use crate::chain::Chain;

/// Per-link intermediate state of a forward-kinematics pass: the link frame
/// *after* its joint rotation, and — for links with a joint — the joint's
/// world-frame anchor and axis *before* its rotation. The latter pair is what
/// the geometric Jacobian needs.
#[derive(Debug, Clone)]
pub struct LinkPose {
    pub name: String,
    /// World transform of this link's frame (post joint rotation).
    pub world: Isometry3<f64>,
    /// World-frame (anchor, axis) of this link's joint, if it has one.
    pub joint_anchor_axis: Option<(
        nalgebra::Point3<f64>,
        nalgebra::Unit<nalgebra::Vector3<f64>>,
    )>,
}

/// Full forward kinematics: returns every link's world pose plus the
/// end-effector pose, with the joint anchor/axis side data the Jacobian needs.
pub fn fk_full(chain: &Chain, q: &[f64]) -> (Vec<LinkPose>, Isometry3<f64>) {
    let mut poses = Vec::with_capacity(chain.links().len());
    let mut t = Isometry3::identity();
    let mut qi = 0;
    for link in chain.links() {
        // Static transform of this link relative to the parent.
        let static_iso = Isometry3::from_parts(Translation3::from(link.translation), link.rotation);
        let pre = t * static_iso;

        let joint_anchor_axis = link.joint.as_ref().map(|j| {
            let anchor_w = pre.transform_point(&j.anchor);
            let axis_w = pre.rotation.transform_vector(&j.axis);
            let axis_w = nalgebra::Unit::new_normalize(axis_w);
            (anchor_w, axis_w)
        });

        let post = match link.joint.as_ref() {
            Some(j) => {
                let q_i = q[qi];
                qi += 1;
                let anchor_iso = Isometry3::from_parts(
                    Translation3::from(j.anchor.coords),
                    UnitQuaternion::identity(),
                );
                let rot = Isometry3::from_parts(
                    Translation3::identity(),
                    UnitQuaternion::from_axis_angle(&j.axis, q_i),
                );
                pre * anchor_iso * rot * anchor_iso.inverse()
            }
            None => pre,
        };

        poses.push(LinkPose {
            name: link.name.clone(),
            world: post,
            joint_anchor_axis,
        });
        t = post;
    }
    let ee = t * chain.tool;
    (poses, ee)
}

/// End-effector pose only.
pub fn fk(chain: &Chain, q: &[f64]) -> Isometry3<f64> {
    fk_full(chain, q).1
}

/// World pose of a named link (post joint rotation), for visualization.
pub fn fk_link(chain: &Chain, q: &[f64], link_name: &str) -> Option<Isometry3<f64>> {
    let (poses, _) = fk_full(chain, q);
    poses
        .into_iter()
        .find(|p| p.name == link_name)
        .map(|p| p.world)
}
