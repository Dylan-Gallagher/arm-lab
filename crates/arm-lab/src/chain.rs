//! Robot chain representation and extraction from a compiled MuJoCo model.

use std::fmt;

use mujoco_rs::prelude::*;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, Unit, UnitQuaternion, Vector3};

/// A single revolute joint of the chain.
#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,
    /// Rotation axis, expressed in the link's local frame (after the link's
    /// static transform, before the joint rotation).
    pub axis: Unit<Vector3<f64>>,
    /// Joint anchor point in the link's local frame (MuJoCo `jnt_pos`).
    pub anchor: Point3<f64>,
    /// Position limits `(lower, upper)` in radians, if the joint is limited.
    pub limits: Option<(f64, f64)>,
    /// Address of this joint's scalar in the MuJoCo `qpos` vector, kept so a
    /// live simulation state can be read directly into `Vec<f64>` joint
    /// coordinates for this chain.
    pub qpos_adr: usize,
    /// Address of this joint's scalar in the MuJoCo velocity/force (`nv`)
    /// vectors (`qvel`, `qfrc_bias`, `qfrc_applied`). Equal to `qpos_adr`
    /// when all model joints are hinges, but not in general (ball/free
    /// joints elsewhere in the model shift it).
    pub dof_adr: usize,
}

/// One link of the chain: a static transform relative to the parent link,
/// plus optionally the joint that moves this link.
#[derive(Debug, Clone)]
pub struct Link {
    pub name: String,
    /// Static offset of this link's frame in the parent link's frame
    /// (MuJoCo `body_pos` / `body_quat`).
    pub translation: Vector3<f64>,
    pub rotation: UnitQuaternion<f64>,
    /// The single hinge joint of this link, if any. Links without a joint are
    /// rigid offsets (like the UR5e base).
    pub joint: Option<Joint>,
    /// MuJoCo body id this link came from (diagnostics / logging).
    pub body_id: usize,
}

/// A serial kinematic chain: `links[0]` is the root (world), each subsequent
/// link hangs off the previous one. The end-effector frame is `tool`, fixed
/// relative to the last link (extracted from a MuJoCo site).
#[derive(Debug, Clone)]
pub struct Chain {
    pub name: String,
    links: Vec<Link>,
    /// End-effector frame relative to the last link.
    pub tool: Isometry3<f64>,
}

/// Error type for chain extraction.
#[derive(Debug)]
pub enum ChainError {
    UnknownBody(String),
    UnknownSite(String),
    SiteNotOnTip {
        site: String,
        site_body: usize,
        tip_body: usize,
    },
    MultipleJointsOnBody {
        body: String,
        count: usize,
    },
    UnsupportedJointType {
        joint: String,
        kind: String,
    },
}

impl fmt::Display for ChainError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownBody(n) => write!(f, "body '{n}' not found in model"),
            Self::UnknownSite(n) => write!(f, "site '{n}' not found in model"),
            Self::SiteNotOnTip {
                site,
                site_body,
                tip_body,
            } => write!(
                f,
                "site '{site}' (body {site_body}) is not on the tip body ({tip_body})"
            ),
            Self::MultipleJointsOnBody { body, count } => write!(
                f,
                "body '{body}' has {count} joints; only 0 or 1 hinge joints per body are supported"
            ),
            Self::UnsupportedJointType { joint, kind } => {
                write!(
                    f,
                    "joint '{joint}' has unsupported type '{kind}' (only hinges)"
                )
            }
        }
    }
}

impl std::error::Error for ChainError {}

impl Chain {
    /// Extract the serial chain from a compiled MuJoCo model.
    ///
    /// Walks from `tip_body` up through `body_parentid` to the world body,
    /// collecting each body's static transform and its (single) hinge joint.
    /// The end-effector frame is taken from `ee_site`, which must be attached
    /// to `tip_body`.
    ///
    /// This keeps the chain exactly consistent with what MuJoCo simulates:
    /// the same offsets, axes, anchors, and limits the physics uses.
    pub fn from_mujoco(
        model: &MjModel,
        name: &str,
        tip_body: &str,
        ee_site: &str,
    ) -> Result<Self, ChainError> {
        let tip_id = model
            .name_to_id(MjtObj::mjOBJ_BODY, tip_body)
            .ok_or_else(|| ChainError::UnknownBody(tip_body.to_string()))?;
        let site_id = model
            .name_to_id(MjtObj::mjOBJ_SITE, ee_site)
            .ok_or_else(|| ChainError::UnknownSite(ee_site.to_string()))?;
        let site_body = model.site_bodyid()[site_id] as usize;
        if site_body != tip_id {
            return Err(ChainError::SiteNotOnTip {
                site: ee_site.to_string(),
                site_body,
                tip_body: tip_id,
            });
        }

        // Collect body ids from tip to world, then reverse.
        let mut path = Vec::new();
        let mut b = tip_id;
        while b != 0 {
            path.push(b);
            b = model.body_parentid()[b] as usize;
        }
        path.push(0); // world body: identity root
        path.reverse();

        let mut links = Vec::with_capacity(path.len());
        for &body in &path {
            let body_name = model
                .id_to_name(MjtObj::mjOBJ_BODY, body)
                .unwrap_or_default()
                .to_string();

            let jnt_num = model.body_jntnum()[body] as usize;
            if jnt_num > 1 {
                return Err(ChainError::MultipleJointsOnBody {
                    body: body_name,
                    count: jnt_num,
                });
            }

            let joint = if jnt_num == 1 {
                let j = model.body_jntadr()[body] as usize;
                let jnt_name = model
                    .id_to_name(MjtObj::mjOBJ_JOINT, j)
                    .unwrap_or_default()
                    .to_string();
                if model.jnt_type()[j] != MjtJoint::mjJNT_HINGE {
                    return Err(ChainError::UnsupportedJointType {
                        joint: jnt_name,
                        kind: format!("{:?}", model.jnt_type()[j]),
                    });
                }
                let axis = Unit::new_normalize(Vector3::new(
                    model.jnt_axis()[j][0],
                    model.jnt_axis()[j][1],
                    model.jnt_axis()[j][2],
                ));
                let limits = if model.jnt_limited()[j] {
                    let r = &model.jnt_range()[j];
                    Some((r[0], r[1]))
                } else {
                    None
                };
                Some(Joint {
                    name: jnt_name,
                    axis,
                    anchor: Point3::new(
                        model.jnt_pos()[j][0],
                        model.jnt_pos()[j][1],
                        model.jnt_pos()[j][2],
                    ),
                    limits,
                    qpos_adr: model.jnt_qposadr()[j] as usize,
                    dof_adr: model.jnt_dofadr()[j] as usize,
                })
            } else {
                None
            };

            let q = &model.body_quat()[body];
            links.push(Link {
                name: body_name,
                translation: Vector3::new(
                    model.body_pos()[body][0],
                    model.body_pos()[body][1],
                    model.body_pos()[body][2],
                ),
                rotation: UnitQuaternion::new_normalize(Quaternion::new(q[0], q[1], q[2], q[3])),
                joint,
                body_id: body,
            });
        }

        let s_pos = &model.site_pos()[site_id];
        let s_quat = &model.site_quat()[site_id];
        let tool = Isometry3::from_parts(
            Translation3::new(s_pos[0], s_pos[1], s_pos[2]),
            UnitQuaternion::new_normalize(Quaternion::new(
                s_quat[0], s_quat[1], s_quat[2], s_quat[3],
            )),
        );

        Ok(Self {
            name: name.to_string(),
            links,
            tool,
        })
    }

    /// Links, root first.
    pub fn links(&self) -> &[Link] {
        &self.links
    }

    /// Number of actuated joints.
    pub fn dof(&self) -> usize {
        self.links.iter().filter_map(|l| l.joint.as_ref()).count()
    }

    /// Joint names in chain order.
    pub fn joint_names(&self) -> Vec<&str> {
        self.links
            .iter()
            .filter_map(|l| l.joint.as_ref().map(|j| j.name.as_str()))
            .collect()
    }

    /// Joint limits `(lower, upper)` in chain order (`None` for unlimited).
    pub fn joint_limits(&self) -> Vec<Option<(f64, f64)>> {
        self.links
            .iter()
            .filter_map(|l| l.joint.as_ref().map(|j| j.limits))
            .collect()
    }

    /// MuJoCo `qpos` addresses of the chain's joints, in chain order.
    pub fn qpos_addresses(&self) -> Vec<usize> {
        self.links
            .iter()
            .filter_map(|l| l.joint.as_ref().map(|j| j.qpos_adr))
            .collect()
    }

    /// MuJoCo `nv`-vector (velocity / generalized force) addresses of the
    /// chain's joints, in chain order. Index into `qvel`, `qfrc_bias`,
    /// `qfrc_applied`, and friends with these.
    pub fn dof_addresses(&self) -> Vec<usize> {
        self.links
            .iter()
            .filter_map(|l| l.joint.as_ref().map(|j| j.dof_adr))
            .collect()
    }

    /// Clamp a joint vector into the chain's limits.
    pub fn clamp_to_limits(&self, q: &mut [f64]) {
        debug_assert_eq!(q.len(), self.dof());
        let mut i = 0;
        for link in &self.links {
            if let Some(j) = &link.joint {
                if let Some((lo, hi)) = j.limits {
                    q[i] = q[i].clamp(lo, hi);
                }
                i += 1;
            }
        }
    }

    /// A zero (rest) configuration.
    pub fn zero(&self) -> Vec<f64> {
        vec![0.0; self.dof()]
    }

    /// Uniform sample inside the joint limits (slightly inset so we do not
    /// sit on a bound). Used by IK restarts and RRT sampling.
    pub fn sample_uniform(&self, rng: &mut crate::rng::Rng) -> Vec<f64> {
        self.joint_limits()
            .into_iter()
            .map(|lim| match lim {
                Some((lo, hi)) => rng.uniform(lo + 0.05, hi - 0.05),
                None => rng.uniform(-3.0, 3.0),
            })
            .collect()
    }

    /// Serialize the chain as a URDF robot description.
    ///
    /// Used by the test suite to cross-check this crate's FK against the
    /// independent `k` implementation, and handy for exporting any MJCF
    /// chain to the wider robotics tool ecosystem. Rotations are emitted as
    /// fixed-axis roll/pitch/yaw with full float precision.
    pub fn to_urdf_string(&self) -> String {
        use std::fmt::Write as _;
        let mut s = String::new();
        let _ = writeln!(s, r#"<robot name="{}">"#, self.name);

        let link_names: Vec<&str> = self.links.iter().map(|l| l.name.as_str()).collect();
        let tool_name = "ee_tool";
        for name in link_names
            .iter()
            .map(|n| n.to_string())
            .chain([tool_name.to_string()])
        {
            let _ = writeln!(s, "  <link name=\"{name}\"/>");
        }

        for (i, link) in self.links.iter().enumerate().skip(1) {
            let parent = link_names[i - 1];
            let (r, p, y) = link.rotation.euler_angles();
            let t = &link.translation;
            let _ = writeln!(
                s,
                "  <joint name=\"{}\" type=\"{}\">",
                link.joint
                    .as_ref()
                    .map(|j| j.name.as_str())
                    .unwrap_or(&format!("{parent}_{}_fixed", link.name)),
                if link.joint.is_some() {
                    "revolute"
                } else {
                    "fixed"
                }
            );
            let _ = writeln!(
                s,
                "    <origin xyz=\"{:.17} {:.17} {:.17}\" rpy=\"{:.17} {:.17} {:.17}\"/>",
                t.x, t.y, t.z, r, p, y
            );
            let _ = writeln!(s, "    <parent link=\"{parent}\"/>");
            let _ = writeln!(s, "    <child link=\"{}\"/>", link.name);
            if let Some(j) = &link.joint {
                let _ = writeln!(
                    s,
                    "    <axis xyz=\"{:.17} {:.17} {:.17}\"/>",
                    j.axis.x, j.axis.y, j.axis.z
                );
                let (lo, hi) = j
                    .limits
                    .unwrap_or((-std::f64::consts::TAU, std::f64::consts::TAU));
                let _ = writeln!(
                    s,
                    "    <limit lower=\"{lo:.17}\" upper=\"{hi:.17}\" effort=\"150\" velocity=\"3.15\"/>"
                );
            }
            let _ = writeln!(s, "  </joint>");
        }

        // Tool frame as a final fixed joint.
        let last = link_names[self.links.len() - 1];
        let (r, p, y) = self.tool.rotation.euler_angles();
        let t = self.tool.translation.vector;
        let _ = writeln!(s, "  <joint name=\"ee_tool_joint\" type=\"fixed\">");
        let _ = writeln!(
            s,
            "    <origin xyz=\"{:.17} {:.17} {:.17}\" rpy=\"{:.17} {:.17} {:.17}\"/>",
            t.x, t.y, t.z, r, p, y
        );
        let _ = writeln!(s, "    <parent link=\"{last}\"/>");
        let _ = writeln!(s, "    <child link=\"{tool_name}\"/>");
        let _ = writeln!(s, "  </joint>");
        let _ = writeln!(s, "</robot>");
        s
    }
}
