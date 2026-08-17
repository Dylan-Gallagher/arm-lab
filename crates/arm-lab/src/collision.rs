//! Collision queries against a compiled MuJoCo scene.
//!
//! The checker writes a joint configuration into `qpos`, runs MuJoCo
//! kinematics + `mj_collision`, and reports a hit if any **MuJoCo-emitted**
//! contact that involves a robot collision geom has signed distance below
//! `contact_threshold`.
//!
//! Parent–child and same-body pairs are already excluded by MuJoCo, so
//! adjacent-link "contacts" never appear. Visual meshes (`contype = 0`)
//! are ignored. Contacts that do not involve the robot (floor-vs-obstacle)
//! are ignored too — those would otherwise flag every scene that has a
//! pillar sitting on the ground plane.
//!
//! Important: `contact_threshold` filters contacts already emitted by MuJoCo;
//! it is not a general pairwise-distance query. With zero geom margin/gap,
//! separated positive-distance pairs might not be emitted even when their
//! distance is below a positive threshold. Use a threshold of `0.0` when the
//! required claim is sampled geometric penetration (`dist < 0`).

use std::ops::Deref;

use mujoco_rs::prelude::*;

use crate::chain::Chain;

/// A robot-involved emitted MuJoCo contact below the checker's threshold.
///
/// `distance_m` is MuJoCo's signed contact distance: negative values are
/// penetration. A positive value only means this pair happened to be emitted;
/// it does not prove that every closer positive-distance pair was enumerated.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotContact {
    pub geom1_id: i32,
    pub geom2_id: i32,
    pub geom1_name: String,
    pub geom2_name: String,
    pub body1_name: String,
    pub body2_name: String,
    pub distance_m: f64,
}

impl RobotContact {
    /// Stable human-readable identity for raw benchmark artifacts.
    pub fn identity(&self) -> String {
        format!(
            "{}[{}]@{} vs {}[{}]@{}",
            self.geom1_name,
            self.geom1_id,
            self.body1_name,
            self.geom2_name,
            self.geom2_id,
            self.body2_name
        )
    }
}

/// MuJoCo-backed collision oracle for a serial chain.
pub struct CollisionChecker<M: Deref<Target = MjModel>> {
    data: MjData<M>,
    qpos_adr: Vec<usize>,
    /// `true` for geoms that belong to the robot and participate in contact
    /// (`contype != 0`). Indexed by MuJoCo geom id.
    robot_geom: Vec<bool>,
    /// An **emitted** contact counts as a collision when
    /// `dist < contact_threshold` (meters).
    ///
    /// This is a filter, not geometric inflation. Positive-distance pairs are
    /// only considered when MuJoCo emitted them based on model margin/gap.
    pub contact_threshold: f64,
}

impl<M: Deref<Target = MjModel>> CollisionChecker<M> {
    /// Build a checker around `model`. `model` may be an owned [`MjModel`]
    /// or a reference; the checker holds the matching [`MjData`].
    pub fn new(model: M, chain: &Chain) -> Self {
        let data = MjData::new(model);
        let qpos_adr = chain.qpos_addresses();

        let model = data.model();
        let nbody = model.body_parentid().len();
        let mut robot_body = vec![false; nbody];
        for link in chain.links() {
            // The extracted chain includes the world body (id 0) as its
            // identity root. World-attached geoms are the floor and any
            // obstacles — not the robot.
            if link.body_id != 0 {
                robot_body[link.body_id] = true;
            }
        }

        let robot_geom: Vec<bool> = model
            .geom_contype()
            .iter()
            .zip(model.geom_bodyid())
            .map(|(&contype, &body)| contype != 0 && robot_body[body as usize])
            .collect();

        Self {
            data,
            qpos_adr,
            robot_geom,
            contact_threshold: 1e-3,
        }
    }

    /// Number of robot collision geoms this checker considers.
    pub fn robot_geom_count(&self) -> usize {
        self.robot_geom.iter().filter(|&&b| b).count()
    }

    /// Write `q` into the live `qpos` (chain joints only).
    pub fn set_q(&mut self, q: &[f64]) {
        debug_assert_eq!(q.len(), self.qpos_adr.len());
        for (&adr, &qi) in self.qpos_adr.iter().zip(q.iter()) {
            self.data.qpos_mut()[adr] = qi;
        }
    }

    /// True if configuration `q` is in collision.
    pub fn collides(&mut self, q: &[f64]) -> bool {
        self.update_contacts(q);
        self.data.contact().iter().any(|contact| {
            contact.dist < self.contact_threshold && self.contact_involves_robot(contact.geom)
        })
    }

    /// Robot-involved emitted contacts below [`Self::contact_threshold`].
    ///
    /// This uses exactly the same contact filter and strict distance comparison
    /// as [`Self::collides`], but retains geom identity and signed distance for
    /// execution audits.
    pub fn robot_contacts(&mut self, q: &[f64]) -> Vec<RobotContact> {
        self.update_contacts(q);
        let model = self.data.model();
        self.data
            .contact()
            .iter()
            .filter(|contact| {
                contact.dist < self.contact_threshold && self.contact_involves_robot(contact.geom)
            })
            .map(|contact| RobotContact {
                geom1_id: contact.geom[0],
                geom2_id: contact.geom[1],
                geom1_name: geom_name(model, contact.geom[0]),
                geom2_name: geom_name(model, contact.geom[1]),
                body1_name: geom_body_name(model, contact.geom[0]),
                body2_name: geom_body_name(model, contact.geom[1]),
                distance_m: contact.dist,
            })
            .collect()
    }

    fn update_contacts(&mut self, q: &[f64]) {
        self.set_q(q);
        self.data.forward_kinematics();
        self.data.collision();
    }

    fn contact_involves_robot(&self, geom: [i32; 2]) -> bool {
        geom.into_iter()
            .any(|id| id >= 0 && self.robot_geom[id as usize])
    }

    /// Borrow the inner [`MjData`] (e.g. to read body poses after `set_q`).
    pub fn data(&self) -> &MjData<M> {
        &self.data
    }

    /// Mutable inner [`MjData`].
    pub fn data_mut(&mut self) -> &mut MjData<M> {
        &mut self.data
    }
}

fn geom_name(model: &MjModel, id: i32) -> String {
    if id < 0 {
        return "none".to_string();
    }
    model
        .id_to_name(MjtObj::mjOBJ_GEOM, id as usize)
        .filter(|name| !name.is_empty())
        .map_or_else(|| format!("geom#{id}"), str::to_string)
}

fn geom_body_name(model: &MjModel, geom_id: i32) -> String {
    if geom_id < 0 {
        return "none".to_string();
    }
    let body_id = model.geom_bodyid()[geom_id as usize] as usize;
    model
        .id_to_name(MjtObj::mjOBJ_BODY, body_id)
        .filter(|name| !name.is_empty())
        .map_or_else(|| format!("body#{body_id}"), str::to_string)
}
