//! Collision queries against a compiled MuJoCo scene.
//!
//! The checker writes a joint configuration into `qpos`, runs MuJoCo
//! kinematics + `mj_collision`, and reports a hit if any contact that
//! involves a robot collision geom has signed distance below `clearance`.
//!
//! Parent–child and same-body pairs are already excluded by MuJoCo, so
//! adjacent-link "contacts" never appear. Visual meshes (`contype = 0`)
//! are ignored. Contacts that do not involve the robot (floor-vs-obstacle)
//! are ignored too — those would otherwise flag every scene that has a
//! pillar sitting on the ground plane.

use std::ops::Deref;

use mujoco_rs::prelude::*;

use crate::chain::Chain;

/// MuJoCo-backed collision oracle for a serial chain.
pub struct CollisionChecker<M: Deref<Target = MjModel>> {
    data: MjData<M>,
    qpos_adr: Vec<usize>,
    /// `true` for geoms that belong to the robot and participate in contact
    /// (`contype != 0`). Indexed by MuJoCo geom id.
    robot_geom: Vec<bool>,
    /// A contact counts as a collision when `dist < clearance` (meters).
    /// Positive clearance inflates obstacles by that amount.
    pub clearance: f64,
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
            clearance: 1e-3,
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
        self.set_q(q);
        self.data.forward_kinematics();
        self.data.collision();
        for c in self.data.contact() {
            if c.dist >= self.clearance {
                continue;
            }
            let g1 = c.geom[0];
            let g2 = c.geom[1];
            let r1 = g1 >= 0 && self.robot_geom[g1 as usize];
            let r2 = g2 >= 0 && self.robot_geom[g2 as usize];
            if r1 || r2 {
                return true;
            }
        }
        false
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
