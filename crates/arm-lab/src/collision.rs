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
//!
//! [`AttachedBoxCollisionChecker`] adds a separate mechanism for a rigid
//! end-effector load: it transforms a compiled mocap box from FK and calls
//! MuJoCo's explicit geom-distance function only for constructor-declared
//! payload/environment pairs. A positive payload clearance therefore does not
//! inflate or otherwise change the robot contact predicate.

use std::collections::HashSet;
use std::fmt;
use std::ops::Deref;

use mujoco_rs::prelude::*;
use nalgebra::{Isometry3, Matrix3, Quaternion, Rotation3, Translation3, UnitQuaternion};

use crate::chain::Chain;
use crate::kinematics::fk;

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

/// Declarative geometry for an end-effector-attached box proxy.
///
/// The named proxy must already exist as a box geom on a MuJoCo mocap body.
/// Its contact masks may be disabled: [`AttachedBoxCollisionChecker`] uses
/// explicit pair-distance queries, not the emitted-contact set, for payload
/// proximity. `proxy_in_ee` is the desired proxy-*geom* pose in the extracted
/// end-effector frame, not the mocap-body pose.
#[derive(Debug, Clone)]
pub struct AttachedBoxSpec {
    pub proxy_geom_name: String,
    pub proxy_in_ee: Isometry3<f64>,
    pub environment_geom_names: Vec<String>,
    pub clearance_m: f64,
}

impl AttachedBoxSpec {
    pub fn new<I, S>(
        proxy_geom_name: impl Into<String>,
        proxy_in_ee: Isometry3<f64>,
        environment_geom_names: I,
        clearance_m: f64,
    ) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        Self {
            proxy_geom_name: proxy_geom_name.into(),
            proxy_in_ee,
            environment_geom_names: environment_geom_names.into_iter().map(Into::into).collect(),
            clearance_m,
        }
    }
}

/// A pair-scoped distance produced by [`AttachedBoxCollisionChecker`].
///
/// Distances below `clearance_m` are exact for the collision decision. Free
/// distances can be capped at the checker's finite query bound (at least 1 m),
/// because MuJoCo's pair query accepts a maximum distance.
#[derive(Debug, Clone, PartialEq)]
pub struct PayloadPairDistance {
    pub proxy_geom_id: usize,
    pub proxy_geom_name: String,
    pub environment_geom_id: usize,
    pub environment_geom_name: String,
    pub distance_m: f64,
    pub clearance_m: f64,
}

impl PayloadPairDistance {
    /// The strict predicate used by the planner.
    pub fn violates_clearance(&self) -> bool {
        !self.distance_m.is_finite() || self.distance_m < self.clearance_m
    }

    /// Stable human-readable pair identity for diagnostics.
    pub fn identity(&self) -> String {
        format!(
            "{}[{}] vs {}[{}]",
            self.proxy_geom_name,
            self.proxy_geom_id,
            self.environment_geom_name,
            self.environment_geom_id
        )
    }
}

/// Construction failures for [`AttachedBoxCollisionChecker`].
#[derive(Debug, Clone, PartialEq)]
pub enum AttachedBoxError {
    EmptyEnvironmentSet,
    UnknownProxyGeom(String),
    ProxyIsNotBox { name: String, geom_type: MjtGeom },
    ProxyBodyIsNotMocap { geom: String, body: String },
    UnknownEnvironmentGeom(String),
    DuplicateEnvironmentGeom(String),
    ProxyInEnvironmentSet(String),
    EnvironmentGeomIsNotContactEnabled(String),
    EnvironmentGeomBelongsToRobot(String),
    NonFiniteProxyTransform,
    InvalidClearance(f64),
}

impl fmt::Display for AttachedBoxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyEnvironmentSet => write!(f, "payload environment set is empty"),
            Self::UnknownProxyGeom(name) => write!(f, "unknown proxy geom '{name}'"),
            Self::ProxyIsNotBox { name, geom_type } => {
                write!(f, "proxy geom '{name}' is {geom_type:?}, not a box")
            }
            Self::ProxyBodyIsNotMocap { geom, body } => {
                write!(f, "proxy geom '{geom}' belongs to non-mocap body '{body}'")
            }
            Self::UnknownEnvironmentGeom(name) => {
                write!(f, "unknown environment geom '{name}'")
            }
            Self::DuplicateEnvironmentGeom(name) => {
                write!(f, "duplicate environment geom '{name}'")
            }
            Self::ProxyInEnvironmentSet(name) => {
                write!(f, "proxy geom '{name}' is also in the environment set")
            }
            Self::EnvironmentGeomIsNotContactEnabled(name) => write!(
                f,
                "environment geom '{name}' has both contact masks disabled"
            ),
            Self::EnvironmentGeomBelongsToRobot(name) => {
                write!(f, "environment geom '{name}' belongs to the robot chain")
            }
            Self::NonFiniteProxyTransform => {
                write!(f, "proxy transform contains a non-finite component")
            }
            Self::InvalidClearance(clearance) => {
                write!(
                    f,
                    "payload clearance must be finite and non-negative, got {clearance}"
                )
            }
        }
    }
}

impl std::error::Error for AttachedBoxError {}

#[derive(Debug, Clone)]
struct ScopedEnvironmentGeom {
    id: usize,
    name: String,
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

/// Pair-scoped collision oracle for a rigid box attached to the end effector.
///
/// This checker combines two deliberately different predicates at each sampled
/// joint state:
///
/// 1. the existing [`CollisionChecker`] emitted-contact rule for the robot, and
/// 2. explicit MuJoCo geom-distance queries from one attached proxy box to a
///    constructor-validated list of environment geoms.
///
/// It does **not** globally inflate the robot, and it does not query the proxy
/// against robot geoms. That latter exclusion is intentional for a scripted
/// grasp whose allowed wrist/load contact region is not modeled.
pub struct AttachedBoxCollisionChecker<M: Deref<Target = MjModel>> {
    robot: CollisionChecker<M>,
    chain: Chain,
    proxy_geom_id: usize,
    proxy_geom_name: String,
    proxy_mocap_id: usize,
    /// Local geom pose relative to its mocap body. The mocap pose is solved so
    /// the *geom*, rather than merely its body origin, matches `proxy_in_ee`.
    proxy_geom_in_body: Isometry3<f64>,
    proxy_in_ee: Isometry3<f64>,
    environment: Vec<ScopedEnvironmentGeom>,
    clearance_m: f64,
    distance_query_bound_m: f64,
}

impl<M: Deref<Target = MjModel>> AttachedBoxCollisionChecker<M> {
    /// Construct and validate a pair-scoped attached-box checker.
    pub fn new(model: M, chain: &Chain, spec: AttachedBoxSpec) -> Result<Self, AttachedBoxError> {
        if !spec.clearance_m.is_finite() || spec.clearance_m < 0.0 {
            return Err(AttachedBoxError::InvalidClearance(spec.clearance_m));
        }
        if !is_finite_isometry(&spec.proxy_in_ee) {
            return Err(AttachedBoxError::NonFiniteProxyTransform);
        }
        if spec.environment_geom_names.is_empty() {
            return Err(AttachedBoxError::EmptyEnvironmentSet);
        }

        let robot = CollisionChecker::new(model, chain);
        let compiled = robot.data.model();
        let proxy_geom_id = compiled
            .name_to_id(MjtObj::mjOBJ_GEOM, &spec.proxy_geom_name)
            .ok_or_else(|| AttachedBoxError::UnknownProxyGeom(spec.proxy_geom_name.clone()))?;
        let proxy_type = compiled.geom_type()[proxy_geom_id];
        if proxy_type != MjtGeom::mjGEOM_BOX {
            return Err(AttachedBoxError::ProxyIsNotBox {
                name: spec.proxy_geom_name,
                geom_type: proxy_type,
            });
        }

        let proxy_body_id = compiled.geom_bodyid()[proxy_geom_id] as usize;
        let proxy_mocap_id = compiled.body_mocapid()[proxy_body_id];
        if proxy_mocap_id < 0 {
            return Err(AttachedBoxError::ProxyBodyIsNotMocap {
                geom: spec.proxy_geom_name,
                body: body_name(compiled, proxy_body_id),
            });
        }

        let proxy_geom_in_body = geom_local_pose(compiled, proxy_geom_id);
        if !is_finite_isometry(&proxy_geom_in_body) {
            return Err(AttachedBoxError::NonFiniteProxyTransform);
        }

        let mut robot_body = vec![false; compiled.body_parentid().len()];
        for link in chain.links() {
            if link.body_id != 0 {
                robot_body[link.body_id] = true;
            }
        }

        let mut seen = HashSet::new();
        let mut environment = Vec::with_capacity(spec.environment_geom_names.len());
        for name in spec.environment_geom_names {
            let id = compiled
                .name_to_id(MjtObj::mjOBJ_GEOM, &name)
                .ok_or_else(|| AttachedBoxError::UnknownEnvironmentGeom(name.clone()))?;
            if id == proxy_geom_id {
                return Err(AttachedBoxError::ProxyInEnvironmentSet(name));
            }
            if !seen.insert(id) {
                return Err(AttachedBoxError::DuplicateEnvironmentGeom(name));
            }
            if compiled.geom_contype()[id] == 0 && compiled.geom_conaffinity()[id] == 0 {
                return Err(AttachedBoxError::EnvironmentGeomIsNotContactEnabled(name));
            }
            let body_id = compiled.geom_bodyid()[id] as usize;
            if robot_body[body_id] {
                return Err(AttachedBoxError::EnvironmentGeomBelongsToRobot(name));
            }
            environment.push(ScopedEnvironmentGeom { id, name });
        }

        Ok(Self {
            robot,
            chain: chain.clone(),
            proxy_geom_id,
            proxy_geom_name: spec.proxy_geom_name,
            proxy_mocap_id: proxy_mocap_id as usize,
            proxy_geom_in_body,
            proxy_in_ee: spec.proxy_in_ee,
            environment,
            clearance_m: spec.clearance_m,
            // Distances beyond the decision threshold are not needed. A 1 m
            // lower bound keeps ordinary free-state diagnostics informative
            // while retaining a finite MuJoCo `distmax`.
            distance_query_bound_m: spec.clearance_m.max(1.0),
        })
    }

    /// Set the unchanged robot emitted-contact threshold used in the combined
    /// predicate. Demo 3 carry uses exactly `0.0 m` (sampled penetration).
    pub fn set_robot_contact_threshold(&mut self, threshold_m: f64) {
        self.robot.contact_threshold = threshold_m;
    }

    pub fn robot_contact_threshold(&self) -> f64 {
        self.robot.contact_threshold
    }

    pub fn payload_clearance_m(&self) -> f64 {
        self.clearance_m
    }

    pub fn proxy_geom_name(&self) -> &str {
        &self.proxy_geom_name
    }

    /// Actual compiled proxy-geom world pose after applying `q` and the
    /// end-effector attachment transform.
    ///
    /// This reads MuJoCo's `geom_xpos`/`geom_xmat`, so tests can verify the
    /// complete FK → mocap-body → nonzero geom-local transform pipeline rather
    /// than inferring it from collision outcomes.
    pub fn proxy_world_pose(&mut self, q: &[f64]) -> Isometry3<f64> {
        self.update_state(q);
        geom_world_pose(&self.robot.data, self.proxy_geom_id)
    }

    /// Environment geom names in the exact deterministic query order.
    pub fn environment_geom_names(&self) -> impl Iterator<Item = &str> {
        self.environment.iter().map(|geom| geom.name.as_str())
    }

    /// True if the existing robot predicate rejects `q`.
    pub fn robot_collides(&mut self, q: &[f64]) -> bool {
        self.update_state(q);
        self.robot_collides_current()
    }

    /// All declared payload/environment pair distances at `q`.
    pub fn payload_distances(&mut self, q: &[f64]) -> Vec<PayloadPairDistance> {
        self.update_state(q);
        self.payload_distances_current()
    }

    /// Only pair distances that violate the strict payload clearance rule.
    pub fn payload_violations(&mut self, q: &[f64]) -> Vec<PayloadPairDistance> {
        self.payload_distances(q)
            .into_iter()
            .filter(PayloadPairDistance::violates_clearance)
            .collect()
    }

    /// True if any declared payload/environment pair is closer than the strict
    /// payload clearance at `q`.
    pub fn payload_collides(&mut self, q: &[f64]) -> bool {
        self.update_state(q);
        self.payload_collides_current()
    }

    /// Combined sampled predicate: unchanged robot rule OR payload proximity.
    pub fn collides(&mut self, q: &[f64]) -> bool {
        self.update_state(q);
        let robot_collision = self.robot_collides_current();
        let payload_collision = self.payload_collides_current();
        robot_collision || payload_collision
    }

    fn update_state(&mut self, q: &[f64]) {
        let desired_proxy_world = fk(&self.chain, q) * self.proxy_in_ee;
        let mocap_body_world = desired_proxy_world * self.proxy_geom_in_body.inverse();
        let translation = mocap_body_world.translation.vector;
        let rotation = mocap_body_world.rotation;
        self.robot.data.mocap_pos_mut()[self.proxy_mocap_id] =
            [translation.x, translation.y, translation.z];
        self.robot.data.mocap_quat_mut()[self.proxy_mocap_id] =
            [rotation.w, rotation.i, rotation.j, rotation.k];
        self.robot.update_contacts(q);
    }

    fn robot_collides_current(&self) -> bool {
        self.robot.data.contact().iter().any(|contact| {
            contact.dist < self.robot.contact_threshold
                && self.robot.contact_involves_robot(contact.geom)
                // The proxy may intentionally touch the wrist/load attachment
                // region. That pair is outside the robot/environment rule and
                // outside the explicitly scoped payload/environment queries.
                && !contact.geom.contains(&(self.proxy_geom_id as i32))
        })
    }

    fn payload_distances_current(&mut self) -> Vec<PayloadPairDistance> {
        let mut distances = Vec::with_capacity(self.environment.len());
        for index in 0..self.environment.len() {
            let environment_geom_id = self.environment[index].id;
            let environment_geom_name = self.environment[index].name.clone();
            let distance_m = self.robot.data.geom_distance(
                self.proxy_geom_id,
                environment_geom_id,
                self.distance_query_bound_m,
                None,
            );
            distances.push(PayloadPairDistance {
                proxy_geom_id: self.proxy_geom_id,
                proxy_geom_name: self.proxy_geom_name.clone(),
                environment_geom_id,
                environment_geom_name,
                distance_m,
                clearance_m: self.clearance_m,
            });
        }
        distances
    }

    fn payload_collides_current(&mut self) -> bool {
        for index in 0..self.environment.len() {
            let distance_m = self.robot.data.geom_distance(
                self.proxy_geom_id,
                self.environment[index].id,
                self.distance_query_bound_m,
                None,
            );
            if !distance_m.is_finite() || distance_m < self.clearance_m {
                return true;
            }
        }
        false
    }
}

fn geom_local_pose(model: &MjModel, geom_id: usize) -> Isometry3<f64> {
    let position = model.geom_pos()[geom_id];
    let quaternion = model.geom_quat()[geom_id];
    Isometry3::from_parts(
        Translation3::new(position[0], position[1], position[2]),
        UnitQuaternion::new_normalize(Quaternion::new(
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
        )),
    )
}

fn geom_world_pose<M: Deref<Target = MjModel>>(data: &MjData<M>, geom_id: usize) -> Isometry3<f64> {
    let position = data.geom_xpos()[geom_id];
    let matrix = Matrix3::from_row_slice(&data.geom_xmat()[geom_id]);
    Isometry3::from_parts(
        Translation3::new(position[0], position[1], position[2]),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(matrix)),
    )
}

fn is_finite_isometry(pose: &Isometry3<f64>) -> bool {
    pose.translation
        .vector
        .iter()
        .all(|value| value.is_finite())
        && pose.rotation.coords.iter().all(|value| value.is_finite())
}

fn body_name(model: &MjModel, id: usize) -> String {
    model
        .id_to_name(MjtObj::mjOBJ_BODY, id)
        .filter(|name| !name.is_empty())
        .map_or_else(|| format!("body#{id}"), str::to_string)
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
