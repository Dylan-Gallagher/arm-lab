//! # arm-lab
//!
//! Serial-chain manipulator stack, written from scratch in Rust.
//!
//! This crate is deliberately *not* a wrapper around a kinematics or planning
//! library. The core algorithms — forward kinematics, geometric Jacobians,
//! damped least-squares inverse kinematics, RRT-Connect, and full-stop S-curve
//! time-parameterization with almost-everywhere jerk bounds — are implemented
//! here directly. External crates are used only at the edges:
//!
//! - [`mujoco_rs::MjModel`] supplies the robot description: the chain is
//!   *extracted* from MuJoCo's compiled model (body offsets, joint axes,
//!   limits, end-effector site), so it is guaranteed consistent with the
//!   simulation. Collision checks call `mj_collision` on that same model.
//! - `k` / `urdf-rs` are dev-dependencies, used **only in tests** as
//!   independent cross-checks of the in-repo FK.
//!
//! Conventions: quaternions are `nalgebra` `(w, x, y, z)`; transforms are
//! `nalgebra::Isometry3<f64>`; the Jacobian stacks `[linear; angular]` rows,
//! with the angular part expressed in the world frame.

pub mod chain;
pub mod collision;
pub mod ik;
pub mod jacobian;
pub mod kinematics;
pub mod plan;
pub mod rng;
pub mod traj;

pub use chain::{Chain, Joint, Link};
pub use collision::{
    AttachedBoxCollisionChecker, AttachedBoxError, AttachedBoxSpec, CollisionChecker,
    PayloadPairDistance, RobotContact,
};
pub use ik::{IkConfig, IkResult};
pub use plan::{PlanConfig, PlanResult, PlanStatus};
pub use rng::Rng;
pub use traj::{TrajLimits, Trajectory, time_parameterize};
