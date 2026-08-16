//! # arm-lab
//!
//! Serial-chain manipulator kinematics, written from scratch in Rust.
//!
//! This crate is deliberately *not* a wrapper around a kinematics library. The
//! core algorithms — forward kinematics, geometric Jacobians, and damped
//! least-squares inverse kinematics — are implemented here directly on
//! `nalgebra` transforms. External crates are used only at the edges:
//!
//! - [`mujoco_rs::MjModel`] supplies the robot description: the chain is
//!   *extracted* from MuJoCo's compiled model (body offsets, joint axes,
//!   limits, end-effector site), so it is guaranteed consistent with the
//!   simulation. No XML parsing of our own.
//! - `k` / `urdf-rs` are dev-dependencies, used **only in tests** as
//!   independent cross-checks of the in-repo FK.
//!
//! Conventions: quaternions are `nalgebra` `(w, x, y, z)`; transforms are
//! `nalgebra::Isometry3<f64>`; the Jacobian stacks `[linear; angular]` rows,
//! with the angular part expressed in the world frame.

pub mod chain;
pub mod ik;
pub mod jacobian;
pub mod kinematics;
pub mod rng;

pub use chain::{Chain, Joint, Link};
pub use ik::{IkConfig, IkResult};
pub use rng::Rng;
