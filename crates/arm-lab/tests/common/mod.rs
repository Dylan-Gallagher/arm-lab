//! Shared test helpers: model loading, chain construction, deterministic RNG.

#![allow(dead_code)]

use arm_lab::Chain;
use mujoco_rs::prelude::*;

pub const UR5E_XML: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/ur5e/ur5e.xml");

pub fn ur5e_model() -> MjModel {
    MjModel::from_xml(UR5E_XML).expect("failed to load vendored UR5e MJCF")
}

pub fn ur5e_chain() -> Chain {
    Chain::from_mujoco(&ur5e_model(), "ur5e", "wrist_3_link", "attachment_site")
        .expect("failed to extract UR5e chain")
}

/// Deterministic SplitMix64 → uniform [0, 1). No `rand` dependency: the whole
/// point of this crate is owning the numerics, and reproducible tests are a
/// feature (fixed seeds → identical statistics on every machine/CI run).
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    pub fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }
}

/// A random joint configuration within the chain's limits.
pub fn random_q(chain: &Chain, rng: &mut Rng) -> Vec<f64> {
    chain
        .joint_limits()
        .into_iter()
        .map(|lim| match lim {
            Some((lo, hi)) => rng.uniform(lo + 0.05, hi - 0.05),
            None => rng.uniform(-3.0, 3.0),
        })
        .collect()
}

/// Push a chain configuration into MuJoCo's `qpos` (by joint name).
pub fn set_qpos<M: std::ops::Deref<Target = MjModel>>(
    data: &mut MjData<M>,
    chain: &Chain,
    q: &[f64],
) {
    let names = chain.joint_names();
    for (name, &qi) in names.iter().zip(q.iter()) {
        data.joint(name).unwrap().view_mut(data).qpos[0] = qi;
    }
}
