# arm-lab

A serial-chain manipulator stack written **from scratch in Rust** — no ROS, no MoveIt, no kinematics library.

![Demo 1 — UR5e stepping through IK-solved targets, simulated in MuJoCo](docs/demo1.gif)

The kinematic chain (offsets, joint axes, limits, end-effector site) is **extracted from the compiled MuJoCo model** — never hand-entered as DH parameters — so the geometry used by the algorithms is guaranteed identical to the one the physics simulates. Forward kinematics, geometric Jacobians, and damped-least-squares inverse kinematics are implemented in this repo on `nalgebra` transforms. The independent `k` + `urdf-rs` stack is used **only inside the test suite** as a cross-check.

Demo 1 (above): the IK solution for each target is commanded into MuJoCo's position actuators, with exact model-based gravity/Coriolis feedforward (`qfrc_bias` → `qfrc_applied`, as a real controller's gravity compensation would); the simulated UR5e then moves itself to the target. Rerun records link transforms, EE pose, targets, and live error plots (`demo_output/demo1.rrd`).

## Numbers (measured, reproducible via `cargo test`)

| Metric | Result |
|---|---|
| IK success, 1000 random reachable targets (full 6-DOF pose task, cold start from home) | **97.8%** |
| Mean IK iterations to converge | 20.7 |
| FK vs independent `k`/`urdf-rs` chain (random configurations) | agreement to < 1e-9 |
| FK vs MuJoCo body/site poses (random configurations) | agreement to < 1e-9 |
| Geometric Jacobian vs numerical differentiation | agreement to < 1e-5 |
| Demo 1 final Cartesian error per target (after gravity compensation) | ≤ 1e-4 m |

Failures in the 2.2% are targets reachable only through joint-limit-blocked regions — the solver honors limits by construction (clamped every iteration, asserted in tests).

## Layout

```
crates/arm-lab        the library: chain extraction, FK, Jacobians, DLS IK
crates/arm-lab-demo   Demo 1 binary: MuJoCo servo loop + Rerun telemetry + offscreen GIF render
assets/ur5e           vendored MuJoCo-Menagerie UR5e (see license note below)
```

## Run it

```bash
# library tests (incl. cross-checks and the 1000-target IK table)
cargo test -p arm-lab

# demo, headless: writes demo_output/demo1.rrd
cargo run --release -p arm-lab-demo

# demo + offscreen video: also writes demo_output/demo1.gif (MuJoCo EGL + ffmpeg)
cargo run --release -p arm-lab-demo -- --render

# demo, streaming live into a Rerun viewer started with `rerun`
cargo run --release -p arm-lab-demo -- --connect
```

Requirements: Rust stable, a C++ toolchain, and (for `--render`) `ffmpeg` on PATH. On Linux without system MuJoCo, `mujoco-rs` auto-downloads MuJoCo 3.9 at build time (`MUJOCO_DOWNLOAD_DIR`, and `LD_LIBRARY_PATH` pointing at its `lib/` at runtime — see the [mujoco-rs docs](https://github.com/sebcrozet/mujoco-rs)).

## Design notes

- **Chain extraction, not re-modeling.** `Chain::from_mujoco` walks `body_parentid` from the tip body to the world, collecting static transforms, hinge axes, anchors, and limits from the *compiled* model, plus the EE site as the tool frame. One source of truth for geometry.
- **DLS IK with adaptive damping.** Each step solves `Δq = Jᵀ(JJᵀ + λ²I)⁻¹e` with a diagonal nullspace bias toward a rest pose; λ scales down with the error so the endgame converges Newton-like while near-singular regions stay damped. Joint limits are clamped every iteration; seeded random restarts (TRAC-IK style) recover from bad basins.
- **Deterministic.** Restart sampling uses an in-repo xorshift RNG with a fixed seed; given the same model and target sequence, results are bit-reproducible.
- **Physics-side servo.** The demo commands joint positions and applies the exact MuJoCo bias force as feedforward — the position actuators' residual steady-state error (gravity sag) disappears, leaving ≤ 0.1 mm tracking at the tool.

## Roadmap

- [x] Demo 1: FK + DLS IK, Rerun telemetry, offscreen-rendered GIF
- [ ] RRT-Connect planning with collision checks; jerk-limited trajectories; PD tracking
- [ ] Pick-and-place with obstacle dodging; benchmark tables

## License & assets

Code: Apache-2.0. The vendored `assets/ur5e` model and meshes are from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (UR5e description © 2018 ROS Industrial Consortium, BSD-3; see `assets/ur5e/LICENSE`), with a local modification adding offscreen buffer size to `scene.xml`.
