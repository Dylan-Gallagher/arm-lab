# arm-lab

A serial-chain manipulator stack written **from scratch in Rust** — no ROS, no MoveIt, no kinematics library, no off-the-shelf planner.

![Demo 2 — UR5e dodging a pillar. RRT-Connect in joint space, jerk-limited S-curve, MuJoCo collision checks, PD + velocity feedforward](docs/demo2.gif)

The kinematic chain (offsets, joint axes, limits, end-effector site) is **extracted from the compiled MuJoCo model** — never hand-entered as DH parameters — so the geometry used by the algorithms is guaranteed identical to the one the physics simulates. Forward kinematics, geometric Jacobians, damped-least-squares inverse kinematics, RRT-Connect with shortcutting, and a rest-to-rest 7-phase S-curve are implemented in this repo. Collision checks call `mj_collision` on interpolated joint-space states. The independent `k` + `urdf-rs` stack is used **only inside the test suite** as a cross-check of FK.

Demo 2 (above): a pillar sits on the home shoulder-pan sweep. The joint-space straight line from home to a +1.1 rad pan collides; RRT-Connect finds a 3-waypoint path around it in **~7 ms**; a jerk-limited S-curve times the polyline (3.50 s, v ≤ 0.55 rad/s); MuJoCo tracks it to **0.005 rad**. Demo 1 (IK target sequence, no obstacles) is in [`docs/demo1.gif`](docs/demo1.gif).

## Numbers (measured, reproducible via `cargo test`)

| Metric | Result |
|---|---|
| IK success, 1000 random reachable targets (full 6-DOF pose task, cold start from home) | **97.8%** |
| Mean IK iterations to converge | 20.7 |
| FK vs independent `k`/`urdf-rs` chain (random configurations) | agreement to < 1e-9 |
| FK vs MuJoCo body/site poses (random configurations) | agreement to < 1e-9 |
| Geometric Jacobian vs numerical differentiation | agreement to < 1e-5 |
| Demo 1 final Cartesian error per target (after gravity compensation) | ≤ 1e-4 m |
| RRT-Connect, cluttered UR5e (pillar on the pan sweep), 11 seeds | **median 6.7 ms** (min 3.2, max 13.4) |
| Same query: tree size → shortcut | 64 nodes → **3 waypoints** |
| Planner determinism | identical path given identical seed |
| Demo 2 S-curve (seed `20260816`, v≤0.55 a≤1.8 j≤8) | **1751 samples, 3.50 s**, peak joint \|qd\| 0.550 rad/s |
| Demo 2 tracking (PD + vel FF + gravity compensation) | worst 0.0046 rad · final goal 0.0002 rad |
| Timed-trajectory determinism | identical `q(t)` given identical seed + limits |

The 1 s median planning-time exit criterion is met by two orders of magnitude. Failures in the 2.2% IK slice are targets reachable only through joint-limit-blocked regions — the solver honors limits by construction (clamped every iteration, asserted in tests).

## Layout

```
crates/arm-lab        the library: chain extraction, FK, Jacobians, DLS IK, RRT-Connect, S-curve, MuJoCo collision
crates/arm-lab-demo   demo1 (IK servo) and demo2 (pillar dodge + S-curve), Rerun + offscreen GIF
assets/ur5e           vendored MuJoCo-Menagerie UR5e + cluttered scene (see license note below)
```

## Run it

```bash
# library tests (cross-checks, 1000-target IK table, planner correctness + timing)
cargo test -p arm-lab

# demo 1, headless: writes demo_output/demo1.rrd
cargo run --release -p arm-lab-demo --bin demo1

# demo 1 + offscreen GIF
cargo run --release -p arm-lab-demo --bin demo1 -- --render

# demo 2: plan around the pillar, execute, write demo_output/demo2.rrd
cargo run --release -p arm-lab-demo --bin demo2

# demo 2 + offscreen GIF
cargo run --release -p arm-lab-demo --bin demo2 -- --render

# stream live into a Rerun viewer started with `rerun`
cargo run --release -p arm-lab-demo --bin demo2 -- --connect
```

Requirements: Rust stable, a C++ toolchain, and (for `--render`) `ffmpeg` on PATH. On Linux without system MuJoCo, `mujoco-rs` auto-downloads MuJoCo 3.9 at build time (`MUJOCO_DOWNLOAD_DIR`, and `LD_LIBRARY_PATH` pointing at its `lib/` at runtime — see the [mujoco-rs docs](https://github.com/davidhozic/mujoco-rs)).

## Design notes

- **Chain extraction, not re-modeling.** `Chain::from_mujoco` walks `body_parentid` from the tip body to the world, collecting static transforms, hinge axes, anchors, and limits from the *compiled* model, plus the EE site as the tool frame. One source of truth for geometry.
- **DLS IK with adaptive damping.** Each step solves `Δq = Jᵀ(JJᵀ + λ²I)⁻¹e` with a diagonal nullspace bias toward a rest pose; λ scales down with the error so the endgame converges Newton-like while near-singular regions stay damped. Joint limits are clamped every iteration; seeded random restarts (TRAC-IK style) recover from bad basins.
- **RRT-Connect, from scratch.** Two trees grow toward each other (Kuffner & LaValle 2000). `EXTEND` takes one joint-space step; `CONNECT` greedily repeats it. Edges are collision-checked by interpolating at `resolution` and calling MuJoCo `mj_collision` on each state. Greedy then random shortcutting removes redundant waypoints; the path is densified to the same resolution for execution. Sampling uses the in-repo SplitMix64 RNG — same seed, same path.
- **Collision filter.** Only contacts that involve a robot collision geom (`contype ≠ 0`, attached to a chain body, not the world) count. Floor-vs-pillar contacts are ignored; parent–child pairs are already excluded by MuJoCo. Visual meshes never participate.
- **Deterministic.** Restart sampling, RRT sampling, and random shortcutting all use the in-repo RNG with a fixed seed. The timed trajectory is bit-stable given the same seed and limits (CI golden test).
- **S-curve time-parameterization.** A rest-to-rest 7-phase bang-bang-jerk profile times the scalar path length `s ∈ [0, L]`. Per-joint `(v, a, j)` limits are converted to path-space limits by the steepest `|dqᵢ/ds|` on the polyline, so no joint exceeds its bound along an edge. Tangent discontinuities at corners produce a one-sample acceleration spike; shortcutting keeps those corners to one or two.
- **Physics-side servo.** The demos apply the exact MuJoCo bias force as gravity/Coriolis feedforward. Demo 2 commands the Menagerie position actuators as a PD tracker with velocity feedforward: `ctrl = q_des + (kv/kp)·qd_des` yields `τ = kp(q_des − q) + kv(qd_des − qd)`. Worst joint-space tracking on the cluttered query is 0.0046 rad.

## Roadmap

- [x] Demo 1: FK + DLS IK, Rerun telemetry, offscreen-rendered GIF
- [x] RRT-Connect with MuJoCo collision checks and shortcutting (Demo 2)
- [x] Jerk-limited S-curve trajectories; joint-space PD + velocity feedforward
- [ ] Pick-and-place with obstacle dodging; benchmark tables

## License & assets

Code: Apache-2.0. The vendored `assets/ur5e` model and meshes are from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (UR5e description © 2018 ROS Industrial Consortium, BSD-3; see `assets/ur5e/LICENSE`), with a local modification adding offscreen buffer size to `scene.xml`. `scene_cluttered.xml` is original to this repo.
