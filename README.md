# arm-lab

A serial-chain manipulator stack written **from scratch in Rust** — no ROS, no MoveIt, no kinematics library, no off-the-shelf planner.

![Demo 3 — UR5e pick-and-place around a pillar. IK, RRT-Connect, scalar S-curve time law, PD + velocity feedforward](docs/demo3.gif)

The kinematic chain (offsets, joint axes, limits, end-effector site) is **extracted from the compiled MuJoCo model** — never hand-entered as DH parameters — so the geometry used by the algorithms is guaranteed identical to the one the physics simulates. Forward kinematics, geometric Jacobians, damped-least-squares inverse kinematics, RRT-Connect with shortcutting, and a rest-to-rest 7-phase scalar S-curve are implemented in this repo. Collision checks call `mj_collision` on interpolated joint-space states. The independent `k` + `urdf-rs` stack is used **only inside the test suite** as a cross-check of FK.

Demo 3 (above): IK solves pick and place poses; the joint-space straight line between them hits a pillar on the table; RRT-Connect carries the cube around it in **~4 ms**; the path coordinate is timed with an S-curve and tracked to **0.004 rad**. Polyline corners are not blended, so the joint trajectory is not globally jerk-limited. The cube is a mocap weld (scripted attach, not contact-rich grasping). Demo 2 (pillar dodge, no object) is in [`docs/demo2.gif`](docs/demo2.gif). Demo 1 (IK target sequence) is in [`docs/demo1.gif`](docs/demo1.gif).

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
| Demo 2 scalar S-curve time law (seed `20260816`, v≤0.55 a≤1.8 j≤8) | **1751 samples, 3.50 s**, peak joint \|qd\| 0.550 rad/s |
| Demo 2 tracking (PD + vel FF + gravity compensation) | worst 0.0046 rad · final goal 0.0002 rad |
| Timed-trajectory determinism | identical `q(t)` given identical seed + limits |
| Demo 3 carry (pick→place around table pillar, seed `20260816`) | **~4 ms**, 4 waypoints after shortcut |
| Demo 3 tracking (full pick-and-place) | worst 0.0037 rad |

The 1 s median planning-time exit criterion is met by two orders of magnitude. The IK stress test samples targets from valid random joint configurations; the 2.2% non-converged slice is reported without assigning an untested cause. The solver honors limits by construction (clamped every iteration, asserted in tests).

## Robustness envelope (simulation)

`robustness_bench` replays one deterministic, collision-free trajectory across seven plant conditions and four controller ablations: nominal, a rigid tool payload, reduced actuator gains, added damping, command latency, an external torque pulse, and a combined shift. Model-based controllers receive gravity/Coriolis bias only from the unchanged nominal model, not the perturbed plant.

The generated [full results](docs/robustness_results.md) and [raw CSV](docs/robustness_results.csv) state their pass thresholds and limitations. This is a simulation stress test, not hardware validation or a sim-to-real guarantee.

## Multi-scene, multi-query extension (simulation)

`multi_query_bench` adds nine fixed scene-query fixtures (three per shipped MJCF scene, six unique joint-pair definitions) while retaining the original declared numeric tracking thresholds. Selected joint pairs are deliberately repeated across scenes to isolate geometry effects. Across five fixed planner seeds per fixture, all **30/30 direct-free** and **15/15 obstructed** trials succeeded. Executed states are checked after every settling, path, and hold step with a threshold of exactly 0.0 m; a case passes only with zero sampled robot contacts whose signed distance is negative. This is a sampled penetration gate, not a positive-clearance certificate.

Position PD meets the numeric gates in **0/18** tracking cases. Desired-velocity feedforward meets the numeric gates in **14/18** cases and passes the full zero-penetration gate in **13/18**. The four sampled-penetration cases are all the retained `reverse_cross_workspace` negative: 25--72 path steps depending on plant/controller, with 0.050--0.079 mm maximum actual penetration; settling and hold remain penetration-free. The generated [report](docs/multi_query_results.md), [45-row planning CSV](docs/multi_query_planning.csv), and [36-row tracking CSV](docs/multi_query_tracking.csv) retain every outcome and include exact joint vectors, seeds, trajectory metrics, per-phase penetration counts/depths and contact identities, pass criteria, and claim boundaries. The fixtures are hand-designed and deterministic, not a sampled task distribution; the results do not estimate hardware or workspace-wide success probability.

## Layout

```
crates/arm-lab        the library: chain extraction, FK, Jacobians, DLS IK, RRT-Connect, S-curve, MuJoCo collision
crates/arm-lab-demo   demo1 (IK), demo2 (pillar dodge), demo3 (pick-and-place), Rerun + offscreen GIF
assets/ur5e           vendored MuJoCo-Menagerie UR5e + cluttered / pick-place scenes (see license)
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

# demo 3: pick-and-place around a pillar, write demo_output/demo3.rrd
cargo run --release -p arm-lab-demo --bin demo3

# demo 3 + offscreen GIF
cargo run --release -p arm-lab-demo --bin demo3 -- --render

# stream live into a Rerun viewer started with `rerun`
cargo run --release -p arm-lab-demo --bin demo3 -- --connect

# deterministic controller-ablation × plant-shift matrix; write Markdown + CSV
cargo run --release -p arm-lab-demo --bin robustness_bench -- --write

# three scenes × three fixed queries; write planning/tracking CSVs + report
cargo run --release -p arm-lab-demo --bin multi_query_bench -- --write

# rerun the bounded matrix and verify deterministic committed fields/outcomes
cargo run --release -p arm-lab-demo --bin multi_query_bench -- --check
```

Requirements: Rust stable, a C++ toolchain, and (for `--render`) `ffmpeg` on PATH. On Linux without system MuJoCo, `mujoco-rs` auto-downloads MuJoCo 3.9 at build time. Set `MUJOCO_DOWNLOAD_DIR` to an absolute directory before building and add its downloaded `lib/` directory to `LD_LIBRARY_PATH` before running; see the [mujoco-rs docs](https://github.com/davidhozic/mujoco-rs).

## Design notes

- **Chain extraction, not re-modeling.** `Chain::from_mujoco` walks `body_parentid` from the tip body to the world, collecting static transforms, hinge axes, anchors, and limits from the *compiled* model, plus the EE site as the tool frame. One source of truth for geometry.
- **DLS IK with adaptive damping.** Each step solves `Δq = Jᵀ(JJᵀ + λ²I)⁻¹e` with a diagonal nullspace bias toward a rest pose; λ scales down with the error so the endgame converges Newton-like while near-singular regions stay damped. Joint limits are clamped every iteration; seeded random restarts (TRAC-IK style) recover from bad basins.
- **RRT-Connect, from scratch.** Two trees grow toward each other (Kuffner & LaValle 2000). `EXTEND` takes one joint-space step; `CONNECT` greedily repeats it. Edges are collision-checked discretely by interpolating at `resolution` (0.05 rad L2 by default) and calling MuJoCo `mj_collision` on each state; this is sampled collision checking, not continuous certification. Greedy then random shortcutting removes redundant waypoints; the path is densified to the same resolution for execution. Sampling uses the in-repo SplitMix64 RNG — same seed, same path.
- **Collision filter.** Only contacts that involve a robot collision geom (`contype ≠ 0`, attached to a chain body, not the world) count. Floor-vs-pillar contacts are ignored; parent–child pairs are already excluded by MuJoCo. Visual meshes never participate.
- **Deterministic.** Restart sampling, RRT sampling, and random shortcutting all use the in-repo RNG with a fixed seed. The timed trajectory is bit-stable given the same seed and limits (CI golden test).
- **Scalar S-curve time law.** A rest-to-rest 7-phase bang-bang-jerk profile times the scalar path length `s ∈ [0, L]`. Per-joint `(v, a, j)` limits are converted to path-space limits by the steepest `|dqᵢ/ds|` on the polyline, so no joint exceeds its bound within an edge. Polyline tangent discontinuities are not blended: joint velocity can jump at a corner, so the complete joint trajectory is not globally acceleration- or jerk-bounded.
- **Physics-side servo.** The demos apply the exact MuJoCo bias force as gravity/Coriolis feedforward. Position actuators are commanded as a PD tracker with velocity feedforward: `ctrl = q_des + (kv/kp)·qd_des` yields `τ = kp(q_des − q) + kv(qd_des − qd)`. Worst joint-space tracking is 0.0046 rad (Demo 2) and 0.0037 rad (Demo 3).
- **Scripted grasp.** Demo 3 welds a mocap cube to the EE after the pick descend and parks it on the place pad after the place descend. That is a kinematics/planning demo, not contact-rich grasping. The cube volume is not represented in the planner collision geometry, and carry planning uses a zero contact threshold that only rejects sampled robot penetration. Attached-load and pair-scoped clearance checks remain future work.

## Roadmap

- [x] Demo 1: FK + DLS IK, Rerun telemetry, offscreen-rendered GIF
- [x] RRT-Connect with MuJoCo collision checks and shortcutting (Demo 2)
- [x] Jerk-bounded scalar S-curve time law; joint-space PD + velocity feedforward
- [x] Pick-and-place with obstacle dodging; benchmark tables
- [x] Reproducible controller robustness matrix with raw CSV and explicit sim-only limits
- [x] Multi-scene, multi-query planning and tracking extension with raw CSVs

## License & assets

Code: Apache-2.0. The vendored `assets/ur5e` model and meshes are from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (UR5e description © 2018 ROS Industrial Consortium, BSD-3; see `assets/ur5e/LICENSE`), with a local modification adding offscreen buffer size to `scene.xml`. `scene_cluttered.xml` and `scene_pickplace.xml` are original to this repo.

Research users can cite the software using [`CITATION.cff`](CITATION.cff).
