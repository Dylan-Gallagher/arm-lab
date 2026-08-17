# Randomized UR5e planning and nominal tracking evaluation (simulation)

This result follows the predeclared `docs/randomized_eval_protocol.md`. It retains the first 100 accepted independently sampled joint-space queries in each of three shipped scenes, without filtering on direct-path or planner outcome. **This is deterministic simulation evidence, not hardware validation, a workspace-uniform task distribution, continuous safety, or sim-to-real evidence.**

## Query sampling and canonical planning

| Scene | Accepted draw | Endpoint/short rejections | Direct baseline | Canonical default RRT | Blocked recovered |
|---|---:|---:|---:|---:|---:|
| open_floor | 597 | 367/130/0 | 33/100 (33.0%; 95% Wilson 24.6--42.7%) | 47/100 (47.0%; 95% Wilson 37.5--56.7%) | 14/67 |
| offset_pillar | 908 | 593/215/0 | 20/100 (20.0%; 95% Wilson 13.3--28.9%) | 46/100 (46.0%; 95% Wilson 36.6--55.7%) | 26/80 |
| tabletop_pillar | 997 | 689/208/0 | 30/100 (30.0%; 95% Wilson 21.9--39.6%) | 45/100 (45.0%; 95% Wilson 35.6--54.8%) | 15/70 |

Equal-scene pooled direct baseline: **83/300 (27.7%; 95% Wilson 22.9--33.0%)**. Canonical default RRT: **138/300 (46.0%; 95% Wilson 40.4--51.7%)**. It recovers **55/217** blocked-direct queries.

All 300 endpoint pairs pass the frozen endpoint predicate by construction. Canonical outcomes retain **138 success** and **162 unconnected** statuses; no failed query is removed.

## Blocked-query goal-bias ablation

| Scene | Blocked queries | Default successes | Zero-goal-bias successes |
|---|---:|---:|---:|
| open_floor | 67 | 67/335 | 69/335 |
| offset_pillar | 80 | 129/400 | 126/400 |
| tabletop_pillar | 70 | 75/350 | 75/350 |

- `default`: 271/1085 successful planner trials. Per-query success histogram (0/5 through 5/5): [161, 1, 1, 1, 0, 53].
- `zero_goal_bias`: 270/1085 successful planner trials. Per-query success histogram (0/5 through 5/5): [161, 1, 1, 1, 1, 52].

Repeated planner trials probe algorithmic seed sensitivity on the same blocked queries; they are not independent task samples and receive no Wilson interval.

## Nominal tracking

Tracking is replayed only when the canonical default plan succeeds. Both controllers receive each identical successful trajectory.

| Controller | Replays | Numeric gates | Full zero-penetration gate |
|---|---:|---:|---:|
| position PD | 138 | 0/138 (0.0%; 95% Wilson 0.0--2.7%) | 0/138 (0.0%; 95% Wilson 0.0--2.7%) |
| PD + velocity FF | 138 | 104/138 (75.4%; 95% Wilson 67.6--81.8%) | 90/138 (65.2%; 95% Wilson 57.0--72.7%) |

### Scene and sampled-penetration breakdown

| Scene | Controller | Replays | Numeric | Full | Penetration cases | Steps S/P/H | Max penetration (m) |
|---|---|---:|---:|---:|---:|---:|---:|
| open_floor | position PD | 47 | 0/47 | 0/47 | 7/47 | 180/729/103 | 0.00026311 |
| open_floor | PD + velocity FF | 47 | 38/47 | 33/47 | 6/47 | 180/889/250 | 0.00090529 |
| offset_pillar | position PD | 46 | 0/46 | 0/46 | 7/46 | 345/685/250 | 0.00090942 |
| offset_pillar | PD + velocity FF | 46 | 35/46 | 30/46 | 7/46 | 345/765/250 | 0.00148761 |
| tabletop_pillar | position PD | 45 | 0/45 | 0/45 | 5/45 | 0/604/250 | 0.00321406 |
| tabletop_pillar | PD + velocity FF | 45 | 31/45 | 27/45 | 5/45 | 0/750/250 | 0.00321387 |

Paired full-gate discordance: **90 FF-pass/PD-fail** trajectories and **0 PD-pass/FF-fail** trajectories. Canonical planning failures with no replay: **162**. Sampled execution penetration occurs in **37/276** tracking cases; maximum depth is **0.00321406 m**.

## Scope and limitations

- The accepted pairs are uniform in compiled joint limits conditional on two collision-free endpoints and at least 0.75-rad separation. They are not uniform in Cartesian workspace or representative of a deployment task distribution.
- Wilson intervals describe repeated draws from this declared conditional generator only. The generator, scenes, and robot model remain fixed.
- Direct and RRT collision checks sample joint-space edges at 0.05-rad L2 spacing and use the existing MuJoCo emitted-contact predicate. They do not certify continuous collision avoidance or positive clearance.
- Tracking uses one canonical successful path per accepted query, the nominal plant only, and sampled `dist < 0` execution checks at 2-ms states. Planning failures receive no tracking replay and remain visible in the denominator.
- Trajectories use a rest-to-rest S-curve on every shortcut edge. Full-stop corners make joint position, velocity, and acceleration continuous and keep jerk bounded almost everywhere, but they are not geometric blends or time-optimal trajectories.
- The goal-bias comparison changes one planner field; it is an ablation, not a comparison with an independent planning implementation.
- There is no randomized plant uncertainty, sensing, localization, grasping, payload, hardware, or sim-to-real evidence in this extension.

## Reproduce

```bash
cargo run --release -p arm-lab-demo --bin randomized_eval -- --write
cargo run --release -p arm-lab-demo --bin randomized_eval -- --check
```

Raw artifacts retain all 300 accepted queries, every canonical and blocked-query replicate plan, and both controller replays for every successful canonical path. `plan_elapsed_ms` is observational wall time and the only field normalized by `--check`.
