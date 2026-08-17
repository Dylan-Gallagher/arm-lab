# UR5e multi-scene, multi-query benchmark (simulation)

This deterministic extension evaluates **nine fixed scene-query fixtures (three per scene, six unique joint-pair definitions) across 3 shipped MJCF scenes**. Repeating selected joint pairs across scenes creates controlled geometry comparisons. Five fixed planner seeds per fixture produce 45 planning trials. The canonical-seed trajectory for each fixture is then replayed using two controller variants against the nominal plant and one fixed combined shift, producing 36 tracking trials. Three fixtures have collision-blocked straight interpolants. **This is simulation evidence, not hardware validation or a sim-to-real guarantee.**

Planner headline: **30/30 direct-free trials** and **15/15 obstructed trials** succeeded.

The numeric tracking limits are reused unchanged from the earlier robustness envelope: temporal RMS six-joint L2 error <= 0.03 rad, maximum error <= 0.10 rad, and final error after a 250-step hold <= 0.02 rad. In addition, a tracking case now passes only with **zero robot-collision steps** across settling, path execution, and hold under the repository's 0.001-m clearance semantics. Signed contact distance, actual penetration, clearance violation, and worst geom pair are retained in the raw CSV.

`plan_elapsed_ms` is observational wall-clock data: it is machine- and load-dependent and is **not byte-stable**. The bounded `--check` mode ignores only that column while verifying every committed deterministic planning field, all tracking fields/outcomes, and this report.

## Planning and trajectory summary

| Scene | Query | Direct interpolant | Planner success | Cost range (rad) | Canonical trajectory |
|---|---|:---:|:---:|---:|---:|
| open_floor | positive_pan | free | 5/5 | 1.100--1.100 | 1267 samples / 2.53 s |
| open_floor | shoulder_elbow | free | 5/5 | 0.805--0.805 | 767 samples / 1.53 s |
| open_floor | wrist_reorientation | free | 5/5 | 1.058--1.058 | 903 samples / 1.80 s |
| offset_pillar | positive_pan | blocked | 5/5 | 1.803--2.541 | 1751 samples / 3.50 s |
| offset_pillar | negative_pan | free | 5/5 | 1.000--1.000 | 1176 samples / 2.35 s |
| offset_pillar | shoulder_elbow | free | 5/5 | 0.805--0.805 | 767 samples / 1.53 s |
| tabletop_pillar | cross_workspace | blocked | 5/5 | 1.518--2.206 | 1515 samples / 3.03 s |
| tabletop_pillar | reverse_cross_workspace | blocked | 5/5 | 1.475--1.715 | 1679 samples / 3.35 s |
| tabletop_pillar | wrist_reorientation | free | 5/5 | 1.058--1.058 | 903 samples / 1.80 s |

## Tracking results

Each cell is RMS / maximum / final six-joint L2 error in radians, followed by collision steps in settle/path/hold and maximum actual penetration. `PASS` requires all three numeric limits and zero collision steps.

| Scene | Query | Plant | Position PD | PD + velocity FF |
|---|---|---|---:|---:|
| open_floor | positive_pan | nominal | 0.0913/0.1095/0.0119; c 0/0/0; pen 0.000 mm FAIL | 0.0117/0.0118/0.0116; c 0/0/0; pen 0.000 mm PASS |
| open_floor | positive_pan | combined moderate shift | 0.0973/0.1163/0.0195; c 0/0/0; pen 0.000 mm FAIL | 0.0195/0.0202/0.0193; c 0/0/0; pen 0.000 mm PASS |
| open_floor | shoulder_elbow | nominal | 0.1117/0.1552/0.0173; c 0/0/0; pen 0.000 mm FAIL | 0.0140/0.0176/0.0176; c 0/0/0; pen 0.000 mm PASS |
| open_floor | shoulder_elbow | combined moderate shift | 0.1188/0.1643/0.0266; c 0/0/0; pen 0.000 mm FAIL | 0.0222/0.0270/0.0271; c 0/0/0; pen 0.000 mm FAIL |
| open_floor | wrist_reorientation | nominal | 0.1245/0.1640/0.0105; c 0/0/0; pen 0.000 mm FAIL | 0.0109/0.0116/0.0098; c 0/0/0; pen 0.000 mm PASS |
| open_floor | wrist_reorientation | combined moderate shift | 0.1339/0.1760/0.0187; c 0/0/0; pen 0.000 mm FAIL | 0.0200/0.0215/0.0176; c 0/0/0; pen 0.000 mm PASS |
| offset_pillar | positive_pan | nominal | 0.1409/0.1719/0.0111; c 0/0/0; pen 0.000 mm FAIL | 0.0097/0.0121/0.0117; c 0/0/0; pen 0.000 mm PASS |
| offset_pillar | positive_pan | combined moderate shift | 0.1498/0.1852/0.0187; c 0/0/0; pen 0.000 mm FAIL | 0.0186/0.0241/0.0194; c 0/0/0; pen 0.000 mm PASS |
| offset_pillar | negative_pan | nominal | 0.0897/0.1095/0.0120; c 0/0/0; pen 0.000 mm FAIL | 0.0117/0.0119/0.0117; c 0/0/0; pen 0.000 mm PASS |
| offset_pillar | negative_pan | combined moderate shift | 0.0957/0.1163/0.0196; c 0/0/0; pen 0.000 mm FAIL | 0.0195/0.0200/0.0193; c 0/0/0; pen 0.000 mm PASS |
| offset_pillar | shoulder_elbow | nominal | 0.1117/0.1552/0.0173; c 0/0/0; pen 0.000 mm FAIL | 0.0140/0.0176/0.0176; c 0/0/0; pen 0.000 mm PASS |
| offset_pillar | shoulder_elbow | combined moderate shift | 0.1188/0.1643/0.0266; c 0/0/0; pen 0.000 mm FAIL | 0.0222/0.0270/0.0271; c 0/0/0; pen 0.000 mm FAIL |
| tabletop_pillar | cross_workspace | nominal | 0.0998/0.1204/0.0152; c 0/0/0; pen 0.000 mm FAIL | 0.0132/0.0151/0.0152; c 0/0/0; pen 0.000 mm PASS |
| tabletop_pillar | cross_workspace | combined moderate shift | 0.1067/0.1278/0.0239; c 0/0/0; pen 0.000 mm FAIL | 0.0218/0.0252/0.0240; c 0/0/0; pen 0.000 mm FAIL |
| tabletop_pillar | reverse_cross_workspace | nominal | 0.1033/0.1213/0.0134; c 0/35/0; pen 0.055 mm FAIL | 0.0129/0.0143/0.0131; c 0/25/0; pen 0.050 mm FAIL |
| tabletop_pillar | reverse_cross_workspace | combined moderate shift | 0.1108/0.1315/0.0221; c 0/72/0; pen 0.079 mm FAIL | 0.0213/0.0241/0.0216; c 0/60/0; pen 0.079 mm FAIL |
| tabletop_pillar | wrist_reorientation | nominal | 0.1245/0.1640/0.0105; c 0/0/0; pen 0.000 mm FAIL | 0.0109/0.0116/0.0098; c 0/0/0; pen 0.000 mm PASS |
| tabletop_pillar | wrist_reorientation | combined moderate shift | 0.1339/0.1760/0.0187; c 0/0/0; pen 0.000 mm FAIL | 0.0200/0.0215/0.0176; c 0/0/0; pen 0.000 mm PASS |

## Aggregate result

- Direct-free planner trials: 30/30 succeeded.
- Obstructed planner trials: 15/15 succeeded.
- Position PD: 0/18 meet the numeric tracking gates; 0/18 pass after the zero-collision requirement.
- PD + velocity feedforward: 14/18 meet the numeric tracking gates; 13/18 pass after the zero-collision requirement.
- Executed-contact audit: 4/36 cases have robot collision steps; 192 total phase-steps, maximum actual penetration 0.000079 m, maximum 0.001-m-clearance violation 0.001079 m.


## Executed collision cases

Collision steps are shown as settle/path/hold. The worst contact is selected by maximum clearance violation; signed distance below zero is actual penetration.

| Scene | Query | Plant | Controller | Steps S/P/H | Max penetration (m) | Max clearance violation (m) | Worst phase / signed distance / geom pair |
|---|---|---|---|---:|---:|---:|---|
| tabletop_pillar | reverse_cross_workspace | nominal | position PD | 0/35/0 | 0.00005480 | 0.00105480 | path / -0.00005480 / geom#27[27]@wrist_2_link vs pillar[31]@pillar |
| tabletop_pillar | reverse_cross_workspace | nominal | PD + velocity FF | 0/25/0 | 0.00005013 | 0.00105013 | path / -0.00005013 / geom#27[27]@wrist_2_link vs pillar[31]@pillar |
| tabletop_pillar | reverse_cross_workspace | combined moderate shift | position PD | 0/72/0 | 0.00007925 | 0.00107925 | path / -0.00007925 / geom#27[27]@wrist_2_link vs pillar[31]@pillar |
| tabletop_pillar | reverse_cross_workspace | combined moderate shift | PD + velocity FF | 0/60/0 | 0.00007922 | 0.00107922 | path / -0.00007922 / geom#27[27]@wrist_2_link vs pillar[31]@pillar |

The fixed `tabletop_pillar/reverse_cross_workspace` fixture is retained in full, including any collision-negative outcomes; no fixture or failed case is removed from either artifact.

## Exact scope and limitations

- The fixtures are deterministic and hand-designed, not sampled from a scene or query distribution. These counts are not estimates of a workspace-wide success probability.
- All scenes use the same UR5e model and actuator interface. The open scene contains only a floor; the other two scenes are distinct layouts but both use pillar-like obstacles.
- Five planner seeds probe sampling variability, but controller tracking uses one canonical path per query. `plan_elapsed_ms` is machine- and load-dependent, is not byte-stable, and is the only field ignored by `--check`.
- Only position PD and its desired-velocity-feedforward variant are compared here. This extension does not show that nominal-bias or integral-residual results generalize across queries.
- The combined plant is one deterministic condition: 1 kg payload at 0.10 m, 80% actuator gains, +1 Nms/rad joint damping, 10 ms command latency, and a 10 Nm / 120 ms shoulder-lift pulse at 45% of each trajectory. It is not a randomized uncertainty distribution.
- Planner collision checks remain discrete at 0.05 rad in joint-space L2. Executed collision checks sample each 2-ms simulation state in settle, path, and hold; neither is a continuous swept-volume certificate. Polyline corners remain unblended, so the scalar time law does not certify global acceleration or jerk.
- There is no sensor noise, contact-rich grasping, hardware experiment, or sim-to-real guarantee.

## Reproduce

```bash
cargo run --release -p arm-lab-demo --bin multi_query_bench -- --write
cargo run --release -p arm-lab-demo --bin multi_query_bench -- --check
```

Raw artifacts: `docs/multi_query_planning.csv` (all 45 planner trials, including exact joint vectors) and `docs/multi_query_tracking.csv` (all 36 tracking trials, numeric metrics, per-phase collision counts/depths, and worst contact identities).
