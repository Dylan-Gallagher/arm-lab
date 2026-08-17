# UR5e multi-scene, multi-query benchmark (simulation)

This deterministic extension evaluates **nine fixed scene-query fixtures (three per scene, six unique joint-pair definitions) across 3 shipped MJCF scenes**. Repeating selected joint pairs across scenes creates controlled geometry comparisons. Five fixed planner seeds per fixture produce 45 planning trials. The canonical-seed trajectory for each fixture is then replayed using two controller variants against the nominal plant and one fixed combined shift, producing 36 tracking trials. Three fixtures have collision-blocked straight interpolants. **This is simulation evidence, not hardware validation or a sim-to-real guarantee.**

Planner headline: **30/30 direct-free trials** and **15/15 obstructed trials** succeeded.

The numeric tracking limits are reused unchanged from the earlier robustness envelope: temporal RMS six-joint L2 error <= 0.03 rad, maximum error <= 0.10 rad, and final error after a 250-step hold <= 0.02 rad. In addition, a tracking case passes only with **zero sampled robot-penetration steps** across settling, path execution, and hold. The audit uses a contact threshold of exactly 0.0 m and counts only MuJoCo-emitted robot contacts with signed distance `< 0`; signed distance, maximum actual penetration, and worst geom/body pair are retained in the raw CSV. This is not a positive-clearance certificate.

`plan_elapsed_ms` is observational wall-clock data: it is machine- and load-dependent and is **not byte-stable**. The bounded `--check` mode ignores only that column while verifying every committed deterministic planning field, all tracking fields/outcomes, and this report.

## Planning and trajectory summary

| Scene | Query | Direct interpolant | Planner success | Cost range (rad) | Canonical trajectory |
|---|---|:---:|:---:|---:|---:|
| open_floor | positive_pan | free | 5/5 | 1.100--1.100 | 1267 samples / 2.53 s |
| open_floor | shoulder_elbow | free | 5/5 | 0.805--0.805 | 767 samples / 1.53 s |
| open_floor | wrist_reorientation | free | 5/5 | 1.058--1.058 | 903 samples / 1.80 s |
| offset_pillar | positive_pan | blocked | 5/5 | 1.803--2.541 | 1919 samples / 3.84 s |
| offset_pillar | negative_pan | free | 5/5 | 1.000--1.000 | 1176 samples / 2.35 s |
| offset_pillar | shoulder_elbow | free | 5/5 | 0.805--0.805 | 767 samples / 1.53 s |
| tabletop_pillar | cross_workspace | blocked | 5/5 | 1.518--2.206 | 1767 samples / 3.53 s |
| tabletop_pillar | reverse_cross_workspace | blocked | 5/5 | 1.475--1.715 | 2060 samples / 4.12 s |
| tabletop_pillar | wrist_reorientation | free | 5/5 | 1.058--1.058 | 903 samples / 1.80 s |

## Tracking results

Each cell is RMS / maximum / final six-joint L2 error in radians, followed by sampled penetration steps in settle/path/hold and maximum actual penetration. `PASS` requires all three numeric limits and zero penetration steps.

| Scene | Query | Plant | Position PD | PD + velocity FF |
|---|---|---|---:|---:|
| open_floor | positive_pan | nominal | 0.0913/0.1094/0.0119; p 0/0/0; pen 0.000 mm FAIL | 0.0117/0.0118/0.0116; p 0/0/0; pen 0.000 mm PASS |
| open_floor | positive_pan | combined moderate shift | 0.0973/0.1163/0.0195; p 0/0/0; pen 0.000 mm FAIL | 0.0195/0.0202/0.0193; p 0/0/0; pen 0.000 mm PASS |
| open_floor | shoulder_elbow | nominal | 0.1117/0.1551/0.0173; p 0/0/0; pen 0.000 mm FAIL | 0.0140/0.0176/0.0176; p 0/0/0; pen 0.000 mm PASS |
| open_floor | shoulder_elbow | combined moderate shift | 0.1187/0.1642/0.0266; p 0/0/0; pen 0.000 mm FAIL | 0.0222/0.0270/0.0271; p 0/0/0; pen 0.000 mm FAIL |
| open_floor | wrist_reorientation | nominal | 0.1245/0.1639/0.0105; p 0/0/0; pen 0.000 mm FAIL | 0.0109/0.0116/0.0098; p 0/0/0; pen 0.000 mm PASS |
| open_floor | wrist_reorientation | combined moderate shift | 0.1338/0.1760/0.0187; p 0/0/0; pen 0.000 mm FAIL | 0.0200/0.0215/0.0176; p 0/0/0; pen 0.000 mm PASS |
| offset_pillar | positive_pan | nominal | 0.1406/0.1899/0.0111; p 0/0/0; pen 0.000 mm FAIL | 0.0093/0.0122/0.0117; p 0/0/0; pen 0.000 mm PASS |
| offset_pillar | positive_pan | combined moderate shift | 0.1494/0.1994/0.0187; p 0/0/0; pen 0.000 mm FAIL | 0.0180/0.0241/0.0194; p 0/0/0; pen 0.000 mm PASS |
| offset_pillar | negative_pan | nominal | 0.0897/0.1094/0.0120; p 0/0/0; pen 0.000 mm FAIL | 0.0116/0.0119/0.0117; p 0/0/0; pen 0.000 mm PASS |
| offset_pillar | negative_pan | combined moderate shift | 0.0957/0.1163/0.0196; p 0/0/0; pen 0.000 mm FAIL | 0.0195/0.0200/0.0193; p 0/0/0; pen 0.000 mm PASS |
| offset_pillar | shoulder_elbow | nominal | 0.1117/0.1551/0.0173; p 0/0/0; pen 0.000 mm FAIL | 0.0140/0.0176/0.0176; p 0/0/0; pen 0.000 mm PASS |
| offset_pillar | shoulder_elbow | combined moderate shift | 0.1187/0.1642/0.0266; p 0/0/0; pen 0.000 mm FAIL | 0.0222/0.0270/0.0271; p 0/0/0; pen 0.000 mm FAIL |
| tabletop_pillar | cross_workspace | nominal | 0.0926/0.1204/0.0152; p 0/0/0; pen 0.000 mm FAIL | 0.0129/0.0151/0.0152; p 0/0/0; pen 0.000 mm PASS |
| tabletop_pillar | cross_workspace | combined moderate shift | 0.0992/0.1277/0.0239; p 0/0/0; pen 0.000 mm FAIL | 0.0213/0.0251/0.0240; p 0/0/0; pen 0.000 mm FAIL |
| tabletop_pillar | reverse_cross_workspace | nominal | 0.0901/0.1362/0.0135; p 0/23/0; pen 0.030 mm FAIL | 0.0128/0.0143/0.0131; p 0/18/0; pen 0.031 mm FAIL |
| tabletop_pillar | reverse_cross_workspace | combined moderate shift | 0.0970/0.1472/0.0222; p 0/54/0; pen 0.079 mm FAIL | 0.0211/0.0241/0.0216; p 0/43/0; pen 0.076 mm FAIL |
| tabletop_pillar | wrist_reorientation | nominal | 0.1245/0.1639/0.0105; p 0/0/0; pen 0.000 mm FAIL | 0.0109/0.0116/0.0098; p 0/0/0; pen 0.000 mm PASS |
| tabletop_pillar | wrist_reorientation | combined moderate shift | 0.1338/0.1760/0.0187; p 0/0/0; pen 0.000 mm FAIL | 0.0200/0.0215/0.0176; p 0/0/0; pen 0.000 mm PASS |

## Aggregate result

- Direct-free planner trials: 30/30 succeeded.
- Obstructed planner trials: 15/15 succeeded.
- Position PD: 0/18 meet the numeric tracking gates; 0/18 pass after the zero-penetration requirement.
- PD + velocity feedforward: 14/18 meet the numeric tracking gates; 13/18 pass after the zero-penetration requirement.
- Executed-penetration audit: 4/36 cases have sampled robot penetration; 138 total phase-steps, maximum actual penetration 0.00007856 m.


## Executed penetration cases

Penetration steps are shown as settle/path/hold. The worst emitted contact is selected by maximum actual penetration; every listed signed distance is below zero.

| Scene | Query | Plant | Controller | Steps S/P/H | Max penetration (m) | Worst phase / signed distance / geom pair |
|---|---|---|---|---:|---:|---|
| tabletop_pillar | reverse_cross_workspace | nominal | position PD | 0/23/0 | 0.00003041 | path / -0.00003041 / geom#27[27]@wrist_2_link vs pillar[31]@pillar |
| tabletop_pillar | reverse_cross_workspace | nominal | PD + velocity FF | 0/18/0 | 0.00003100 | path / -0.00003100 / geom#27[27]@wrist_2_link vs pillar[31]@pillar |
| tabletop_pillar | reverse_cross_workspace | combined moderate shift | position PD | 0/54/0 | 0.00007856 | path / -0.00007856 / geom#27[27]@wrist_2_link vs pillar[31]@pillar |
| tabletop_pillar | reverse_cross_workspace | combined moderate shift | PD + velocity FF | 0/43/0 | 0.00007565 | path / -0.00007565 / geom#27[27]@wrist_2_link vs pillar[31]@pillar |

The fixed `tabletop_pillar/reverse_cross_workspace` fixture is retained in full, including any penetration-failing outcomes; no fixture or failed case is removed from either artifact.

## Exact scope and limitations

- The fixtures are deterministic and hand-designed, not sampled from a scene or query distribution. These counts are not estimates of a workspace-wide success probability.
- All scenes use the same UR5e model and actuator interface. The open scene contains only a floor; the other two scenes are distinct layouts but both use pillar-like obstacles.
- Five planner seeds probe sampling variability, but controller tracking uses one canonical path per query. `plan_elapsed_ms` is machine- and load-dependent, is not byte-stable, and is the only field ignored by `--check`.
- Only position PD and its desired-velocity-feedforward variant are compared here. This extension does not show that nominal-bias or integral-residual results generalize across queries.
- The combined plant is one deterministic condition: 1 kg payload at 0.10 m, 80% actuator gains, +1 Nms/rad joint damping, 10 ms command latency, and a 10 Nm / 120 ms shoulder-lift pulse at 45% of each trajectory. It is not a randomized uncertainty distribution.
- Planner collision checks remain discrete at 0.05 rad in joint-space L2 and use MuJoCo's emitted-contact set. The repository's positive contact threshold filters emitted candidates; with these zero-margin geoms it does not establish positive geometric clearance. Planner behavior is retained, but no 1-mm-clearance claim is made.
- The execution gate checks signed distance `< 0` at each 2-ms simulation state in settle, path, and hold. It detects sampled penetration, not positive-distance near misses or continuous swept-volume collision between samples. Each shortcut edge uses a rest-to-rest S-curve, making position, velocity, and acceleration continuous at full-stop corners while keeping jerk bounded almost everywhere; this is not geometric corner blending or a time-optimal parameterization.
- There is no sensor noise, contact-rich grasping, hardware experiment, or sim-to-real guarantee.

## Reproduce

```bash
cargo run --release -p arm-lab-demo --bin multi_query_bench -- --write
cargo run --release -p arm-lab-demo --bin multi_query_bench -- --check
```

Raw artifacts: `docs/multi_query_planning.csv` (all 45 planner trials, including exact joint vectors) and `docs/multi_query_tracking.csv` (all 36 tracking trials, numeric metrics, per-phase penetration counts/depths, and worst emitted-contact identities).
