# UR5e multi-scene, multi-query benchmark (simulation)

This deterministic extension evaluates **nine fixed scene-query fixtures (three per scene, six unique joint-pair definitions) across 3 shipped MJCF scenes**. Repeating selected joint pairs across scenes creates controlled geometry comparisons. Five fixed planner seeds per fixture produce 45 planning trials. The canonical-seed trajectory for each fixture is then replayed using two controller variants against the nominal plant and one fixed combined shift, producing 36 tracking trials. Three fixtures have collision-blocked straight interpolants. **This is simulation evidence, not hardware validation or a sim-to-real guarantee.**

The tracking pass limits are reused unchanged from the earlier robustness envelope: temporal RMS six-joint L2 error <= 0.03 rad, maximum error <= 0.10 rad, and final error after a 250-step hold <= 0.02 rad. They were declared in code before this benchmark was executed.

## Planning and trajectory summary

| Scene | Query | Direct interpolant | Planner success | Median plan (ms) | Cost range (rad) | Canonical trajectory |
|---|---|:---:|:---:|---:|---:|---:|
| open_floor | positive_pan | free | 5/5 | 0.122 | 1.100--1.100 | 1267 samples / 2.53 s |
| open_floor | shoulder_elbow | free | 5/5 | 0.094 | 0.805--0.805 | 767 samples / 1.53 s |
| open_floor | wrist_reorientation | free | 5/5 | 0.123 | 1.058--1.058 | 903 samples / 1.80 s |
| offset_pillar | positive_pan | blocked | 5/5 | 6.869 | 1.803--2.541 | 1751 samples / 3.50 s |
| offset_pillar | negative_pan | free | 5/5 | 0.124 | 1.000--1.000 | 1176 samples / 2.35 s |
| offset_pillar | shoulder_elbow | free | 5/5 | 0.112 | 0.805--0.805 | 767 samples / 1.53 s |
| tabletop_pillar | cross_workspace | blocked | 5/5 | 4.346 | 1.518--2.206 | 1515 samples / 3.03 s |
| tabletop_pillar | reverse_cross_workspace | blocked | 5/5 | 5.831 | 1.475--1.715 | 1679 samples / 3.35 s |
| tabletop_pillar | wrist_reorientation | free | 5/5 | 0.134 | 1.058--1.058 | 903 samples / 1.80 s |

## Tracking results

Each cell is RMS / maximum / final six-joint L2 error in radians. `PASS` requires all three declared limits.

| Scene | Query | Plant | Position PD | PD + velocity FF |
|---|---|---|---:|---:|
| open_floor | positive_pan | nominal | 0.0913 / 0.1095 / 0.0119 FAIL | 0.0117 / 0.0118 / 0.0116 PASS |
| open_floor | positive_pan | combined moderate shift | 0.0973 / 0.1163 / 0.0195 FAIL | 0.0195 / 0.0202 / 0.0193 PASS |
| open_floor | shoulder_elbow | nominal | 0.1117 / 0.1552 / 0.0173 FAIL | 0.0140 / 0.0176 / 0.0176 PASS |
| open_floor | shoulder_elbow | combined moderate shift | 0.1188 / 0.1643 / 0.0266 FAIL | 0.0222 / 0.0270 / 0.0271 FAIL |
| open_floor | wrist_reorientation | nominal | 0.1245 / 0.1640 / 0.0105 FAIL | 0.0109 / 0.0116 / 0.0098 PASS |
| open_floor | wrist_reorientation | combined moderate shift | 0.1339 / 0.1760 / 0.0187 FAIL | 0.0200 / 0.0215 / 0.0176 PASS |
| offset_pillar | positive_pan | nominal | 0.1409 / 0.1719 / 0.0111 FAIL | 0.0097 / 0.0121 / 0.0117 PASS |
| offset_pillar | positive_pan | combined moderate shift | 0.1498 / 0.1852 / 0.0187 FAIL | 0.0186 / 0.0241 / 0.0194 PASS |
| offset_pillar | negative_pan | nominal | 0.0897 / 0.1095 / 0.0120 FAIL | 0.0117 / 0.0119 / 0.0117 PASS |
| offset_pillar | negative_pan | combined moderate shift | 0.0957 / 0.1163 / 0.0196 FAIL | 0.0195 / 0.0200 / 0.0193 PASS |
| offset_pillar | shoulder_elbow | nominal | 0.1117 / 0.1552 / 0.0173 FAIL | 0.0140 / 0.0176 / 0.0176 PASS |
| offset_pillar | shoulder_elbow | combined moderate shift | 0.1188 / 0.1643 / 0.0266 FAIL | 0.0222 / 0.0270 / 0.0271 FAIL |
| tabletop_pillar | cross_workspace | nominal | 0.0998 / 0.1204 / 0.0152 FAIL | 0.0132 / 0.0151 / 0.0152 PASS |
| tabletop_pillar | cross_workspace | combined moderate shift | 0.1067 / 0.1278 / 0.0239 FAIL | 0.0218 / 0.0252 / 0.0240 FAIL |
| tabletop_pillar | reverse_cross_workspace | nominal | 0.1033 / 0.1213 / 0.0134 FAIL | 0.0129 / 0.0143 / 0.0131 PASS |
| tabletop_pillar | reverse_cross_workspace | combined moderate shift | 0.1108 / 0.1315 / 0.0221 FAIL | 0.0213 / 0.0241 / 0.0216 FAIL |
| tabletop_pillar | wrist_reorientation | nominal | 0.1245 / 0.1640 / 0.0105 FAIL | 0.0109 / 0.0116 / 0.0098 PASS |
| tabletop_pillar | wrist_reorientation | combined moderate shift | 0.1339 / 0.1760 / 0.0187 FAIL | 0.0200 / 0.0215 / 0.0176 PASS |

## Aggregate result

- Planner: 45/45 fixed-seed trials succeeded.
- Position PD: 0/18 tracking cases passed.
- PD + velocity feedforward: 14/18 tracking cases passed.

All 4 velocity-feedforward misses pass the RMS and maximum-error gates but fail the unchanged final-hold gate, with final errors from 0.0216 to 0.0271 rad.

## Exact scope and limitations

- The fixtures are deterministic and hand-designed, not sampled from a scene or query distribution. These counts are not estimates of a workspace-wide success probability.
- All scenes use the same UR5e model and actuator interface. The open scene contains only a floor; the other two scenes are distinct layouts but both use pillar-like obstacles.
- Five planner seeds probe sampling variability, but controller tracking uses one canonical path per query. Wall-clock planning times are machine- and load-dependent.
- Only position PD and its desired-velocity-feedforward variant are compared here. This extension does not show that nominal-bias or integral-residual results generalize across queries.
- The combined plant is one deterministic condition: 1 kg payload at 0.10 m, 80% actuator gains, +1 Nms/rad joint damping, 10 ms command latency, and a 10 Nm / 120 ms shoulder-lift pulse at 45% of each trajectory. It is not a randomized uncertainty distribution.
- Collision checks remain discrete at 0.05 rad in joint-space L2. Polyline corners remain unblended, so the scalar time law does not certify global acceleration or jerk. There is no sensor noise, contact-rich grasping, or hardware experiment.

## Reproduce

```bash
cargo run --release -p arm-lab-demo --bin multi_query_bench -- --write
```

Raw artifacts: `docs/multi_query_planning.csv` (all 45 planner trials, including exact joint vectors) and `docs/multi_query_tracking.csv` (all 36 tracking trials and complete metrics).
