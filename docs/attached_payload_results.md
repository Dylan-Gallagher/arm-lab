# Demo 3 attached-payload proxy results

The [protocol](attached_payload_protocol.md) was committed before implementation
or result generation as
`5c36fd461749888c3a9e9168205ff0283a4ed8ac`. The implementation keeps the
original arm collision predicate and RRT-Connect algorithm, then adds a second,
pair-scoped predicate during the carry only: the 50 mm cube proxy must remain at
least 5 mm from the named `floor`, `table`, and `pillar` geoms at sampled states.

Targeted reproduction (after configuring the MuJoCo shared-library path as
described in the repository README):

```bash
cargo test --release -p arm-lab --test pickplace attached_cube_blocks_straight_carry_and_planned_path_is_sampled_clear -- --exact --nocapture
```

## Fixed-query result

All figures below are deterministic geometry/path fields from seed `20260816`;
wall-clock time is not part of the reproducibility claim.

| Gate | Result |
|---|---:|
| Pick-approach endpoint | robot-free and payload-clear |
| Place-approach endpoint | robot-free and payload-clear |
| Straight-edge samples at 0.05 rad L2 | 23 |
| Straight samples with robot collision | 9 |
| Straight samples violating payload margin | 11 |
| Straight minimum cube/environment signed distance | **-0.065709578 m**, cube vs pillar |
| Combined fixed-seed RRT result | **success** |
| Shortcut waypoints / densified samples | **6 / 47** |
| Densified path robot-collision samples | **0** |
| Densified path payload-margin violations | **0** |
| Densified path minimum cube/environment distance | **0.011848632 m**, cube vs pillar |
| Margin above the declared 0.005 m payload threshold | **0.006848632 m** at sampled states |
| Joint-space path cost | 2.251250420 rad |

Both predicates independently block the naive straight edge (`robot=true`,
`payload=true`). Two fresh checker/planner instances produce bit-identical
shortcut waypoints and densified paths. A fresh third checker re-audits every
returned state rather than trusting the planner result object.

The compiled-pose regression also uses a separate MJCF fixture with a nonzero
EE attachment transform and a nonzero proxy geom-local translation and
rotation. MuJoCo's resulting `geom_xpos`/`geom_xmat` agree with
`FK(q) * T_EE_proxy` to less than `1e-10` m and rad, demonstrating that the
checker positions the compiled geom—not merely the mocap body origin—as
declared.

Two repeated headless Demo 3 runs planned the attached-cube carry in 5.6 ms and
5.4 ms, respectively. Both produced the same 6 shortcut waypoints, 5.13 s
corner-stop timed carry, and 0.0022 rad worst joint-tracking error. The timing is
illustrative and can vary with the machine and load; the path fields and
audited outcomes are the deterministic evidence.

## Negative outcomes retained

- The naive straight carry is unsafe under both scoped checks: 9/23 sampled
  states have a robot collision and 11/23 violate the cube margin.
- Adding the payload gate changes the old 4-waypoint carry into a 6-waypoint,
  47-sample path. The later corner-stop time law increases carry duration while
  reducing the repeated full-demo worst tracking error to 0.0022 rad; neither
  change is hidden.
- A global 40 mm robot threshold is not used. The positive 5 mm rule applies
  only to the declared cube/environment pairs; the original zero-penetration
  robot rule remains unchanged during carry.

## Claim limits

This is a sampled discrete check at 0.05 rad joint-space spacing, not continuous
swept-volume collision detection. The proxy is one rigid box at a fixed EE
transform. It does not model grasp uncertainty, object slip, compliance,
fingers, calibration error, contact-rich grasping, or unlisted scene geometry.
Cube-versus-robot checks are intentionally excluded because the attachment
region needs an explicit allowed-contact model; the configured pair scope and
constructor tests prevent wrist geoms from entering the environment list by
accident. A contact-enabled synthetic fixture additionally proves that an
actual penetrating robot/proxy contact is excluded while the independently
scoped payload/environment predicate remains active. The result is simulation
evidence for one deterministic query, not a workspace-wide or hardware-safety
certificate.
