# Corner-stop trajectory results

The [protocol](corner_stop_protocol.md) was committed before implementation or
result-bearing reruns. The implementation replaces a single nonzero-speed
polyline traversal with an independent rest-to-rest seven-phase S-curve on
every nonzero shortcut edge. It samples each interior waypoint once at zero
joint velocity and acceleration.

## Fixed-seed derivative audit

The fixed Demo 2 plan (`seed = 20260816`) has three shortcut waypoints and one
interior corner. At the declared 2 ms sample interval, its generated trajectory
has:

| Quantity | Measured | Declared limit |
|---|---:|---:|
| Samples | 1,919 | — |
| Duration | 3.836 s | — |
| Peak joint speed | 0.550 rad/s | 0.550 rad/s |
| Peak joint acceleration | 1.799 rad/s² | 1.800 rad/s² |
| Peak joint jerk | 7.996 rad/s³ | 8.000 rad/s³ |
| Interior waypoint stops | 1/1 | 1/1 |

The test suite also exercises a non-collinear three-joint corner, checks exact
zero velocity and acceleration at its single waypoint sample, checks analytic
velocity/acceleration/jerk against every joint limit, and bounds the
finite-difference acceleration and jerk across every adjacent sample. Repeated
generation must match exactly, including sampled jerk and timestamps.

The old Demo 2 profile took 3.500 s. Full-stop corner handling increases the
fixed trajectory duration by 0.336 s (9.6%) while reducing the recorded
feedforward-tracking maximum from 0.0046 to 0.0020 rad. This is a measured
fixed-case comparison, not a general performance claim.

## Retained benchmark outcomes

All result-bearing suites were rerun with their previously declared planning,
tracking, and sampled-penetration gates unchanged.

- The 45 fixed multi-query planner trials retain 30/30 direct-free and 15/15
  obstructed successes. Position PD remains 0/18 at the numeric gate; velocity
  feedforward remains 14/18 numeric and 13/18 after the zero-sampled-
  penetration gate.
- The randomized evaluator retains all 300 accepted queries, 138/300 canonical
  planning successes, and 276 controller replays. Position PD remains 0/138;
  velocity feedforward remains 104/138 numeric and 90/138 after the sampled-
  penetration gate.
- The four fixed multi-query penetration cases remain visible. Their aggregate
  sampled path-penetration count changes from 192 to 138 phase-steps and the
  maximum depth changes from 0.07925 to 0.07856 mm.
- The randomized execution audit still finds penetration in 37/276 replays;
  the maximum sampled depth changes from 3.22250 to 3.21406 mm.

The randomized planning rows are byte-identical to the earlier artifact because
trajectory generation is downstream of planning. The tracking artifacts were
regenerated and retain every pass and failure.

## Artifact integrity

| Artifact | Rows including header | SHA-256 |
|---|---:|---|
| `robustness_results.csv` | 29 | `31739fefb228f4073251c95d48d372ead825da1d3c57ac2eba065225542341e2` |
| `multi_query_planning.csv` | 46 | `e517789dd8404afc731836238dd593c98db2a89d93444d9dca4b045280d9a56b` |
| `multi_query_tracking.csv` | 37 | `651f355bf1e7ccb6838a6eab67f5c625a046fef5890d6ca1b8aad1377f73b34f` |
| `randomized_eval_planning.csv` | 2,254 | `b739009c0a4b3691bc910f67ad079a944e1b16c83d341bcd078179bda6c13055` |
| `randomized_eval_tracking.csv` | 277 | `7d4e245efc732ce4fc828b3cd115493b6889a9521232ab32cbdfc5257448e419` |

The benchmark `--check` modes rerun the full deterministic matrices and compare
the committed outputs. Only observational planner wall time is normalized.

## Claim boundary

Position, velocity, and acceleration are continuous across the complete
corner-stop trajectory. Jerk is bounded almost everywhere but can jump between
finite one-sided values at phase and waypoint boundaries, where its classical
derivative need not exist. The method is deliberately full-stop, not a
geometrically blended or time-optimal trajectory.

Planner collision checks remain discrete at 0.05 rad in joint-space L2, and
execution penetration checks sample each 2 ms simulation state. These results
do not establish continuous swept-volume safety, positive robot clearance,
torque feasibility, hardware performance, or sim-to-real transfer.
