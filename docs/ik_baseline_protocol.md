# Independent IK baseline protocol

Frozen before implementing the evaluator or running any new result-bearing IK
comparison. The repository's existing test already reports the default
arm-lab solver at 97.8% on this cohort; that control is therefore **not blind**.
The cold-start ablation, the independent-solver outcomes, paired cells, and
all derived comparisons are unknown at protocol freeze time.

## Question

On one deterministic set of full-pose targets that are reachable by
construction, how often do:

1. arm-lab's adaptive damped-least-squares solver;
2. the same solver without random restarts; and
3. the independently maintained `k` crate's Jacobian solver, with and without
   the same restart starts,

meet the same strict pose-error thresholds from the same initial conditions?

This is a convergence and cross-implementation study. It is not a runtime
ranking, a collision-aware reachability benchmark, or evidence about hardware.
Every result is retained regardless of which implementation wins.

## Frozen source and dependencies

- arm-lab base revision: `a5321fe6e0c4851bf5d5f5a964268b4f0212ea95`
- public evidence release: `v0.3.0` at `392e7980b50f4e544a3163339bbdc6469f7bb942`
- robot model: vendored `assets/ur5e/ur5e.xml`
- arm-lab target and evaluation kinematics: chain extracted from the compiled
  MuJoCo model, tip body `wrist_3_link`, site `attachment_site`
- independent solver: crates.io `k` exactly `0.32.0`
- independent model input: arm-lab's generated URDF, parsed through
  `urdf-rs`; the existing 50-configuration cross-check establishes agreement
  below `1e-9` in position and orientation for this generated chain

The implementation may add a new artifact generator and pin the already-used
development dependencies exactly. It may not alter the library IK algorithm,
the compiled robot model, the existing 1,000-target test, or any released
artifact while producing this evidence.

## Target cohort

The evaluator recreates the exact existing `ik_stats` cohort rather than
selecting a new favorable sample:

- target count: **1,000**;
- deterministic SplitMix64 seed: `0x1C2026`;
- each target joint is sampled independently and uniformly from the compiled
  joint interval with a `0.05 rad` inset at each bound;
- every target pose is `arm_lab::kinematics::fk(q_target)`;
- all six position and orientation task rows are active;
- initial and arm-lab rest configuration, in chain order:
  `[-pi/2, -pi/2, pi/2, -pi/2, -pi/2, 0]`.

The complete target joint vectors and target poses are serialized before the
result rows in `docs/ik_baseline_targets.csv`. The evaluator must regenerate
them rather than reading outcomes from the existing test.

Targets are reachable under the exact arm-lab kinematic model because they are
generated with forward kinematics from in-limit configurations. They are not
uniform in Cartesian workspace, filtered for collision, representative of a
task distribution, or guaranteed to have a collision-free approach from home.

## Variants

All variants are evaluated on all 1,000 targets in the following fixed order.
Success is recomputed with arm-lab FK from the returned joint vector; a solver's
own status is not sufficient.

### `arm_cold`

- `IkConfig::default()` except `restarts = 0`;
- adaptive damping, `lambda = 0.05`;
- diagonal rest-pose bias, `nullspace_gain = 0.05`;
- step gain `1.0`;
- maximum 200 iterations;
- tolerances `1e-4 m` position and `1e-3 rad` orientation.

### `arm_restarts`

- unchanged `IkConfig::default()`;
- the home start followed, on failure, by at most eight deterministic uniform
  restart starts;
- restart RNG seed `0 ^ 0x1CF7EE` and the compiled chain's existing sampler;
- all other fields identical to `arm_cold`.

### `k_cold`

- `k::JacobianIkSolver` constructed with position tolerance `1e-4`, rotation
  tolerance `1e-3`, its documented/default Jacobian multiplier `0.5`, and 200
  maximum iterations;
- one solve from the same home configuration;
- joint clamping and the generated URDF limits are handled by `k`.

### `k_restarts`

- the same independent solver configuration as `k_cold`;
- home followed, on failure, by the **same eight restart joint vectors** used
  by `arm_restarts`, in the same order;
- at most nine solver calls of at most 200 iterations each.

The maximum attempt and iteration counts are matched between the two restart
conditions, but the implementations use different linear algebra, update
rules, damping, early stopping, and per-iteration work. Do not compare wall
time or claim compute matching.

## Common outcome rule

For a returned joint vector `q`, compute the end-effector pose with arm-lab FK.
The row is successful only when all of the following hold:

- every joint is finite and within its compiled limit (tolerance `1e-12`);
- position error is strictly below `1e-4 m`;
- shortest-representation world-frame orientation error is strictly below
  `1e-3 rad`.

For an independent-solver error or exhausted restart sequence, the row is a
failure. Periodic or alternative joint solutions are allowed; only the common
pose and limit rule determines success. Failure errors may be retained for
diagnosis but are not compared across implementations because `k` restores an
attempt's initial state after a failed call while arm-lab returns its best
attempt.

## Required artifacts

- `docs/ik_baseline_targets.csv`: exactly 1,000 target rows with full joint and
  pose serialization;
- `docs/ik_baseline_results.csv`: exactly 4,000 rows, one per target/variant,
  including common success status, successful solution joints, and common
  final pose errors;
- `docs/ik_baseline_results.md`: generated aggregate report with all outcomes;
- `docs/ik_baseline_validation.md`: exact commands, environment, hashes,
  repeatability result, and claim limitations;
- one deterministic artifact generator supporting `--write` and `--check`.

No wall-clock field belongs in a generated artifact. Numeric serialization must
use fixed precision sufficient to reproduce the common success rule.

## Predeclared analyses

Report, without outcome-dependent subgroup selection:

1. success count/rate and Wilson 95% interval for all four variants;
2. paired `arm_cold` versus `arm_restarts` cells and restart-only recoveries;
3. paired `k_cold` versus `k_restarts` cells and restart-only recoveries;
4. paired `arm_restarts` versus `k_restarts` cells (`both`, `arm only`,
   `k only`, `neither`);
5. an exact two-sided McNemar/binomial sign test for the discordant
   `arm_restarts`/`k_restarts` cells, labeled descriptive for this single
   deterministic target cohort;
6. the union of targets solved by either restart-enabled implementation; and
7. successful-row worst and median common pose errors for each variant.

Do not add post-hoc target categories, tune any solver field after observing
outcomes, omit failures, or headline iteration/runtime numbers. Unexpected
failures or an independent baseline that outperforms arm-lab are publishable
results, not reasons to change the cohort or configuration.

## Verification gates

Before publication:

- the target and result row counts and unique keys must be exact;
- all target and successful solution joint vectors must pass the compiled-limit
  audit;
- every stored success boolean must equal a fresh common FK/error check;
- both restart-enabled variants must use byte-identical restart vectors for a
  given restart index;
- two consecutive full runs must match every generated byte;
- `--check` must reject a one-byte stale-artifact mutation in a temporary copy
  or an equivalent unit test must prove stale artifacts fail;
- workspace formatting, strict Clippy, all existing release tests, and the new
  generator tests must pass;
- existing randomized, multi-scene, attached-payload, corner-stop, planner,
  and robustness artifact checks must remain unchanged.

Only after these gates pass may the report and README be updated. Publication
language must say full-pose, reachable-by-construction, simulation-only,
collision-unaware IK; it must preserve the known/non-blind status of the 97.8%
control and avoid universal solver rankings.
