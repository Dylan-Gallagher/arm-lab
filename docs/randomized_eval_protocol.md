# Randomized planning and nominal-tracking evaluation protocol

Frozen before generating any result-bearing randomized-evaluation artifact on
2026-08-17 (Europe/Dublin).

## Purpose

The v0.2.0 multi-query benchmark uses nine transparent, hand-designed
scene-query fixtures. This extension asks whether the same planner and two
minimum controller variants behave similarly on joint-space queries drawn from
a declared distribution. It adds evidence; it does not replace or reinterpret
the fixed benchmark.

No planner, controller, collision predicate, trajectory limit, actuator, or
pass threshold may be tuned from this evaluation's outcomes. Every accepted
query and every failure will be retained.

## Query distribution

Evaluate all three shipped scenes independently: `scene.xml`,
`scene_cluttered.xml`, and `scene_pickplace.xml`.

For each scene:

1. initialize the repository's SplitMix64 generator with seed
   `202608170100 + scene_index`;
2. draw start and goal independently using `Chain::sample_uniform`, which is
   uniform inside each compiled joint limit with the repository's existing
   0.05-rad inset;
3. reject the pair if either endpoint triggers the unchanged planner collision
   predicate or if start-goal joint-space L2 distance is below 0.75 rad; and
4. accept the first 100 remaining pairs, without filtering on direct-path
   feasibility or planner outcome.

The maximum is 100,000 attempted pairs per scene. Reaching it before accepting
100 is an implementation/evaluation failure, not permission to weaken the
distribution. Record accepted draw index and aggregate rejection counts.

This is a deterministic sample from a conditional joint-space distribution,
not a workspace-uniform task distribution. The three samples are independent
across scenes and are not paired configurations.

## Planning comparisons

The direct-interpolation baseline uses the existing `edge_free` predicate at
0.05-rad joint-space L2 spacing.

For every accepted query, run the unchanged default RRT-Connect configuration:

- 2,000 maximum iterations;
- 0.25-rad extension step;
- 0.05-rad collision-check resolution;
- 0.05 goal bias;
- 40 random shortcut attempts; and
- densified output.

The planner seed is
`202608180000 + 10000*scene_index + 10*query_index + replicate`, where the
zero-based replicate is 0 for the canonical run.

For every query whose direct interpolant is blocked, run five replicates
(`replicate = 0..4`) of both:

1. the unchanged default planner; and
2. a single ablation that changes only `goal_bias` from 0.05 to 0.0.

The canonical default trial is therefore included in the five default
replicates; do not duplicate it in the artifact. Direct-free queries receive
only the canonical trial because both RRT configurations take the identical
early direct-path return before sampling.

Primary planning outputs, by scene and equally weighted pooled sample, are:

- direct-interpolation success count and Wilson 95% interval;
- canonical default RRT success count and Wilson 95% interval;
- number of blocked-direct queries recovered by canonical RRT;
- all planner status, iterations, nodes, waypoints, sampled path length, and
  non-authoritative wall time; and
- for blocked queries, default-versus-zero-goal-bias replicate success counts
  and per-query counts out of five.

Five planner repeats on one query are not five independent task samples. Do
not attach query-distribution confidence intervals to repeated-trial counts.

## Nominal tracking comparison

For every query whose canonical default plan succeeds, time-parameterize that
exact path with the unchanged limits (0.55 rad/s, 1.8 rad/s^2, 8 rad/s^3) and
the scene's 2-ms simulation step. Replay the same trajectory on the unchanged
nominal plant using:

1. position PD; and
2. position PD plus desired-velocity feedforward with `kv/kp = 0.2`.

Use the existing 250-step settle and 250-step final hold. Retain temporal RMS,
maximum, and final six-joint L2 error, maximum end-effector position error,
peak actuator-force fraction, saturated-step fraction, and sampled emitted
robot penetration in settle/path/hold.

The unchanged numeric gates are RMS <= 0.03 rad, maximum <= 0.10 rad, and final
<= 0.02 rad. A full pass additionally requires zero sampled penetration
(`dist < 0`) at every 2-ms state. Report both numeric and full pass counts,
Wilson 95% intervals by controller, and the paired counts `FF pass / PD fail`
and `PD pass / FF fail`. Planning failures remain in the query denominator and
are explicitly reported as having no tracking replay.

## Artifacts and reproducibility

Add one bounded executable with `--write` and `--check` modes and commit:

- `docs/randomized_eval_queries.csv` — every accepted query and sampling
  provenance;
- `docs/randomized_eval_planning.csv` — every canonical and blocked-query
  replicate outcome;
- `docs/randomized_eval_tracking.csv` — both controller outcomes for every
  successful canonical plan; and
- `docs/randomized_eval_results.md` — aggregate results, intervals, retained
  failures, and scope limitations.

`--check` must regenerate and exact-compare all deterministic fields and
outcomes. Only measured planner wall time may be normalized. Unit tests must
cover seed derivation, cohort layout, interval calculation, pass gating, and
stale-artifact rejection. CI must run the full bounded check.

## Interpretation and promotion rules

- No outcome threshold controls whether failures are published or retained.
- A negative or mixed result cannot trigger query removal, seed replacement,
  threshold changes, or a new post-hoc planner configuration.
- Wilson intervals describe repeated draws from the declared conditional
  generator; they do not certify arbitrary workspaces, scenes, obstacles, or
  real tasks.
- Collision checks remain sampled rather than continuous; tracking remains
  simulation-only; no hardware, grasp, uncertainty, or sim-to-real claim is
  introduced.
- The branch may be merged only after formatting, strict linting, all existing
  tests, the new unit tests, full `--check`, artifact-schema audits, and a
  result-independent source review pass. The exact result, favorable or not,
  determines the wording of any paper or release claim—not whether the raw
  evidence is preserved.
