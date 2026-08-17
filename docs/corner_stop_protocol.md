# Global corner-stop trajectory protocol

Frozen before implementation or result-bearing reruns on 2026-08-17
(Europe/Dublin).

## Motivation and claim boundary

The existing scalar S-curve follows an entire joint-space polyline at nonzero
path speed. Its per-edge joint velocity, acceleration, and jerk respect the
declared limits, but a polyline tangent can change discontinuously at an
interior waypoint. The complete joint trajectory is therefore not globally
acceleration- or jerk-bounded.

This change will replace that behavior with an explicit full stop at every
shortcut waypoint. It will not implement geometric corner blending, time-
optimal path parameterization, continuous collision checking, torque limits,
or hardware validation.

## Frozen design

1. Parameterize every nonzero shortcut edge with an independent rest-to-rest
   seven-phase scalar S-curve.
2. Convert the joint velocity, acceleration, and jerk limits to scalar limits
   using that edge's constant unit tangent.
3. Round each edge duration up to an integer number of the requested fixed
   sample intervals. Realize the rounded duration by time-scaling the original
   S-curve, which can only reduce velocity, acceleration, and jerk.
4. Concatenate edges without duplicate timestamps. At every interior waypoint,
   joint velocity and acceleration must be exactly zero. Position, velocity,
   and acceleration are therefore continuous; joint jerk may jump between
   finite left and right values but must remain within the declared bound.
5. Add sampled joint jerk to `Trajectory` so callers and tests can audit the
   same analytic derivatives used by the generator.
6. Pass the planner's shortcut `waypoints`, not its collision-check-densified
   `path`, to the trajectory generator. The densified path remains the planning
   and sampled-collision artifact. This avoids an unnecessary stop at every
   0.05-rad collision-check sample.
7. Preserve deterministic output for identical path, limits, and sample time.
   A stationary path remains one rest sample.

## Required tests

- straight-line endpoint and velocity/acceleration/jerk limits;
- a non-collinear multi-joint corner with an exact waypoint sample whose
  velocity and acceleration are zero;
- uniform timestamps, including an exact final timestamp;
- duplicate/zero-length waypoint handling;
- deterministic repeat output including jerk;
- an analytic or finite-difference check around each corner showing no
  velocity or acceleration impulse;
- the fixed-seed Demo 2 plan using shortcut waypoints; and
- all existing workspace tests and strict lints.

## Result-bearing reruns

If implementation gates pass, regenerate and verify every committed artifact
whose trajectory duration, sample count, tracking result, or controller result
can change:

- Demo 2 golden trajectory test;
- `robustness_bench` Markdown and CSV;
- `multi_query_bench` planning/tracking CSVs and Markdown; and
- `randomized_eval` planning/tracking CSVs and Markdown.

Planning outcomes must remain unchanged because this change is downstream of
planning. Tracking results may improve or regress and will be retained without
post-hoc threshold changes. Existing numeric and sampled-penetration pass gates
remain unchanged.

## Promotion gate

Promote the branch only if:

- all analytic continuity and derivative-limit tests pass;
- all existing tests and strict lints pass;
- generated artifacts pass their own deterministic `--check` modes;
- the README and generated reports distinguish full-stop corner handling from
  blended motion and retain the simulation/sampled-collision limitations; and
- an independent diff and claim audit finds no unsupported global-smoothness,
  continuous-safety, or sim-to-real claim.
