# Attached-payload collision proxy: pre-result protocol

This protocol is fixed before implementing or running the attached-payload
experiment. Its purpose is to close one narrow Demo 3 planning limitation
without changing the arm planner or overstating what a sampled simulator check
can prove.

## Scope and fixed semantics

- Scene: `assets/ur5e/scene_pickplace.xml`.
- Query: the existing Demo 3 carry from the deterministic pick-approach IK
  solution to the deterministic place-approach IK solution.
- Planner seed: `20260816`.
- Joint-space edge sampling: 0.05 rad L2, unchanged from `PlanConfig`.
- Robot rule: preserve the existing carry predicate exactly: a sampled state is
  rejected only when MuJoCo emits a robot-involved contact with signed distance
  `< 0.0 m`.
- Payload proxy: reuse the scene's visual-only `cube` box geom (half-extents
  0.025 m). At every queried joint state, set its world transform to
  `T_world_EE * T_EE_cube`, where `T_EE_cube` is a 0.035 m translation along EE
  +Z with no relative rotation, matching the scripted Demo 3 weld.
- Payload pair scope: call MuJoCo's explicit `mj_geomDistance` wrapper only for
  `cube` versus the named collision geoms `floor`, `table`, and `pillar`.
- Payload rule: reject a sampled state when any scoped signed pair distance is
  strictly `< 0.005 m`. This 5 mm value is a declared planning buffer, not a
  tolerance estimate or a hardware guarantee.
- Intended load/tool proximity is excluded by construction: no cube-versus-
  robot pair is queried. In particular, wrist/load proximity cannot make the
  payload predicate fail. Visual-only `place_pad` is also outside the pair set.
- Combined carry rule: reject when either the unchanged robot rule or the
  payload rule rejects the state. All non-carry Demo 3 segments retain their
  existing robot-only predicates and thresholds.

The explicit pair-distance query is materially different from setting a global
positive `contact_threshold`: the latter only filters contacts MuJoCo already
emitted and applies to every robot pair. This protocol requests geometric
distance only for the three declared payload/environment pairs.

## Predeclared regression gates

The implementation is acceptable only if all of these deterministic gates
hold:

1. The carry start and goal are free under both the unchanged robot predicate
   and the new payload predicate.
2. The 0.05 rad sampled straight carry edge is blocked by the combined
   predicate. The test records whether the robot, payload, or both are
   responsible instead of attributing the result after the fact.
3. RRT-Connect with the fixed seed succeeds under the combined predicate, and
   every state in its returned densified path is re-audited as robot-free and
   payload-clear using independent checker state.
4. The configured pair scope is exactly `floor`, `table`, and `pillar`; it
   excludes the cube itself, all chain-body geoms (including the wrist), and the
   visual place pad.
5. Two fresh checkers and planners produce bit-identical waypoints and densified
   paths for the fixed input and seed. Wall-clock timing is explicitly excluded
   from determinism claims.
6. Invalid specifications fail at construction: unknown or non-box proxy,
   proxy not attached to a mocap body, unknown/duplicate/non-contact environment
   geom, robot geom in the environment set, non-finite transform, and negative
   or non-finite clearance.

## Claim boundary

Passing establishes only sampled, discrete clearance of one rigid box proxy
against three named environment geoms at the planner's 0.05 rad joint-space
resolution. It is not continuous swept-volume collision detection, does not
model grasp uncertainty, cube motion relative to the EE, compliance, fingers,
contact-rich grasping, calibration error, or unlisted geometry, and does not
certify hardware safety. Payload-versus-robot self-collision is intentionally
not checked because the wrist/load attachment region would require a separately
defined allowed-contact model.
