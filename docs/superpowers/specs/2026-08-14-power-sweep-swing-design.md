# Power-Sweep Swing Design

## Problem

The current "push" swing (`plan_fixed_joint_swing_quadratic` in `src/robot/motion/physics.rs`) solves one Cartesian impact pose via IK, then gives **every** joint an independent constant-acceleration profile over the **same shared duration**. Because the duration is shared, whichever joint needs the largest angular delta (often not j0/j2) sets the pace — j0 (yaw, dual MX-64, highest torque in the arm) and j2 (elbow, MX-28, low reflected inertia) end up moving however little IK happens to require, not as much as their motors can deliver. The racket doesn't feel powered by the strong joints.

Separately, a constant-acceleration-from-rest profile is slowest at the start and only reaches peak velocity in the single instant it arrives at the target. If the ball's predicted arrival time is off by even a little, the racket is caught still ramping up rather than at speed.

## Goal

Redesign the push swing so:
1. **j0 and j2 drive the swing.** Their motion is sized to use the joint-speed ceiling, not whatever IK residue is left over.
2. **They sustain near-peak speed across a window**, not just an instant, so ball-timing error doesn't cost impact speed.
3. **j3 (wrist) stays cocked and snaps late** — held near its start pose, then a short, fast quadratic burst ending exactly at the impact instant, contributing extra tip speed like a whip.
4. **j1 (shoulder) stays a passive follower** — no special treatment, it just interpolates to wherever IK puts it.

## Kinematic model

### Ramp-then-cruise profile (j0, j2)

A new one-DOF segment: accelerate from rest at fixed acceleration `a` up to a peak velocity `v_peak`, then hold `v_peak` (zero acceleration) for the rest of the duration. Unlike constant-acceleration-from-rest, this is deliberately front-loaded — peak speed is reached *before* the impact instant and held through it.

Given a **fixed total duration** `T` (shared by the whole pre-impact segment, same role as today's `FIXED_JOINT_SWING_DURATION_SECS`) and a required travel `Δq`, `v_peak` is uniquely determined:

```
Δq = v_peak·T − v_peak²/(2a)
```

(ramp phase covers `0.5·v_peak·t_ramp` at average speed `v_peak/2`; cruise phase covers `v_peak·(T − t_ramp)`; `t_ramp = v_peak/a`.) Solving the quadratic for the physically valid (smaller) root:

```
v_peak = a·T − √((a·T)² − 2·a·Δq)
```

This has a real solution only if `Δq ≤ 0.5·a·T²` (the distance reachable by accelerating for the entire duration and never cruising). If `Δq` exceeds that, the swing is infeasible for this `T` — surfaced as a planning error so the existing push-distance bisection can retry a shorter push.

`T = FIXED_JOINT_SWING_RAMP_SECS + FIXED_JOINT_SWING_CRUISE_SECS` (both new tunable constants) and `a = arm.max_joint_speed / FIXED_JOINT_SWING_RAMP_SECS` (the acceleration that reaches the joint-speed ceiling in exactly the ramp window). Both j0 and j2 solve their own `v_peak` against the same `T`/`a` — whichever needs more travel naturally solves closer to `v_max`; the existing push-distance bisection (already in the codebase) pushes both toward the ceiling by searching for the largest feasible push distance.

Sanity check: when `Δq` exactly equals `0.5·a·T²`, `v_peak = a·T` and `t_ramp = T` — the profile degenerates to a plain constant-acceleration segment for the whole duration, i.e. today's quadratic push is the boundary case of this one.

### Delayed-burst profile (j3)

Held at its start value (zero velocity) until `T − FIXED_JOINT_SWING_SNAP_DURATION_SECS`, then a plain constant-acceleration-from-rest quadratic burst for the remaining `FIXED_JOINT_SWING_SNAP_DURATION_SECS`, reaching the IK-derived target exactly at `T`. Position and velocity are continuous at the hold→burst boundary (velocity is 0 on both sides); acceleration jumps from 0 to the burst's constant value, which is fine — the existing feasibility check (torque/kinematic/collision) already samples the whole segment and will reject anything the wrist can't actually do.

### j1 (shoulder)

No new profile — keeps the existing plain constant-acceleration-from-rest quadratic segment over the same shared `T`.

## Architecture

- **New file** `src/robot/motion/ramp_cruise_segment.rs`: `RampCruiseSegment`, the one-DOF primitive above.
- **`src/robot/motion/quadratic_segment.rs`**: add `DelayedQuadraticSegment` (hold-then-burst wrapper around the existing `QuadraticSegment`).
- **`src/robot/motion/trajectory.rs`**: add a per-joint `PreImpactJointProfile` enum (`Quadratic` / `DelayedQuadratic{delay}` / `RampCruise{accel}`) and a `pre_impact_profiles: Vec<PreImpactJointProfile>` field on `Trajectory`, populated only by a new constructor `Trajectory::with_power_sweep(...)`. Existing constructors (`new`, `with_follow_through`, `with_quadratic_push`) leave this empty and keep behaving exactly as today — `pre_impact_segments()` only consults per-joint profiles when the vector is non-empty, so no existing swing (quintic or quadratic) changes behavior.
- **`src/robot/motion/physics.rs`**: new `plan_fixed_joint_swing_power_sweep` / `_from_alignment` / `_to_pose`, mirroring the structure of the existing `plan_fixed_joint_swing_quadratic*` functions (same IK-once, same push-distance bisection loop) but building a `Trajectory::with_power_sweep` instead, with `POWER_SWEEP_JOINT_INDICES = [0, 2]` driving the ramp-cruise profile, the wrist index (`arm.wrist_joint_index()`) driving the delayed burst, and everything else on plain quadratic.
- **`src/defaults/motion.rs`**: three new constants — `FIXED_JOINT_SWING_RAMP_SECS`, `FIXED_JOINT_SWING_CRUISE_SECS`, `FIXED_JOINT_SWING_SNAP_DURATION_SECS`.
- **`src/robot/motion/planner.rs`**: expose `Planner::fixed_joint_swing_power_sweep(_from_alignment)`.
- **`src/real/control_worker.rs`**: swap the currently-wired `Planner::fixed_joint_swing_quadratic_from_alignment` call for the new power-sweep variant.

## Feasibility and error handling

`plan_fixed_joint_swing_power_sweep_to_pose` validates `RampCruiseSegment::new(...)` for both `POWER_SWEEP_JOINT_INDICES` *before* building the trajectory; a `None` (distance unreachable in time `T` at any speed) maps to `SwingPlanError::JointOrTorqueLimit`, which the existing outer push-distance bisection (unchanged logic, reused verbatim) treats like any other infeasible candidate and retries at a shorter push distance. The existing three-part `evaluate_trajectory_feasibility` (kinematic/torque/table-collision) still runs on the assembled trajectory exactly as it does today for the quadratic swing — no changes needed there since it samples generically through `PreImpactSegment`.

## Testing strategy

Each new primitive gets direct unit tests (hold behavior, ramp-then-cruise shape, degenerate boundary case, continuity at phase transitions). `Trajectory::with_power_sweep` gets a continuity/shape test mirroring the existing `quadratic_push_trajectory_reaches_targets_and_is_continuous_through_the_knot` test. The top-level planner gets tests mirroring the existing `fixed_joint_swing_pushes_forward_without_backswing` and `fixed_joint_swing_quadratic_finds_faster_push_than_fixed_ladder` tests, plus new assertions specific to this design: j0/j2 sustain near-ceiling speed for a window (not just an instant) ending at impact, and j3 stays at its start value until the snap window.
