# Pass-Through Snap Test Design

## Problem

Earlier testing established two things about the real 4-DOF arm's power-sweep swing:

1. **j0 (yaw) is the only joint whose motion meaningfully drives the forward push** at this arm's geometry (Jacobian analysis) — j1 is nearly orthogonal to the push direction, j2 works slightly against it.
2. **j0 is already running near its physical speed ceiling**, and simply increasing the push distance to give it more room backfires — the required joint-angle sweep grows faster than the time budget can cover, collapsing the swing to its emergency fallback (verified: 20/25/30cm push distances all made things worse, not better, than the current ~10cm).

Given that, the user wants to explore a different lever entirely: instead of pushing the racket further in a straight line to the ball's exact contact point, plan a trajectory that continues **past** the contact point (an overshoot target), so the racket is still accelerating — not decelerating into a stop — at the moment it actually reaches the ball. Combined with this, the wrist (j3) should do a real two-phase motion: an active backswing away from ready, then a fast snap to its mechanical limit, timed so the snap's completion coincides with the ball-contact instant. All four joints move simultaneously through the hit.

This is a new motion shape that doesn't exist anywhere in the codebase. The goal here is **exploratory**: build a standalone real-hardware test tool to physically observe and tune this motion, independent of whether it ever becomes the production swing. No changes to the production planner (`src/robot/motion/physics.rs`) or the real control loop (`src/real/control_worker.rs`).

## Non-goals

- Not integrating this into `plan_fixed_joint_swing_power_sweep` or `control_worker.rs`. If this motion proves out, production integration is a separate, later design effort.
- Not replicating the production alignment geometry exactly (ball radius / racket half-thickness / contact-below-center offsets). The tool targets the racket **center** directly at a specified world-frame point for simplicity — precise ball-contact geometry isn't the thing being tested here, the motion *shape* is.
- Not moving the linear rail (AXL). The tool assumes a fixed rail position (the arm's default rail x) purely as an IK input.
- Not adding vision/ball-tracking. All positions and timings are CLI arguments, standing in for what a real Prediction would supply.

## Design

### Tool shape

A new standalone binary, `tools/pass_through_snap_test`, following the existing pattern of `tools/measure_joint_speed` and `tools/verify_mirror`: a `Cargo.toml` depending on `pingpong-bot` with `features = ["real"]`, an `args.rs` (clap `Parser`), a thin `main.rs`, and a `run.rs` with the actual logic. Connects directly to `DynamixelBus` (not `RealHardware`) — this test doesn't need rail/vision/control-loop plumbing, just raw 4-joint streaming, matching `measure_joint_speed`'s approach.

### Motion composition

All angles/positions computed once at start from the arm's IK/FK (`pingpong_bot::defaults::robot()` → `Arc<Arm>`) and the current hardware joint angles (`bus.read_joints()`). Rail x is fixed at `arm.rail.map_or(0.0, |r| r.default_x())` for the IK call.

**Overshoot target.** Given `--target-x/-y/-z` (the ball's contact point) and `--overshoot-m`:
```
push_direction = horizontal_normalize(toward_opponent_center(target))   // same formula as physics.rs's ball_alignment_geometry, simplified (no ball/racket offsets)
overshoot_position = target_position + push_direction * overshoot_m
target_normal = push_direction
```
IK-solve `overshoot_position`/`target_normal` at the fixed rail x, hinted from the current joint pose, via `Arm::inverse_pose_at_fixed_rail_best_normal(..., IkSearch::Global)` — the same call the production swing planner uses. This gives the overshoot joint angles, `overshoot_joints`.

**j0, j2** (indices `POWER_SWEEP_JOINT_INDICES = [0, 2]`, matching production): `RampCruiseSegment::new(current_angle, overshoot_joints[i], total_duration_secs, accel)` where `accel = arm.max_joint_speed / ramp_secs` (`--ramp-secs`, default matching production's `FIXED_JOINT_SWING_RAMP_SECS` = 0.06). If `RampCruiseSegment::new` returns `None` (the sweep is unreachable in `total_duration_secs` even at max speed), the tool reports this plainly and exits before commanding any real motion — the same feasibility check the production planner performs internally.

**j1** (index 1): plain `QuadraticSegment::new(current_angle, 0.0, overshoot_joints[1], total_duration_secs)` — passive follower, matching production's role for this joint. Always feasible (no `None` case).

**j3** (`arm.wrist_joint_index()`): two phases, computed as follows.
- *Snap target*: whichever of `arm.joint_limit(wrist_index).{min,max}` is on the opposite side of `current_angle` from `--wrist-cocked-deg` — i.e., if the cocked angle is below current, the snap target is the joint's max limit, and vice versa. This makes "snap forward" self-determining from the cocked direction, rather than a separately-specified target.
- *Snap duration*: `2 · |snap_target − cocked_angle| / (arm.max_joint_speed · margin)`, where `margin` is `--snap-velocity-margin` (default 0.85, matching the production constant `FIXED_JOINT_SWING_SNAP_VELOCITY_MARGIN` — reused as the default value only, not imported, since this tool doesn't touch production code paths).
- *Phase A* (backswing), valid for `t ∈ [0, backswing_secs]` (`--backswing-duration-secs`): `QuadraticSegment::new(current_angle, 0.0, cocked_angle, backswing_secs)`.
- *Phase B* (hold + snap), valid for `t ∈ [backswing_secs, impact_time_secs]`: `DelayedQuadraticSegment::new(cocked_angle, snap_target, phase_b_duration, hold_secs)` sampled at `t − backswing_secs`, where `phase_b_duration = impact_time_secs − backswing_secs` and `hold_secs = phase_b_duration − snap_duration`. If `hold_secs < 0` (the snap alone doesn't fit in the time left before impact), the tool reports this plainly and exits — the backswing and/or total timing needs adjusting.
- *Phase C* (post-snap hold), valid for `t ∈ [impact_time_secs, total_duration_secs]`: constant at `snap_target` — matching the production swing's convention of holding position after the commit knot rather than doing a separate follow-through shape for this experimental tool.

### Feasibility summary (checked before any hardware command)

1. `impact_time_secs < total_duration_secs` (overshoot target must be reached after the real contact instant).
2. `RampCruiseSegment::new` succeeds for both j0 and j2.
3. `hold_secs ≥ 0` for j3's phase B.

Any failure prints which check failed and the relevant numbers (required vs. available), then exits without commanding hardware — same spirit as the production planner's `SwingPlanError` reporting, but as plain CLI output since this tool has no `Result<_, DomainError>` caller to report to.

### Execution

Once feasibility passes, the tool prints a summary (overshoot target, all joint deltas, snap window, computed peak speeds) and requires a typed `y` confirmation — matching `measure_joint_speed`/`verify_mirror`'s safety pattern — before commanding anything. It then runs its own streaming loop (`--poll-hz`, default 200.0, matching `RealHardware`'s stream rate): at each tick, sample all four joints' position functions at the current elapsed time and send them together via one `bus.write_joints(&Joints{values})` call, continuing until `total_duration_secs` has elapsed.

### Reporting

While streaming, the tool also polls `bus.read_joints()` (interleaved with the write ticks, matching `measure_joint_speed`'s single-bus polling loop — write then immediately read back) and records `(elapsed_secs, [q0..q3])`. After the run, it reports, per joint: peak measured speed (finite-difference between consecutive samples) and — specifically — the measured angle and velocity at the sample closest to `impact_time_secs`, since that's the moment being evaluated: was the racket still accelerating through contact, or had it already leveled off?

## Testing strategy

The tool's pure logic (overshoot geometry, snap-target/duration computation, phase-boundary sampling, feasibility checks) is implemented as free functions taking plain `f64`/`Point3`/`Vector3` values — no hardware access — so it can be unit-tested within the tool crate itself (`#[cfg(test)]`, following the existing precedent in `tools/clip_review`, `tools/detect_full`, etc.), without needing real hardware or even the `real` feature. Tests cover: overshoot position/target-normal computation against a known ball position, snap-target side selection for both cocked-below and cocked-above cases, snap-duration formula against a hand-computed example, phase-boundary continuity (position matches at each segment handoff), and each of the three feasibility checks correctly failing on a constructed infeasible input.

No hardware-in-the-loop automated test is possible or attempted — real-world validation is the point of running the tool, done manually by the user on the bench.
