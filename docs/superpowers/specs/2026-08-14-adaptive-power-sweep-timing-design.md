# Adaptive Power-Sweep Timing Design

## Problem

On real hardware, the power-sweep swing (added in the previous work: `docs/superpowers/plans/2026-08-14-power-sweep-swing.md`) visibly fails to push forward — the racket mostly goes upward — and the wrist snap is imperceptible.

**Root cause (confirmed by direct instrumentation):** the wrist (j3) must rotate up to **‑24°** at full 10cm push distance to keep the racket face correctly oriented, and that requirement scales roughly linearly with push distance (‑2.5° at 10%, ‑7.6° at 30%, ‑13.6° at 50%, ‑24° at 100%). The wrist's snap window was a **fixed** `FIXED_JOINT_SWING_SNAP_DURATION_SECS` (50ms), which can only accommodate rotations up to about 10% push distance before the trajectory feasibility check rejects it for exceeding the joint-speed limit ("관절 속도"). Every push distance from 30% through 100% fails this check.

The push-distance bisection search only probes *upward* from a 30% anchor — if that anchor itself is infeasible, the search is never entered, and the whole swing (`plan_fixed_joint_swing_power_sweep_to_pose`) falls straight through to the hardcoded absolute fallback of **0.020m (2cm)**, replanning the *entire* trajectory at that trivial distance. This drags j0/j2 down with it even though they were independently capable of the full 10cm sweep (confirmed reachable at 100% push in isolation). The result: a ~2cm push dominated visually by whatever residual motion is left, and a wrist rotation (~5° at that scale) too small and quick to read as a "snap."

Separately, the user asked why the swing's pre-impact duration (`FIXED_JOINT_SWING_RAMP_SECS + FIXED_JOINT_SWING_CRUISE_SECS`, fixed at 120ms) is hardcoded rather than derived from the ball's estimated arrival time, wanting to test with the real, live estimate instead.

## Goals

1. Make the wrist snap window **adapt** to however much rotation is actually required, instead of a fixed 50ms — this is the primary fix for the collapse.
2. Make the swing's **total pre-impact duration** derive from the ball's live estimated arrival time (`predicted_arrival_at`, already computed in `control_worker.rs`) rather than a hardcoded constant, while preserving today's exact trigger cadence and nominal numbers so behavior in the common case is unchanged — the difference shows up when the control loop runs late (today silently uses a stale duration; this correctly shrinks it) and makes the number honestly reflect what it claims to.

## Non-goals

- Not changing *when* the swing is triggered (`swing_due_at`, still `predicted_arrival − FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS`) — only what duration is computed once triggered. Moving the trigger earlier trades prediction accuracy for time budget and is a separate decision the user deferred.
- Not touching the push-distance bisection's search structure (still anchors at 30% and searches upward, still falls back to the 2cm absolute fallback if even that anchor fails) — adaptive snap sizing resolves the case that was breaking it (30%+ requiring more snap time than available), so the existing structure is expected to work correctly again without restructuring it.
- Not touching the old quintic/quadratic swing functions, the sim's swing path, or `FIXED_JOINT_SWING_DURATION_SECS`/`FIXED_JOINT_SWING_LEAD_SECS` (sim-only, unaffected).

## Design

### Adaptive snap duration

Replace the fixed snap window with one computed from the wrist's actual required rotation, using the same closed-form minimum-time-at-ceiling-speed idea already used for j0/j2's ramp:

```
Δq3 = impact_pose.joints[wrist_index] - start.joints[wrist_index]
snap_duration = clamp(2 · |Δq3| / arm.max_joint_speed, MIN_SNAP_SECS, impact_time)
```

`2·|Δq3|/max_joint_speed` is the minimum time to cover `Δq3` from rest at the joint's speed ceiling (same formula as a plain constant-acceleration segment's rest-to-rest minimum time). `MIN_SNAP_SECS` (new constant, keeping today's value of 0.050s) is a floor so a near-zero `Δq3` doesn't produce a degenerately instantaneous "snap." The upper clamp to `impact_time` prevents the snap from claiming more time than the whole swing has.

`FIXED_JOINT_SWING_SNAP_DURATION_SECS` is renamed to `FIXED_JOINT_SWING_MIN_SNAP_SECS` to reflect this new role (floor, not fixed value); its value (0.050) is unchanged.

### Dynamic total impact time

`FIXED_JOINT_SWING_CRUISE_SECS` is removed as a fixed addend — cruise time is now simply whatever's left of the caller-supplied total duration after the fixed ramp: `cruise = target_impact_time − FIXED_JOINT_SWING_RAMP_SECS` (implicit, not a separate variable — `RampCruiseSegment` already takes total duration directly).

A new constant `FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS = 0.120` (same value as today's fixed `RAMP_SECS + CRUISE_SECS`) becomes the floor every computed duration is clamped to, so the swing never gets shorter than what shipped today.

A new constant `FIXED_JOINT_SWING_IMPACT_MARGIN_SECS = 0.200` names the margin the system already achieves today (the gap between when the swing was triggered and when it actually reaches its target, under the old fixed-duration math: `LEAD(0.320) − DURATION(0.120) = 0.200`), now computed explicitly rather than falling out of unrelated constants by coincidence.

`control_worker.rs` computes, at the same point it already triggers the swing (`swing_due_at`, unchanged):

```
predicted_arrival_at:  now stored in BallControlState::Aligning (new field), set
                        alongside swing_due_at/return_due_at when alignment (re)commits
remaining = predicted_arrival_at.saturating_duration_since(Instant::now())
target_impact_time_secs = max(
    remaining.as_secs_f64() - FIXED_JOINT_SWING_IMPACT_MARGIN_SECS,
    FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS,
)
```

In the nominal on-time case this evaluates to exactly `0.320 − 0.200 = 0.120` — identical to today's fixed value, because `FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS` (0.320, unchanged, still governs *when* the swing fires) and the new margin constant were chosen to reproduce it. The only behavioral difference is when the control loop services the due swing late: today's code keeps using a now-stale 120ms regardless; this version correctly computes less time is left and clamps to the same 120ms floor rather than either extrapolating a wrong duration or silently drifting the actual margin below 200ms without anyone noticing.

`target_impact_time_secs` is threaded through `Planner::fixed_joint_swing_power_sweep_from_alignment` → `physics::plan_fixed_joint_swing_power_sweep_from_alignment` → `plan_fixed_joint_swing_power_sweep_to_pose`, replacing the internal `FIXED_JOINT_SWING_RAMP_SECS + FIXED_JOINT_SWING_CRUISE_SECS` computation. The physics-layer functions clamp their received value to `FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS` internally (defense in depth — callers besides `control_worker.rs`, e.g. tests, don't have to reproduce the clamp themselves).

The `remaining/margin/clamp` arithmetic in `control_worker.rs` is extracted into a small pure function, `target_impact_time_secs(predicted_arrival_at: Instant, now: Instant) -> f64`, rather than inlined at the call site — `Instant` arithmetic can't be constructed arbitrarily in tests (no `Instant::from_secs`), but two `Instant`s a known `Duration` apart can, so a pure function taking both instants is directly unit-testable where the inlined version wouldn't be.

### Interface changes

- `plan_fixed_joint_swing_power_sweep(arm, start, target_impact_time_secs: f64)`
- `plan_fixed_joint_swing_power_sweep_from_alignment(arm, start, aligned, target_impact_time_secs: f64)`
- `plan_fixed_joint_swing_power_sweep_to_pose(arm, start, target_position, target_normal, target_impact_time_secs: f64)` (private)
- `Planner::fixed_joint_swing_power_sweep(arm, start, target_impact_time_secs: f64)`
- `Planner::fixed_joint_swing_power_sweep_from_alignment(arm, start, aligned, target_impact_time_secs: f64)`
- `BallControlState::Aligning` gains `predicted_arrival_at: Instant`

All existing call sites of these functions (three in `physics.rs` tests, one in `control_worker.rs`) are updated. Tests that want today's exact fixed-duration behavior pass `pingpong_bot::defaults::FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS` explicitly.

## Testing strategy

- `RampCruiseSegment`/`DelayedQuadraticSegment` primitives are unchanged — no new tests needed there.
- Existing power-sweep planner tests (`fixed_joint_swing_power_sweep_j0_j2_sustain_ceiling_speed_before_impact`, `fixed_joint_swing_power_sweep_holds_wrist_until_the_snap_window`, `fixed_joint_swing_power_sweep_stays_within_joint_limits`) updated to pass `FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS`, reproducing today's exact numbers as a regression baseline.
- New test: adaptive snap duration actually scales with required rotation — two different target poses requiring different `Δq3` produce different snap durations, both respecting the `MIN_SNAP_SECS` floor and the `≤ impact_time` ceiling.
- New test (the actual regression test for the reported bug): a realistic full-height impact target now achieves a push distance much closer to the intended 10cm than the 2cm emergency fallback, using the same scenario that previously collapsed.
- New test: `target_impact_time_secs` below the floor is clamped to `FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS` rather than producing an unreasonably short/infeasible swing.
- `control_worker.rs`: existing lead-time test (`real_fixed_swing_lead_matches_power_sweep_lead_constant`) unaffected (still asserts `FIXED_SWING_LEAD` against `FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS`, both untouched by this change). New test verifying `target_impact_time_secs` computed at the swing-trigger site matches `remaining_time − margin` (clamped), using a synthetic `predicted_arrival_at`.
