# Adaptive Power-Sweep Timing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the power-sweep swing collapsing to a 2cm emergency fallback on real hardware by making the wrist's snap window adapt to how much rotation it actually needs, and make the swing's total pre-impact duration derive from the ball's live estimated arrival time instead of a hardcoded constant.

**Architecture:** Two independent-but-related changes to `plan_fixed_joint_swing_power_sweep*` in `physics.rs`: (1) the wrist's `DelayedQuadratic` snap window is sized from its actual required rotation (same minimum-time-at-ceiling-speed math already used for j0/j2) instead of a fixed 50ms; (2) the functions gain a `target_impact_time_secs: f64` parameter (clamped to a floor internally) replacing the hardcoded `RAMP_SECS + CRUISE_SECS` sum. `control_worker.rs` computes that parameter from a new `predicted_arrival_at` field stored on `BallControlState::Aligning`, via a small pure helper function that's unit-testable despite `Instant` having no public constructor.

**Tech Stack:** Rust, cargo test / cargo build / cargo clippy.

**Spec:** `docs/superpowers/specs/2026-08-14-adaptive-power-sweep-timing-design.md`

## Global Constraints

- Do not touch the old quintic/quadratic swing functions, the sim's swing path, `FIXED_JOINT_SWING_DURATION_SECS`, or `FIXED_JOINT_SWING_LEAD_SECS` — sim-only, unaffected by this change.
- Do not change *when* the swing triggers (`swing_due_at` computation, `FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS`) — only what duration gets computed once triggered.
- Do not restructure the push-distance bisection search (still anchors at 30%, still falls back to the hardcoded 0.020m absolute fallback) — adaptive snap sizing is expected to make that search succeed again on its own.
- New/renamed constants: `FIXED_JOINT_SWING_MIN_SNAP_SECS` (renamed from `FIXED_JOINT_SWING_SNAP_DURATION_SECS`, same value 0.050), `FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS = 0.120` (new), `FIXED_JOINT_SWING_IMPACT_MARGIN_SECS = 0.200` (new). `FIXED_JOINT_SWING_CRUISE_SECS` is removed.

---

### Task 1: Rename/add constants in `defaults/motion.rs`

**Files:**
- Modify: `src/defaults/motion.rs`
- Modify: `src/defaults/mod.rs` (re-export list)

**Interfaces:**
- Produces: `pub const FIXED_JOINT_SWING_MIN_SNAP_SECS: f64 = 0.050`, `pub const FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS: f64 = 0.120`, `pub const FIXED_JOINT_SWING_IMPACT_MARGIN_SECS: f64 = 0.200`. Removes `FIXED_JOINT_SWING_CRUISE_SECS` and `FIXED_JOINT_SWING_SNAP_DURATION_SECS`.

- [ ] **Step 1: Rename and add the constants**

In `src/defaults/motion.rs`, find this block:

```rust
/// 가속 뒤 첨두속도를 그대로 유지(순항)하는 시간 [s] — 공 도착 시각 예측
/// 오차를 흡수하는 창이다. `FIXED_JOINT_SWING_RAMP_SECS`와 합이 파워 스윙의
/// 전체 타격-전 시간이 된다.
pub const FIXED_JOINT_SWING_CRUISE_SECS: f64 = 0.060;
/// 손목(j3)이 접힌 자세로 대기하다 등가속 스냅으로 목표각까지 움직이는
/// 시간 [s] — 파워 스윙 전체 시간의 마지막 구간이다.
pub const FIXED_JOINT_SWING_SNAP_DURATION_SECS: f64 = 0.050;
/// 예상 공 도착 시각보다 파워 스윙 명령을 앞서 시작할 시간 [s] —
/// [`FIXED_JOINT_SWING_LEAD_SECS`]의 파워 스윙 전용 짝.
/// 타격-전 시간이 `FIXED_JOINT_SWING_RAMP_SECS + FIXED_JOINT_SWING_CRUISE_SECS`
/// (0.12s)로 quadratic 스윙(0.20s)보다 짧아진 만큼(0.08s), 원래 의도했던
/// "예상 도착보다 0.20초 먼저 임팩트" 여유를 그대로 유지하도록
/// `FIXED_JOINT_SWING_LEAD_SECS`(0.400)에서 그만큼 줄였다.
pub const FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS: f64 = 0.320;
```

Replace it with:

```rust
/// 손목(j3)이 등가속 스냅에 쓸 최소 시간 [s] — 실제 스냅 시간은
/// `2·|Δq3|/max_joint_speed`(정지에서 관절 속도 상한까지 걸리는 최소
/// 시간)로 요구 회전량에 맞춰 계산하고, 이 값은 그 계산이 0에 가까운
/// 회전량에도 지나치게 짧은 스냅을 만들지 않게 막는 하한이다.
/// (2026-08-14 이전에는 이 값이 고정 스냅 시간 자체였다 — 임팩트까지
/// j3 요구 회전량이 커질수록(최대 -24°) 50ms 창을 넘어서면서 궤적 전체가
/// 강제로 2cm 비상 폴백까지 떨어지는 문제가 있었다.)
pub const FIXED_JOINT_SWING_MIN_SNAP_SECS: f64 = 0.050;
/// 파워 스윙 타격-전 시간의 하한 [s] — `FIXED_JOINT_SWING_RAMP_SECS`(0.06)
/// + 이전에 고정이었던 순항 시간(0.06)과 같은 값으로, 오늘 출하되는
/// 스윙보다 짧아지지 않도록 막는다. 실제 타격-전 시간은
/// `control_worker`가 예상 도착 시각까지 남은 시간에서
/// [`FIXED_JOINT_SWING_IMPACT_MARGIN_SECS`]를 뺀 값으로 동적으로 계산하고,
/// 이 하한으로 클램프한다.
pub const FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS: f64 = 0.120;
/// 예상 공 도착 시각보다 몇 초 먼저 임팩트가 나야 하는지 — 파워 스윙의
/// 목표 타격-전 시간을 `남은 시간 − 이 값`으로 계산하는 데 쓴다.
/// [`FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS`](0.320)와 짝을 이뤄 제때
/// 스윙이 트리거되면 `0.320 − 0.200 = 0.120`으로
/// [`FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS`]와 정확히 같아진다 — 오늘
/// 고정값과 동일한 여유를 그대로 재현하되, 제어 루프가 늦게 응답한
/// 경우에는 오래된 고정값을 그대로 쓰는 대신 실제 남은 시간을 반영해
/// 자연히 하한까지 줄어든다.
pub const FIXED_JOINT_SWING_IMPACT_MARGIN_SECS: f64 = 0.200;
/// 예상 공 도착 시각보다 파워 스윙 명령을 앞서 시작할 시간 [s] —
/// [`FIXED_JOINT_SWING_LEAD_SECS`]의 파워 스윙 전용 짝. 스윙이 언제
/// 트리거되는지만 정하고, 스윙 자체의 소요 시간은
/// [`FIXED_JOINT_SWING_IMPACT_MARGIN_SECS`]로 별도 계산한다.
pub const FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS: f64 = 0.320;
```

- [ ] **Step 2: Verify it compiles (expect errors from other files)**

Run: `cargo build --lib 2>&1 | grep "error\[" | head -20`
Expected: several `error[E0425]: cannot find value \`FIXED_JOINT_SWING_CRUISE_SECS\`` / `FIXED_JOINT_SWING_SNAP_DURATION_SECS` in `src/robot/motion/physics.rs` — this is expected; Task 2 fixes them. Do not fix them here.

- [ ] **Step 3: Commit**

```bash
git add src/defaults/motion.rs
git commit -m "refactor(motion): rename swing snap constant to a floor, add adaptive-timing constants"
```

Note: `src/defaults/mod.rs`'s re-export list does not need editing in this task — it re-exports `FIXED_JOINT_SWING_LEAD_SECS`/`FIXED_JOINT_SWING_DURATION_SECS`/`FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS` already, and none of the new/renamed constants need top-level re-export (they're only consumed via `pingpong_bot::defaults::motion::` or `crate::defaults::motion::` qualified paths in the tasks below, matching how `FIXED_JOINT_SWING_RAMP_SECS` is already consumed in `physics.rs`).

---

### Task 2: Adaptive snap + dynamic duration in `physics.rs`

**Files:**
- Modify: `src/robot/motion/physics.rs`

**Interfaces:**
- Consumes: `FIXED_JOINT_SWING_MIN_SNAP_SECS`, `FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS` (Task 1).
- Produces: `pub fn plan_fixed_joint_swing_power_sweep(arm: &Arm, start: &robot::Pose, target_impact_time_secs: f64) -> Result<FixedJointSwing, DomainError>`, `pub fn plan_fixed_joint_swing_power_sweep_from_alignment(arm: &Arm, start: &robot::Pose, aligned: &robot::Pose, target_impact_time_secs: f64) -> Result<FixedJointSwing, DomainError>` — both gain a new trailing `f64` parameter. Task 3 (`planner.rs`) calls these with the same new signatures.

- [ ] **Step 1: Update the import block**

In `src/robot/motion/physics.rs`, find:

```rust
use crate::defaults::motion::{
    ALIGNMENT_CONTACT_BELOW_RACKET_CENTER_M, ALIGNMENT_TARGET_HEIGHT_OFFSET_M,
    DETECTION_WINDUP_DISTANCE_M, DETECTION_WINDUP_MIN_DURATION_SECS, FIXED_IMPACT_PUSH_SPEED_M_S,
    FIXED_JOINT_PUSH_DISTANCE_M, FIXED_JOINT_PUSH_LIFT_M, FIXED_JOINT_SNAP_SPEED_RATIO,
    FIXED_JOINT_SWING_CRUISE_SECS, FIXED_JOINT_SWING_DURATION_SECS,
    FIXED_JOINT_SWING_FOLLOW_THROUGH_SECS, FIXED_JOINT_SWING_RAMP_SECS,
    FIXED_JOINT_SWING_SNAP_DURATION_SECS, IMPACT_CENTER_BELOW_BALL_M, IMPACT_UPWARD_TILT_DEG,
    READY_PREWIND_DISTANCE_M, RETURN_TO_CENTER_GROWTH, RETURN_TO_CENTER_MAX_SECS,
    RETURN_TO_CENTER_MIN_SECS, ready_racket_height_m, ready_racket_y_m,
};
```

Replace with:

```rust
use crate::defaults::motion::{
    ALIGNMENT_CONTACT_BELOW_RACKET_CENTER_M, ALIGNMENT_TARGET_HEIGHT_OFFSET_M,
    DETECTION_WINDUP_DISTANCE_M, DETECTION_WINDUP_MIN_DURATION_SECS, FIXED_IMPACT_PUSH_SPEED_M_S,
    FIXED_JOINT_PUSH_DISTANCE_M, FIXED_JOINT_PUSH_LIFT_M, FIXED_JOINT_SNAP_SPEED_RATIO,
    FIXED_JOINT_SWING_DURATION_SECS, FIXED_JOINT_SWING_FOLLOW_THROUGH_SECS,
    FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS, FIXED_JOINT_SWING_MIN_SNAP_SECS,
    FIXED_JOINT_SWING_RAMP_SECS, IMPACT_CENTER_BELOW_BALL_M, IMPACT_UPWARD_TILT_DEG,
    READY_PREWIND_DISTANCE_M, RETURN_TO_CENTER_GROWTH, RETURN_TO_CENTER_MAX_SECS,
    RETURN_TO_CENTER_MIN_SECS, ready_racket_height_m, ready_racket_y_m,
};
```

- [ ] **Step 2: Thread `target_impact_time_secs` through the three power-sweep functions**

Find the three functions (`plan_fixed_joint_swing_power_sweep`, `plan_fixed_joint_swing_power_sweep_from_alignment`, `plan_fixed_joint_swing_power_sweep_to_pose`) and replace them in full:

```rust
/// j0(요)·j2(팔꿈치)가 임팩트를 만드는 "파워 스윙" —
/// [`plan_fixed_joint_swing_quadratic`]의 관절 배분을 뒤집는다. 그 버전은
/// 모든 관절이 같은 고정 시간을 나눠 쓰므로, IK가 우연히 큰 델타를 준 약한
/// 관절이 전체 스윙 속도의 병목이 될 수 있었다. 이 버전은:
///
/// - j0·j2(이 팔에서 가장 토크가 큰 관절)가 [`RampCruiseSegment`]로 관절
///   속도 상한까지 가속한 뒤 임팩트 앞에서 그 속도를 유지한다 — 공 도착
///   시각 예측 오차에 강건하도록 순간이 아니라 구간으로 첨두속도를 낸다.
/// - j1(어깨)은 그대로 IK가 요구하는 만큼만 따라간다.
/// - j3(손목)는 요구 회전량에서 계산한 최소 시간만큼만 접힌 자세로
///   대기하다 마지막 구간에서 등가속 스냅으로 목표각에 도달한다 — 회전량이
///   크면 스냅 창도 그만큼 넓어진다([`FIXED_JOINT_SWING_MIN_SNAP_SECS`]는
///   하한일 뿐이다).
///
/// `target_impact_time_secs`는 타격-전 전체 시간의 목표값이다 —
/// [`FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS`] 미만이면 그 값으로 클램프한다.
pub fn plan_fixed_joint_swing_power_sweep(
    arm: &Arm,
    start: &robot::Pose,
    target_impact_time_secs: f64,
) -> Result<FixedJointSwing, DomainError> {
    return plan_fixed_joint_swing_power_sweep_from_alignment(
        arm,
        start,
        start,
        target_impact_time_secs,
    );
}

/// [`plan_fixed_joint_swing_power_sweep`]의 정렬-기준 버전 —
/// [`plan_fixed_joint_swing_quadratic_from_alignment`]와 같은 이유로 실측
/// 대신 마지막 절대 정렬 자세를 밀기 기준으로 쓴다.
pub fn plan_fixed_joint_swing_power_sweep_from_alignment(
    arm: &Arm,
    start: &robot::Pose,
    aligned: &robot::Pose,
    target_impact_time_secs: f64,
) -> Result<FixedJointSwing, DomainError> {
    let aligned_racket = arm
        .forward_kinematics_with_rail(aligned.rail_x, &aligned.joints)
        .ok_or_else(|| {
            DomainError::InfeasibleSwing(SwingPlanError::InverseKinematicsNoSolution {
                target_x: aligned.rail_x,
                target_y: 0.0,
                target_z: table::SURFACE_Z,
            })
        })?;
    let joint_count = start.joints.values.len();
    if joint_count < 4 {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::JointOrTorqueLimit {
                target_x: aligned_racket.position.x,
                target_y: aligned_racket.position.y,
                target_z: aligned_racket.position.z,
            },
        ));
    }
    let horizontal_normal = Vector3::new(aligned_racket.normal.x, aligned_racket.normal.y, 0.0);
    if horizontal_normal.norm_squared() <= 1e-12 {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::RacketOrientationUnreachable {
                target_x: aligned_racket.position.x,
                target_y: aligned_racket.position.y,
                target_z: aligned_racket.position.z,
                normal_x: aligned_racket.normal.x,
                normal_y: aligned_racket.normal.y,
                normal_z: aligned_racket.normal.z,
            },
        ));
    };
    let push_direction = horizontal_normal.normalize();
    let target_normal = aligned_racket.normal;

    let try_push_distance = |push_distance_m: f64| -> Result<FixedJointSwing, DomainError> {
        let lift_m = FIXED_JOINT_PUSH_LIFT_M * push_distance_m / FIXED_JOINT_PUSH_DISTANCE_M;
        let target_position = Point3::from(
            aligned_racket.position.coords
                + push_direction * push_distance_m
                + Vector3::z() * lift_m,
        );
        return plan_fixed_joint_swing_power_sweep_to_pose(
            arm,
            start,
            target_position,
            target_normal,
            target_impact_time_secs,
        );
    };

    if let Ok(planned) = try_push_distance(FIXED_JOINT_PUSH_DISTANCE_M) {
        return Ok(planned);
    }

    let low_fraction = 0.30;
    if let Ok(mut best) = try_push_distance(FIXED_JOINT_PUSH_DISTANCE_M * low_fraction) {
        let mut low = low_fraction;
        let mut high = 1.0_f64;
        for _ in 0..PUSH_DISTANCE_BISECTION_STEPS {
            let mid = (low + high) * 0.5;
            match try_push_distance(FIXED_JOINT_PUSH_DISTANCE_M * mid) {
                Ok(planned) => {
                    low = mid;
                    best = planned;
                }
                Err(_) => {
                    high = mid;
                }
            }
        }
        return Ok(best);
    }

    let fallback_distance_m = 0.020;
    let fallback_lift_m =
        FIXED_JOINT_PUSH_LIFT_M * fallback_distance_m / FIXED_JOINT_PUSH_DISTANCE_M;
    let fallback_position = Point3::from(
        aligned_racket.position.coords
            + push_direction * fallback_distance_m
            + Vector3::z() * fallback_lift_m,
    );
    return plan_fixed_joint_swing_power_sweep_to_pose(
        arm,
        start,
        fallback_position,
        target_normal,
        target_impact_time_secs,
    );
}

/// j0·j2 인덱스 — 이 팔에서 토크가 가장 큰 두 관절(이중/단일 MX-64).
const POWER_SWEEP_JOINT_INDICES: [usize; 2] = [0, 2];

fn plan_fixed_joint_swing_power_sweep_to_pose(
    arm: &Arm,
    start: &robot::Pose,
    target_position: Point3,
    target_normal: Vector3<f64>,
    target_impact_time_secs: f64,
) -> Result<FixedJointSwing, DomainError> {
    let joint_count = start.joints.values.len();
    let (impact_pose, _) = arm
        .inverse_pose_at_fixed_rail_best_normal(
            start.rail_x,
            target_position,
            target_normal,
            start,
            robot::IkSearch::Global,
        )
        .map_err(DomainError::InfeasibleSwing)?;

    let impact_time = target_impact_time_secs.max(FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS);
    let ramp_accel = arm.max_joint_speed / FIXED_JOINT_SWING_RAMP_SECS;

    let mut profiles = vec![PreImpactJointProfile::Quadratic; joint_count];
    for &index in &POWER_SWEEP_JOINT_INDICES {
        let q0 = start.joints.values[index];
        let qf = impact_pose.joints.values[index];
        if RampCruiseSegment::new(q0, qf, impact_time, ramp_accel).is_none() {
            return Err(DomainError::InfeasibleSwing(
                SwingPlanError::JointOrTorqueLimit {
                    target_x: target_position.x,
                    target_y: target_position.y,
                    target_z: target_position.z,
                },
            ));
        }
        profiles[index] = PreImpactJointProfile::RampCruise { accel: ramp_accel };
    }
    if let Some(wrist_index) = arm.wrist_joint_index() {
        let dq3 = impact_pose.joints.values[wrist_index] - start.joints.values[wrist_index];
        let snap_duration = (2.0 * dq3.abs() / arm.max_joint_speed)
            .max(FIXED_JOINT_SWING_MIN_SNAP_SECS)
            .min(impact_time);
        let delay = (impact_time - snap_duration).max(0.0);
        profiles[wrist_index] = PreImpactJointProfile::DelayedQuadratic { delay };
    }

    let skipped_joint_indices = (0..joint_count)
        .filter(|index| {
            (impact_pose.joints.values[*index] - start.joints.values[*index]).abs() < 1e-6
        })
        .collect();

    let follow_time = FIXED_JOINT_SWING_FOLLOW_THROUGH_SECS;
    let mut end_values = impact_pose.joints.values.clone();
    let impact_velocity: Vec<f64> = (0..joint_count)
        .map(|i| {
            profiles[i]
                .build(
                    start.joints.values[i],
                    impact_pose.joints.values[i],
                    impact_time,
                )
                .sample(impact_time)
                .1
        })
        .collect();
    for (index, (value, velocity)) in end_values.iter_mut().zip(impact_velocity.iter()).enumerate()
    {
        *value += velocity * follow_time * 0.5;
        if let Some(limit) = arm.joint_limit(index) {
            *value = (*value).clamp(limit.min, limit.max);
        }
    }
    let follow_through_velocity = vec![0.0; joint_count];
    // 레일은 이 스윙 동안 움직이지 않는다(`Rail::fixed`) — 팔로스루 레일
    // 위치도 그대로 시작 위치.
    let follow_rail_x = start.rail_x;

    let trajectory = Trajectory::with_power_sweep(
        start.joints.clone(),
        impact_pose.joints.clone(),
        Joints {
            values: end_values,
        },
        profiles,
        follow_through_velocity,
        impact_time,
        impact_time + follow_time,
        Rail::fixed(start.rail_x),
        follow_rail_x,
        0.0,
    );
    evaluate_trajectory_feasibility(arm, &trajectory, start.rail_x)
        .map_err(DomainError::InfeasibleSwing)?;
    return Ok(FixedJointSwing {
        trajectory,
        skipped_joint_indices,
    });
}
```

(This is the same code as today except: both public functions gained the `target_impact_time_secs: f64` parameter and thread it to the two `try_push_distance`/fallback call sites of `_to_pose`; `_to_pose` gained the same parameter, computes `impact_time` from it instead of the old constant sum, and computes `snap_duration`/`delay` from `dq3` instead of the fixed constant.)

- [ ] **Step 3: Update the three existing tests to the new signature**

In the `#[cfg(test)] mod tests` block, find and replace each call. First, `fixed_joint_swing_power_sweep_j0_j2_sustain_ceiling_speed_before_impact`:

```rust
    #[test]
    fn fixed_joint_swing_power_sweep_j0_j2_sustain_ceiling_speed_before_impact() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let home = robot::Pose::new(
            arm.rail.as_ref().map_or(0.0, |rail| rail.default_x()),
            robot::Joints::from_slice(&crate::defaults::READY_JOINTS_4DOF),
        );
        let planned = plan_fixed_joint_swing_power_sweep(
            arm,
            &home,
            FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS,
        )
        .expect("power sweep swing");
        let trajectory = planned.trajectory;
        for &index in &[0usize, 2usize] {
            let v_end = trajectory.sample_velocity_at(trajectory.impact_time_secs)[index];
            let cruise_probe = trajectory.impact_time_secs - FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS * 0.5;
            let v_mid_cruise = trajectory.sample_velocity_at(cruise_probe.max(0.0))[index];
            assert!(
                v_end.abs() > 1e-6,
                "j{index} should be moving at impact: v={v_end}"
            );
            assert!(
                (v_mid_cruise.abs() - v_end.abs()).abs() < 1e-2,
                "j{index} should already be near peak speed mid-cruise, not just at impact: mid={v_mid_cruise} end={v_end}"
            );
        }
        assert!(
            trajectory.peak_joint_speed() <= arm.max_joint_speed * (1.0 + 1e-9),
            "peak speed must stay within the motor ceiling"
        );
    }
```

(The `cruise_probe` line switches from `FIXED_JOINT_SWING_CRUISE_SECS` to `FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS` — both are 0.06/0.12 in the old code's relationship, but since `FIXED_JOINT_SWING_CRUISE_SECS` no longer exists, probing at `impact_time - MIN_IMPACT_TIME/2` — i.e. 60ms before impact, still comfortably inside the cruise phase for this test's scenario — preserves the same intent: sample partway through the back half of the swing, not just at the very last instant.)

Next, `fixed_joint_swing_power_sweep_holds_wrist_until_the_snap_window`:

```rust
    #[test]
    fn fixed_joint_swing_power_sweep_holds_wrist_until_the_snap_window() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let ready = robot::Pose::new(
            arm.rail.as_ref().map_or(0.0, |rail| rail.default_x()),
            arm.default_joints.clone(),
        );
        let alignment = plan_ball_alignment(
            arm,
            &ready,
            Point3::new(table::WIDTH_X * 0.5, ready_racket_y_m(), 0.95),
        )
        .expect("alignment");
        let start = robot::Pose::new(
            alignment.follow_through_rail_x,
            alignment.follow_through.clone(),
        );
        let planned = plan_fixed_joint_swing_power_sweep(
            arm,
            &start,
            FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS,
        )
        .expect("power sweep swing");
        let trajectory = planned.trajectory;
        let wrist_index = arm.wrist_joint_index().expect("4dof arm has a wrist");
        let dq3 = trajectory.end.values[wrist_index] - trajectory.start.values[wrist_index];
        let expected_snap = (2.0 * dq3.abs() / arm.max_joint_speed)
            .max(FIXED_JOINT_SWING_MIN_SNAP_SECS)
            .min(trajectory.impact_time_secs);
        let hold_end = (trajectory.impact_time_secs - expected_snap).max(0.0);
        if hold_end > 1e-3 {
            let mid_hold = trajectory.sample_at(hold_end * 0.5);
            assert!(
                (mid_hold.values[wrist_index] - start.joints.values[wrist_index]).abs() < 1e-6,
                "wrist should still be cocked mid-hold: {:?}",
                mid_hold.values[wrist_index]
            );
        }
        let v_during_snap =
            trajectory.sample_velocity_at((hold_end + trajectory.impact_time_secs) * 0.5)
                [wrist_index];
        assert!(
            v_during_snap.abs() > 1e-6,
            "wrist should be moving during the snap window"
        );
    }
```

(`hold_end` is now computed from the same adaptive formula the implementation uses, rather than a fixed constant — this test asserts the *shape* of the profile, not a specific hardcoded window.)

Finally, `fixed_joint_swing_power_sweep_stays_within_joint_limits`:

```rust
    #[test]
    fn fixed_joint_swing_power_sweep_stays_within_joint_limits() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let rail_x = arm.rail.as_ref().map_or(0.0, |rail| rail.default_x());
        let mut joints = arm.default_joints.clone();
        joints.values[3] = arm.joint_limit(3).expect("q3 limit").min;
        let start = robot::Pose::new(rail_x, joints);
        let planned = plan_fixed_joint_swing_power_sweep(
            arm,
            &start,
            FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS,
        )
        .expect("power sweep from wrist limit");
        assert!(arm.joints_in_limits(&planned.trajectory.end));
    }
```

- [ ] **Step 4: Run the three updated tests**

Run: `cargo test --lib fixed_joint_swing_power_sweep -- --nocapture`
Expected: all 3 tests PASS (plus whatever other `fixed_joint_swing` tests already existed, e.g. `fixed_joint_swing_from_wrist_limit_stays_inside_limits`, `fixed_joint_swing_quadratic_*` — those are untouched and should still pass).

- [ ] **Step 5: Write the failing regression test for the reported bug**

Add a new test in the same `mod tests` block, after `fixed_joint_swing_power_sweep_stays_within_joint_limits`:

```rust
    /// 2026-08-14 실기 관찰: j3 요구 회전량이 push 거리에 비례해 커지는데
    /// (최대 -24°) 스냅 창이 고정 50ms였던 시절엔 30% 이상 push가 전부
    /// 관절속도 한계로 막혀 이분탐색 저점(30%)조차 실패, 결국 2cm 비상
    /// 폴백까지 떨어졌다 — j0·j2는 100%에서도 충분히 가능했는데 손목 때문에
    /// 전체가 끌려 내려갔다. 스냅 창을 요구 회전량에 맞춰 계산하면 이
    /// 시나리오에서 훨씬 큰 push 거리(대략 80%대)를 찾아야 한다.
    #[test]
    fn fixed_joint_swing_power_sweep_reaches_a_real_push_not_the_emergency_fallback() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let ready = robot::Pose::new(
            arm.rail.as_ref().map_or(0.0, |rail| rail.default_x()),
            arm.default_joints.clone(),
        );
        let alignment = plan_ball_alignment(
            arm,
            &ready,
            Point3::new(table::WIDTH_X * 0.5, ready_racket_y_m(), 0.95),
        )
        .expect("alignment");
        let start = robot::Pose::new(
            alignment.follow_through_rail_x,
            alignment.follow_through.clone(),
        );
        let aligned_racket = arm
            .forward_kinematics_with_rail(start.rail_x, &start.joints)
            .expect("aligned FK");
        let planned = plan_fixed_joint_swing_power_sweep(
            arm,
            &start,
            FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS,
        )
        .expect("power sweep swing");
        let impact = arm
            .forward_kinematics_with_rail(planned.trajectory.rail.end, &planned.trajectory.end)
            .expect("impact FK");
        let horizontal_normal =
            Vector3::new(aligned_racket.normal.x, aligned_racket.normal.y, 0.0).normalize();
        let pushed_distance = (impact.position - aligned_racket.position).dot(&horizontal_normal);
        assert!(
            pushed_distance > 0.05,
            "push distance collapsed toward the 2cm emergency fallback: {pushed_distance:.4}m"
        );
    }
```

- [ ] **Step 6: Run it, confirm it fails against the pre-fix behavior conceptually, then confirm it passes against the code as written in Step 2**

Run: `cargo test --lib fixed_joint_swing_power_sweep_reaches_a_real_push -- --nocapture`
Expected: PASS (the fix is already in place from Step 2 — this step's purpose is confirming the regression guard holds, not a red/green cycle on this specific test since the implementation was written in Step 2). If it fails, the `pushed_distance` assertion threshold or the adaptive-snap implementation has a bug — do not loosen the threshold to make it pass; investigate why the push distance is still collapsing.

- [ ] **Step 7: Write and run a test for the minimum-duration clamp**

Add:

```rust
    #[test]
    fn fixed_joint_swing_power_sweep_clamps_too_short_target_duration_to_the_floor() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let home = robot::Pose::new(
            arm.rail.as_ref().map_or(0.0, |rail| rail.default_x()),
            robot::Joints::from_slice(&crate::defaults::READY_JOINTS_4DOF),
        );
        let planned = plan_fixed_joint_swing_power_sweep(arm, &home, 0.010)
            .expect("power sweep swing even with a too-short requested duration");
        assert!(
            (planned.trajectory.impact_time_secs - FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS).abs()
                < 1e-9,
            "impact_time_secs={}",
            planned.trajectory.impact_time_secs
        );
    }
```

Run: `cargo test --lib fixed_joint_swing_power_sweep_clamps_too_short -- --nocapture`
Expected: PASS.

- [ ] **Step 8: Write and run a test that adaptive snap duration actually scales with required rotation**

Add:

```rust
    /// 손목 요구 회전량이 다른 두 임팩트 자세를 직접 비교해, 스냅 창이
    /// 고정값이 아니라 회전량에 비례해 커지는지 확인한다.
    #[test]
    fn fixed_joint_swing_power_sweep_snap_window_scales_with_required_rotation() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let ready = robot::Pose::new(
            arm.rail.as_ref().map_or(0.0, |rail| rail.default_x()),
            arm.default_joints.clone(),
        );
        let alignment = plan_ball_alignment(
            arm,
            &ready,
            Point3::new(table::WIDTH_X * 0.5, ready_racket_y_m(), 0.95),
        )
        .expect("alignment");
        let start = robot::Pose::new(
            alignment.follow_through_rail_x,
            alignment.follow_through.clone(),
        );
        let aligned_racket = arm
            .forward_kinematics_with_rail(start.rail_x, &start.joints)
            .expect("aligned FK");
        let horizontal_normal =
            Vector3::new(aligned_racket.normal.x, aligned_racket.normal.y, 0.0).normalize();

        let snap_window_for = |push_distance_m: f64| -> f64 {
            let lift_m = FIXED_JOINT_PUSH_LIFT_M * push_distance_m / FIXED_JOINT_PUSH_DISTANCE_M;
            let target_position = Point3::from(
                aligned_racket.position.coords
                    + horizontal_normal * push_distance_m
                    + Vector3::z() * lift_m,
            );
            let planned = plan_fixed_joint_swing_power_sweep_to_pose(
                arm,
                &start,
                target_position,
                aligned_racket.normal,
                FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS,
            )
            .expect("power sweep to a specific push distance");
            let wrist_index = arm.wrist_joint_index().expect("4dof arm has a wrist");
            let dq3 = planned.trajectory.end.values[wrist_index]
                - planned.trajectory.start.values[wrist_index];
            return (2.0 * dq3.abs() / arm.max_joint_speed)
                .max(FIXED_JOINT_SWING_MIN_SNAP_SECS)
                .min(planned.trajectory.impact_time_secs);
        };

        let small_push_snap = snap_window_for(FIXED_JOINT_PUSH_DISTANCE_M * 0.10);
        let large_push_snap = snap_window_for(FIXED_JOINT_PUSH_DISTANCE_M * 0.60);
        assert!(
            large_push_snap > small_push_snap + 1e-3,
            "snap window should grow with required rotation: small={small_push_snap:.4} large={large_push_snap:.4}"
        );
    }
```

Run: `cargo test --lib fixed_joint_swing_power_sweep_snap_window_scales -- --nocapture`
Expected: PASS.

- [ ] **Step 9: Run the full physics test module**

Run: `cargo test --lib physics::`
Expected: same pass/fail set as the pre-existing baseline (`fixed_joint_swing_pushes_forward_without_backswing` is a known pre-existing failure unrelated to this work — see `docs/superpowers/plans/2026-08-14-power-sweep-swing.md`'s history; do not attempt to fix it here), plus all new/updated power-sweep tests passing.

- [ ] **Step 10: Commit**

```bash
git add src/robot/motion/physics.rs
git commit -m "fix(motion): adapt power-sweep snap window and duration instead of hardcoding them"
```

---

### Task 3: Update `Planner` wrappers

**Files:**
- Modify: `src/robot/motion/planner.rs`

**Interfaces:**
- Consumes: `physics::plan_fixed_joint_swing_power_sweep(arm, start, target_impact_time_secs)`, `physics::plan_fixed_joint_swing_power_sweep_from_alignment(arm, start, aligned, target_impact_time_secs)` (Task 2).
- Produces: `Planner::fixed_joint_swing_power_sweep(arm: &Arm, start: &robot::Pose, target_impact_time_secs: f64) -> Result<physics::FixedJointSwing, DomainError>`, `Planner::fixed_joint_swing_power_sweep_from_alignment(arm: &Arm, start: &robot::Pose, aligned: &robot::Pose, target_impact_time_secs: f64) -> Result<physics::FixedJointSwing, DomainError>`.

- [ ] **Step 1: Update the two wrapper functions**

In `src/robot/motion/planner.rs`, find:

```rust
    /// [`Self::fixed_joint_swing_quadratic`]를 대체하는 파워 스윙 — j0·j2가
    /// 관절 속도 상한까지 가속-순항하며 임팩트를 만들고, j3는 접힌 자세로
    /// 대기하다 임팩트 직전에만 스냅한다.
    pub fn fixed_joint_swing_power_sweep(
        arm: &Arm,
        start: &robot::Pose,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing_power_sweep(arm, start);
    }

    /// [`Self::fixed_joint_swing_power_sweep`]의 정렬-기준 버전.
    pub fn fixed_joint_swing_power_sweep_from_alignment(
        arm: &Arm,
        start: &robot::Pose,
        aligned: &robot::Pose,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing_power_sweep_from_alignment(arm, start, aligned);
    }
```

Replace with:

```rust
    /// [`Self::fixed_joint_swing_quadratic`]를 대체하는 파워 스윙 — j0·j2가
    /// 관절 속도 상한까지 가속-순항하며 임팩트를 만들고, j3는 요구
    /// 회전량에 맞춘 스냅 창으로 접힌 자세를 유지하다 스냅한다.
    /// `target_impact_time_secs`는 타격-전 전체 시간의 목표값이다
    /// (`FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS` 미만이면 그 값으로
    /// 클램프된다).
    pub fn fixed_joint_swing_power_sweep(
        arm: &Arm,
        start: &robot::Pose,
        target_impact_time_secs: f64,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing_power_sweep(arm, start, target_impact_time_secs);
    }

    /// [`Self::fixed_joint_swing_power_sweep`]의 정렬-기준 버전.
    pub fn fixed_joint_swing_power_sweep_from_alignment(
        arm: &Arm,
        start: &robot::Pose,
        aligned: &robot::Pose,
        target_impact_time_secs: f64,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing_power_sweep_from_alignment(
            arm,
            start,
            aligned,
            target_impact_time_secs,
        );
    }
```

- [ ] **Step 2: Verify it compiles (expect errors from control_worker.rs)**

Run: `cargo build --lib 2>&1 | tail -20`
Expected: `src/robot/motion/planner.rs` compiles cleanly on its own (it's part of the lib target). The bin target (`cargo build --bin pingpong-bot`) will fail at this point because `control_worker.rs` still calls the old 3-argument form — that's expected; Task 4 fixes it. Do not fix `control_worker.rs` in this task.

- [ ] **Step 3: Commit**

```bash
git add src/robot/motion/planner.rs
git commit -m "feat(motion): thread target_impact_time_secs through Planner power-sweep wrappers"
```

---

### Task 4: Dynamic duration in `control_worker.rs`

**Files:**
- Modify: `src/real/control_worker.rs`

**Interfaces:**
- Consumes: `Planner::fixed_joint_swing_power_sweep_from_alignment(arm, start, aligned, target_impact_time_secs: f64)` (Task 3), `pingpong_bot::defaults::FIXED_JOINT_SWING_IMPACT_MARGIN_SECS`, `pingpong_bot::defaults::FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS` (Task 1 — need adding to the re-export list in `src/defaults/mod.rs` since `control_worker.rs` is in the bin crate and uses fully-qualified `pingpong_bot::defaults::` paths, unlike `physics.rs` which uses `crate::defaults::motion::`).
- Produces: `fn target_impact_time_secs(predicted_arrival_at: Instant, now: Instant) -> f64` (private, in `control_worker.rs`), `BallControlState::Aligning { .., predicted_arrival_at: Instant, .. }` (new field).

- [ ] **Step 1: Add the two new constants to the `defaults/mod.rs` re-export list**

`control_worker.rs` references constants via `pingpong_bot::defaults::CONST_NAME` (fully-qualified, since it's a separate binary crate consuming the library), so they must be re-exported at the `defaults` module root, not just defined in `defaults::motion`.

In `src/defaults/mod.rs`, find:

```rust
    FIXED_JOINT_SWING_DURATION_SECS, FIXED_JOINT_SWING_LEAD_SECS,
    FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS, HOME_RETURN_SPEED_RATIO,
```

Replace with:

```rust
    FIXED_JOINT_SWING_DURATION_SECS, FIXED_JOINT_SWING_IMPACT_MARGIN_SECS,
    FIXED_JOINT_SWING_LEAD_SECS, FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS,
    FIXED_JOINT_SWING_MIN_SNAP_SECS, FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS,
    HOME_RETURN_SPEED_RATIO,
```

(`FIXED_JOINT_SWING_MIN_SNAP_SECS` isn't consumed by `control_worker.rs` in this task, but re-exporting it now keeps the module-root list matching what actually exists in `defaults::motion` for future consumers, following the pattern already established for the other `FIXED_JOINT_SWING_*` constants.)

- [ ] **Step 2: Verify it compiles**

Run: `cargo build --lib 2>&1 | tail -10`
Expected: succeeds.

- [ ] **Step 3: Write the failing tests for the pure timing helper**

`Instant` has no public constructor from an absolute value, so the helper takes two `Instant`s and is tested via `Instant::now() + Duration` arithmetic, which *is* constructible.

Find the test module's imports near the top of the `#[cfg(test)] mod tests` block in `src/real/control_worker.rs` (it already imports `use super::*;`, giving access to `target_impact_time_secs` once defined, and `std::time::{Duration, Instant}` are already imported at the top of the file per Step 6 below). Add these three tests anywhere in the `mod tests` block, e.g. right before `real_fixed_swing_lead_matches_power_sweep_lead_constant`:

```rust
    #[test]
    fn target_impact_time_secs_matches_todays_fixed_value_when_on_schedule() {
        let now = Instant::now();
        let predicted_arrival_at = now + FIXED_SWING_LEAD;
        let target = target_impact_time_secs(predicted_arrival_at, now);
        assert!(
            (target - pingpong_bot::defaults::FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS).abs()
                < 1e-9,
            "target={target}"
        );
    }

    #[test]
    fn target_impact_time_secs_uses_remaining_time_when_above_the_floor() {
        let now = Instant::now();
        // 500ms left, margin 200ms -> 300ms raw, above the 120ms floor.
        let predicted_arrival_at = now + Duration::from_millis(500);
        let target = target_impact_time_secs(predicted_arrival_at, now);
        assert!((target - 0.300).abs() < 1e-9, "target={target}");
    }

    #[test]
    fn target_impact_time_secs_clamps_to_the_floor_when_late() {
        let now = Instant::now();
        // Only 50ms left -- far less than the 200ms margin needs.
        let predicted_arrival_at = now + Duration::from_millis(50);
        let target = target_impact_time_secs(predicted_arrival_at, now);
        assert!(
            (target - pingpong_bot::defaults::FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS).abs()
                < 1e-9,
            "target={target}"
        );
    }
```

- [ ] **Step 4: Run the tests to verify they fail**

Run: `cargo test --bin pingpong-bot target_impact_time_secs`
Expected: FAIL with "cannot find function `target_impact_time_secs`".

- [ ] **Step 5: Implement the helper function**

Add this function near the top of `src/real/control_worker.rs`, right after the constant block (after `const RAIL_ERROR_WARN_M: f64 = 0.020;` or wherever the constants block ends — place it as a standalone `fn` at module scope, before `pub fn run` or whatever the next item is):

```rust
/// 예상 도착 시각까지 남은 시간에서 여유를 뺀 만큼을 파워 스윙의 목표
/// 타격-전 시간으로 쓴다. `Instant`는 임의 절대값으로 만들 수 없어서(원점이
/// 없음) 순수함수로 분리해야 단위 테스트가 가능하다 — 실제 호출부는 항상
/// `predicted_arrival_at`과 `Instant::now()`를 넘긴다.
fn target_impact_time_secs(predicted_arrival_at: Instant, now: Instant) -> f64 {
    let remaining = predicted_arrival_at
        .saturating_duration_since(now)
        .as_secs_f64();
    return (remaining - pingpong_bot::defaults::FIXED_JOINT_SWING_IMPACT_MARGIN_SECS)
        .max(pingpong_bot::defaults::FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS);
}
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `cargo test --bin pingpong-bot target_impact_time_secs`
Expected: all 3 PASS.

- [ ] **Step 7: Add `predicted_arrival_at` to `BallControlState::Aligning`**

Find:

```rust
enum BallControlState {
    Idle,
    Aligning {
        swing_due_at: Instant,
        swing_attempted: bool,
        return_due_at: Instant,
        measurement: PendingAlignmentMeasurement,
    },
    Waiting,
}
```

Replace with:

```rust
enum BallControlState {
    Idle,
    Aligning {
        swing_due_at: Instant,
        swing_attempted: bool,
        return_due_at: Instant,
        /// 파워 스윙 목표 타격-전 시간을 스윙 트리거 시점에 다시 계산하는 데
        /// 쓴다 (`target_impact_time_secs`) — 매 새 예측마다 `swing_due_at`과
        /// 함께 갱신된다.
        predicted_arrival_at: Instant,
        measurement: PendingAlignmentMeasurement,
    },
    Waiting,
}
```

- [ ] **Step 8: Update the construction site to set the new field**

Find (around where `swing_due_at`/`return_due_at` are computed from `predicted_arrival_at`):

```rust
            let predicted_arrival_at = request.trajectory.origin + target.t;
            let swing_due_at = predicted_arrival_at
                .checked_sub(FIXED_SWING_LEAD)
                .unwrap_or(issued_at);
            let return_due_at = predicted_arrival_at
                + Duration::from_secs_f64(pingpong_bot::defaults::POST_ALIGNMENT_HOLD_SECS);
            state = BallControlState::Aligning {
                swing_due_at,
                swing_attempted: false,
                return_due_at,
                measurement: PendingAlignmentMeasurement {
                    track_seq,
                    rail_commanded_m,
                    joints_commanded: alignment.follow_through.clone(),
                },
            };
```

Replace the `state = BallControlState::Aligning { ... }` block with:

```rust
            state = BallControlState::Aligning {
                swing_due_at,
                swing_attempted: false,
                return_due_at,
                predicted_arrival_at,
                measurement: PendingAlignmentMeasurement {
                    track_seq,
                    rail_commanded_m,
                    joints_commanded: alignment.follow_through.clone(),
                },
            };
```

(`predicted_arrival_at` is already in scope as a local variable computed on the line just above — field-init shorthand.)

- [ ] **Step 9: Update the swing-trigger match arm to carry `predicted_arrival_at` through**

Find:

```rust
            let due_swing = match &state {
                BallControlState::Aligning {
                    swing_due_at,
                    swing_attempted,
                    measurement,
                    ..
                } if !swing_attempted && Instant::now() >= *swing_due_at => {
                    Some((
                        measurement.track_seq,
                        *swing_due_at,
                        pingpong_bot::robot::Pose::new(
                            measurement.rail_commanded_m,
                            measurement.joints_commanded.clone(),
                        ),
                    ))
                }
                BallControlState::Idle
                | BallControlState::Waiting
                | BallControlState::Aligning { .. } => None,
            };
            if let Some((track_seq, swing_due_at, aligned_target)) = due_swing {
```

Replace with:

```rust
            let due_swing = match &state {
                BallControlState::Aligning {
                    swing_due_at,
                    swing_attempted,
                    predicted_arrival_at,
                    measurement,
                    ..
                } if !swing_attempted && Instant::now() >= *swing_due_at => {
                    Some((
                        measurement.track_seq,
                        *swing_due_at,
                        *predicted_arrival_at,
                        pingpong_bot::robot::Pose::new(
                            measurement.rail_commanded_m,
                            measurement.joints_commanded.clone(),
                        ),
                    ))
                }
                BallControlState::Idle
                | BallControlState::Waiting
                | BallControlState::Aligning { .. } => None,
            };
            if let Some((track_seq, swing_due_at, predicted_arrival_at, aligned_target)) = due_swing {
```

- [ ] **Step 10: Compute the dynamic duration and pass it to the planner call**

Find:

```rust
                match hardware.read_pose() {
                    Ok(swing_start) => {
                        let swing_arm = arm_for_rail_position(&arm, swing_start.rail_x);
                        match Planner::fixed_joint_swing_power_sweep_from_alignment(
                            &swing_arm,
                            &swing_start,
                            &aligned_target,
                        ) {
```

Replace with:

```rust
                match hardware.read_pose() {
                    Ok(swing_start) => {
                        let swing_arm = arm_for_rail_position(&arm, swing_start.rail_x);
                        let swing_target_impact_time_secs =
                            target_impact_time_secs(predicted_arrival_at, Instant::now());
                        match Planner::fixed_joint_swing_power_sweep_from_alignment(
                            &swing_arm,
                            &swing_start,
                            &aligned_target,
                            swing_target_impact_time_secs,
                        ) {
```

- [ ] **Step 11: Log the computed duration for observability**

Find (inside the `Ok(())` arm after `command_result`):

```rust
                                    info!(
                                        target: "latency",
                                        track_seq,
                                        scheduled_lead_secs = FIXED_SWING_LEAD.as_secs_f64(),
                                        start_late_ms = f2(swing_due_at.elapsed().as_secs_f64() * 1e3),
                                        command_send_ms = f2(command_send_ms),
                                        swing_duration_secs = f4(swing.duration_secs),
```

Replace with:

```rust
                                    info!(
                                        target: "latency",
                                        track_seq,
                                        scheduled_lead_secs = FIXED_SWING_LEAD.as_secs_f64(),
                                        start_late_ms = f2(swing_due_at.elapsed().as_secs_f64() * 1e3),
                                        command_send_ms = f2(command_send_ms),
                                        target_impact_time_secs = f4(swing_target_impact_time_secs),
                                        swing_duration_secs = f4(swing.duration_secs),
```

- [ ] **Step 12: Update the test-only construction site**

Find:

```rust
        let mut state = BallControlState::Aligning {
            swing_due_at: Instant::now(),
            swing_attempted: false,
            return_due_at: Instant::now(),
            measurement: PendingAlignmentMeasurement {
                track_seq: 9,
                rail_commanded_m: rail.default_x(),
                joints_commanded: robot.arm.default_joints.clone(),
            },
        };
```

Replace with:

```rust
        let mut state = BallControlState::Aligning {
            swing_due_at: Instant::now(),
            swing_attempted: false,
            return_due_at: Instant::now(),
            predicted_arrival_at: Instant::now(),
            measurement: PendingAlignmentMeasurement {
                track_seq: 9,
                rail_commanded_m: rail.default_x(),
                joints_commanded: robot.arm.default_joints.clone(),
            },
        };
```

- [ ] **Step 13: Build and run the full control_worker test suite**

Run: `cargo build --bin pingpong-bot 2>&1 | tail -30`
Expected: succeeds with no errors.

Run: `cargo test --bin pingpong-bot real::control_worker`
Expected: same pass/fail set as the pre-existing baseline (`alignment_target_applies_remaining_negative_x_correction` and `delayed_vision_request_is_advanced_instead_of_dropped` are known pre-existing failures unrelated to this work — do not attempt to fix them here), plus all `target_impact_time_secs_*` tests passing and `real_fixed_swing_lead_matches_power_sweep_lead_constant` still passing unchanged.

- [ ] **Step 14: Commit**

```bash
git add src/defaults/mod.rs src/real/control_worker.rs
git commit -m "feat(real): derive power-sweep duration from estimated ball arrival time"
```

---

### Task 5: Full-suite verification

**Files:** none — verification only.

- [ ] **Step 1: Run the full lib test suite**

Run: `cargo test --lib 2>&1 | tail -20`
Expected: same pre-existing failures as the established baseline (`defaults::tests::presets_validate`, `robot::motion::physics::tests::fixed_joint_swing_pushes_forward_without_backswing`, `sim::eval::protocol::smoke::protocol_runs_and_prints_score`, `sim::physics::arm_bodies::tests::spawns_four_joints_dual_yaw_torque_from_entry`, `sim::physics::world::tests::direct_control_on_shoot_moves_rail_to_centered_ball`, `sim::physics::world::tests::every_joint_reaches_commanded_pose_at_real_ball_contact`, `sim::physics::world::tests::quintic_swing_moves_robot_joints`), no new failures, and all new/updated tests from Tasks 2 and 4 passing.

- [ ] **Step 2: Run the full bin test suite**

Run: `cargo test --bin pingpong-bot 2>&1 | tail -20`
Expected: same pre-existing failures as baseline (`real::control_worker::tests::alignment_target_applies_remaining_negative_x_correction`, `real::control_worker::tests::delayed_vision_request_is_advanced_instead_of_dropped`), no new failures.

- [ ] **Step 3: Run clippy on the touched files**

Run: `cargo clippy --lib --bin pingpong-bot 2>&1 | grep -E "^(warning|error)" | grep -v needless_return | grep -iE "physics\.rs|planner\.rs|control_worker\.rs|motion\.rs" `
Expected: no output beyond pre-existing categories already present in this codebase (index-loop-variable style, doc-comment overindent, etc. — this codebase does not gate on `clippy -D warnings`; see the previous plan's Task 5 note). If a genuinely new category of warning appears in these files that wasn't already present before this plan's changes, fix it.

- [ ] **Step 4: Manual sanity check of the fix**

Run: `cargo test --lib fixed_joint_swing_power_sweep_reaches_a_real_push -- --nocapture` and confirm the `pushed_distance` in the assertion failure message (if it were to fail) or a quick temporary `eprintln!` shows a value meaningfully larger than the old ~0.02m fallback — this is the concrete evidence the reported bug is fixed. Do not leave any temporary debug output in the committed code.

- [ ] **Step 5: No commit needed for this task** — it's verification only. If Step 3 required a fix, commit that fix separately with an appropriate message.
