# Home-Position Return Speed Ratio Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the linear rail (and the arm joints moving with it) travel noticeably slower than rally speed whenever the robot moves to a home/ready position — i.e. on startup ("go to center") and when switching test-control zones (modes 1/2/3, keys `1`/`2`/`3`) — without changing the speed of any rally/ball-tracking move.

**Architecture:** Every stop-to-stop move (home return, rally alignment, ready-prewind, etc.) is planned by one shared function, `physics::plan_move_to`, which picks the *shortest feasible* trajectory duration for a given start/target pose. There is currently no way to ask it for a slower move. We add a `speed_ratio` parameter (1.0 = current/full speed) that scales the duration search's time estimate and its min/max bounds, expose it through the existing `Planner` delegate layer as new sibling methods (the existing zero-arg-ratio methods are untouched, so no other caller or test is affected), and use it only inside `plan_neutral_return_segments` in `src/real/control_worker.rs` — the single function both home-position switching (`move_to_ready`) and startup centering (`initialize_pose_attempt`) funnel through.

**Tech Stack:** Rust (edition 2024), plain `#[test]` unit tests (no external test framework), `cargo test` / `cargo build`.

## Global Constraints

- The slowdown must apply to home-position switching (modes 1/2/3) and to startup "go to center", and must NOT apply to rally ball-alignment moves (`plan_ball_alignment`, `plan_ball_alignment_fixed_rail`) or the ready-prewind move (`plan_ready_prewind`).
- The whole move (rail AND arm joints together) slows down together — this was an explicit choice over a rail-only change, because it reuses the existing duration-search planner instead of decoupling rail/joint timing.
- The slowdown factor must be a single named, tunable constant, not a hardcoded literal scattered across call sites. Initial value: `1.0 / 3.0` (i.e. home moves take about 3x as long / move about 3x slower than the equivalent rally move would).
- All changes must be purely additive: every existing public function signature (`plan_move_to`, `plan_return_to_center_at`, `Planner::move_to`, `Planner::return_to_center_at`, etc.) keeps its current signature and behavior unchanged, so no existing call site or test needs to change.

---

## File Structure

- Modify: `src/robot/motion/physics.rs` — add `plan_move_to_at_speed_ratio` and `plan_return_to_center_at_speed_ratio`; `plan_move_to`/`plan_return_to_center_at` become thin wrappers around them with `speed_ratio = 1.0`.
- Modify: `src/robot/motion/planner.rs` — add `Planner::move_to_at_speed_ratio` and `Planner::return_to_center_at_speed_ratio` delegate methods.
- Modify: `src/defaults/motion.rs` — add `HOME_RETURN_SPEED_RATIO: f64 = 1.0 / 3.0`.
- Modify: `src/defaults/mod.rs` — re-export `HOME_RETURN_SPEED_RATIO`.
- Modify: `src/real/control_worker.rs` — `plan_neutral_return_segments` calls the new `_speed_ratio` variants with `HOME_RETURN_SPEED_RATIO` instead of the full-speed ones.

---

### Task 1: Add `plan_move_to_at_speed_ratio` to `physics.rs`

**Files:**
- Modify: `src/robot/motion/physics.rs:1004-1076` (the `plan_move_to` function)
- Test: same file, `#[cfg(test)] mod tests` block (ends around line 1870+; add new tests near the existing `return_to_center_at_targets_the_given_rail_x` test at line 1657-1669)

**Interfaces:**
- Produces: `pub fn plan_move_to_at_speed_ratio(arm: &Arm, start: &robot::Pose, center_joints: Joints, center_rail_x: f64, speed_ratio: f64) -> Result<Trajectory, DomainError>` — same contract as `plan_move_to`, but the planned trajectory's `duration_secs` is inflated by roughly `1.0 / speed_ratio` (a `speed_ratio` of `1.0` reproduces `plan_move_to`'s exact result).
- Consumes: nothing new — uses the same `RETURN_TO_CENTER_MIN_SECS`, `RETURN_TO_CENTER_MAX_SECS`, `RETURN_TO_CENTER_GROWTH` constants already imported at the top of `physics.rs`, and the existing private `build_feasible_trajectory` helper.

**Design note (corrected after implementation surfaced a bug in the first draft):** the original draft tried to re-run the same shortest-feasible-duration *search* with every internal estimate divided by `speed_ratio`. That search's estimates are only a *starting guess* — `build_feasible_trajectory`'s pass/fail depends solely on the arm/rail's real physical limits, not on `speed_ratio`. When the rail distance dominates (the common case for home moves), the scaled starting guess lands right back on approximately the same physically-minimal duration the full-speed search would have found, so the "slow" plan ended up barely slower at all (empirically ~1.09x instead of the intended 3x). The fix: compute the actual full-speed duration first (by calling the untouched original search, factored into a private helper), then build a single trajectory at exactly `full_speed.duration_secs / speed_ratio`. This is deterministic and exact, and it reuses `build_feasible_trajectory` directly instead of re-deriving a duration through search.

- [ ] **Step 1: Write the failing tests**

Add these two tests inside the existing `#[cfg(test)] mod tests { ... }` block in `src/robot/motion/physics.rs`, right after the `return_to_center_at_targets_the_given_rail_x` test (around line 1669):

```rust
    #[test]
    fn plan_move_to_at_speed_ratio_one_matches_plan_move_to() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let rail = arm.rail.expect("rail 있는 로봇");
        let start = robot::Pose::new(rail.x_max, arm.default_joints.clone());

        let via_plain =
            plan_move_to(arm, &start, arm.default_joints.clone(), rail.x_min).expect("plan_move_to");
        let via_ratio =
            plan_move_to_at_speed_ratio(arm, &start, arm.default_joints.clone(), rail.x_min, 1.0)
                .expect("plan_move_to_at_speed_ratio ratio=1.0");

        assert_eq!(via_plain.duration_secs, via_ratio.duration_secs);
    }

    #[test]
    fn plan_move_to_at_speed_ratio_slows_down_for_ratio_below_one() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let rail = arm.rail.expect("rail 있는 로봇");
        let start = robot::Pose::new(rail.x_max, arm.default_joints.clone());

        let full_speed =
            plan_move_to_at_speed_ratio(arm, &start, arm.default_joints.clone(), rail.x_min, 1.0)
                .expect("전속 이동 계획");
        let slow =
            plan_move_to_at_speed_ratio(arm, &start, arm.default_joints.clone(), rail.x_min, 1.0 / 3.0)
                .expect("저속 이동 계획");

        assert!(
            (slow.duration_secs - full_speed.duration_secs * 3.0).abs() < 1e-9,
            "slow={} full={}",
            slow.duration_secs,
            full_speed.duration_secs
        );
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib plan_move_to_at_speed_ratio -- --nocapture`
Expected: FAIL to compile — `plan_move_to_at_speed_ratio` does not exist yet (`cannot find function` error).

- [ ] **Step 3: Implement `plan_move_to_at_speed_ratio` on top of a new private `plan_move_to_full_speed` helper**

First, extract the *entire current body* of `plan_move_to` (`src/robot/motion/physics.rs:1004-1076`, everything between the outer `{ }`) verbatim into a new private function `plan_move_to_full_speed` with the same parameters — do not change a single line of its logic, only its name and visibility (`fn` instead of `pub fn`, no doc comment needed since it's now a private implementation detail). Then replace the original `plan_move_to`'s doc comment and signature, and add the new public function, so the whole region becomes:

```rust
/// 정지 → 정지로 임의의 포즈까지 잇는 최단 실행가능 궤적.
///
/// [`plan_return_to_center`]가 목표만 센터로 고정한 특수형이고, real의 coarse 선추종도
/// 같은 것이 필요하다 — 임팩트 근처로 미리 옮겨두면 커밋 스윙이 이동까지 떠맡지 않는다.
pub fn plan_move_to(
    arm: &Arm,
    start: &robot::Pose,
    center_joints: Joints,
    center_rail_x: f64,
) -> Result<Trajectory, DomainError> {
    return plan_move_to_full_speed(arm, start, center_joints, center_rail_x);
}

/// [`plan_move_to`]와 같지만 관절·레일 속도를 `speed_ratio`(0보다 크고 1 이하)만큼
/// 늦춘 궤적을 계획한다 — 홈 포지션 복귀처럼 랠리보다 느려도 되는 이동에 쓴다.
/// `speed_ratio == 1.0`이면 [`plan_move_to`]와 완전히 같은 결과를 낸다.
///
/// 전속 탐색의 추정 시작값을 `speed_ratio`로 나눠 다시 탐색하지 않는다 — 레일
/// 거리가 지배적인 이동에서는 그 추정값이 실제 물리적 최단 시간과 우연히
/// 비슷해서, 탐색이 곧바로 성공해 버리면 사실상 느려지지 않는다(실측: 3배
/// 느리길 기대했는데 1.09배). 대신 전속 탐색이 찾아낸 **실제** 최단 시간을
/// `1/speed_ratio`로 늘려 그대로 쓴다 — 정지→정지 quintic은 시간을 늘릴수록
/// 필요 속도·가속도·토크가 줄어들므로, 전속에서 성공한 궤적은 그보다 긴
/// 시간에서도 성공한다.
///
/// `duration_secs`는 항상 `건네준 duration + follow_time`(고정 팔로스루
/// 유지시간)이다(`trajectory_with_follow_through`). `full_speed.duration_secs`를
/// 그대로 `speed_ratio`로 나눠 `duration` 인자로 되돌리면 follow_time이
/// 두 번(전속 결과에 한 번, 저속 궤적에 다시 한 번) 늘어나 총 시간이 정확히
/// `1/speed_ratio`배가 되지 않는다(실측: 0.06초 어긋남) — 미리 `follow_time`을
/// 빼서 보정한다.
pub fn plan_move_to_at_speed_ratio(
    arm: &Arm,
    start: &robot::Pose,
    center_joints: Joints,
    center_rail_x: f64,
    speed_ratio: f64,
) -> Result<Trajectory, DomainError> {
    let full_speed = plan_move_to_full_speed(arm, start, center_joints.clone(), center_rail_x)?;
    if speed_ratio >= 1.0 {
        return Ok(full_speed);
    }
    let follow_time = defaults::ControlParams::default().swing_follow_through_secs;
    let slow_duration = full_speed.duration_secs / speed_ratio - follow_time;
    let start_velocity = vec![0.0; start.joints.values.len()];
    let end_velocity = vec![0.0; center_joints.values.len()];
    let rail = Rail {
        start: start.rail_x,
        end: center_rail_x,
        start_velocity: 0.0,
        end_velocity: 0.0,
    };
    return build_feasible_trajectory(
        arm,
        &start.joints,
        center_joints,
        start_velocity,
        end_velocity,
        slow_duration,
        rail,
    )
    .map_err(DomainError::InfeasibleSwing);
}

/// [`plan_move_to`]의 실제 탐색 로직 — 전속 결과가 [`plan_move_to_at_speed_ratio`]의
/// 감속 기준(실제 최단 시간)으로도 쓰인다.
fn plan_move_to_full_speed(
    arm: &Arm,
    start: &robot::Pose,
    center_joints: Joints,
    center_rail_x: f64,
) -> Result<Trajectory, DomainError> {
    let start_velocity = vec![0.0; start.joints.values.len()];
    let end_velocity = vec![0.0; center_joints.values.len()];

    // 끝속도가 항상 0이라 `fit_end_velocity`의 스케일링은 아무 것도 못 바꾼다
    // (0에 뭘 곱해도 0) — 첫 시도부터 웬만하면 통과하도록, 실제 이동 거리
    // 기준 등속 근사(0.5배 여유, quintic 첨두 속도가 평균보다 크므로)로 시작
    // 시간을 추정해 무의미한 재시도(각 32회 반복)를 줄인다.
    let joint_distance = start
        .joints
        .values
        .iter()
        .zip(center_joints.values.iter())
        .map(|(actual, home)| (actual - home).abs())
        .fold(0.0_f64, f64::max);
    let rail_distance = (start.rail_x - center_rail_x).abs();
    let joint_time_estimate = if arm.max_joint_speed > 0.0 {
        joint_distance / (arm.max_joint_speed * 0.5)
    } else {
        0.0
    };
    let rail_time_estimate = arm.rail.as_ref().map_or(0.0, |rail| {
        if rail.max_speed > 0.0 {
            rail_distance / (rail.max_speed * 0.5)
        } else {
            0.0
        }
    });

    let mut duration = joint_time_estimate
        .max(rail_time_estimate)
        .max(RETURN_TO_CENTER_MIN_SECS);
    let mut last_error = None;
    while duration <= RETURN_TO_CENTER_MAX_SECS {
        let rail = Rail {
            start: start.rail_x,
            end: center_rail_x,
            start_velocity: 0.0,
            end_velocity: 0.0,
        };
        match build_feasible_trajectory(
            arm,
            &start.joints,
            center_joints.clone(),
            start_velocity.clone(),
            end_velocity.clone(),
            duration,
            rail,
        ) {
            Ok(trajectory) => return Ok(trajectory),
            Err(error) => {
                last_error = Some(error);
                duration *= RETURN_TO_CENTER_GROWTH;
            }
        }
    }
    return Err(DomainError::InfeasibleSwing(last_error.unwrap_or(
        SwingPlanError::InverseKinematicsNoSolution {
            target_x: center_rail_x,
            target_y: 0.0,
            target_z: table::SURFACE_Z,
        },
    )));
}
```

Note `plan_move_to_full_speed`'s body is byte-for-byte the original `plan_move_to` body — only the function name changed and the `pub` was dropped. This guarantees `plan_move_to`'s behavior is completely unchanged (verified by the `plan_move_to_at_speed_ratio_one_matches_plan_move_to` test).

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib plan_move_to_at_speed_ratio -- --nocapture`
Expected: both tests PASS. The second test now checks *exact* equality (`slow.duration_secs == full_speed.duration_secs * 3.0`, within `1e-9`) rather than a loose bound, since the corrected design makes the ratio deterministic.

- [ ] **Step 5: Commit**

```bash
git add src/robot/motion/physics.rs
git commit -m "feat(motion): add speed-ratio variant of plan_move_to"
```

---

### Task 2: Add `plan_return_to_center_at_speed_ratio` to `physics.rs`

**Files:**
- Modify: `src/robot/motion/physics.rs:518-531` (the `plan_return_to_center_at` function)
- Test: same file, `#[cfg(test)] mod tests` block

**Interfaces:**
- Consumes: `plan_move_to_at_speed_ratio` from Task 1.
- Produces: `pub fn plan_return_to_center_at_speed_ratio(arm: &Arm, start: &robot::Pose, rail_x: f64, speed_ratio: f64) -> Result<Trajectory, DomainError>` — same contract as `plan_return_to_center_at` (clamps `rail_x` to the rail's travel range, targets `arm.default_joints`), but slowed by `speed_ratio` via `plan_move_to_at_speed_ratio`.

- [ ] **Step 1: Write the failing tests**

Add these two tests right after the two added in Task 1:

```rust
    #[test]
    fn return_to_center_at_speed_ratio_one_matches_return_to_center_at() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let rail = arm.rail.expect("rail 있는 로봇");
        let start = robot::Pose::new(rail.default_x(), arm.default_joints.clone());

        let via_plain =
            plan_return_to_center_at(arm, &start, rail.x_min).expect("plan_return_to_center_at");
        let via_ratio = plan_return_to_center_at_speed_ratio(arm, &start, rail.x_min, 1.0)
            .expect("plan_return_to_center_at_speed_ratio ratio=1.0");

        assert_eq!(via_plain.duration_secs, via_ratio.duration_secs);
    }

    #[test]
    fn return_to_center_at_speed_ratio_still_targets_given_rail_x() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let rail = arm.rail.expect("rail 있는 로봇");
        let start = robot::Pose::new(rail.default_x(), arm.default_joints.clone());

        let moved = plan_return_to_center_at_speed_ratio(arm, &start, rail.x_min, 1.0 / 3.0)
            .expect("느린 복귀도 x_min에 도달");

        assert!((moved.follow_through_rail_x - rail.x_min).abs() < 1e-9);
        assert_eq!(moved.follow_through, arm.default_joints);
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib return_to_center_at_speed_ratio -- --nocapture`
Expected: FAIL to compile — `plan_return_to_center_at_speed_ratio` does not exist yet.

- [ ] **Step 3: Implement `plan_return_to_center_at_speed_ratio`**

Replace the current `plan_return_to_center_at` function (`src/robot/motion/physics.rs:518-531`) with:

```rust
/// [`plan_return_to_center`]과 같은 중립 자세를, 목표 레일 x만 호출측이 고른
/// 값으로 계획한다 — 좌/센터/우 존 테스트 컨트롤이 준비 위치를 바꿀 때 쓴다.
pub fn plan_return_to_center_at(
    arm: &Arm,
    start: &robot::Pose,
    rail_x: f64,
) -> Result<Trajectory, DomainError> {
    return plan_return_to_center_at_speed_ratio(arm, start, rail_x, 1.0);
}

/// [`plan_return_to_center_at`]와 같지만 [`plan_move_to_at_speed_ratio`]로 계획해
/// `speed_ratio`만큼 늦춘다 — 시작 자세 초기화·수동 홈 포지션 복귀가 쓴다.
pub fn plan_return_to_center_at_speed_ratio(
    arm: &Arm,
    start: &robot::Pose,
    rail_x: f64,
    speed_ratio: f64,
) -> Result<Trajectory, DomainError> {
    let center_joints = arm.default_joints.clone();
    let center_rail_x = arm
        .rail
        .as_ref()
        .map_or(start.rail_x, |rail| rail.clamp_x(rail_x));
    return plan_move_to_at_speed_ratio(arm, start, center_joints, center_rail_x, speed_ratio);
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib return_to_center_at_speed_ratio -- --nocapture`
Expected: both tests PASS. Also re-run Task 1's tests and the pre-existing `return_to_center_at_targets_the_given_rail_x` test to make sure nothing broke:

Run: `cargo test --lib plan_move_to -- --nocapture` and `cargo test --lib return_to_center_at -- --nocapture`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/robot/motion/physics.rs
git commit -m "feat(motion): add speed-ratio variant of plan_return_to_center_at"
```

---

### Task 3: Wire the slow ratio into home-position return (`Planner` + `control_worker.rs` + constant)

**Files:**
- Modify: `src/defaults/motion.rs` (add constant, near line 26)
- Modify: `src/defaults/mod.rs:64-71` (re-export the constant)
- Modify: `src/robot/motion/planner.rs:88-96` and `:145-153` (add two delegate methods)
- Modify: `src/real/control_worker.rs:1340-1412` (`plan_neutral_return_segments` — use the slow variants)
- Test: `src/real/control_worker.rs`, `#[cfg(test)] mod tests` block (existing tests around line 1571-1603, plus one new test)

**Interfaces:**
- Consumes: `plan_move_to_at_speed_ratio` and `plan_return_to_center_at_speed_ratio` from Tasks 1-2.
- Produces: `pingpong_bot::defaults::HOME_RETURN_SPEED_RATIO: f64` (= `1.0 / 3.0`); `Planner::move_to_at_speed_ratio(arm, start, end_joints, end_rail_x, speed_ratio)`; `Planner::return_to_center_at_speed_ratio(arm, start, rail_x, speed_ratio)`. After this task, every trajectory `plan_neutral_return_segments` returns is planned at `HOME_RETURN_SPEED_RATIO`, so both `move_to_ready` (mode 1/2/3 switching) and `initialize_pose_attempt` (startup centering) — the only two callers of `plan_neutral_return_segments` outside tests — automatically inherit the slower speed.

- [ ] **Step 1: Write the failing test**

Add this test in `src/real/control_worker.rs`'s `#[cfg(test)] mod tests` block, right after `extreme_logged_alignment_pose_has_a_safe_ready_return` (around line 1603):

```rust
    #[test]
    fn plan_neutral_return_segments_is_slower_than_full_speed_return() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let start = Pose::new(rail.x_max, robot.arm.default_joints.clone());

        let home_segments = plan_neutral_return_segments(&robot.arm, &start, rail.x_min)
            .expect("홈 포지션 복귀 계획");
        let full_speed = Planner::return_to_center_at(&robot.arm, &start, rail.x_min)
            .expect("전속 복귀 계획(비교 기준)");

        let home_duration: f64 = home_segments.iter().map(|segment| segment.duration_secs).sum();
        assert!(
            home_duration > full_speed.duration_secs * 2.0,
            "home_duration={home_duration} full_speed={}",
            full_speed.duration_secs
        );
    }
```

This test picks `start`/target rail positions far apart (`rail.x_max` to `rail.x_min`) so the direct return succeeds in one segment (matching the `Ok(vec![direct])` early-return path in `plan_neutral_return_segments`), making the duration comparison clean.

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --lib plan_neutral_return_segments_is_slower_than_full_speed_return -- --nocapture`
Expected: FAIL — the assertion fails because today `plan_neutral_return_segments` calls the full-speed `Planner::return_to_center_at`, so `home_duration` roughly equals `full_speed.duration_secs` (not more than double it).

- [ ] **Step 3: Add the constant**

In `src/defaults/motion.rs`, add this right after the `RETURN_TO_CENTER_GROWTH` constant (after line 26):

```rust
/// 모드 1/2/3 홈 포지션 변경·시작 시 센터(ready) 복귀는 랠리처럼 빠를 필요가
/// 없다 — [`crate::robot::motion::plan_move_to_at_speed_ratio`]로 관절·레일
/// 속도를 이 비율만큼 늦춘다.
pub const HOME_RETURN_SPEED_RATIO: f64 = 1.0 / 3.0;
```

In `src/defaults/mod.rs`, update the `pub use motion::{...}` block (lines 64-71) to include it, keeping the existing alphabetical ordering:

```rust
pub use motion::{
    ALIGNMENT_CONTACT_BELOW_RACKET_CENTER_M, ALIGNMENT_LAUNCHER_RIGHT_OFFSET_M,
    COARSE_TRACK_JOINT_FRACTION, HOME_RETURN_SPEED_RATIO, JACOBIAN_DAMPING, JDOT_STEP,
    MAGNUS_OMEGA_MAX, MAX_INTERCEPT_SAMPLES, MAX_PLAN_TIME_SECS, MIN_TIME_TO_GO_SECS,
    PLAN_DT_SECS, POSITION_TOLERANCE_RAD_OR_M, POST_ALIGNMENT_HOLD_SECS,
    RACKET_DIRECTION_TOLERANCE_DEG, RACKET_SPEED_RATIO_TOLERANCE, RAIL_ACCEL_M_S2,
    RETURN_TO_CENTER_GROWTH, RETURN_TO_CENTER_MAX_SECS, RETURN_TO_CENTER_MIN_SECS,
    TIME_TO_GO_BIAS,
};
```

- [ ] **Step 4: Add the `Planner` delegate methods**

In `src/robot/motion/planner.rs`, right after the existing `return_to_center_at` method (after line 96), add:

```rust
    /// [`Self::return_to_center_at`]와 같지만 `speed_ratio`만큼 늦춘 궤적을 계획한다 —
    /// 홈 포지션 복귀·시작 자세 초기화처럼 랠리보다 느려도 되는 이동에 쓴다.
    pub fn return_to_center_at_speed_ratio(
        arm: &Arm,
        start: &robot::Pose,
        rail_x: f64,
        speed_ratio: f64,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_return_to_center_at_speed_ratio(arm, start, rail_x, speed_ratio);
    }
```

Right after the existing `move_to` method (after line 153), add:

```rust
    /// [`Self::move_to`]와 같지만 `speed_ratio`만큼 늦춘 궤적을 계획한다.
    pub fn move_to_at_speed_ratio(
        arm: &Arm,
        start: &robot::Pose,
        end_joints: crate::robot::Joints,
        end_rail_x: f64,
        speed_ratio: f64,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_move_to_at_speed_ratio(arm, start, end_joints, end_rail_x, speed_ratio);
    }
```

- [ ] **Step 5: Wire `plan_neutral_return_segments` to use the slow variants**

In `src/real/control_worker.rs`, inside `plan_neutral_return_segments` (lines 1340-1412), make these three replacements:

Replace (line 1346):
```rust
    let direct_error = match Planner::return_to_center_at(arm, &planning_start, rail_x) {
```
with:
```rust
    let direct_error = match Planner::return_to_center_at_speed_ratio(
        arm,
        &planning_start,
        rail_x,
        pingpong_bot::defaults::HOME_RETURN_SPEED_RATIO,
    ) {
```

Replace (lines 1388-1389):
```rust
        let lift =
            match Planner::move_to(arm, &planning_start, lifted_joints, planning_start.rail_x) {
```
with:
```rust
        let lift = match Planner::move_to_at_speed_ratio(
            arm,
            &planning_start,
            lifted_joints,
            planning_start.rail_x,
            pingpong_bot::defaults::HOME_RETURN_SPEED_RATIO,
        ) {
```

Replace (line 1398):
```rust
        match Planner::return_to_center_at(arm, &lifted_pose, rail_x) {
```
with:
```rust
        match Planner::return_to_center_at_speed_ratio(
            arm,
            &lifted_pose,
            rail_x,
            pingpong_bot::defaults::HOME_RETURN_SPEED_RATIO,
        ) {
```

- [ ] **Step 6: Run the new test and the pre-existing regression tests**

Run: `cargo test --lib plan_neutral_return_segments -- --nocapture`
Expected: all three PASS — the new `plan_neutral_return_segments_is_slower_than_full_speed_return`, plus the pre-existing `logged_follow_through_pose_has_a_safe_ready_return` and `extreme_logged_alignment_pose_has_a_safe_ready_return` (these only assert feasibility/segment count/final pose, not duration, so they should be unaffected by the slower planning — but must be re-checked since the trajectories they get back now take a different code path speed-wise).

- [ ] **Step 7: Commit**

```bash
git add src/defaults/motion.rs src/defaults/mod.rs src/robot/motion/planner.rs src/real/control_worker.rs
git commit -m "feat(real): slow down home-position return moves to 1/3 speed"
```

---

### Task 4: Full-suite verification

**Files:** none (verification only)

- [ ] **Step 1: Run the full test suite**

Run: `cargo test`
Expected: all tests PASS (no pre-existing test depended on `plan_move_to`, `plan_return_to_center_at`, `Planner::move_to`, or `Planner::return_to_center_at` producing a specific duration value at rally speed — those signatures/behavior are untouched — so this should be a clean pass).

- [ ] **Step 2: Run a full build to catch anything the targeted `--lib` test runs skipped**

Run: `cargo build`
Expected: builds cleanly with no new warnings from the changed files (`src/robot/motion/physics.rs`, `src/robot/motion/planner.rs`, `src/defaults/motion.rs`, `src/defaults/mod.rs`, `src/real/control_worker.rs`).

- [ ] **Step 3: Manually sanity-check the ratio math (no code change — just confirm before closing out)**

Confirm: `HOME_RETURN_SPEED_RATIO = 1.0 / 3.0` means the home-return trajectory's planned duration is roughly 3x what the same move would take at rally speed, which (since the AXL rail's commanded velocity in `src/hardware/rail/axl_rail.rs::command_abs_in_secs` is derived as `distance / duration_secs`) makes the actual commanded rail velocity roughly 1/3 of what it would be for the same move during rally. If the resulting real-hardware speed still feels too fast or too slow, the only thing that needs to change is the single `HOME_RETURN_SPEED_RATIO` constant in `src/defaults/motion.rs` — no other file needs touching.
