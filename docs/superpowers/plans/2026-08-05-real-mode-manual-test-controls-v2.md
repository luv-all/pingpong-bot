# Real-mode Manual Test Controls Implementation Plan (v2 — rebased onto the alignment-only control model)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an operator running `--mode real --preview` control the shooter-fed, no-rally test protocol (10 balls each at left/center/right) with keyboard buttons: force an immediate return to ready pose (`r`), clear per-ball state and return to the current ready pose once idle (`w`), and pick which zone's rail-x the ready pose targets (`1`/`2`/`3`).

**Why v2 exists:** This feature was already fully implemented and reviewed once, on `codex/wrist-linear-control-base` at commit `6288585`. While that work was in flight, the base branch rewrote `control_worker.rs`'s entire per-ball control model — from a swing-and-hit sequence (`Planner::aligned_impact_sequence`, `PredictionStage::{Provisional,Refined}`, `BallControlState::Struck`) to a stop-and-align model (`Planner::ball_alignment`, no stage concept, `BallControlState::Aligning`) — in the exact function this feature modifies. That is not a mechanical merge conflict; see the design doc's "v2 리베이스" section for the full mapping. This plan re-implements the same operator-facing feature against the new base (current tip as of this plan: `ee37930`).

**Architecture:** A new `TestControl` enum (`ResetPosition | Wait | SetZone(TestZone)`) travels over a dedicated channel from the existing `--preview` highgui window (which already surfaces unmapped keys as `PreviewAction::Key(i32)`, currently discarded) into `control_worker`. `control_worker` gains a `home_rail_x: f64` local that a new `Planner::return_to_center_at` (a rail-x-targeted sibling of the existing `Planner::return_to_center`, which the current alignment-only model already uses for its automatic post-align return) targets instead of the hardcoded `arm.rail.default_x()`. `ResetPosition` applies immediately (cancelling busy hardware first); `Wait`/`SetZone` are queued and applied the next time the loop is idle, reusing the same idle gate the automatic post-align return already uses. Plan failures during a manual control are non-fatal (matches the base branch's already-established convention for its automatic return path); hardware failures are fatal. A new `RuntimeEvent::TestZoneChanged` reports the zone actually applied back to the preview window's status panel.

**Tech Stack:** Rust, crossbeam-channel (existing dependency, one new `unbounded()` channel), OpenCV (`opencv` crate — existing `camera::Preview` facade, no new dependency).

**Spec:** [`docs/superpowers/specs/2026-08-05-real-mode-manual-test-controls-design.md`](../specs/2026-08-05-real-mode-manual-test-controls-design.md) — read its "v2 리베이스" section before starting; it documents exactly what changed underneath this feature and why each task below differs from a hypothetical straight port of the original.

## Global Constraints

- Scope is `src/real/*` (`test_control.rs` new; `control_worker.rs`, `run.rs`, `preview.rs`, `runtime_event.rs`, `mod.rs` modified) plus `src/robot/motion/physics.rs` and `src/robot/motion/planner.rs`. `estimator_worker.rs`, `camera_worker.rs`, and `CommitRequest`'s shape are untouched. Do **not** touch `Planner::aligned_impact_sequence`, `Planner::ready_prewind`, or `PredictionStage` — they still exist in the codebase (other callers/tests use them) but are irrelevant to this feature; do not "clean them up" as part of this work.
- Keybindings: `1`=`SetZone(Left)`, `2`=`SetZone(Center)`, `3`=`SetZone(Right)`, `w`/`W`=`Wait`, `r`/`R`=`ResetPosition`. `q`/`Q`/ESC stay `Quit` (handled upstream in `camera::io::preview::show_bgr`, unchanged). Unmapped keys are ignored.
- Zone → rail-x mapping: `Left = rail.x_min + margin`, `Center = rail.default_x()` (today's fixed behavior), `Right = rail.x_max - margin`, where `margin = (rail.x_max - rail.x_min) * RAIL_ZONE_SAFETY_MARGIN_RATIO`. `RAIL_ZONE_SAFETY_MARGIN_RATIO = 0.05` (5%) lives in `src/defaults/hardware.rs`, re-exported through `src/defaults/mod.rs` — this is the one place to tune it later. The `Left = x_min` / `Right = x_max` direction is an unverified assumption pending real-hardware confirmation — flag it in a doc comment.
- `ResetPosition` applies immediately: if `hardware.is_busy()`, call `hardware.cancel()` and wait for it to settle (`while hardware.is_busy() && !shutdown.is_down() { thread::sleep(BUSY_POLL); }`) before commanding the ready-pose move. `Wait`/`SetZone` only apply once `pending_verification.is_none() && !hardware.is_busy()` — same gate the existing automatic post-align return already used. If multiple `Wait`/`SetZone` presses queue up before the loop goes idle, only the latest is applied; a `ResetPosition` arriving in the same drain batch wins outright and clears anything queued.
- Applying any of the three test controls resets `CommandLatch` to `CommandLatch::default()` and `BallControlState` to `Idle`, regardless of prior state — this is a manual override, not a per-ball transition, and it intentionally bypasses the existing "one alignment per ball" latch permanence for the ball currently in flight.
- **Error-handling convention (new in v2 — matches what the base branch already does for its own automatic return path):** when `move_to_ready` (the renamed, rail-x-parameterized `move_to_center`) fails with `MoveError::Hardware`, treat it as fatal — send `RuntimeEvent::Failed` and have the caller `break` the control loop. When it fails with `MoveError::Plan`, treat it as non-fatal — send `RuntimeEvent::Failed`, reset `state` to `BallControlState::Idle` and emit `ControlState{Idle}`, then keep running (the caller does not `break`). This mirrors `control_worker.rs`'s existing `due_for_return` branch (see the base file around the `if let Err(error) = move_to_center(...)` block) — do not invent a different policy for the manual-control paths.
- `RuntimeEvent` and the two `match` blocks over it in `run.rs` (`main_loop`'s event loop, and `log_event`) are exhaustive with no wildcard arm — adding `RuntimeEvent::TestZoneChanged` in Task 3 requires a (stub, `{ .. } => {}`) arm in both immediately or the binary stops compiling. Task 4 replaces both stubs with real behavior. Do not add a wildcard `_ =>` arm as a shortcut.
- No new external dependencies.
- `src/real/*` is part of the `main.rs` **binary** target (`mod real;` in `src/main.rs`, not `src/lib.rs`) — its tests only run with `cargo test --bin pingpong-bot <filter>` (default features already include `gui`/`real`, no extra flags needed). `src/robot/motion/*` is part of the **library** crate — its tests run with `cargo test --lib <filter>`. Run the exact command shown per step, not just at the end.
- This codebase writes explicit `return` statements at the end of every function (not bare tail expressions) — match that style in all new code.

---

### Task 1: `TestZone` / `TestControl` types + rail-zone safety margin constant

**Files:**
- Create: `src/real/test_control.rs`
- Modify: `src/real/mod.rs` (register the module, re-export the two types)
- Modify: `src/defaults/hardware.rs` (add `RAIL_ZONE_SAFETY_MARGIN_RATIO`)
- Modify: `src/defaults/mod.rs` (re-export it — `hardware` is currently a private `mod hardware;` with **no** existing `pub use hardware::...` line at all, so nothing in it is reachable outside the lib crate today; this task adds the first one)
- Test: `src/real/test_control.rs` (`#[cfg(test)] mod tests`)

**Interfaces:**
- Consumes: `pingpong_bot::robot::LinearRail` (existing — fields `x_min: f64`, `x_max: f64`, method `default_x(self) -> f64`).
- Produces: `enum TestZone { Left, Center, Right }` (`Debug, Clone, Copy, PartialEq, Eq`) with `fn rail_x(self, rail: LinearRail) -> f64` and `fn label(self) -> &'static str`; `enum TestControl { ResetPosition, Wait, SetZone(TestZone) }` (`Debug, Clone, Copy, PartialEq, Eq`) with `fn from_key(key: i32) -> Option<Self>`; `pub const RAIL_ZONE_SAFETY_MARGIN_RATIO: f64` reachable as `pingpong_bot::defaults::RAIL_ZONE_SAFETY_MARGIN_RATIO`. Tasks 3 and 4 consume `TestZone`/`TestControl`.

- [ ] **Step 1: Add the safety-margin constant and re-export it**

In `src/defaults/hardware.rs`, right after the existing `RAIL_READY_X_M` constant:

```rust
/// 실기에서 눈으로 맞춘 레일 중앙 준비 위치 [m].
pub const RAIL_READY_X_M: f64 = 0.71;
```

add:

```rust

/// 좌/우 존 준비 위치가 레일 양 끝에서 안쪽으로 확보하는 안전 여유 — 레일
/// 전체 구간(`x_max - x_min`) 대비 비율. 초기값 5% — 실기 벤치에서 하드
/// 스탑까지의 여유를 눈으로 확인한 뒤 조정한다.
pub const RAIL_ZONE_SAFETY_MARGIN_RATIO: f64 = 0.05;
```

In `src/defaults/mod.rs`, the `hardware` module is declared as a private `mod hardware;` (alongside `pub mod dxl_limits;` and `mod impact;`) and currently has no `pub use hardware::...` line — nothing in it is reachable from outside the lib crate today. Add one, placed between the existing `pub use dxl_limits::{ ... };` block and `pub use impact::ImpactParams;` (matching the `mod` declaration order: `dxl_limits`, then `hardware`, then `impact`):

```rust
pub use hardware::RAIL_ZONE_SAFETY_MARGIN_RATIO;
```

- [ ] **Step 2: Register the module and write the failing tests**

Add to `src/real/mod.rs`, in the `mod` block (alphabetically between `mod sim_update;` and `mod throttle;` — check current alphabetical order in the file and place it there):

```rust
mod test_control;
```

And to the `pub use` block, alphabetically (between `pub use sim_update::{PoseMsg, SimUpdate};` and `pub use throttle::Throttle;`):

```rust
pub use test_control::{TestControl, TestZone};
```

Create `src/real/test_control.rs` with only the test module first (the types it references don't exist yet — this is the point):

```rust
//! 실기 수동 테스트 컨트롤 — reset / wait / zone 버튼이 이 타입으로 들어온다.

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::robot::LinearRail;

    fn test_rail() -> LinearRail {
        return LinearRail {
            mount_y: 1.0,
            mount_z: 0.2,
            x_min: 0.0,
            x_max: 1.41,
            default_x: 0.71,
            max_speed: 1.0,
        };
    }

    #[test]
    fn zone_rail_x_insets_left_and_right_by_the_safety_margin() {
        let rail = test_rail();
        let margin = (rail.x_max - rail.x_min) * pingpong_bot::defaults::RAIL_ZONE_SAFETY_MARGIN_RATIO;
        assert_eq!(TestZone::Left.rail_x(rail), rail.x_min + margin);
        assert_eq!(TestZone::Center.rail_x(rail), rail.default_x());
        assert_eq!(TestZone::Right.rail_x(rail), rail.x_max - margin);
    }

    #[test]
    fn zone_label_is_upper_case_name() {
        assert_eq!(TestZone::Left.label(), "LEFT");
        assert_eq!(TestZone::Center.label(), "CENTER");
        assert_eq!(TestZone::Right.label(), "RIGHT");
    }

    #[test]
    fn digit_keys_map_to_set_zone() {
        assert_eq!(
            TestControl::from_key(i32::from(b'1')),
            Some(TestControl::SetZone(TestZone::Left))
        );
        assert_eq!(
            TestControl::from_key(i32::from(b'2')),
            Some(TestControl::SetZone(TestZone::Center))
        );
        assert_eq!(
            TestControl::from_key(i32::from(b'3')),
            Some(TestControl::SetZone(TestZone::Right))
        );
    }

    #[test]
    fn w_and_r_map_to_wait_and_reset_case_insensitively() {
        assert_eq!(TestControl::from_key(i32::from(b'w')), Some(TestControl::Wait));
        assert_eq!(TestControl::from_key(i32::from(b'W')), Some(TestControl::Wait));
        assert_eq!(
            TestControl::from_key(i32::from(b'r')),
            Some(TestControl::ResetPosition)
        );
        assert_eq!(
            TestControl::from_key(i32::from(b'R')),
            Some(TestControl::ResetPosition)
        );
    }

    #[test]
    fn unmapped_keys_are_ignored() {
        assert_eq!(TestControl::from_key(i32::from(b'x')), None);
        assert_eq!(TestControl::from_key(27), None);
        assert_eq!(TestControl::from_key(-1), None);
    }
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cargo test --bin pingpong-bot real::test_control::tests::zone_rail_x_insets_left_and_right_by_the_safety_margin`

Expected: FAIL to compile — `cannot find type `TestZone` in this scope` (and similarly for `TestControl`).

- [ ] **Step 4: Implement the types**

Replace the top of `src/real/test_control.rs` (everything above `#[cfg(test)]`) with:

```rust
//! 실기 수동 테스트 컨트롤 — reset / wait / zone 버튼이 이 타입으로 들어온다.
//!
//! 슈터(발사기)가 좌/센터/우로 공을 쏘는 무랠리 테스트 프로토콜에서, 운영자가
//! `--preview` 창의 키 입력으로 로봇의 준비 자세와 내부 상태를 직접 통제한다.

use pingpong_bot::defaults::RAIL_ZONE_SAFETY_MARGIN_RATIO;
use pingpong_bot::robot::LinearRail;

/// 슈터가 겨누는 존 — 이 존의 레일 x가 다음 준비 자세 목표가 된다.
///
/// `Left = x_min`, `Right = x_max`는 미검증 가정이다 — 실기에서 방향이
/// 반대로 확인되면 이 매핑만 뒤집으면 된다. 좌/우는 레일 양 끝단에서
/// [`RAIL_ZONE_SAFETY_MARGIN_RATIO`]만큼 안쪽으로 물러난 위치를 목표로 한다 —
/// 준비 자세가 기계적 하드 스탑에 바짝 붙지 않게 하는 안전 여유다.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestZone {
    Left,
    Center,
    Right,
}

impl TestZone {
    /// 이 존의 준비 자세 레일 x [m] — 좌/우는 안전 여유만큼 안쪽으로 물러난다.
    pub fn rail_x(self, rail: LinearRail) -> f64 {
        let margin = (rail.x_max - rail.x_min) * RAIL_ZONE_SAFETY_MARGIN_RATIO;
        return match self {
            Self::Left => rail.x_min + margin,
            Self::Center => rail.default_x(),
            Self::Right => rail.x_max - margin,
        };
    }

    /// 프리뷰 패널 표시용 라벨.
    pub fn label(self) -> &'static str {
        return match self {
            Self::Left => "LEFT",
            Self::Center => "CENTER",
            Self::Right => "RIGHT",
        };
    }
}

/// `--preview` 창 키 입력이 `control_worker`로 보내는 수동 컨트롤.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestControl {
    /// 즉시 적용 — 하드웨어가 움직이는 중이어도 멈추고 준비 자세로 복귀한다.
    ResetPosition,
    /// 다음 idle 시점에 적용 — 존 변경 없이 latch·상태만 정리하고 현재
    /// home 레일 x로 복귀한다.
    Wait,
    /// 다음 idle 시점에 적용 — home 레일 x를 이 존으로 바꾸고 `Wait`과
    /// 동일하게 정리한다.
    SetZone(TestZone),
}

impl TestControl {
    /// highgui 키코드 → 수동 컨트롤. 매핑에 없는 키는 `None`(무시).
    ///
    /// `1`/`2`/`3` = 좌/센터/우 존, `w` = wait(존 유지), `r` = 즉시 리셋.
    /// `q`/ESC는 프리뷰 창 자체가 Quit으로 소비하므로 여기 없다.
    pub fn from_key(key: i32) -> Option<Self> {
        return match key {
            k if k == i32::from(b'1') => Some(Self::SetZone(TestZone::Left)),
            k if k == i32::from(b'2') => Some(Self::SetZone(TestZone::Center)),
            k if k == i32::from(b'3') => Some(Self::SetZone(TestZone::Right)),
            k if k == i32::from(b'w') || k == i32::from(b'W') => Some(Self::Wait),
            k if k == i32::from(b'r') || k == i32::from(b'R') => Some(Self::ResetPosition),
            _ => None,
        };
    }
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cargo test --bin pingpong-bot real::test_control::tests`

Expected: PASS (5/5).

- [ ] **Step 6: Regression-check the `defaults` crate**

Run: `cargo test --lib defaults::`

Expected: PASS — confirms the new `pub use` line didn't break `defaults`'s own existing tests.

- [ ] **Step 7: Commit**

```bash
git add src/real/test_control.rs src/real/mod.rs src/defaults/hardware.rs src/defaults/mod.rs
git commit -m "feat(real): add TestZone/TestControl types and rail-zone safety margin"
```

---

### Task 2: `Planner::return_to_center_at`

**Files:**
- Modify: `src/robot/motion/physics.rs` (extract a hint-based helper out of `plan_return_to_center`, add `plan_return_to_center_at`)
- Modify: `src/robot/motion/planner.rs` (add `Planner::return_to_center_at`, right after the existing `return_to_center`)
- Test: `src/robot/motion/physics.rs` (`#[cfg(test)] mod tests`)

**Interfaces:**
- Consumes: existing `plan_move_to`, `Arm`, `robot::Pose`, `DomainError`, `Trajectory` (all already imported in `physics.rs`).
- Produces: `pub fn plan_return_to_center_at(arm: &Arm, start: &robot::Pose, rail_x: f64) -> Result<Trajectory, DomainError>` in `physics.rs`; `pub fn return_to_center_at(arm: &Arm, start: &robot::Pose, rail_x: f64) -> Result<Trajectory, DomainError>` in `Planner` (`planner.rs`). Task 3 consumes `Planner::return_to_center_at` (indirectly, via a generalized `plan_neutral_return_segments`).

- [ ] **Step 1: Write the failing test**

First locate the existing `plan_return_to_center` and its test in `src/robot/motion/physics.rs` — search for `fn plan_return_to_center` (around the `///` block that begins "레일의 `home_x`(원점, x=0)는..."). Add a new test to the `#[cfg(test)] mod tests` block in the same file (place it near any existing test that exercises `plan_return_to_center`, if one exists — otherwise anywhere in the test module):

```rust
    #[test]
    fn return_to_center_at_targets_the_given_rail_x() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let rail = arm.rail.expect("rail 있는 로봇");
        let start = robot::Pose::new(rail.default_x(), arm.default_joints.clone());

        let moved =
            plan_return_to_center_at(arm, &start, rail.x_min).expect("return to center at x_min");

        assert!((moved.follow_through_rail_x - rail.x_min).abs() < 1e-9);
        assert_eq!(moved.follow_through, arm.default_joints);
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --lib robot::motion::physics::tests::return_to_center_at_targets_the_given_rail_x`

Expected: FAIL to compile — `cannot find function `plan_return_to_center_at` in this scope`.

- [ ] **Step 3: Implement**

In `src/robot/motion/physics.rs`, find the existing function:

```rust
pub fn plan_return_to_center(arm: &Arm, start: &robot::Pose) -> Result<Trajectory, DomainError> {
    let center_joints = arm.default_joints.clone();
    let center_rail_x = arm
        .rail
        .as_ref()
        .map(|rail| rail.default_x())
        .unwrap_or(start.rail_x);
    return plan_move_to(arm, start, center_joints, center_rail_x);
}
```

Replace it with:

```rust
pub fn plan_return_to_center(arm: &Arm, start: &robot::Pose) -> Result<Trajectory, DomainError> {
    let center_rail_x = arm
        .rail
        .as_ref()
        .map(|rail| rail.default_x())
        .unwrap_or(start.rail_x);
    return plan_return_to_center_at(arm, start, center_rail_x);
}

/// [`plan_return_to_center`]과 같은 중립 자세를, 목표 레일 x만 호출측이 고른
/// 값으로 계획한다 — 좌/센터/우 존 테스트 컨트롤이 준비 위치를 바꿀 때 쓴다.
pub fn plan_return_to_center_at(
    arm: &Arm,
    start: &robot::Pose,
    rail_x: f64,
) -> Result<Trajectory, DomainError> {
    let center_joints = arm.default_joints.clone();
    let center_rail_x = arm
        .rail
        .as_ref()
        .map_or(start.rail_x, |rail| rail.clamp_x(rail_x));
    return plan_move_to(arm, start, center_joints, center_rail_x);
}
```

In `src/robot/motion/planner.rs`, find the existing method:

```rust
    pub fn return_to_center(arm: &Arm, start: &robot::Pose) -> Result<Trajectory, DomainError> {
        return physics::plan_return_to_center(arm, start);
    }
```

Right after it, add:

```rust

    /// [`Self::return_to_center`]과 같지만 목표 레일 x를 호출측이 고른다 —
    /// 좌/센터/우 존 테스트 컨트롤이 쓴다.
    pub fn return_to_center_at(
        arm: &Arm,
        start: &robot::Pose,
        rail_x: f64,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_return_to_center_at(arm, start, rail_x);
    }
```

- [ ] **Step 4: Run tests to verify they pass, and that existing behavior is unchanged**

Run: `cargo test --lib robot::motion::physics::tests::return_to_center_at_targets_the_given_rail_x`

Expected: PASS.

Run: `cargo test --lib robot::motion::physics::tests`

Expected: PASS (full module) — confirms the `plan_return_to_center` extraction didn't change behavior for any pre-existing test that exercises it (e.g. `startup_initialization_sets_ready_rail_and_all_joints` in `control_worker.rs` also depends on `Planner::return_to_center` transitively via `plan_neutral_return_segments`, but that's a binary-target test — covered in Task 3's regression sweep, not here).

- [ ] **Step 5: Commit**

```bash
git add src/robot/motion/physics.rs src/robot/motion/planner.rs
git commit -m "feat(robot): add Planner::return_to_center_at for caller-chosen center rail x"
```

---

### Task 3: `control_worker` accepts and applies `TestControl`

**Files:**
- Modify: `src/real/runtime_event.rs` (add `RuntimeEvent::TestZoneChanged`)
- Modify: `src/real/control_worker.rs` (`spawn` signature and body, generalize `plan_neutral_return_segments`, rename `move_to_center` → `move_to_ready`, new `apply_test_control`)
- Modify: `src/real/run.rs` (minimal: wire the new channel into `control_worker::spawn`'s call site, add stub match arms so the exhaustive `RuntimeEvent` matches keep compiling — Task 4 fills in the real behavior)
- Test: `src/real/control_worker.rs` (`#[cfg(test)] mod tests`)

**Interfaces:**
- Consumes: `TestControl`, `TestZone` (Task 1), `Planner::return_to_center_at` (Task 2).
- Produces: `RuntimeEvent::TestZoneChanged { zone: TestZone, home_rail_x: f64 }`; `control_worker::spawn(hardware, arm, rx, test_control_rx: Receiver<TestControl>, sim_tx, event_tx, shutdown)` (new 4th positional parameter, right after `rx`). Task 4 consumes both.

- [ ] **Step 1: Add the `RuntimeEvent` variant**

In `src/real/runtime_event.rs`, add the import:

```rust
use pingpong_bot::Point3;
use pingpong_bot::robot;

use super::TestZone;
```

Then add a variant to `RuntimeEvent`, right after `ControlState`:

```rust
    /// 현재 공 처리 상태가 바뀌었다 — 프리뷰 상태 패널이 소비한다.
    ControlState { state: ControlStateSnapshot },
    /// 준비 자세 레일 x가 존 선택으로 바뀌었거나, 수동 컨트롤로 재적용됐다.
    /// 프리뷰 상태 패널이 소비한다.
    TestZoneChanged { zone: TestZone, home_rail_x: f64 },
    /// 현재 공의 계획 생략 또는 하드웨어 오류. 계획 생략은 다음 공을 계속 처리한다.
    Failed {
        track_seq: Option<u64>,
        reason: String,
    },
```

- [ ] **Step 2: Write the failing tests**

In `src/real/control_worker.rs`, update the `use super::{...}` import to add `TestControl, TestZone`:

```rust
use super::{CommitRequest, ControlStateSnapshot, PoseMsg, RuntimeEvent, Shutdown, SimUpdate};
```

becomes:

```rust
use super::{
    CommitRequest, ControlStateSnapshot, PoseMsg, RuntimeEvent, Shutdown, SimUpdate, TestControl,
    TestZone,
};
```

Add to the `#[cfg(test)] mod tests` block, after the existing `aligning_blocks_only_its_own_track` test:

```rust
    #[test]
    fn apply_test_control_set_zone_moves_home_clears_latch_and_emits_zone_event() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let mut hardware = PoseApplyingHardware {
            pose: Pose::new(rail.default_x(), robot.arm.default_joints.clone()),
        };
        let mut latch = CommandLatch::default();
        latch.should_send(9);
        latch.mark_finished();
        let mut state = BallControlState::Aligning {
            track_seq: 9,
            return_due_at: Instant::now(),
            measurement: PendingAlignmentMeasurement {
                track_seq: 9,
                rail_commanded_m: rail.default_x(),
                joints_commanded: robot.arm.default_joints.clone(),
            },
        };
        let mut home_rail_x = rail.default_x();
        let mut current_zone = TestZone::Center;
        let (event_tx, event_rx) = crossbeam_channel::unbounded();

        apply_test_control(
            TestControl::SetZone(TestZone::Left),
            &mut hardware,
            &robot.arm,
            &mut home_rail_x,
            &mut current_zone,
            &mut latch,
            &mut state,
            None,
            &event_tx,
        )
        .expect("apply set zone");

        assert_eq!(current_zone, TestZone::Left);
        assert!((home_rail_x - TestZone::Left.rail_x(rail)).abs() < 1e-9);
        assert!(matches!(state, BallControlState::Idle));
        assert!(latch.should_send(9));
        assert!((hardware.pose.rail_x - TestZone::Left.rail_x(rail)).abs() < 1e-6);

        let events: Vec<_> = event_rx.try_iter().collect();
        assert!(events.iter().any(|event| matches!(
            event,
            RuntimeEvent::ControlState {
                state: ControlStateSnapshot::Idle
            }
        )));
        assert!(events.iter().any(|event| matches!(
            event,
            RuntimeEvent::TestZoneChanged {
                zone: TestZone::Left,
                ..
            }
        )));
    }

    #[test]
    fn apply_test_control_wait_keeps_current_zone() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let mut hardware = PoseApplyingHardware {
            pose: Pose::new(rail.x_max, robot.arm.default_joints.clone()),
        };
        let mut latch = CommandLatch::default();
        let mut state = BallControlState::Idle;
        let mut home_rail_x = rail.x_max;
        let mut current_zone = TestZone::Right;
        let (event_tx, _event_rx) = crossbeam_channel::unbounded();

        apply_test_control(
            TestControl::Wait,
            &mut hardware,
            &robot.arm,
            &mut home_rail_x,
            &mut current_zone,
            &mut latch,
            &mut state,
            None,
            &event_tx,
        )
        .expect("apply wait");

        assert_eq!(current_zone, TestZone::Right);
        assert!((home_rail_x - rail.x_max).abs() < 1e-9);
    }
```

Note: `latch.should_send(9)` / `latch.mark_finished()` (no `stage` argument) match this base's already-simplified `CommandLatch` — do not add a stage parameter back.

- [ ] **Step 3: Run tests to verify they fail**

Run: `cargo test --bin pingpong-bot real::control_worker::tests::apply_test_control_set_zone_moves_home_clears_latch_and_emits_zone_event`

Expected: FAIL to compile — `cannot find function `apply_test_control` in this scope` (and `PendingAlignmentMeasurement` is private to the module, but the test is inside `mod tests` which has `use super::*;`, so once `apply_test_control` exists this resolves).

- [ ] **Step 4: Generalize `plan_neutral_return_segments` and rename `move_to_center` → `move_to_ready`**

In `src/real/control_worker.rs`, find:

```rust
/// 시작 자세 초기화와 공 제어 후 복귀에 같은 전체축 이동을 사용한다.
fn move_to_center(hardware: &mut dyn Hardware, arm: &Arm) -> Result<(), MoveError> {
    let start = hardware.read_pose().map_err(MoveError::Hardware)?;
    let trajectories = plan_neutral_return_segments(arm, &start).map_err(MoveError::Plan)?;
    if trajectories.len() > 1 {
        info!(
            segments = trajectories.len(),
            "직접 복귀 관통 회피 — 위로 든 뒤 준비 자세 복귀"
        );
    }
    for trajectory in trajectories {
        hardware.command(&trajectory).map_err(MoveError::Hardware)?;
        while hardware.is_busy() {
            thread::sleep(BUSY_POLL);
        }
    }
    return Ok(());
}

/// 직접 복귀가 테이블을 스치면 안전한 상승 중간 자세를 거치는 2구간을 찾는다.
/// 모든 구간은 실행 전에 속도·토크·테이블 충돌 검사를 통과해야 한다.
fn plan_neutral_return_segments(
    arm: &Arm,
    start: &pingpong_bot::robot::Pose,
) -> Result<Vec<pingpong_bot::robot::motion::Trajectory>, DomainError> {
    match Planner::return_to_center(arm, start) {
        Ok(direct) => return Ok(vec![direct]),
        Err(error) => {
            if !matches!(
                error,
                DomainError::InfeasibleSwing(
                    pingpong_bot::error::SwingPlanError::TablePenetration { .. }
                )
            ) {
                return Err(error);
            }
        }
    }

    let racket = arm
        .forward_kinematics_with_rail(start.rail_x, &start.joints)
        .ok_or_else(|| {
            DomainError::InfeasibleSwing(
                pingpong_bot::error::SwingPlanError::InverseKinematicsNoSolution {
                    target_x: start.rail_x,
                    target_y: 0.0,
                    target_z: 0.0,
                },
            )
        })?;
    let mut last_error = None;
    for lift_m in [0.03, 0.06, 0.10, 0.15] {
        let lifted_target = pingpong_bot::Point3::new(
            racket.position.x,
            racket.position.y,
            racket.position.z + lift_m,
        );
        let lifted_joints = match arm.rail.as_ref() {
            Some(rail) => arm.inverse_kinematics_with_rail(
                rail,
                start.rail_x,
                lifted_target,
                Some(&start.joints),
            ),
            None => arm.inverse_kinematics_near(lifted_target, Some(&start.joints)),
        };
        let lifted_joints = match lifted_joints {
            Ok(joints) => joints,
            Err(error) => {
                last_error = Some(DomainError::InfeasibleSwing(error));
                continue;
            }
        };
        let lift = match Planner::move_to(arm, start, lifted_joints, start.rail_x) {
            Ok(trajectory) => trajectory,
            Err(error) => {
                last_error = Some(error);
                continue;
            }
        };
        let lifted_pose =
            pingpong_bot::robot::Pose::new(lift.follow_through_rail_x, lift.follow_through.clone());
        match Planner::return_to_center(arm, &lifted_pose) {
            Ok(ready) => return Ok(vec![lift, ready]),
            Err(error) => last_error = Some(error),
        }
    }
    return Err(last_error.unwrap_or_else(|| {
        DomainError::InfeasibleSwing(
            pingpong_bot::error::SwingPlanError::InverseKinematicsNoSolution {
                target_x: start.rail_x,
                target_y: racket.position.y,
                target_z: racket.position.z,
            },
        )
    }));
}
```

Replace both functions with:

```rust
/// 시작 자세 초기화와 공 제어 후 복귀·수동 테스트 컨트롤이 같은 전체축 이동을 쓴다.
fn move_to_ready(hardware: &mut dyn Hardware, arm: &Arm, rail_x: f64) -> Result<(), MoveError> {
    let start = hardware.read_pose().map_err(MoveError::Hardware)?;
    let trajectories =
        plan_neutral_return_segments(arm, &start, rail_x).map_err(MoveError::Plan)?;
    if trajectories.len() > 1 {
        info!(
            segments = trajectories.len(),
            "직접 복귀 관통 회피 — 위로 든 뒤 준비 자세 복귀"
        );
    }
    for trajectory in trajectories {
        hardware.command(&trajectory).map_err(MoveError::Hardware)?;
        while hardware.is_busy() {
            thread::sleep(BUSY_POLL);
        }
    }
    return Ok(());
}

/// 직접 복귀가 테이블을 스치면 안전한 상승 중간 자세를 거치는 2구간을 찾는다.
/// 모든 구간은 실행 전에 속도·토크·테이블 충돌 검사를 통과해야 한다. 목표
/// 레일 x는 호출측이 고른다 — 시작 자세 초기화는 항상 `rail.default_x()`를,
/// 수동 테스트 컨트롤은 존 선택에 따른 값을 넘긴다.
fn plan_neutral_return_segments(
    arm: &Arm,
    start: &pingpong_bot::robot::Pose,
    rail_x: f64,
) -> Result<Vec<pingpong_bot::robot::motion::Trajectory>, DomainError> {
    match Planner::return_to_center_at(arm, start, rail_x) {
        Ok(direct) => return Ok(vec![direct]),
        Err(error) => {
            if !matches!(
                error,
                DomainError::InfeasibleSwing(
                    pingpong_bot::error::SwingPlanError::TablePenetration { .. }
                )
            ) {
                return Err(error);
            }
        }
    }

    let racket = arm
        .forward_kinematics_with_rail(start.rail_x, &start.joints)
        .ok_or_else(|| {
            DomainError::InfeasibleSwing(
                pingpong_bot::error::SwingPlanError::InverseKinematicsNoSolution {
                    target_x: start.rail_x,
                    target_y: 0.0,
                    target_z: 0.0,
                },
            )
        })?;
    let mut last_error = None;
    for lift_m in [0.03, 0.06, 0.10, 0.15] {
        let lifted_target = pingpong_bot::Point3::new(
            racket.position.x,
            racket.position.y,
            racket.position.z + lift_m,
        );
        let lifted_joints = match arm.rail.as_ref() {
            Some(rail) => arm.inverse_kinematics_with_rail(
                rail,
                start.rail_x,
                lifted_target,
                Some(&start.joints),
            ),
            None => arm.inverse_kinematics_near(lifted_target, Some(&start.joints)),
        };
        let lifted_joints = match lifted_joints {
            Ok(joints) => joints,
            Err(error) => {
                last_error = Some(DomainError::InfeasibleSwing(error));
                continue;
            }
        };
        let lift = match Planner::move_to(arm, start, lifted_joints, start.rail_x) {
            Ok(trajectory) => trajectory,
            Err(error) => {
                last_error = Some(error);
                continue;
            }
        };
        let lifted_pose =
            pingpong_bot::robot::Pose::new(lift.follow_through_rail_x, lift.follow_through.clone());
        match Planner::return_to_center_at(arm, &lifted_pose, rail_x) {
            Ok(ready) => return Ok(vec![lift, ready]),
            Err(error) => last_error = Some(error),
        }
    }
    return Err(last_error.unwrap_or_else(|| {
        DomainError::InfeasibleSwing(
            pingpong_bot::error::SwingPlanError::InverseKinematicsNoSolution {
                target_x: start.rail_x,
                target_y: racket.position.y,
                target_z: racket.position.z,
            },
        )
    }));
}
```

Now find the one remaining caller of the old 2-argument `plan_neutral_return_segments` — inside `initialize_pose_attempt`:

```rust
    let trajectories = plan_neutral_return_segments(arm, &measured).map_err(MoveError::Plan)?;
    let ready_joints = arm.default_joints.clone();
    let ready_rail_x = arm
        .rail
        .as_ref()
        .map_or(measured.rail_x, |rail| rail.default_x());
```

Replace with (only the first line changes — reorder so `ready_rail_x` is computed first, then passed in):

```rust
    let ready_joints = arm.default_joints.clone();
    let ready_rail_x = arm
        .rail
        .as_ref()
        .map_or(measured.rail_x, |rail| rail.default_x());
    let trajectories =
        plan_neutral_return_segments(arm, &measured, ready_rail_x).map_err(MoveError::Plan)?;
```

- [ ] **Step 5: Add `apply_test_control`**

Right after the `move_to_ready` function (which Step 4 just created), add:

```rust

/// 존 변경(있다면) → 준비 자세 이동 → latch·상태 초기화 → 이벤트 발행까지 한 번에 한다.
/// `Wait`/`SetZone`은 idle일 때만 호출부가 부르고, `ResetPosition`은 즉시 부른다.
fn apply_test_control(
    control: TestControl,
    hardware: &mut dyn Hardware,
    arm: &Arm,
    home_rail_x: &mut f64,
    current_zone: &mut TestZone,
    latch: &mut CommandLatch,
    state: &mut BallControlState,
    sim_tx: Option<&Sender<SimUpdate>>,
    event_tx: &Sender<RuntimeEvent>,
) -> Result<(), MoveError> {
    if let TestControl::SetZone(zone) = control
        && let Some(rail) = arm.rail
    {
        *current_zone = zone;
        *home_rail_x = zone.rail_x(rail);
    }
    move_to_ready(hardware, arm, *home_rail_x)?;
    *latch = CommandLatch::default();
    *state = BallControlState::Idle;
    if let Ok(pose) = hardware.read_pose()
        && let Some(sim_tx) = sim_tx
    {
        let _ = sim_tx.try_send(SimUpdate {
            pose: Some(PoseMsg::from(&pose)),
            ..SimUpdate::default()
        });
    }
    info!(
        control = ?control,
        zone = ?current_zone,
        home_rail_x = f4(*home_rail_x),
        "테스트 컨트롤 적용 — 준비 자세 복귀"
    );
    let _ = event_tx.send(RuntimeEvent::ControlState {
        state: ControlStateSnapshot::Idle,
    });
    let _ = event_tx.send(RuntimeEvent::TestZoneChanged {
        zone: *current_zone,
        home_rail_x: *home_rail_x,
    });
    return Ok(());
}
```

- [ ] **Step 6: Wire `spawn` — signature, locals, drain loop, idle-apply, error-handling convention**

In `src/real/control_worker.rs`, update the `spawn` signature:

```rust
pub fn spawn(
    mut hardware: Box<dyn Hardware>,
    arm: Arc<Arm>,
    rx: Receiver<CommitRequest>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<RuntimeEvent>,
    shutdown: Shutdown,
) -> JoinHandle<()> {
```

becomes:

```rust
pub fn spawn(
    mut hardware: Box<dyn Hardware>,
    arm: Arc<Arm>,
    rx: Receiver<CommitRequest>,
    test_control_rx: Receiver<TestControl>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<RuntimeEvent>,
    shutdown: Shutdown,
) -> JoinHandle<()> {
```

Right after `let window = motion::InterceptWindow::default();` and before the `if let Some(sim_tx) = &sim_tx {` block, add:

```rust
        let mut home_rail_x = arm.rail.map(|rail| rail.default_x()).unwrap_or(pose.rail_x);
        let mut current_zone = TestZone::Center;
```

Right after the existing `let _ = event_tx.send(RuntimeEvent::ControlState { state: ControlStateSnapshot::Idle, });` (the one sent at startup, before `info!("공 위치·방향 정렬 준비...")`), add:

```rust
        let _ = event_tx.send(RuntimeEvent::TestZoneChanged {
            zone: current_zone,
            home_rail_x,
        });
```

Change:

```rust
        let mut latch = CommandLatch::default();
        let mut last_command: Option<Instant> = None;
        let mut pending_verification: Option<PendingVerification> = None;
        let mut state = BallControlState::Idle;
        let mut consecutive_misses: u8 = 0;

        while !shutdown.is_down() {
```

to:

```rust
        let mut latch = CommandLatch::default();
        let mut last_command: Option<Instant> = None;
        let mut pending_verification: Option<PendingVerification> = None;
        let mut state = BallControlState::Idle;
        let mut consecutive_misses: u8 = 0;
        let mut pending_test_control: Option<TestControl> = None;

        'control: while !shutdown.is_down() {
            while let Ok(control) = test_control_rx.try_recv() {
                match control {
                    TestControl::ResetPosition => {
                        pending_test_control = None;
                        if hardware.is_busy() {
                            hardware.cancel();
                            while hardware.is_busy() && !shutdown.is_down() {
                                thread::sleep(BUSY_POLL);
                            }
                        }
                        if shutdown.is_down() {
                            break 'control;
                        }
                        pending_verification = None;
                        match apply_test_control(
                            TestControl::ResetPosition,
                            hardware.as_mut(),
                            &arm,
                            &mut home_rail_x,
                            &mut current_zone,
                            &mut latch,
                            &mut state,
                            sim_tx.as_ref(),
                            &event_tx,
                        ) {
                            Ok(()) => {}
                            Err(MoveError::Hardware(error)) => {
                                let _ = event_tx.send(RuntimeEvent::Failed {
                                    track_seq: latch.track_seq,
                                    reason: format!("수동 리셋 중 하드웨어 오류: {error}"),
                                });
                                break 'control;
                            }
                            Err(MoveError::Plan(error)) => {
                                warn!(%error, "수동 리셋 중 준비 자세 계획 실패 — 세션은 유지");
                                let _ = event_tx.send(RuntimeEvent::Failed {
                                    track_seq: latch.track_seq,
                                    reason: format!("수동 리셋 중 준비 자세 계획 실패: {error}"),
                                });
                                state = BallControlState::Idle;
                                let _ = event_tx.send(RuntimeEvent::ControlState {
                                    state: ControlStateSnapshot::Idle,
                                });
                            }
                        }
                    }
                    other => pending_test_control = Some(other),
                }
            }
```

Note the deliberate difference from `MoveError::Hardware`'s handling of the `initialize_pose_attempt`/other paths: there's no `MoveError::StartupAlignmentTimeout` arm needed here because `move_to_ready`/`apply_test_control` never produce that variant (it's only returned by `initialize_pose_attempt`, called once before `spawn` even starts) — but `match` on `MoveError` requires all three variants; add a catch-all only if the compiler demands it (it will, since `MoveError` has 3 variants) — extend the second `Err` arm's pattern instead:

```rust
                            Err(error @ MoveError::Plan(_))
                            | Err(error @ MoveError::StartupAlignmentTimeout { .. }) => {
```

Use this combined arm (treating `StartupAlignmentTimeout` the same as `Plan` — non-fatal, log and continue — since it can never actually occur here but the match must be exhaustive) in place of the plain `Err(MoveError::Plan(error)) =>` arm shown above, and reference `error` via `{error}` in the `format!`/`warn!` calls exactly as before (the binding name is still `error`).

Now find the existing `due_for_return` handling block:

```rust
            let due_for_return = match &state {
                BallControlState::Aligning { return_due_at, .. } => {
                    Instant::now() >= *return_due_at
                }
                BallControlState::Idle => false,
            };
            if pending_verification.is_none() && due_for_return && !hardware.is_busy() {
```

Replace it with (adding the idle-apply branch as a sibling `if`/`else if`, keyed off the same `pending_verification.is_none() && !hardware.is_busy()` gate):

```rust
            let due_for_return = match &state {
                BallControlState::Aligning { return_due_at, .. } => {
                    Instant::now() >= *return_due_at
                }
                BallControlState::Idle => false,
            };
            let idle_ready = pending_verification.is_none() && !hardware.is_busy();
            if idle_ready && let Some(control) = pending_test_control.take() {
                match apply_test_control(
                    control,
                    hardware.as_mut(),
                    &arm,
                    &mut home_rail_x,
                    &mut current_zone,
                    &mut latch,
                    &mut state,
                    sim_tx.as_ref(),
                    &event_tx,
                ) {
                    Ok(()) => {}
                    Err(MoveError::Hardware(error)) => {
                        let _ = event_tx.send(RuntimeEvent::Failed {
                            track_seq: latch.track_seq,
                            reason: format!("테스트 컨트롤 적용 중 하드웨어 오류: {error}"),
                        });
                        break;
                    }
                    Err(error @ MoveError::Plan(_))
                    | Err(error @ MoveError::StartupAlignmentTimeout { .. }) => {
                        warn!(%error, "테스트 컨트롤 적용 중 준비 자세 계획 실패 — 세션은 유지");
                        let _ = event_tx.send(RuntimeEvent::Failed {
                            track_seq: latch.track_seq,
                            reason: format!("테스트 컨트롤 적용 중 준비 자세 계획 실패: {error}"),
                        });
                        state = BallControlState::Idle;
                        let _ = event_tx.send(RuntimeEvent::ControlState {
                            state: ControlStateSnapshot::Idle,
                        });
                    }
                }
            } else if idle_ready && due_for_return {
```

Everything from the original `if let BallControlState::Aligning { measurement, .. } = &state {` line through the matching closing brace of that `if pending_verification.is_none() && due_for_return && !hardware.is_busy() {` block stays **exactly as it is today** — it's now the body of the `else if idle_ready && due_for_return {` branch instead of the original `if`. The only two things inside that body to update:

1. `move_to_center(hardware.as_mut(), &arm)` → `move_to_ready(hardware.as_mut(), &arm, home_rail_x)`.
2. Nothing else — the existing `MoveError::Hardware`-vs-other split inside that block (`let fatal_hardware_error = matches!(error, MoveError::Hardware(_));`) already implements exactly the error-handling convention this task's Global Constraints describe; leave it untouched.

Confirm the final shape reads as: `if idle_ready && let Some(control) = ... { /* apply_test_control handling */ } else if idle_ready && due_for_return { /* existing move_to_center-turned-move_to_ready handling, byte-identical otherwise */ }`.

- [ ] **Step 7: Keep `run.rs` compiling — minimal call-site + stub match arms**

In `src/real/run.rs`, in the `run()` function, add a placeholder channel right after `let (event_tx, event_rx) = unbounded();`:

```rust
    // Task 4가 실제 키 입력으로 채운다 — 지금은 컴파일 유지용.
    let (_test_control_tx, test_control_rx) = unbounded();
```

Update the `control_worker::spawn(...)` call to pass it through:

```rust
    let control_handle = control_worker::spawn(
        Box::new(hardware),
        Arc::clone(&arm),
        commit_rx,
        test_control_rx,
        sim_tx,
        event_tx,
        shutdown,
    );
```

In `main_loop`'s event-handling `match &event { ... }` block, add a stub arm right after the `RuntimeEvent::ControlState { state } => { ... }` arm:

```rust
                RuntimeEvent::TestZoneChanged { .. } => {}
```

In `log_event`'s `match event { ... }`, add a stub arm right after the `RuntimeEvent::ControlState { state } => debug!(?state, "제어 상태 전이"),` arm:

```rust
        RuntimeEvent::TestZoneChanged { .. } => {}
```

- [ ] **Step 8: Run tests to verify they pass**

Run: `cargo test --bin pingpong-bot real::control_worker::tests`

Expected: PASS (all tests in the module, including the two new ones and every pre-existing one — `each_vision_track_is_sent_only_once`, `new_track_resets_latch`, `aligned_track_is_permanently_blocked_even_after_returning_to_idle`, `due_command_needs_two_stable_readbacks`, `idle_blocks_nothing`, `aligning_blocks_only_its_own_track`, `startup_initialization_sets_ready_rail_and_all_joints`, `logged_follow_through_pose_has_a_safe_ready_return`, `delayed_vision_request_is_advanced_instead_of_dropped`, `vision_request_is_rejected_only_after_prediction_has_ended`).

Run: `cargo build --bin pingpong-bot`

Expected: builds cleanly (confirms `run.rs`'s stub wiring compiles).

- [ ] **Step 9: Commit**

```bash
git add src/real/runtime_event.rs src/real/control_worker.rs src/real/run.rs
git commit -m "feat(real): control_worker applies TestControl (reset/wait/zone) on the alignment model"
```

---

### Task 4: Preview window controls + `run.rs` key routing

**Files:**
- Modify: `src/real/preview.rs` (`PreviewWindow::show()` return type, `set_zone`, status panel)
- Modify: `src/real/run.rs` (real channel wiring, key → `TestControl` routing, `TestZoneChanged` display + logging)
- Test: `src/real/preview.rs` (`#[cfg(test)] mod tests`)

**Interfaces:**
- Consumes: `TestZone`, `TestControl::from_key` (Task 1), `RuntimeEvent::TestZoneChanged` and `control_worker::spawn`'s new signature (Task 3), `camera::PreviewAction` (existing).
- Produces: `PreviewWindow::show(&mut self) -> PreviewAction` (was `-> bool`); `PreviewWindow::set_zone(&mut self, zone: TestZone, home_rail_x: f64)`. Nothing downstream of this task.

- [ ] **Step 1: Write the failing test**

In `src/real/preview.rs`, add the import `use super::TestZone;` alongside the existing `use super::ControlStateSnapshot;` / `use super::PreviewEvent;` lines:

```rust
use super::ControlStateSnapshot;
use super::PreviewEvent;
use super::TestZone;
```

Add to the `#[cfg(test)] mod tests` block, after the existing `use` lines and before `idle_pixel`:

```rust
    #[test]
    fn set_zone_stores_the_current_zone_and_home_x() {
        let mut window = PreviewWindow::new("test");
        assert!(window.current_zone.is_none());
        window.set_zone(TestZone::Right, 1.34);
        assert_eq!(window.current_zone, Some((TestZone::Right, 1.34)));
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --bin pingpong-bot real::preview::tests::set_zone_stores_the_current_zone_and_home_x`

Expected: FAIL to compile — `no method named `set_zone` found` / `no field `current_zone` on type `PreviewWindow``.

- [ ] **Step 3: Implement — `PreviewWindow` field, `set_zone`, `show()` return type**

Add a field to the `PreviewWindow` struct, right after `control_state: Option<ControlStateSnapshot>,`:

```rust
    /// 최근 제어 상태 — 다음 상태가 올 때까지 남는다.
    control_state: Option<ControlStateSnapshot>,
    /// 현재 테스트 존과 그 준비 레일 x — 상태 패널이 소비한다.
    current_zone: Option<(TestZone, f64)>,
}
```

Update `PreviewWindow::new` to initialize it:

```rust
    pub fn new(window: impl Into<String>) -> Self {
        return Self {
            window: window.into(),
            panels: BTreeMap::new(),
            hud: Vec::new(),
            sticky: Vec::new(),
            last_target: BTreeMap::new(),
            control_state: None,
            current_zone: None,
        };
    }
```

Add, right after `set_control_state`:

```rust
    /// 현재 테스트 존과 그 준비 레일 x를 화면에 반영한다.
    pub fn set_zone(&mut self, zone: TestZone, home_rail_x: f64) {
        self.current_zone = Some((zone, home_rail_x));
    }
```

Replace `show`:

```rust
    /// 창을 갱신한다. 반환 `true` = 사용자가 종료(ESC/`q`)를 눌렀다.
    pub fn show(&mut self) -> bool {
        if self.panels.is_empty() {
            return false;
        }
        return match self.render() {
            Ok(action) => action == PreviewAction::Quit,
            Err(error) => {
                warn!(%error, "프리뷰 렌더 실패");
                false
            }
        };
    }
```

with:

```rust
    /// 창을 갱신한다. 반환값은 highgui 키 입력 그대로 — 호출측이 Quit/Key를 처리한다.
    pub fn show(&mut self) -> PreviewAction {
        if self.panels.is_empty() {
            return PreviewAction::Continue;
        }
        return match self.render() {
            Ok(action) => action,
            Err(error) => {
                warn!(%error, "프리뷰 렌더 실패");
                PreviewAction::Continue
            }
        };
    }
```

Update the one call site inside `render`:

```rust
        if let Some(state) = &self.control_state {
            draw_control_state_panel(&mut mosaic, state)?;
        }
```

becomes:

```rust
        if let Some(state) = &self.control_state {
            draw_control_state_panel(&mut mosaic, state, self.current_zone)?;
        }
```

- [ ] **Step 4: Implement — status panel draws zone + key legend**

Change the constant:

```rust
const STATE_PANEL_H: i32 = 110;
```

to:

```rust
const STATE_PANEL_H: i32 = 150;
```

Change the `draw_control_state_panel` signature:

```rust
fn draw_control_state_panel(image: &mut Mat, state: &ControlStateSnapshot) -> opencv::Result<()> {
```

becomes:

```rust
fn draw_control_state_panel(
    image: &mut Mat,
    state: &ControlStateSnapshot,
    zone: Option<(TestZone, f64)>,
) -> opencv::Result<()> {
```

Replace the tail of the function — from the existing `if let ControlStateSnapshot::Aligning { ... } = state { ... }` block through the final `return Ok(());` — with the same block plus the new zone/legend drawing appended before the `return`:

```rust
    if let ControlStateSnapshot::Aligning {
        track_seq,
        return_due_at,
        rail_commanded_m,
        aim_commanded_rad,
    } = state
    {
        let remaining = return_due_at
            .saturating_duration_since(Instant::now())
            .as_secs_f64();
        let line1 = format!("track {track_seq}  returns {remaining:.2}s");
        let line2 = format!(
            "rail {:.3}m  aim {:.1}deg",
            rail_commanded_m,
            aim_commanded_rad.to_degrees()
        );
        camera::Preview::draw_text_at_px(
            image,
            camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 80)),
            &line1,
            0.4,
            STATE_ACTIVE_COLOR,
            1,
        )?;
        camera::Preview::draw_text_at_px(
            image,
            camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 98)),
            &line2,
            0.4,
            STATE_ACTIVE_COLOR,
            1,
        )?;
    }

    if let Some((zone, home_rail_x)) = zone {
        let zone_line = format!("ZONE {}  x={home_rail_x:.3}", zone.label());
        camera::Preview::draw_text_at_px(
            image,
            camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 116)),
            &zone_line,
            0.42,
            STATE_ACTIVE_COLOR,
            1,
        )?;
    }
    camera::Preview::draw_text_at_px(
        image,
        camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 134)),
        "1/2/3 zone  w wait  r reset",
        0.35,
        STATE_IDLE_COLOR,
        1,
    )?;
    return Ok(());
}
```

Update the two pre-existing tests' calls to `draw_control_state_panel` to pass the new `zone` argument (both pass `None` — they only assert node colors):

```rust
        draw_control_state_panel(&mut img, &ControlStateSnapshot::Idle, None).unwrap();
```

and

```rust
        draw_control_state_panel(&mut img, &state, None).unwrap();
```

- [ ] **Step 5: Run preview tests to verify they pass**

Run: `cargo test --bin pingpong-bot real::preview::tests`

Expected: PASS (5/5 — the new `set_zone` test plus the four pre-existing tests, two of which now pass `None`).

- [ ] **Step 6: Wire `run.rs` — real channel, key routing, zone display**

Update imports:

```rust
use crossbeam_channel::{Receiver, bounded, unbounded};
```

becomes:

```rust
use crossbeam_channel::{Receiver, Sender, bounded, unbounded};
```

```rust
use pingpong_bot::camera::{Calibration, CamCliArgs, CamStreamArgs, StereoOfflineArgs};
```

becomes:

```rust
use pingpong_bot::camera::{Calibration, CamCliArgs, CamStreamArgs, PreviewAction, StereoOfflineArgs};
```

```rust
use super::{
    Options, PacedSource, PreviewEvent, PreviewWindow, RuntimeEvent, ShutdownGuard, control_worker,
    shutdown_channel, sim_host,
};
```

becomes:

```rust
use super::{
    Options, PacedSource, PreviewEvent, PreviewWindow, RuntimeEvent, ShutdownGuard, TestControl,
    control_worker, shutdown_channel, sim_host,
};
```

Replace the Task-3 placeholder line:

```rust
    // Task 4가 실제 키 입력으로 채운다 — 지금은 컴파일 유지용.
    let (_test_control_tx, test_control_rx) = unbounded();
```

with:

```rust
    let (test_control_tx, test_control_rx) = unbounded();
```

Update the `main_loop` call:

```rust
    let outcome = main_loop(&options, &event_rx, preview_rx, guard);
```

becomes:

```rust
    let outcome = main_loop(&options, &event_rx, preview_rx, test_control_tx, guard);
```

Update the `main_loop` signature:

```rust
fn main_loop(
    options: &Options,
    event_rx: &Receiver<RuntimeEvent>,
    preview_rx: Option<Receiver<PreviewEvent>>,
    guard: ShutdownGuard,
) -> Outcome {
```

becomes:

```rust
fn main_loop(
    options: &Options,
    event_rx: &Receiver<RuntimeEvent>,
    preview_rx: Option<Receiver<PreviewEvent>>,
    test_control_tx: Sender<TestControl>,
    guard: ShutdownGuard,
) -> Outcome {
```

Replace the Task-3 stub arm in the event-handling `match &event { ... }` block:

```rust
                RuntimeEvent::TestZoneChanged { .. } => {}
```

with:

```rust
                RuntimeEvent::TestZoneChanged { zone, home_rail_x } => {
                    if let Some(preview) = &mut preview {
                        preview.set_zone(*zone, *home_rail_x);
                    }
                }
```

Replace the `Some(preview) => { ... }` arm of the final `match &mut preview { ... }` block:

```rust
            Some(preview) => {
                if let Some(rx) = &preview_rx {
                    while let Ok(event) = rx.try_recv() {
                        preview.push(event);
                    }
                }
                if preview.show() {
                    outcome.last = LastState::Quit;
                    break outcome;
                }
            }
```

with:

```rust
            Some(preview) => {
                if let Some(rx) = &preview_rx {
                    while let Ok(event) = rx.try_recv() {
                        preview.push(event);
                    }
                }
                match preview.show() {
                    PreviewAction::Quit => {
                        outcome.last = LastState::Quit;
                        break outcome;
                    }
                    PreviewAction::Key(key) => {
                        if let Some(control) = TestControl::from_key(key) {
                            let _ = test_control_tx.send(control);
                        }
                    }
                    PreviewAction::Continue => {}
                }
            }
```

Replace the Task-3 stub arm in `log_event`:

```rust
        RuntimeEvent::TestZoneChanged { .. } => {}
```

with:

```rust
        RuntimeEvent::TestZoneChanged { zone, home_rail_x } => info!(
            ?zone,
            home_rail_x = f2(*home_rail_x),
            "테스트 존 변경 — 준비 자세 레일 x 갱신"
        ),
```

- [ ] **Step 7: Full build and regression sweep**

Run: `cargo build --bin pingpong-bot`

Expected: builds cleanly, and — since this is the last task — check the warning output for anything mentioning `TestZone`, `TestControl`, or `TestZoneChanged`; there should be none (everything added across Tasks 1-4 is now genuinely read/constructed/called somewhere).

Run: `cargo test --bin pingpong-bot real::`

Expected: PASS — every test under `real::` (`control_worker`, `preview`, `test_control`, and any others already in the module), none broken by this task's changes.

Run: `cargo test --lib robot::motion::physics::tests`

Expected: PASS — confirms Task 2's `physics.rs` change is still solid alongside everything else.

Run: `cargo test --lib defaults::`

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add src/real/preview.rs src/real/run.rs
git commit -m "feat(real): route preview keys to TestControl, show current test zone"
```

---

## Manual Smoke Test (after Task 4, not automatable)

This plan's automated tests cover the logic; the hardware interaction itself needs a human at the bench:

1. `cargo run --bin pingpong-bot -- --mode real --preview --dry-run --home` (dry-run first — no motor motion, but the full pipeline including preview and key handling runs).
2. Confirm the status panel shows `ZONE CENTER x=0.710` (or whatever `RAIL_READY_X_M` currently is) right after startup.
3. Press `1`, confirm the panel updates to `ZONE LEFT x=0.070` (5% of the rail's `1.41 m` span inset from `RAIL_X_MIN_M = 0.0` — the true `f64` value rounds to `.070` with `{:.3}`, not `.071`; verify against the actual constants in `src/defaults/hardware.rs` if they've since changed) and the log line `테스트 존 변경` (or, in dry-run, `테스트 컨트롤 적용`) appears.
4. Press `3`, confirm it updates to `ZONE RIGHT x=1.339` (the same 5% margin inset from `RAIL_X_MAX_M = 1.41`).
5. Press `w`, confirm the zone stays `RIGHT` but a fresh `테스트 컨트롤 적용` log line appears.
6. Press `r`, confirm the same happens even if pressed while `--dry-run` reports busy right after a simulated alignment.
7. Repeat without `--dry-run` on the real bench, confirming actual rail motion matches the panel's `x=` value, before running the real 10-ball-per-zone protocol. Confirm empirically whether `1`/`3` land on the physically-left/right sides of the table — if reversed, swap `TestZone::rail_x`'s two arms (Task 1, `src/real/test_control.rs`) and re-run this smoke test. Also confirm 5% actually clears the hard stops by a comfortable margin at both ends; adjust `RAIL_ZONE_SAFETY_MARGIN_RATIO` (`src/defaults/hardware.rs`) if not.
