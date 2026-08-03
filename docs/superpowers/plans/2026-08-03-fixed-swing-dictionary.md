# Fixed Swing Dictionary Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a third, IK-free swing mode — a hard-coded start/end joint-angle "swing dictionary" — that decides only (1) the linear rail's target x from the estimated ball trajectory, (2) when to start the fixed swing so it lands on the predicted impact time, and (3) triggers it. No inverse kinematics is used anywhere in this mode, in either sim or real hardware.

**Architecture:** A new pure module (`robot/motion/fixed_swing.rs`) holds the two fixed joint-angle poses and two small functions: `plan_fixed_swing` (a torque/speed/accel-feasible quintic between the two fixed poses, reusing the already-existing `Planner::move_to_fastest`) and `should_start_fixed_swing` (a duration comparison, no search). Sim and real each get a thin integration that feeds this module a single `Prediction`/`HitTarget` already produced by existing (non-IK) trajectory code, and commit through the same `Hardware::command`/`robot::State::replace_swing` surfaces every other swing mode already uses. `tools/jog` gets a manual mode for real-hardware dry-run/apply testing.

**Tech Stack:** Rust, existing `pingpong_bot` crate (nalgebra, no new dependencies), egui (sim GUI), `tools/jog` (kiss3d/egui real-hardware jog tool).

## Global Constraints

- No inverse kinematics anywhere in this feature's code path — not the 5D pose IK (`inverse_pose_with_rail`), not the 3D position IK (`robot::control::position_only_goal`), not the 3-joint chain IK. Rail target comes from clamping a predicted position's x directly; joint targets are the two fixed dictionaries.
- Swing dictionary values are exact, in degrees, joint order `[j0 yaw, j1 shoulder, j2 elbow, j3 wrist]` (matches `tools/jog`'s `JOINT_LABELS` and the sim "Motor Test" panel's joint order):
  - start: `[-10.0, 0.0, 50.0, -30.0]`
  - end: `[40.0, 0.0, -12.0, -70.0]`
- "100% of motor torque limit" = the already-existing `Planner::move_to_fastest` path (no gentle `RETURN_TO_CENTER_MIN_SECS` floor; duration search stops at the shortest quintic that still respects `peak_torque_utilization <= 1.0`, `kinematic_limit_violation`, and collision checks). Do not build a second, separate "torque-saturated" planner.
- This branch (`feat/predefined-swing-dictionary`) is based on `codex/control-logic-rewrite`, not `main`. Reuse its `robot::control::{HitTargetSelector, HitTarget}`, `estimator::{BallTrajectory, TrajectorySample}`, and `real/control_worker.rs`-adjacent plumbing (`CommitRequest`, `Hardware` trait) — do not reintroduce `main`'s `Planner::plan_best_swing`/`solve_impact_target` pipeline.
- Every new sim toggle must follow the existing `use_bang_bang_swing` wiring pattern exactly: `SimRuntimeControls` field → `panel_ui_state.rs`/`panel.rs` checkbox → `session.rs` physics-thread sync → `SimWorld` setter/field. Do not invent a second plumbing mechanism.
- Existing swing modes (`plan_best_swing`/`PositionController`/bang-bang) must keep working unmodified — this is an additive mode selected by a toggle/flag, not a replacement.

---

## File Structure

- Create `src/robot/motion/fixed_swing.rs` — the dictionary constants, `plan_fixed_swing`, `fixed_swing_rail_target`, `should_start_fixed_swing`. Pure, no I/O, fully unit-testable.
- Modify `src/robot/motion/mod.rs` — register the new module.
- Modify `src/robot/motion/planner.rs` — expose the three functions above through `Planner::` (matches the codebase's "everything channels through `Planner`" convention).
- Modify `src/sim/session/controls.rs` — add `use_fixed_swing_dictionary: bool` to `SimRuntimeControls`.
- Modify `src/sim/session/session.rs` — sync the new toggle into `SimWorld` each physics tick.
- Modify `src/sim/physics/world.rs` — add the field/setter and `try_fixed_swing_dictionary`, wired into `try_auto_swing`.
- Modify `src/sim/gui/viewer/panel_ui_state.rs`, `src/sim/gui/viewer/panel.rs` — GUI checkbox next to the existing "Bang-bang swing (debug)" toggle.
- Create `src/real/fixed_swing_worker.rs` — real-hardware control worker for this mode (parallel to, not a rewrite of, `control_worker.rs`).
- Modify `src/real/mod.rs` — register the new worker module.
- Modify `src/real/options.rs`, `src/cli/args.rs`, `src/real/run.rs` — `--fixed-swing-dictionary` flag, threaded through to pick which worker `run()` spawns.
- Modify `tools/jog/src/plan/kind.rs`, `tools/jog/src/plan/mod.rs`, `tools/jog/src/panel.rs` — a `Kind::FixedSwing` manual mode for real-hardware dry-run/apply testing.

---

### Task 1: Core fixed-swing module

**Files:**
- Create: `src/robot/motion/fixed_swing.rs`
- Modify: `src/robot/motion/mod.rs`
- Modify: `src/robot/motion/planner.rs`
- Test: inline `#[cfg(test)]` in `src/robot/motion/fixed_swing.rs`

**Interfaces:**
- Consumes: `crate::robot::{Arm, Joints, LinearRail, Pose}`, `crate::robot::motion::{Planner, Trajectory}` (`Planner::move_to_fastest` already exists on this branch per the carried-over WIP diff in `physics.rs`/`planner.rs`), `crate::error::DomainError`, `crate::defaults::MIN_TIME_TO_GO_SECS` (already defined in `src/defaults/motion.rs:20`).
- Produces (used by Tasks 2 and 4):
  - `pub const FIXED_SWING_START_DEG: [f64; 4]`, `pub const FIXED_SWING_END_DEG: [f64; 4]`
  - `pub fn fixed_swing_start_joints() -> Joints`
  - `pub fn fixed_swing_end_joints() -> Joints`
  - `pub fn plan_fixed_swing(arm: &Arm, rail_x: f64) -> Result<Trajectory, DomainError>`
  - `pub fn fixed_swing_rail_target(rail: &LinearRail, predicted_impact_x: f64) -> f64`
  - `pub fn should_start_fixed_swing(time_to_impact_secs: f64, swing_duration_secs: f64) -> bool`
  - Re-exported as `Planner::fixed_swing_start_joints()`, `Planner::fixed_swing_end_joints()`, `Planner::plan_fixed_swing(arm, rail_x)`.

- [ ] **Step 1: Commit the current baseline before adding new code**

The branch already carries the "Motor Test" WIP diff (`physics.rs`, `planner.rs`, `panel.rs`, `panel_ui_state.rs`) and the 41 restored `docs/superpowers`/`docs/wp*.md` files as uncommitted changes. Commit them first so this task's diff is isolated.

```bash
git add -A
git status --short
```

Expected: only the known WIP files (motor test panel) and the 41 restored docs are staged — no unrelated files.

```bash
git commit -m "$(cat <<'EOF'
chore: carry Motor Test WIP onto predefined-swing-dictionary branch, restore superpowers docs

Rebased the in-progress Motor Test panel (fixed start/end joint-angle
testing, move_to_fastest) from main onto codex/control-logic-rewrite, and
restored the docs/superpowers and docs/wp*.md files that branch deleted —
main's code comments still cite them by path.
EOF
)"
```

- [ ] **Step 2: Write the failing tests**

Create `src/robot/motion/fixed_swing.rs`:

```rust
//! 사전 정의된 스윙 딕셔너리 — IK 없이 시작/끝 관절각만으로 스윙한다.
//!
//! `Planner::plan_best_swing`/`robot::control::PositionController`와 달리 라켓
//! 위치·자세를 IK로 풀지 않는다. 스윙 "모양"은 [`FIXED_SWING_START_DEG`] →
//! [`FIXED_SWING_END_DEG`]로 고정이고, 호출부는 레일 x(기하만, IK 없음)와
//! 스윙을 시작할 타이밍만 정한다.

use crate::defaults;
use crate::error::DomainError;
use crate::robot::{Arm, Joints, LinearRail, Pose};

use super::{Planner, Trajectory};

/// 스윙 시작(백스윙/준비) 자세 [deg] — j0 yaw, j1 shoulder, j2 elbow, j3 wrist.
pub const FIXED_SWING_START_DEG: [f64; 4] = [-10.0, 0.0, 50.0, -30.0];
/// 스윙 끝(임팩트) 자세 [deg] — 관절 순서는 시작과 동일.
pub const FIXED_SWING_END_DEG: [f64; 4] = [40.0, 0.0, -12.0, -70.0];

pub fn fixed_swing_start_joints() -> Joints {
    return Joints::from_slice(&FIXED_SWING_START_DEG.map(f64::to_radians));
}

pub fn fixed_swing_end_joints() -> Joints {
    return Joints::from_slice(&FIXED_SWING_END_DEG.map(f64::to_radians));
}

/// 레일 `rail_x`에 고정한 채, IK 없이 시작→끝 관절각을 모터 한계(속도·가속·
/// 토크) 100%로 잇는 가장 빠른 quintic. `should_start_fixed_swing`이 이
/// 결과의 `duration_secs`를 스윙 시작 타이밍 판정에 쓴다.
pub fn plan_fixed_swing(arm: &Arm, rail_x: f64) -> Result<Trajectory, DomainError> {
    let start = Pose::new(rail_x, fixed_swing_start_joints());
    return Planner::move_to_fastest(arm, &start, fixed_swing_end_joints(), rail_x);
}

/// 예측 임팩트 x를 레일 사거리 안으로 자른다 — IK 없이 기하만으로 리니어
/// 목표를 정한다.
pub fn fixed_swing_rail_target(rail: &LinearRail, predicted_impact_x: f64) -> f64 {
    return rail.clamp_x(predicted_impact_x);
}

/// 남은 시간이 고정 스윙 소요 시간 이하가 되는 즉시 스윙을 시작해야 한다.
/// 소요 시간이 고정이라(끝속도 탐색 없음) `plan_swing`처럼 duration을
/// 남은 시간에 맞춰 늘리지 않는다 — 대신 "지금이 그 타이밍인가"만 본다.
pub fn should_start_fixed_swing(time_to_impact_secs: f64, swing_duration_secs: f64) -> bool {
    return time_to_impact_secs.is_finite()
        && time_to_impact_secs > defaults::MIN_TIME_TO_GO_SECS
        && time_to_impact_secs <= swing_duration_secs;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dictionary_joints_convert_degrees_to_radians() {
        let start = fixed_swing_start_joints();
        let end = fixed_swing_end_joints();
        for (actual, expected_deg) in start.values.iter().zip(FIXED_SWING_START_DEG) {
            assert!((actual.to_degrees() - expected_deg).abs() < 1e-9);
        }
        for (actual, expected_deg) in end.values.iter().zip(FIXED_SWING_END_DEG) {
            assert!((actual.to_degrees() - expected_deg).abs() < 1e-9);
        }
    }

    #[test]
    fn plan_fixed_swing_starts_and_ends_at_the_dictionary_poses_with_rail_held() {
        let robot = crate::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail");
        let rail_x = rail.default_x();

        let trajectory = plan_fixed_swing(&robot.arm, rail_x).expect("fixed swing plan");

        for (actual, expected) in trajectory
            .start
            .values
            .iter()
            .zip(fixed_swing_start_joints().values)
        {
            assert!((actual - expected).abs() < 1e-9);
        }
        for (actual, expected) in trajectory
            .goal_joints()
            .values
            .iter()
            .zip(fixed_swing_end_joints().values)
        {
            assert!((actual - expected).abs() < 1e-9);
        }
        assert!((trajectory.rail.start - rail_x).abs() < 1e-12);
        assert!((trajectory.rail.end - rail_x).abs() < 1e-12);
        assert!(trajectory.duration_secs > 0.0);
    }

    #[test]
    fn fixed_swing_rail_target_clamps_to_rail_range() {
        let robot = crate::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail");

        assert!((fixed_swing_rail_target(&rail, rail.x_min - 1.0) - rail.x_min).abs() < 1e-12);
        assert!((fixed_swing_rail_target(&rail, rail.x_max + 1.0) - rail.x_max).abs() < 1e-12);
        let mid = (rail.x_min + rail.x_max) * 0.5;
        assert!((fixed_swing_rail_target(&rail, mid) - mid).abs() < 1e-12);
    }

    #[test]
    fn should_start_fixed_swing_fires_only_inside_the_duration_window() {
        let swing_duration = 0.30;
        assert!(!should_start_fixed_swing(0.50, swing_duration), "too early");
        assert!(should_start_fixed_swing(0.30, swing_duration), "exactly at duration");
        assert!(should_start_fixed_swing(0.10, swing_duration), "inside window");
        assert!(
            !should_start_fixed_swing(defaults::MIN_TIME_TO_GO_SECS * 0.5, swing_duration),
            "degenerate tti"
        );
        assert!(!should_start_fixed_swing(f64::NAN, swing_duration), "non-finite");
    }
}
```

- [ ] **Step 3: Run tests to verify they fail (module not registered yet)**

Run: `cargo test -p pingpong-bot --lib fixed_swing 2>&1 | tail -30`
Expected: compile error — `fixed_swing` is not a module of `robot::motion` yet.

- [ ] **Step 4: Register the module and expose it through `Planner`**

In `src/robot/motion/mod.rs`, add the module and re-export:

```rust
pub mod bang_bang;
pub mod feasibility;
pub mod fixed_swing;
pub mod impact_candidate;
```

and in the `pub use` block:

```rust
pub use fixed_swing::{
    FIXED_SWING_END_DEG, FIXED_SWING_START_DEG, fixed_swing_end_joints, fixed_swing_rail_target,
    fixed_swing_start_joints, should_start_fixed_swing,
};
```

In `src/robot/motion/planner.rs`, add to `impl Planner` (near `move_to_fastest`):

```rust
    /// [`super::fixed_swing::fixed_swing_start_joints`].
    pub fn fixed_swing_start_joints() -> crate::robot::Joints {
        return super::fixed_swing::fixed_swing_start_joints();
    }

    /// [`super::fixed_swing::fixed_swing_end_joints`].
    pub fn fixed_swing_end_joints() -> crate::robot::Joints {
        return super::fixed_swing::fixed_swing_end_joints();
    }

    /// [`super::fixed_swing::plan_fixed_swing`].
    pub fn plan_fixed_swing(arm: &Arm, rail_x: f64) -> Result<Trajectory, DomainError> {
        return super::fixed_swing::plan_fixed_swing(arm, rail_x);
    }
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cargo test -p pingpong-bot --lib fixed_swing 2>&1 | tail -40`
Expected: all 4 tests in `robot::motion::fixed_swing::tests` PASS.

- [ ] **Step 6: Run the full library test suite for regressions**

Run: `cargo test -p pingpong-bot --lib 2>&1 | tail -20`
Expected: no new failures (pre-existing failures/warnings, if any, are unrelated).

- [ ] **Step 7: Commit**

```bash
git add src/robot/motion/fixed_swing.rs src/robot/motion/mod.rs src/robot/motion/planner.rs
git commit -m "$(cat <<'EOF'
feat(robot): add IK-free fixed swing dictionary

New robot::motion::fixed_swing module: hard-coded start/end joint-angle
poses, a torque/speed/accel-feasible quintic between them via the
existing move_to_fastest, and a pure duration-comparison timing check.
No inverse kinematics anywhere in this path — callers supply rail x from
plain trajectory geometry (fixed_swing_rail_target) instead.
EOF
)"
```

---

### Task 2: Sim physics-world integration

**Files:**
- Modify: `src/sim/session/controls.rs`
- Modify: `src/sim/session/session.rs`
- Modify: `src/sim/physics/world.rs`
- Test: `src/sim/physics/world.rs` (`#[cfg(test)]` module, existing tests are inline in that file)

**Interfaces:**
- Consumes: `Planner::plan_fixed_swing`, `Planner::fixed_swing_start_joints`, `motion::fixed_swing_rail_target`, `motion::should_start_fixed_swing` (Task 1). `robot::State::{is_swinging, set_rail_target, replace_motion_and_return, set_auto_return_to_center, rail_x, joints}` (already exist, `src/robot/state.rs`). `estimator::Prediction` (already exists).
- Produces: `SimWorld::set_use_fixed_swing_dictionary(&mut self, enabled: bool)`, `SimWorld::use_fixed_swing_dictionary(&self) -> bool`, both used by Task 3 (GUI) and by `session.rs`.

- [ ] **Step 1: Write the failing test**

Add to the `#[cfg(test)] mod tests` block at the bottom of `src/sim/physics/world.rs` (find it via `grep -n "mod tests" src/sim/physics/world.rs` — append inside the existing block, following the file's existing test-construction pattern of building a `SimWorld` and stepping it):

```rust
    /// 고정 스윙 딕셔너리 모드는 커밋 시 START/END 딕셔너리 그대로 재생해야
    /// 한다 — IK로 고른 임의 자세가 아니라 정확히 그 두 포즈.
    #[test]
    fn fixed_swing_dictionary_commits_the_exact_dictionary_poses() {
        let mut world = test_world();
        world.set_use_ground_truth(true);
        world.set_use_fixed_swing_dictionary(true);

        let settings = crate::sim::launch::Settings::default();
        world.shoot_ball(&settings);

        let dt = 1.0 / 1000.0;
        let mut committed_end: Option<Vec<f64>> = None;
        for _ in 0..4000 {
            world.step(dt, None);
            if let Some(trajectory) = world.robot.active_trajectory() {
                committed_end = Some(trajectory.goal_joints().values.clone());
                break;
            }
        }
        let end = committed_end.expect("고정 스윙이 커밋돼야 한다");
        for (actual, expected) in end
            .iter()
            .zip(crate::robot::motion::fixed_swing_end_joints().values)
        {
            assert!((actual - expected).abs() < 1e-9);
        }
    }
```

If the file has no `test_world()` helper, use whatever existing helper the file's other tests use to build a `SimWorld` (e.g. search for `SimWorld::new(` in the same test module and copy its exact call) instead of inventing a new one.

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p pingpong-bot --lib fixed_swing_dictionary_commits_the_exact_dictionary_poses 2>&1 | tail -30`
Expected: compile error — `set_use_fixed_swing_dictionary` does not exist on `SimWorld`.

- [ ] **Step 3: Add the `SimRuntimeControls` field**

In `src/sim/session/controls.rs`, add next to `use_bang_bang_swing`:

```rust
    /// true면 commit 시 quintic 대신 IK 없는 고정 스윙 딕셔너리
    /// (`robot::motion::fixed_swing`)로 계획한다 - GUI 체크박스가 매 프레임 반영한다.
    pub use_fixed_swing_dictionary: bool,
```

and in `Default::default()`:

```rust
            use_bang_bang_swing: false,
            use_fixed_swing_dictionary: false,
```

- [ ] **Step 4: Add the `SimWorld` field, setter, and dictionary commit method**

In `src/sim/physics/world.rs`, add the field next to `use_bang_bang_swing: bool,` (around line 94):

```rust
    use_bang_bang_swing: bool,
    /// true면 commit 시 quintic 대신 IK 없는 고정 스윙 딕셔너리로 계획한다
    /// - GUI 디버그 토글 전용.
    use_fixed_swing_dictionary: bool,
```

Initialize it next to `use_bang_bang_swing: false,` in the constructor (around line 406):

```rust
            use_bang_bang_swing: false,
            use_fixed_swing_dictionary: false,
```

Add the setter/getter next to `set_use_bang_bang_swing`/`use_bang_bang_swing` (around line 480-486). Turning the mode on moves the arm to the dictionary's start pose (so the very first commit's assumed start matches reality) and disables the generic center auto-return — this mode returns to its own start pose instead (Step 5), not `arm.default_joints`:

```rust
    pub fn set_use_bang_bang_swing(&mut self, enabled: bool) {
        self.use_bang_bang_swing = enabled;
    }

    pub fn use_bang_bang_swing(&self) -> bool {
        return self.use_bang_bang_swing;
    }

    /// 고정 스윙 딕셔너리 모드 on/off. 켜지는 순간 팔을
    /// [`motion::fixed_swing_start_joints`]로 이동시킨다 — 이 모드의 모든
    /// 커밋은 그 자세에서 시작한다고 가정하므로, 실제로 거기 있어야 한다.
    /// 일반 중앙 자동복귀([`robot::State::set_auto_return_to_center`])는
    /// 끈다 — 이 모드는 [`Self::try_fixed_swing_dictionary`]가 직접
    /// 시작 자세로 복귀시킨다.
    pub fn set_use_fixed_swing_dictionary(&mut self, enabled: bool) {
        if enabled && !self.use_fixed_swing_dictionary {
            let rail_x = self
                .arm
                .rail
                .map_or(self.robot.rail_x(), |rail| rail.default_x());
            let start = robot::Pose::new(self.robot.rail_x(), self.robot.joints().clone());
            if let Ok(trajectory) = motion::Planner::move_to(
                &self.arm,
                &start,
                motion::fixed_swing_start_joints(),
                rail_x,
            ) {
                self.robot.replace_swing(trajectory);
            }
        }
        self.use_fixed_swing_dictionary = enabled;
        self.robot.set_auto_return_to_center(!enabled);
    }

    pub fn use_fixed_swing_dictionary(&self) -> bool {
        return self.use_fixed_swing_dictionary;
    }
```

Add `try_fixed_swing_dictionary` as a new private method near `try_auto_swing` (after it, so it reads naturally alongside `poll_and_advance_bang_bang`):

```rust
    /// IK 없이 고정 스윙 딕셔너리로 커밋한다. 레일 x는 가장 임박한 예측의
    /// 임팩트 x를 그대로 클램프하고([`motion::fixed_swing_rail_target`]),
    /// 스윙은 [`motion::FIXED_SWING_START_DEG`]→[`motion::FIXED_SWING_END_DEG`]를
    /// 그대로 재생한다. 남은 시간이 그 고정 스윙의 소요 시간 이하가 되는
    /// 순간 커밋한다([`motion::should_start_fixed_swing`]) — quintic 재적합으로
    /// duration을 남은 시간에 맞추는 일반 경로와 달리, 이 경로는 duration이
    /// 고정이라 "지금이 그 타이밍인가"만 판정한다.
    fn try_fixed_swing_dictionary(&mut self, predictions: &[Prediction]) {
        if self.swing_committed || self.robot.is_swinging() {
            return;
        }
        let Some(rail) = self.arm.rail else {
            return;
        };
        let Some(prediction) = predictions
            .iter()
            .min_by(|left, right| {
                left.time_to_impact_secs
                    .total_cmp(&right.time_to_impact_secs)
            })
        else {
            return;
        };
        let target_rail_x =
            motion::fixed_swing_rail_target(&rail, prediction.impact_position.coords.x);
        self.robot.set_rail_target(target_rail_x);

        let Ok(trajectory) = motion::Planner::plan_fixed_swing(&self.arm, target_rail_x) else {
            return;
        };
        if !motion::should_start_fixed_swing(
            prediction.time_to_impact_secs,
            trajectory.duration_secs,
        ) {
            return;
        }
        let return_pose = robot::Pose::new(target_rail_x, motion::fixed_swing_start_joints());
        self.robot.replace_motion_and_return(trajectory, return_pose);
        self.mark_swing_committed();
        info!(
            shot = self.shot_seq,
            rail_x = target_rail_x,
            time_to_impact_secs = prediction.time_to_impact_secs,
            "shot: fixed swing dictionary commit"
        );
    }
```

Wire it into `try_auto_swing`, right after the existing bang-bang branch (`if self.use_bang_bang_swing { ... return; }`, ends around line 849) and before `self.debug_snap.commit_phase = CommitPhase::InWindow;` (line 850):

```rust
        if self.use_fixed_swing_dictionary {
            self.try_fixed_swing_dictionary(&predictions);
            return;
        }
        self.debug_snap.commit_phase = CommitPhase::InWindow;
```

- [ ] **Step 5: Sync the toggle from the physics thread**

In `src/sim/session/session.rs`, extend the tuple destructure and the `SimWorld` sync call (around lines 110-136) to also read and apply `use_fixed_swing_dictionary`:

```rust
                    let (
                        shoot,
                        park,
                        shooter,
                        use_bang_bang_swing,
                        use_fixed_swing_dictionary,
                        rail_frame,
                        intercept,
                    ) = {
                        let mut ctrl = physics_controls.lock().expect("sim controls");
                        let shoot = ctrl.shoot_requested;
                        let park = ctrl.park_requested;
                        ctrl.shoot_requested = false;
                        ctrl.park_requested = false;
                        (
                            shoot,
                            park,
                            ctrl.shooter.clone(),
                            ctrl.use_bang_bang_swing,
                            ctrl.use_fixed_swing_dictionary,
                            ctrl.rail_frame,
                            ctrl.intercept,
                        )
                    };
                    let mut w = physics_world.lock().expect("sim 월드");
                    w.set_use_bang_bang_swing(use_bang_bang_swing);
                    w.set_use_fixed_swing_dictionary(use_fixed_swing_dictionary);
```

(the rest of the `w.step(...)` call is unchanged.)

- [ ] **Step 6: Run test to verify it passes**

Run: `cargo test -p pingpong-bot --lib fixed_swing_dictionary_commits_the_exact_dictionary_poses 2>&1 | tail -40`
Expected: PASS.

- [ ] **Step 7: Run the full library test suite for regressions**

Run: `cargo test -p pingpong-bot --lib 2>&1 | tail -20`
Expected: no new failures — in particular, `use_bang_bang_swing`-related tests (e.g. `world.rs:2187`) still pass unmodified, confirming the new branch doesn't interfere with the existing default path.

- [ ] **Step 8: Commit**

```bash
git add src/sim/session/controls.rs src/sim/session/session.rs src/sim/physics/world.rs
git commit -m "$(cat <<'EOF'
feat(sim): wire the fixed swing dictionary into the physics world

New SimWorld::set_use_fixed_swing_dictionary toggle, mirroring
use_bang_bang_swing's plumbing exactly. When on, the arm parks at the
dictionary's start pose, commits are IK-free (rail x from prediction
geometry, joints from the fixed dictionary), and the swing returns to
its own start pose afterward instead of the generic center.
EOF
)"
```

---

### Task 3: Sim GUI checkbox

**Files:**
- Modify: `src/sim/gui/viewer/panel_ui_state.rs`
- Modify: `src/sim/gui/viewer/panel.rs`

**Interfaces:**
- Consumes: `SimRuntimeControls::use_fixed_swing_dictionary` (Task 2), the existing `debug_checkbox` helper in `panel.rs` (already used for `use_bang_bang_swing` at `panel.rs:543`).
- Produces: nothing new consumed elsewhere — this is the leaf UI.

- [ ] **Step 1: Add the UI-state field**

In `src/sim/gui/viewer/panel_ui_state.rs`, add next to `use_bang_bang_swing` (check its exact field name via `grep -n use_bang_bang_swing src/sim/gui/viewer/panel_ui_state.rs` — the struct already carries `PanelUiState::joint_test_start_deg` etc. from the carried-over WIP, so the field ordering may differ slightly from this plan's earlier `git diff` snapshot):

```rust
    pub use_bang_bang_swing: bool,
    pub use_fixed_swing_dictionary: bool,
```

and in its constructor from `SimRuntimeControls` (`PanelUiState::new`-equivalent — the function that currently does `use_bang_bang_swing: controls.use_bang_bang_swing,`):

```rust
            use_bang_bang_swing: controls.use_bang_bang_swing,
            use_fixed_swing_dictionary: controls.use_fixed_swing_dictionary,
```

- [ ] **Step 2: Add the checkbox and wire it to `SimRuntimeControls`**

In `src/sim/gui/viewer/panel.rs`, find the existing bang-bang checkbox call (`&mut ui_state.use_bang_bang_swing` around line 543) and add a matching one directly after it, using the same `debug_checkbox` helper and in the same window/section:

```rust
    debug_checkbox(
        ui,
        &mut ui_state.use_bang_bang_swing,
        "Bang-bang swing (debug)",
    );
    debug_checkbox(
        ui,
        &mut ui_state.use_fixed_swing_dictionary,
        "Fixed swing dictionary (debug, no IK)",
    );
```

(Match whatever the surrounding call's exact argument order/label style is — inspect the real call at that line before editing, since the WIP diff may have shifted nearby line numbers.)

In the same file's `draw()` function, find `ctrl.use_bang_bang_swing = ui_state.use_bang_bang_swing;` (around line 238) and add directly after it:

```rust
        ctrl.use_bang_bang_swing = ui_state.use_bang_bang_swing;
        ctrl.use_fixed_swing_dictionary = ui_state.use_fixed_swing_dictionary;
```

- [ ] **Step 3: Build and run the sim GUI manually**

Run: `cargo build -p pingpong-bot --bin pingpong-bot 2>&1 | tail -30`
Expected: builds clean.

Run: `cargo run -p pingpong-bot --release -- --mode sim` (or the project's existing sim launch command — check `run-sim-macos.sh` if on macOS) and in the GUI:
1. Check "Fixed swing dictionary (debug, no IK)" — the arm should visibly move to the dictionary's start pose (j0=-10°, j1=0°, j2=50°, j3=-30°).
2. Click Shoot — the rail should track the ball's x, and at some point the arm should swing directly from the start pose to the end pose (j0=40°, j1=0°, j2=-12°, j3=-70°) and then return to the start pose.
3. Uncheck it and confirm normal quintic swinging (the default mode) still works.

Report what you observed — this is a manual UI check per the project's stated testing practice ("start the dev server and use the feature in a browser" analog for a native GUI); do not claim success without having actually run it.

- [ ] **Step 4: Commit**

```bash
git add src/sim/gui/viewer/panel_ui_state.rs src/sim/gui/viewer/panel.rs
git commit -m "feat(sim): add GUI toggle for the fixed swing dictionary mode"
```

---

### Task 4: Real-hardware control worker (no IK, no `PositionController`)

**Files:**
- Create: `src/real/fixed_swing_worker.rs`
- Modify: `src/real/mod.rs`

**Interfaces:**
- Consumes: `CommitRequest { trajectory: BallTrajectory, stage, ball_x, ball_y, ball_vx, raw_ball_x, at }` (`src/real/commit_request.rs`, unchanged), `Hardware` trait (`command`, `read_pose`, `is_busy`, `cancel`, `command_initial_pose`; `src/hardware/hardware.rs`, unchanged), `robot::control::{HitTargetSelector, HitTarget}` (`src/robot/control.rs` — geometry-only interpolation, **not** `PositionController`, no IK), `Planner::{plan_fixed_swing, fixed_swing_start_joints}`, `motion::fixed_swing_rail_target`, `motion::should_start_fixed_swing` (Task 1), `ShotEvent`/`ControlStatus`/`Shutdown`/`SimUpdate`/`PoseMsg`/`SwingMsg` (`src/real/mod.rs` re-exports, unchanged).
- Produces: `pub fn spawn(hardware, arm, intercept, home, rx, status_tx, sim_tx, event_tx, shutdown) -> JoinHandle<()>` — **same signature as `control_worker::spawn`** so `real/run.rs` (Task 5) can pick either worker with a plain `if`/`else` and no changes to the channel/event contract.

- [ ] **Step 1: Write the failing test**

Add `#[cfg(test)] mod tests` at the bottom of the new file, testing the pure decision logic this worker uses (mirroring `control_worker.rs`'s own `TwoStageLatch` unit tests, which test logic without touching real hardware):

```rust
#[cfg(test)]
mod tests {
    use pingpong_bot::Point3;
    use pingpong_bot::estimator::{BallTrajectory, TrajectorySample};
    use pingpong_bot::robot::motion::InterceptWindow;

    use super::target_from_ball_trajectory;

    #[test]
    fn target_from_ball_trajectory_reads_x_and_time_with_no_ik() {
        let window = InterceptWindow {
            y_min: 0.2,
            y_max: 0.4,
            sample_step: 0.03,
        };
        let trajectory = BallTrajectory::new(
            vec![],
            vec![
                TrajectorySample::new(
                    Point3::new(0.2, 0.2, 0.4),
                    nalgebra::Vector3::new(0.0, -2.0, 0.0),
                    0.10,
                ),
                TrajectorySample::new(
                    Point3::new(0.4, 0.4, 0.2),
                    nalgebra::Vector3::new(0.0, -2.0, 0.0),
                    0.20,
                ),
            ],
            std::time::Instant::now(),
        )
        .expect("valid trajectory");

        let target = target_from_ball_trajectory(&trajectory, window).expect("target");
        assert!((target.position.x - 0.3).abs() < 1e-9);
        assert!((target.time_secs - 0.15).abs() < 1e-9);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p pingpong-bot fixed_swing_worker 2>&1 | tail -30`
Expected: compile error — the module/file/function don't exist yet.

- [ ] **Step 3: Implement the worker**

Create `src/real/fixed_swing_worker.rs`:

```rust
//! 실물 고정 스윙 딕셔너리 제어 워커 — IK 없음.
//!
//! `control_worker.rs`(2단계 `PositionController`, 5차원/3차원 IK)와 별개의
//! 워커다. 이 경로는 `HitTargetSelector::select`의 기하 보간(궤적 예측 행
//! 사이 선형보간)만 써서 레일 x·타이밍을 정하고, 관절은 항상 고정 딕셔너리
//! (`robot::motion::{fixed_swing_start_joints, fixed_swing_end_joints}`)를
//! 그대로 재생한다.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};
use pingpong_bot::error::HwError;
use pingpong_bot::estimator::BallTrajectory;
use pingpong_bot::hardware::Hardware;
use pingpong_bot::robot::control::{HitTarget, HitTargetSelector, TargetSelectionError};
use pingpong_bot::robot::motion::{InterceptWindow, Planner};
use pingpong_bot::robot::Arm;
use tracing::{info, info_span, warn};

use super::{CommitRequest, ControlStatus, PoseMsg, ShotEvent, Shutdown, SimUpdate, SwingMsg};

const MAX_REQUEST_AGE_SECS: f64 = 0.250;
const COMMAND_THROTTLE: Duration = Duration::from_millis(20);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);

/// `trajectory.predicted`에서 `window`의 중앙 y를 기하 보간만으로 읽는다 —
/// IK 없음. `HitTargetSelector::select`가 이미 하는 일을 그대로 노출해
/// 단위테스트를 IK/하드웨어 없이 돌릴 수 있게 한 얇은 래퍼.
fn target_from_ball_trajectory(
    trajectory: &BallTrajectory,
    window: InterceptWindow,
) -> Result<HitTarget, TargetSelectionError> {
    let selector = HitTargetSelector::new(window.y_min, window.y_max)
        .map_err(|_| TargetSelectionError::InvalidWindow)?;
    return selector.select(trajectory);
}

/// 제어 워커를 띄운다. `control_worker::spawn`과 같은 시그니처 — `real/run.rs`가
/// 둘 중 하나를 고른다.
pub fn spawn(
    mut hardware: Box<dyn Hardware>,
    arm: Arc<Arm>,
    intercept: InterceptWindow,
    home: bool,
    rx: Receiver<CommitRequest>,
    status_tx: Sender<ControlStatus>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<ShotEvent>,
    shutdown: Shutdown,
) -> JoinHandle<()> {
    return thread::spawn(move || {
        let _span = info_span!("fixed_swing_control").entered();

        if home && let Err(error) = move_to_start(hardware.as_mut(), &arm) {
            warn!(%error, "초기 스윙 시작 자세 정렬 실패 — 2단계 제어를 시작하지 않는다");
            let _ = event_tx.send(ShotEvent::Failed {
                shot_seq: 1,
                reason: format!("초기 정렬 실패: {error}"),
            });
            let _ = event_tx.send(ShotEvent::Done);
            return;
        }

        let pose = match hardware.read_pose() {
            Ok(pose) => pose,
            Err(error) => {
                let _ = event_tx.send(ShotEvent::Failed {
                    shot_seq: 1,
                    reason: format!("시작 포즈 읽기 실패: {error}"),
                });
                let _ = event_tx.send(ShotEvent::Done);
                return;
            }
        };
        if let Some(sim_tx) = &sim_tx {
            let _ = sim_tx.try_send(SimUpdate {
                pose: Some(PoseMsg::from(&pose)),
                ..SimUpdate::default()
            });
        }
        let mut shot_seq: u64 = 1;
        let _ = event_tx.send(ShotEvent::Armed { shot_seq, pose });
        let _ = status_tx.send(ControlStatus::Ready { shot_seq });
        info!("고정 스윙 딕셔너리 제어 준비 — IK 없음");

        let mut last_command: Option<Instant> = None;
        let mut committed_this_ball = false;

        while !shutdown.is_down() {
            let request = match rx.recv_timeout(RECV_TIMEOUT) {
                Ok(request) => request,
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => continue,
            };
            if committed_this_ball
                || request.age_secs() > MAX_REQUEST_AGE_SECS
                || last_command.is_some_and(|at| at.elapsed() < COMMAND_THROTTLE)
                || hardware.is_busy()
            {
                continue;
            }

            let Ok(target) = target_from_ball_trajectory(&request.trajectory, intercept) else {
                continue;
            };
            let Some(rail) = arm.rail else {
                continue;
            };
            let rail_x = pingpong_bot::robot::motion::fixed_swing_rail_target(&rail, target.position.x);

            let Ok(trajectory) = Planner::plan_fixed_swing(&arm, rail_x) else {
                continue;
            };
            let remaining_secs =
                target.time_secs - request.trajectory.reference_time.elapsed().as_secs_f64();
            if !pingpong_bot::robot::motion::should_start_fixed_swing(
                remaining_secs,
                trajectory.duration_secs,
            ) {
                continue;
            }

            if let Err(error) = hardware.command(&trajectory) {
                let _ = event_tx.send(ShotEvent::Failed {
                    shot_seq,
                    reason: format!("고정 스윙 명령 실패: {error}"),
                });
                break;
            }
            if let Some(sim_tx) = &sim_tx {
                let _ = sim_tx.try_send(SimUpdate {
                    swing: Some(SwingMsg::from_trajectory(&trajectory)),
                    ..SimUpdate::default()
                });
            }
            committed_this_ball = true;
            last_command = Some(Instant::now());
            let _ = event_tx.send(ShotEvent::Committed {
                shot_seq,
                time_to_impact_secs: remaining_secs.max(0.0),
                duration_secs: trajectory.duration_secs,
                impact: target.position,
                rail_start: trajectory.rail.start,
                rail_end: trajectory.rail.end,
                peak_joint_speed: trajectory.peak_joint_speed(),
            });
            info!(
                shot = shot_seq,
                rail_x,
                duration_secs = trajectory.duration_secs,
                remaining_secs,
                "real shot: fixed swing dictionary commit (no IK)"
            );

            // 재생 완료를 기다린 뒤 시작 자세로 복귀하고 다음 공을 받는다.
            while hardware.is_busy() {
                thread::sleep(Duration::from_millis(5));
            }
            if let Err(error) = move_to_start(hardware.as_mut(), &arm) {
                warn!(%error, "스윙 시작 자세 복귀 실패 — 현재 자세에서 계속");
            }
            let _ = status_tx.send(ControlStatus::Recovering { shot_seq });
            shot_seq = shot_seq.saturating_add(1);
            committed_this_ball = false;
            let _ = status_tx.send(ControlStatus::Ready { shot_seq });
        }

        let _ = event_tx.send(ShotEvent::Done);
    });
}

/// 레일 중앙 + 고정 딕셔너리 시작 자세로 이동한다 — `control_worker::move_to_center`와
/// 같은 자리지만 목표가 `arm.default_joints`가 아니라 스윙 시작 딕셔너리다.
fn move_to_start(hardware: &mut dyn Hardware, arm: &Arm) -> Result<(), HwError> {
    let rail_center = arm.rail.map_or(0.0, |rail| rail.default_x());
    return hardware.command_initial_pose(rail_center, &Planner::fixed_swing_start_joints());
}

#[cfg(test)]
mod tests {
    use pingpong_bot::Point3;
    use pingpong_bot::estimator::{BallTrajectory, TrajectorySample};
    use pingpong_bot::robot::motion::InterceptWindow;

    use super::target_from_ball_trajectory;

    #[test]
    fn target_from_ball_trajectory_reads_x_and_time_with_no_ik() {
        let window = InterceptWindow {
            y_min: 0.2,
            y_max: 0.4,
            sample_step: 0.03,
        };
        let trajectory = BallTrajectory::new(
            vec![],
            vec![
                TrajectorySample::new(
                    Point3::new(0.2, 0.2, 0.4),
                    nalgebra::Vector3::new(0.0, -2.0, 0.0),
                    0.10,
                ),
                TrajectorySample::new(
                    Point3::new(0.4, 0.4, 0.2),
                    nalgebra::Vector3::new(0.0, -2.0, 0.0),
                    0.20,
                ),
            ],
            std::time::Instant::now(),
        )
        .expect("valid trajectory");

        let target = target_from_ball_trajectory(&trajectory, window).expect("target");
        assert!((target.position.x - 0.3).abs() < 1e-9);
        assert!((target.time_secs - 0.15).abs() < 1e-9);
    }
}
```

(Note: the plan's Step 1 test block and the file's own `#[cfg(test)] mod tests` are the same code — Step 1 exists only to prove it fails before Step 3 makes it compile; don't duplicate it twice in the real file, keep the one inside `fixed_swing_worker.rs`.)

Register the module in `src/real/mod.rs` — add `mod fixed_swing_worker;` next to `mod control_worker;` (check the exact existing line via `grep -n "mod control_worker" src/real/mod.rs`), and `pub(crate) use fixed_swing_worker;` or equivalent visibility matching how `control_worker` is currently exposed to `run.rs` (check whether it's `pub(crate) mod control_worker;` or re-exported some other way, and match it exactly).

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p pingpong-bot fixed_swing_worker 2>&1 | tail -40`
Expected: `target_from_ball_trajectory_reads_x_and_time_with_no_ik` PASSES.

- [ ] **Step 5: Build the whole workspace**

Run: `cargo build -p pingpong-bot 2>&1 | tail -30`
Expected: clean build.

- [ ] **Step 6: Commit**

```bash
git add src/real/fixed_swing_worker.rs src/real/mod.rs
git commit -m "$(cat <<'EOF'
feat(real): add IK-free fixed swing dictionary control worker

New real::fixed_swing_worker, parallel to control_worker.rs — same
spawn() signature and CommitRequest/ShotEvent contract, but reads rail
x and swing timing from HitTargetSelector's plain trajectory
interpolation (no IK at all) and always plays the fixed start/end
joint-angle dictionary from robot::motion::fixed_swing.
EOF
)"
```

---

### Task 5: CLI flag and worker selection

**Files:**
- Modify: `src/cli/args.rs`
- Modify: `src/real/options.rs`
- Modify: `src/real/run.rs`

**Interfaces:**
- Consumes: `real::fixed_swing_worker::spawn` (Task 4), `real::control_worker::spawn` (unchanged).
- Produces: `--fixed-swing-dictionary` CLI flag, `Options.fixed_swing_dictionary: bool`.

- [ ] **Step 1: Add the CLI flag**

In `src/cli/args.rs`, add next to `release_torque` (following the existing bare-bool-flag style, e.g. `dry_run`/`release_torque`):

```rust
    /// real: IK 없는 고정 스윙 딕셔너리 모드로 실행한다 (`control_worker` 대신
    /// `fixed_swing_worker`).
    #[arg(long)]
    pub fixed_swing_dictionary: bool,
```

- [ ] **Step 2: Thread it through `Options`**

In `src/real/options.rs`, add the field and its `from_args` mapping:

```rust
    /// IK 없는 고정 스윙 딕셔너리 모드 (`fixed_swing_worker`)로 실행.
    pub fixed_swing_dictionary: bool,
```

```rust
            fixed_swing_dictionary: args.fixed_swing_dictionary,
```

- [ ] **Step 3: Branch the worker spawn in `run()`**

In `src/real/run.rs`, replace the single `control_worker::spawn(...)` call (around line 109-119) with a branch that picks the worker but keeps every other line (hardware, arm, estimator wiring, channels) identical:

```rust
    let control_handle = if options.fixed_swing_dictionary {
        super::fixed_swing_worker::spawn(
            Box::new(hardware),
            Arc::clone(&arm),
            args.intercept_window(),
            options.home,
            commit_rx,
            status_tx,
            sim_tx,
            event_tx,
            shutdown,
        )
    } else {
        control_worker::spawn(
            Box::new(hardware),
            Arc::clone(&arm),
            args.intercept_window(),
            options.home,
            commit_rx,
            status_tx,
            sim_tx,
            event_tx,
            shutdown,
        )
    };
```

Add the import (`use super::fixed_swing_worker;` or match however `control_worker` is currently imported at the top of `run.rs`).

- [ ] **Step 4: Build and smoke-test both paths compile and parse**

Run: `cargo build -p pingpong-bot --bin pingpong-bot 2>&1 | tail -30`
Expected: clean build.

Run: `cargo run -p pingpong-bot -- --mode real --dry-run --fixed-swing-dictionary --timeout-secs 2 2>&1 | tail -40`
Expected: it attempts to open hardware/cameras (fails fast without real cameras/serial attached, which is expected on a dev machine) but the CLI parses `--fixed-swing-dictionary` without error and the log shows `"고정 스윙 딕셔너리 제어 준비 — IK 없음"` if it gets far enough to spawn the worker, or a clean camera/hardware-open error otherwise — either is fine for this step; a `clap` parse error is not.

- [ ] **Step 5: Commit**

```bash
git add src/cli/args.rs src/real/options.rs src/real/run.rs
git commit -m "feat(real): add --fixed-swing-dictionary flag to select the IK-free worker"
```

---

### Task 6: Manual real-hardware testing via `tools/jog`

**Files:**
- Modify: `tools/jog/src/plan/kind.rs`
- Modify: `tools/jog/src/plan/mod.rs`
- Modify: `tools/jog/src/panel.rs`

**Interfaces:**
- Consumes: `Planner::plan_fixed_swing`, `Planner::fixed_swing_start_joints` (Task 1). `plan::compose`'s existing `move_traj` helper and `Draft`/`JogApp` Sync→Preview→Apply flow (`tools/jog/src/plan/mod.rs`, `tools/jog/src/state/jog_app.rs` — unchanged).
- Produces: `Kind::FixedSwing`, selectable in the jog GUI, previewable and applicable to real hardware (or dry-run) through the existing `Action::{Sync,Preview,Apply,Discard}` machinery — this is this plan's actual "test on the real robot" tool, per `docs/two-stage-position-control.md`'s convention of dry-run-first hardware verification.

- [ ] **Step 1: Add the `Kind` variant**

In `tools/jog/src/plan/kind.rs`:

```rust
pub enum Kind {
    Joint,
    Angles,
    RailAbs,
    Ik,
    Pose,
    /// 슈터가 쏜 공의 예측 도달점으로 임팩트 스윙.
    Swing,
    /// 고정 스윙 딕셔너리(IK 없음) — 현재 레일 x에서 START → END 관절각.
    FixedSwing,
}
```

and in `Kind::label`:

```rust
            Self::Swing => "스윙 (슈터 공)",
            Self::FixedSwing => "고정 스윙 딕셔너리 (IK 없음)",
```

- [ ] **Step 2: Handle it in `compose`**

In `tools/jog/src/plan/mod.rs`, add a match arm in `compose` (next to `Kind::Swing`'s `anyhow::bail!`):

```rust
        Kind::Swing => anyhow::bail!("스윙은 plan_swing()으로 계획합니다"),
        Kind::FixedSwing => motion::Planner::plan_fixed_swing(arm, start.rail_x)
            .map_err(|error| anyhow::anyhow!("{error}"))
            .context("고정 스윙 딕셔너리"),
```

Also update `reach_ok`'s wildcard arm — it already returns `true` for unmatched kinds via its `_ => true` catch-all, so `Kind::FixedSwing` needs no change there, but double check by reading the current match before assuming.

- [ ] **Step 3: Add it to the panel's motion-kind selector**

In `tools/jog/src/panel.rs`, add `Kind::FixedSwing` to the `for kind in [...]` list in `draw_motion` (next to `Kind::Swing`):

```rust
            for kind in [
                Kind::Joint,
                Kind::Angles,
                Kind::RailAbs,
                Kind::Ik,
                Kind::Pose,
                Kind::Swing,
                Kind::FixedSwing,
            ] {
```

Since `compose` for `Kind::FixedSwing` needs no extra draft fields (no reach delta, no tilt, no shooter), no new match arm is needed in `draw_motion`'s `match app.draft.kind { ... }` body — the default (no extra widgets drawn) is correct; confirm this by checking whether that match is exhaustive (if it is, add `Kind::FixedSwing => {}` with a short label reminding the user it uses whatever rail x Sync captured).

- [ ] **Step 4: Build**

Run: `cargo build -p jog 2>&1 | tail -30`
Expected: clean build.

- [ ] **Step 5: Manual dry-run verification on real hardware**

This is the actual "test on the real robot" step. Follow the project's existing dry-run-first convention (`tools/jog/README.md`, `RealHardware::dry_run_with_arm`):

```bash
cargo run -p jog --release -- --dry-run
```

In the jog GUI:
1. Click "동기화" (Sync) to read the (simulated, since dry-run) starting pose.
2. Select "고정 스윙 딕셔너리 (IK 없음)" from the motion dropdown.
3. Click "미리보기" (Preview) — the sim view should animate the arm moving from the fixed start pose (j0=-10°, j1=0°, j2=50°, j3=-30°) to the fixed end pose (j0=40°, j1=0°, j2=-12°, j3=-70°) at the currently-synced rail x.
4. Confirm the reported duration in the status area is sane (not near-zero, not implausibly long) — compare against the `plan_fixed_swing` test's `trajectory.duration_secs > 0.0` from Task 1, and sanity check it against `1/DYNAMIXEL_MAX_JOINT_SPEED_RAD_S`-scale expectations.
5. Click "적용" (Apply) — in dry-run this drives the simulated/mirrored path only; confirm no errors.

Report the observed preview duration and whether Apply completed without error. Do not proceed to a live (non-dry-run) hardware run without the user present and explicitly approving it — `RealHardware::new` (non-dry-run) drives actual motors.

- [ ] **Step 6: Commit**

```bash
git add tools/jog/src/plan/kind.rs tools/jog/src/plan/mod.rs tools/jog/src/panel.rs
git commit -m "feat(jog): add a fixed swing dictionary mode for real-hardware dry-run/apply testing"
```

---

## Self-Review Notes

- **Spec coverage:** (1) rail x from trajectory, no IK — `fixed_swing_rail_target` (Task 1) + its use in `try_fixed_swing_dictionary`/`fixed_swing_worker` (Tasks 2, 4). (2) swing timing from trajectory — `should_start_fixed_swing` (Task 1). (3) swing at that timing — the `replace_motion_and_return`/`hardware.command` calls gated on it (Tasks 2, 4). "100% torque" — `move_to_fastest` reuse (Task 1), documented in Global Constraints. "New branch" — done at session start (`feat/predefined-swing-dictionary`, based on `codex/control-logic-rewrite`). "Debug/test on real robot" — Task 6 (`tools/jog`) plus Task 5's `--dry-run --fixed-swing-dictionary` CLI path.
- **Placeholder scan:** no TBD/TODO; every step has concrete code or an exact command. Task 6 Steps 2-3 ask the implementer to check an existing match's exhaustiveness before editing rather than presupposing its exact current shape — that's a real uncertainty (the file wasn't fully read line-by-line in planning), not a placeholder, and the instruction tells them exactly how to resolve it.
- **Type consistency:** `Trajectory`, `Joints`, `Pose`, `LinearRail`, `DomainError`, `Prediction`, `BallTrajectory`, `HitTarget`, `TargetSelectionError`, `InterceptWindow` are used with the exact field/method names verified by reading the current source (`src/robot/motion/trajectory.rs`, `src/robot/joints.rs`, `src/robot/pose.rs`, `src/robot/rail/linear.rs`, `src/estimator/prediction.rs`, `src/estimator/trajectory.rs`, `src/robot/control.rs`) rather than assumed.
