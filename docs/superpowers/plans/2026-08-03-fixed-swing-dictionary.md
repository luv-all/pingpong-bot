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
- **Task 3b (correction, inserted after live-GUI testing exposed a design defect):** modify `src/robot/motion/fixed_swing.rs`/`planner.rs`/`mod.rs`, `src/sim/physics/world.rs`, `src/sim/session/controls.rs`/`session.rs`, `src/sim/gui/viewer/panel_ui_state.rs`/`panel.rs` again — replace "gate on the swing's full duration" with "gate on an assumed impact instant partway through the swing" (`ImpactTimeStrategy::{Midpoint, PeakRacketSpeed}`), with a live GUI selector to compare both.
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

### Task 3b: Impact-time-aware swing timing (correction — hit mid-swing, not end-of-swing)

**Why this task exists:** Tasks 1-2's `should_start_fixed_swing(time_to_impact_secs, swing_duration_secs)` compared the ball's remaining flight time against the *entire* swing's duration, implicitly treating "the swing finishes" as "the racket meets the ball." That's wrong: the racket sweeps continuously from the start pose to the end pose, and the ball must be met *during* that sweep, not at its end. Live-GUI testing during Task 3 confirmed the practical symptom: for `defaults::robot()` and the default sim shot, `plan_fixed_swing`'s duration (0.5297s) exceeded even the most generous available lead time (0.517s, the y=0 hit-plane) — so the old "start when `time_to_impact_secs <= duration_secs`" condition was already true on the very first tick after launch, and the swing still finished too late to make contact.

**Fix:** introduce an explicit "assumed impact instant within the swing" (`impact_time_secs`, `0 < impact_time_secs < duration_secs`) and gate `should_start_fixed_swing` on that instead of the full duration. Two selectable strategies for computing it (the user wants to compare both live):
- **Midpoint** — `duration_secs * 0.5`. Trivial, no FK evaluation.
- **PeakRacketSpeed** — the elapsed time within the swing where the racket center's Cartesian speed (via forward-kinematics finite difference over the fixed joint trajectory) is highest — a common "this is where the swing is moving fastest, and fastest predictably reproducible motion" heuristic.

Because the racket's Cartesian *velocity profile* over the fixed START→END joint trajectory does not depend on rail x (the rail is held fixed during the swing; rail_x only translates the racket's position, not its velocity-over-time shape), both strategies can be computed once per planned trajectory and don't need to be recomputed per candidate rail position.

`should_start_fixed_swing`'s function body is unchanged — only what callers pass as its second argument changes (from `trajectory.duration_secs` to the newly computed `impact_time_secs`).

**Files:**
- Modify: `src/robot/motion/fixed_swing.rs` — add `ImpactTimeStrategy`, `DEFAULT_IMPACT_TIME_STRATEGY`, `fixed_swing_impact_time_secs`; update `should_start_fixed_swing`'s doc comment (no body change).
- Modify: `src/robot/motion/planner.rs` — expose `Planner::fixed_swing_impact_time_secs`.
- Modify: `src/sim/physics/world.rs` — `try_fixed_swing_dictionary` computes and uses the impact time instead of `trajectory.duration_secs`; add a `fixed_swing_impact_strategy: ImpactTimeStrategy` field + setter/getter, mirroring the existing `use_fixed_swing_dictionary` field.
- Modify: `src/sim/session/controls.rs` — add `fixed_swing_impact_strategy: ImpactTimeStrategy` to `SimRuntimeControls`.
- Modify: `src/sim/session/session.rs` — sync the new field into `SimWorld` each physics tick, alongside `use_fixed_swing_dictionary`.
- Modify: `src/sim/gui/viewer/panel_ui_state.rs`, `src/sim/gui/viewer/panel.rs` — a small selector (e.g. two `ui.radio_value` buttons) next to the "Fixed swing dictionary" checkbox so the strategy can be flipped live for comparison.

**Interfaces:**
- Consumes: `Trajectory` (`src/robot/motion/trajectory.rs`, unchanged — reads `.duration_secs` and `.sample_at(t)`), `Arm::forward_kinematics_with_rail` (unchanged).
- Produces: `Planner::fixed_swing_impact_time_secs(arm: &Arm, rail_x: f64, trajectory: &Trajectory, strategy: motion::ImpactTimeStrategy) -> f64`, consumed by Task 2's already-merged `try_fixed_swing_dictionary` (modified here) and by Task 4 (not yet started — its brief already reflects this).

- [ ] **Step 1: Write the failing tests**

Add to `src/robot/motion/fixed_swing.rs`'s existing `#[cfg(test)] mod tests` block:

```rust
    #[test]
    fn midpoint_strategy_is_exactly_half_duration() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let trajectory = plan_fixed_swing(&robot.arm, rail_x).expect("fixed swing plan");
        let impact_time = fixed_swing_impact_time_secs(
            &robot.arm,
            rail_x,
            &trajectory,
            ImpactTimeStrategy::Midpoint,
        );
        assert!((impact_time - trajectory.duration_secs * 0.5).abs() < 1e-9);
    }

    #[test]
    fn peak_speed_strategy_picks_a_time_strictly_inside_the_swing() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let trajectory = plan_fixed_swing(&robot.arm, rail_x).expect("fixed swing plan");
        let impact_time = fixed_swing_impact_time_secs(
            &robot.arm,
            rail_x,
            &trajectory,
            ImpactTimeStrategy::PeakRacketSpeed,
        );
        assert!(impact_time > 0.0, "impact_time={impact_time}");
        assert!(
            impact_time < trajectory.duration_secs,
            "impact_time={impact_time} duration={}",
            trajectory.duration_secs
        );
    }

    #[test]
    fn should_start_fixed_swing_now_gates_on_impact_time_not_full_duration() {
        // 회귀 방지: `duration_secs`(전체 소요)가 아니라 그보다 짧은
        // `impact_time_secs`(스윙 내부 임팩트 시각)를 기준으로 삼아야, 스윙이
        // "끝나는" 시점이 아니라 "공을 맞히는" 시점에 남은 시간을 맞춘다.
        let duration_secs = 0.53;
        let impact_time_secs = duration_secs * 0.5;
        // 남은 시간이 절반(임팩트 시각)보다 크면 아직 시작하면 안 된다 — 예전
        // 로직(`duration_secs` 기준)이었다면 이 값에서 이미 시작했을 것이다.
        assert!(!should_start_fixed_swing(0.45, impact_time_secs));
        assert!(should_start_fixed_swing(impact_time_secs, impact_time_secs));
        assert!(should_start_fixed_swing(0.10, impact_time_secs));
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p pingpong-bot --lib fixed_swing 2>&1 | tail -40`
Expected: compile error — `ImpactTimeStrategy`/`fixed_swing_impact_time_secs` don't exist yet.

- [ ] **Step 3: Implement the impact-time functions**

In `src/robot/motion/fixed_swing.rs`, add (near `should_start_fixed_swing`):

```rust
/// 고정 스윙 내부에서 라켓이 공과 만난다고 가정하는 시각을 고르는 전략.
///
/// 라켓은 START→END를 실시간으로 스윕하므로, 공은 그 스윕 **도중** 만나야
/// 한다 — 스윙이 끝나는 순간(= END 자세 도달)을 임팩트로 보면 안 된다
/// (2026-08-03 실측 회귀: 기본 슈터 샷에서 스윙 전체 소요 시간이 로봇
/// 접수창의 어떤 평면에 대해서도 남은 비행 시간보다 길어, "지금 시작해도
/// 이미 늦음"이 발사 첫 틱부터 참이었다).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImpactTimeStrategy {
    /// 스윙 소요 시간의 정확히 절반.
    Midpoint,
    /// 라켓 중심의 순간 속력(FK 유한차분)이 최대가 되는 시각.
    PeakRacketSpeed,
}

/// 두 전략을 비교 중이라 기본값은 더 단순하고 예측 가능한 쪽으로 둔다.
pub const DEFAULT_IMPACT_TIME_STRATEGY: ImpactTimeStrategy = ImpactTimeStrategy::Midpoint;

/// `strategy`에 따라 고정 스윙 내부의 가정 임팩트 시각 [s]을 고른다
/// (`0 < 반환값 < trajectory.duration_secs`).
///
/// 레일은 스윙 도중 고정이라(`plan_fixed_swing`이 `rail_x`를 시작=끝으로 둔다)
/// 라켓의 시간에 따른 **속도** 형태는 `rail_x`와 무관하다 — 위치만
/// 평행이동한다. 그래서 이 계산은 궤적당 한 번만 하면 되고, 레일 위치가
/// 바뀌어도 다시 스윕할 필요가 없다(다만 인터페이스는 `rail_x`를 그대로
/// 받아 FK 위치를 실제로 구한다 — 속도만 rail_x 불변이라는 뜻).
pub fn fixed_swing_impact_time_secs(
    arm: &Arm,
    rail_x: f64,
    trajectory: &Trajectory,
    strategy: ImpactTimeStrategy,
) -> f64 {
    return match strategy {
        ImpactTimeStrategy::Midpoint => trajectory.duration_secs * 0.5,
        ImpactTimeStrategy::PeakRacketSpeed => {
            peak_racket_speed_time(arm, rail_x, trajectory)
        }
    };
}

/// 라켓 중심 속력이 최대인 시각을 유한차분으로 찾는다 — 균등 표본
/// 중심차분, 표본 수는 정확도와 계산량의 실용적 절충.
fn peak_racket_speed_time(arm: &Arm, rail_x: f64, trajectory: &Trajectory) -> f64 {
    const SAMPLES: usize = 64;
    let duration = trajectory.duration_secs;
    if duration <= 0.0 {
        return 0.0;
    }
    let step = duration / SAMPLES as f64;
    let position_at = |t: f64| -> Option<nalgebra::Vector3<f64>> {
        return arm
            .forward_kinematics_with_rail(rail_x, &trajectory.sample_at(t))
            .map(|pose| pose.position.coords);
    };
    let mut best_time = duration * 0.5;
    let mut best_speed = -1.0_f64;
    for index in 0..=SAMPLES {
        let t = step * index as f64;
        let before = (t - step * 0.5).max(0.0);
        let after = (t + step * 0.5).min(duration);
        let span = (after - before).max(1e-9);
        if let (Some(p0), Some(p1)) = (position_at(before), position_at(after)) {
            let speed = (p1 - p0).norm() / span;
            if speed > best_speed {
                best_speed = speed;
                best_time = t;
            }
        }
    }
    return best_time;
}
```

Update `should_start_fixed_swing`'s doc comment (function body unchanged) to:

```rust
/// 남은 시간이 스윙 **내부의 가정 임팩트 시각**([`fixed_swing_impact_time_secs`]) 이하가
/// 되는 즉시 스윙을 시작해야 한다 — 스윙 전체 소요 시간이 아니다. 스윙은
/// START→END를 실시간으로 스윕하므로, 공은 그 스윕 도중 만나야 한다.
pub fn should_start_fixed_swing(time_to_impact_secs: f64, impact_time_secs: f64) -> bool {
    return time_to_impact_secs.is_finite()
        && time_to_impact_secs > defaults::MIN_TIME_TO_GO_SECS
        && time_to_impact_secs <= impact_time_secs;
}
```

In `src/robot/motion/mod.rs`, add `ImpactTimeStrategy`, `DEFAULT_IMPACT_TIME_STRATEGY`, `fixed_swing_impact_time_secs` to the `fixed_swing` re-export list.

In `src/robot/motion/planner.rs`, add:

```rust
    /// [`super::fixed_swing::fixed_swing_impact_time_secs`].
    pub fn fixed_swing_impact_time_secs(
        arm: &Arm,
        rail_x: f64,
        trajectory: &Trajectory,
        strategy: super::fixed_swing::ImpactTimeStrategy,
    ) -> f64 {
        return super::fixed_swing::fixed_swing_impact_time_secs(arm, rail_x, trajectory, strategy);
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p pingpong-bot --lib fixed_swing 2>&1 | tail -40`
Expected: all `fixed_swing` tests (the 4 from Task 1 + the 3 new ones) PASS.

- [ ] **Step 5: Update the sim commit path to use impact time, not full duration**

In `src/sim/physics/world.rs`, add a field next to `use_fixed_swing_dictionary` (added in Task 2):

```rust
    use_fixed_swing_dictionary: bool,
    /// 고정 스윙 내부 임팩트 시각을 고르는 전략 — GUI에서 두 전략을 실시간
    /// 비교할 수 있게 노출한다.
    fixed_swing_impact_strategy: motion::ImpactTimeStrategy,
```

Initialize it in the constructor next to `use_fixed_swing_dictionary: false,`:

```rust
            use_fixed_swing_dictionary: false,
            fixed_swing_impact_strategy: motion::DEFAULT_IMPACT_TIME_STRATEGY,
```

Add setter/getter next to `set_use_fixed_swing_dictionary`/`use_fixed_swing_dictionary`:

```rust
    pub fn set_fixed_swing_impact_strategy(&mut self, strategy: motion::ImpactTimeStrategy) {
        self.fixed_swing_impact_strategy = strategy;
    }

    pub fn fixed_swing_impact_strategy(&self) -> motion::ImpactTimeStrategy {
        return self.fixed_swing_impact_strategy;
    }
```

In `try_fixed_swing_dictionary` (added in Task 2), replace the `should_start_fixed_swing` call:

```rust
        let Ok(trajectory) = motion::Planner::plan_fixed_swing(&self.arm, target_rail_x) else {
            return;
        };
        let impact_time = motion::Planner::fixed_swing_impact_time_secs(
            &self.arm,
            target_rail_x,
            &trajectory,
            self.fixed_swing_impact_strategy,
        );
        if !motion::should_start_fixed_swing(prediction.time_to_impact_secs, impact_time) {
            return;
        }
```

- [ ] **Step 6: Write the failing sim-level regression test**

Add to `src/sim/physics/world.rs`'s test module, next to `fixed_swing_dictionary_commits_the_exact_dictionary_poses`:

```rust
    /// 회귀 방지: 스윙은 남은 비행시간이 스윙 **전체 소요 시간**만큼 남았을 때가
    /// 아니라, 스윙 내부 임팩트 시각(절반)만큼 남았을 때 시작해야 한다 — 즉
    /// 발사 직후 곧바로 커밋하면 안 되고, 공이 접근해 tti가 그 절반 수준으로
    /// 줄어들 때까지 실제로 기다려야 한다.
    #[test]
    fn fixed_swing_dictionary_waits_for_the_midpoint_not_the_full_duration() {
        let robot = crate::defaults::robot().expect("robot");
        let mut world = SimWorld::new(robot);
        world.set_use_ground_truth(true);
        world.set_use_fixed_swing_dictionary(true);
        world.set_fixed_swing_impact_strategy(crate::robot::motion::ImpactTimeStrategy::Midpoint);

        world.shoot_ball(&launch::Settings::default());
        // 발사 바로 다음 스텝에서는 아직 커밋되지 않아야 한다 — 예전(전체
        // 소요 시간 기준) 로직이었다면 이 시점에 이미 커밋했을 것이다.
        world.step(1.0 / 1000.0, None);
        assert!(
            world.robot.active_trajectory().is_none(),
            "발사 즉시 커밋되면 안 된다 — 스윙 절반 시각만큼 남기고 시작해야 한다"
        );

        let dt = 1.0 / 1000.0;
        let mut committed = false;
        for _ in 0..4000 {
            world.step(dt, None);
            if world.robot.active_trajectory().is_some() {
                committed = true;
                break;
            }
        }
        assert!(committed, "결국은 커밋돼야 한다");
    }
```

- [ ] **Step 7: Run test to verify it fails, then passes after Step 5's fix**

Run: `cargo test -p pingpong-bot --lib fixed_swing_dictionary_waits_for_the_midpoint_not_the_full_duration -- --nocapture 2>&1 | tail -30`

If you're implementing Steps 5 and 6 in order (recommended), this test should already pass once Step 5 is done — if so, that's fine; confirm it passes and move on. If you write this test before Step 5's change, expect it to fail first (proving the old behavior was indeed "commits on the very first tick"), then pass after Step 5.

- [ ] **Step 8: Sim GUI comparison selector**

In `src/sim/session/controls.rs`, add to `SimRuntimeControls`:

```rust
    /// 고정 스윙 딕셔너리의 내부 임팩트 시각 전략 ("Fixed swing dictionary" 옆
    /// 선택기). 공이 주차된 동안만 반영해도 되지만, 이 값은 위험이 없어
    /// 비행 중에도 즉시 반영한다.
    pub fixed_swing_impact_strategy: crate::robot::motion::ImpactTimeStrategy,
```

and in its `Default`:

```rust
            fixed_swing_impact_strategy: crate::robot::motion::DEFAULT_IMPACT_TIME_STRATEGY,
```

In `src/sim/session/session.rs`, extend the same tuple/sync block Task 2 added (alongside `use_fixed_swing_dictionary`) to also read and apply `fixed_swing_impact_strategy`:

```rust
                    let (
                        shoot,
                        park,
                        shooter,
                        use_bang_bang_swing,
                        use_fixed_swing_dictionary,
                        fixed_swing_impact_strategy,
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
                            ctrl.fixed_swing_impact_strategy,
                            ctrl.rail_frame,
                            ctrl.intercept,
                        )
                    };
                    let mut w = physics_world.lock().expect("sim 월드");
                    w.set_use_bang_bang_swing(use_bang_bang_swing);
                    w.set_use_fixed_swing_dictionary(use_fixed_swing_dictionary);
                    w.set_fixed_swing_impact_strategy(fixed_swing_impact_strategy);
```

In `src/sim/gui/viewer/panel_ui_state.rs`, add next to `use_fixed_swing_dictionary`:

```rust
    pub fixed_swing_impact_strategy: crate::robot::motion::ImpactTimeStrategy,
```

and in its constructor-from-controls:

```rust
            fixed_swing_impact_strategy: controls.fixed_swing_impact_strategy,
```

In `src/sim/gui/viewer/panel.rs`, directly below the "Fixed swing dictionary" checkbox added in Task 3, add a small selector:

```rust
    ui.horizontal(|ui| {
        ui.label("임팩트 시각:");
        ui.radio_value(
            &mut ui_state.fixed_swing_impact_strategy,
            crate::robot::motion::ImpactTimeStrategy::Midpoint,
            "중간 시점",
        );
        ui.radio_value(
            &mut ui_state.fixed_swing_impact_strategy,
            crate::robot::motion::ImpactTimeStrategy::PeakRacketSpeed,
            "최대 속도 시점",
        );
    });
```

and in `draw()`, next to `ctrl.use_fixed_swing_dictionary = ui_state.use_fixed_swing_dictionary;`:

```rust
        ctrl.fixed_swing_impact_strategy = ui_state.fixed_swing_impact_strategy;
```

- [ ] **Step 9: Build and run the full test suite**

Run: `cargo build -p pingpong-bot 2>&1 | tail -30`
Expected: clean build.

Run: `cargo test -p pingpong-bot --lib 2>&1 | tail -20`
Expected: the same 7 pre-existing failures from this branch's base commit (confirmed pre-existing, unrelated to this feature — see prior tasks' reports), plus all `fixed_swing`-related tests passing, no new failures.

- [ ] **Step 10: Manual GUI comparison (best-effort, report what you can and cannot verify)**

Run the sim GUI (check `run-sim-macos.sh` or the project's existing sim launch invocation). Toggle "Fixed swing dictionary" on, try both "중간 시점" and "최대 속도 시점" with the default shooter settings, and observe:
1. With "중간 시점" (Midpoint): does the swing now visibly wait after launch before starting (rather than starting immediately), and does contact look closer to correct?
2. With "최대 속도 시점" (PeakRacketSpeed): same questions.

As in Task 3, you cannot click/interact with the native window yourself — build clean, launch briefly to confirm no panic, and report plainly that full visual comparison needs the user's own hands-on check.

- [ ] **Step 11: Commit**

```bash
git add src/robot/motion/fixed_swing.rs src/robot/motion/mod.rs src/robot/motion/planner.rs \
        src/sim/physics/world.rs src/sim/session/controls.rs src/sim/session/session.rs \
        src/sim/gui/viewer/panel_ui_state.rs src/sim/gui/viewer/panel.rs
git commit -m "$(cat <<'EOF'
fix(robot): hit the ball mid-swing, not at the end of the swing

should_start_fixed_swing was gated on the swing's full duration, which
implicitly treated "the swing finishes" as "the racket meets the ball."
The racket actually sweeps continuously from the start pose to the end
pose, so impact should happen partway through, not at the end. Add
ImpactTimeStrategy (Midpoint, PeakRacketSpeed via FK finite-difference)
and gate on that instead, with a live GUI selector to compare both.

Confirmed via direct measurement before this fix: the default sim shot's
lead time to every candidate hit-plane (0.39-0.52s) was already shorter
than the swing's full duration (0.53s), so the old gate fired on the
very first physics tick after launch — exactly the "starts swinging the
instant the ball is launched" symptom reported from the live GUI.
EOF
)"
```

---

### Task 4: Real-hardware control worker (no IK, no `PositionController`)

**Files:**
- Create: `src/real/fixed_swing_worker.rs`
- Modify: `src/real/mod.rs`

**Interfaces:**
- Consumes: `CommitRequest { trajectory: BallTrajectory, stage, ball_x, ball_y, ball_vx, raw_ball_x, at }` (`src/real/commit_request.rs`, unchanged), `Hardware` trait (`command`, `read_pose`, `is_busy`, `cancel`, `command_initial_pose`; `src/hardware/hardware.rs`, unchanged), `robot::control::{HitTargetSelector, HitTarget}` (`src/robot/control.rs` — geometry-only interpolation, **not** `PositionController`, no IK), `Planner::{plan_fixed_swing, fixed_swing_start_joints, fixed_swing_impact_time_secs}` (the last one added in Task 3b), `motion::{fixed_swing_rail_target, should_start_fixed_swing, DEFAULT_IMPACT_TIME_STRATEGY}` (Task 1 + Task 3b — gate on `fixed_swing_impact_time_secs`'s result, never on `trajectory.duration_secs` directly), `ShotEvent`/`ControlStatus`/`Shutdown`/`SimUpdate`/`PoseMsg`/`SwingMsg` (`src/real/mod.rs` re-exports, unchanged).
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
            // Task 3b: 스윙 전체 소요 시간이 아니라 스윙 내부 임팩트 시각을
            // 기준으로 삼는다 — 라켓은 START→END를 스윕하는 도중 공을 만나야
            // 하고, 스윙이 "끝나는" 시점을 임팩트로 보면 안 된다.
            let impact_time = Planner::fixed_swing_impact_time_secs(
                &arm,
                rail_x,
                &trajectory,
                pingpong_bot::robot::motion::DEFAULT_IMPACT_TIME_STRATEGY,
            );
            if !pingpong_bot::robot::motion::should_start_fixed_swing(remaining_secs, impact_time)
            {
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

### Task 5b: Per-joint phase-offset support in `Trajectory` (backward-compatible extension)

**Why this task exists:** live-GUI testing (user) showed the fixed swing's racket velocity is well-aligned with the forward/return direction (96-100% of speed vector, measured directly) but the peak speed itself is low (0.879 m/s measured for the current dictionary) despite large joint excursions (j0 50°, j2 62°, j3 40°). Root cause: `plan_fixed_swing` uses `Planner::move_to_fastest`, which builds a single quintic where **all 4 joints share the same start/end time** — every joint reaches its own peak angular velocity at roughly the same instant, with no "kinetic chain" effect (proximal joints building momentum that distal joints' motion, arriving later, adds to). User confirmed: build a staggered/phase-offset ("whip") trajectory as the fix, compare it against the current synchronized shape live in the GUI.

**Why split from Task 5c:** `Trajectory` (`src/robot/motion/trajectory.rs`) is the type every other swing mode in the codebase depends on (`plan_swing`, `plan_bang_bang_swing`, `plan_move_to`/`plan_move_to_fastest`, `plan_return_to_center`, coarse-track). This task extends it **additively** — a new `Option` field that is `None` for every existing call site, preserving their behavior byte-for-byte. Task 5c (which actually builds the staggered fixed-swing trajectory) depends on this extension existing and tested first, in isolation, before any feature code uses it — so a mistake here is caught by this task's own regression tests, not discovered later mixed in with new feature logic.

**Files:**
- Modify: `src/robot/motion/trajectory.rs` — add `joint_phase_offsets: Option<Vec<(f64, f64)>>` field (defaults to `None` in both existing constructors), a `with_phase_offsets` builder, a private local-time-mapping helper, and update `pre_impact_segments`/`sample_at`/`sample_velocity_at`/`sample_acceleration_at` to respect it when present. Follow-through segments are untouched — offsets apply to the pre-impact phase only.

**Interfaces:**
- Consumes: nothing new — pure extension of the existing `Trajectory` struct and its own methods.
- Produces: `Trajectory::with_phase_offsets(self, offsets: Vec<(f64, f64)>) -> Self` (chainable builder), consumed by Task 5c. The field itself (`pub joint_phase_offsets`) is also directly readable/settable for tests.

- [ ] **Step 1: Write the failing tests**

Add to `src/robot/motion/trajectory.rs`'s existing `#[cfg(test)] mod tests` block:

```rust
    #[test]
    fn joint_phase_offsets_default_none_preserves_existing_sampling() {
        let trajectory = Trajectory::new(
            Joints::from_slice(&[0.0, 0.0]),
            Joints::from_slice(&[1.0, 2.0]),
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            0.5,
            Rail::fixed(0.0),
        );
        assert!(trajectory.joint_phase_offsets.is_none());
        let mid = trajectory.sample_at(0.25);
        assert!(mid.values[0] > 0.0 && mid.values[0] < 1.0);
        assert!(mid.values[1] > 0.0 && mid.values[1] < 2.0);
        // 오프셋이 없으면 두 관절이 같은 진행률로 움직여야 한다(회귀 확인).
        let ratio0 = mid.values[0] / 1.0;
        let ratio1 = mid.values[1] / 2.0;
        assert!(
            (ratio0 - ratio1).abs() < 1e-9,
            "오프셋 없으면 두 관절이 같은 진행률로 움직여야 함: {ratio0} vs {ratio1}"
        );
    }

    #[test]
    fn joint_phase_offsets_some_holds_before_and_after_its_own_window() {
        // 관절0은 [0.0, 0.2]에서만, 관절1은 [0.3, 0.5]에서만 움직인다.
        let trajectory = Trajectory::new(
            Joints::from_slice(&[0.0, 0.0]),
            Joints::from_slice(&[1.0, 1.0]),
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            0.5,
            Rail::fixed(0.0),
        )
        .with_phase_offsets(vec![(0.0, 0.2), (0.3, 0.2)]);

        let at0 = trajectory.sample_at(0.0);
        assert!((at0.values[0] - 0.0).abs() < 1e-9);
        assert!((at0.values[1] - 0.0).abs() < 1e-9);

        // t=0.2: 관절0은 이미 자기 구간이 끝나 끝값(1.0)에서 정지, 관절1은
        // 아직 자기 구간(0.3~0.5) 전이라 시작값(0.0) 그대로.
        let mid = trajectory.sample_at(0.2);
        assert!((mid.values[0] - 1.0).abs() < 1e-6, "관절0={}", mid.values[0]);
        assert!((mid.values[1] - 0.0).abs() < 1e-6, "관절1={}", mid.values[1]);

        // t=0.5: 관절0은 계속 끝값 유지, 관절1도 자기 구간이 끝나 끝값(1.0).
        let end = trajectory.sample_at(0.5);
        assert!((end.values[0] - 1.0).abs() < 1e-6);
        assert!((end.values[1] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn joint_phase_offsets_velocity_is_zero_outside_its_own_window() {
        let trajectory = Trajectory::new(
            Joints::from_slice(&[0.0]),
            Joints::from_slice(&[1.0]),
            vec![0.0],
            vec![0.0],
            0.5,
            Rail::fixed(0.0),
        )
        .with_phase_offsets(vec![(0.1, 0.2)]);
        assert!((trajectory.sample_velocity_at(0.05)[0]).abs() < 1e-9, "자기 구간 전");
        assert!((trajectory.sample_velocity_at(0.4)[0]).abs() < 1e-9, "자기 구간 후");
    }

    #[test]
    fn other_swing_modes_never_set_joint_phase_offsets() {
        // 회귀 가드: 다른 모든 스윙 플래너는 이 필드를 건드리지 않아야 한다.
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let start = crate::robot::Pose::new(rail_x, robot.arm.default_joints.clone());
        let trajectory = crate::robot::motion::Planner::return_to_center(&robot.arm, &start)
            .expect("return_to_center");
        assert!(trajectory.joint_phase_offsets.is_none());
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p pingpong-bot --lib joint_phase_offsets 2>&1 | tail -40`
Expected: compile error — `joint_phase_offsets`/`with_phase_offsets` don't exist yet.

- [ ] **Step 3: Add the field, builder, and local-time helper**

Add the field to the `Trajectory` struct, next to `follow_through_rail_velocity`:

```rust
    pub follow_through_rail_velocity: f64,
    /// 관절별 `(로컬 시작 오프셋 [s], 로컬 구간 길이 [s])` — pre-impact 구간에만
    /// 적용된다. `None`이면 모든 관절이 `impact_time_secs`를 그대로 공유한다
    /// (기존 동작, 이 필드가 없던 시절과 동일). `Some`이면 관절 i는 전역 시간
    /// `[offset, offset+duration]` 구간에서만 움직이고, 그 밖에서는 시작/끝
    /// 값에 정지한다 — 근위→원위 순서로 어긋난 채찍형 스윙
    /// ([`crate::robot::motion::fixed_swing`])에 쓰인다. 팔로스루 구간은
    /// 영향받지 않는다.
    pub joint_phase_offsets: Option<Vec<(f64, f64)>>,
```

In both `Trajectory::new` and `Trajectory::with_follow_through`, add `joint_phase_offsets: None,` to the returned `Self { ... }` block (this is the only change needed to keep every existing call site's behavior identical).

Add the builder and helper, near `pre_impact_segments`:

```rust
    /// 관절별 위상 오프셋을 부여한 사본을 돌려준다 — 채찍형 스윙 전용
    /// (`crate::robot::motion::fixed_swing`). `offsets.len()`은 관절 수와
    /// 같아야 한다(호출부 책임 — 검증은 하지 않는다, 내부 전용 빌더이므로).
    pub fn with_phase_offsets(mut self, offsets: Vec<(f64, f64)>) -> Self {
        self.joint_phase_offsets = Some(offsets);
        return self;
    }

    /// 관절 `joint_index`의 전역 시간 `global_t`를 로컬(자기 구간 기준) 시간으로
    /// 변환한다. `joint_phase_offsets`가 없으면 전역 시간을 그대로 쓴다(기존
    /// 동작). 있으면 자기 구간 밖에서는 경계값으로 클램프한다 — 그 결과
    /// `QuinticSegment::sample`이 구간 시작/끝의 경계 조건(위치·속도·가속도)을
    /// 그대로 반환하므로, 관절이 자기 구간 전/후에는 정지해 있는 것처럼
    /// 보인다(양끝 속도가 0인 한).
    fn pre_impact_local_time(&self, joint_index: usize, global_t: f64) -> f64 {
        let Some(offsets) = &self.joint_phase_offsets else {
            return global_t;
        };
        let (offset, duration) = offsets[joint_index];
        return (global_t - offset).clamp(0.0, duration);
    }
```

Modify `pre_impact_segments` to use per-joint duration:

```rust
    fn pre_impact_segments(&self) -> Vec<QuinticSegment> {
        let n = self.start.values.len();
        assert_eq!(self.end.values.len(), n, "impact joint count");
        assert_eq!(self.start_velocity.len(), n, "start velocity count");
        assert_eq!(self.end_velocity.len(), n, "impact velocity count");
        let mut segments = Vec::with_capacity(n);
        for i in 0..n {
            let impact_accel = self.impact_acceleration.get(i).copied().unwrap_or(0.0);
            let duration = self
                .joint_phase_offsets
                .as_ref()
                .map_or(self.impact_time_secs, |offsets| offsets[i].1);
            segments.push(QuinticSegment::new(
                self.start.values[i],
                self.end.values[i],
                self.start_velocity[i],
                self.end_velocity[i],
                0.0,
                impact_accel,
                duration,
            ));
        }
        return segments;
    }
```

Modify `sample_at`'s pre-impact branch to map through `pre_impact_local_time` per joint:

```rust
    pub fn sample_at(&self, t: f64) -> Joints {
        let values = if t <= self.impact_time_secs || self.duration_secs <= self.impact_time_secs {
            self.pre_impact_segments()
                .into_iter()
                .enumerate()
                .map(|(i, segment)| segment.sample(self.pre_impact_local_time(i, t)).0)
                .collect()
        } else {
            self.follow_through_segments()
                .into_iter()
                .map(|segment| segment.sample(t - self.impact_time_secs).0)
                .collect()
        };
        return Joints { values };
    }
```

Apply the exact same pattern (enumerate + `self.pre_impact_local_time(i, t)` in the pre-impact branch, follow-through branch unchanged) to `sample_velocity_at` and `sample_acceleration_at`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p pingpong-bot --lib joint_phase_offsets 2>&1 | tail -40`
Expected: all 4 new tests PASS.

- [ ] **Step 5: Run the full library test suite for regressions**

Run: `cargo test -p pingpong-bot --lib 2>&1 | tail -20`
Expected: the same 7 pre-existing failures documented across every prior task's report in this plan (confirmed pre-existing on this branch's base, unrelated), no new failures. This is the critical check for this task — a regression here would mean the additive field isn't as additive as designed, and it would show up as an EXISTING swing-mode test failing (e.g. anything in `physics.rs`'s or `trajectory.rs`'s own test modules), not one of the 7 known ones.

- [ ] **Step 6: Commit**

```bash
git add src/robot/motion/trajectory.rs
git commit -m "$(cat <<'EOF'
feat(robot): add backward-compatible per-joint phase offsets to Trajectory

Additive extension: a new Option<Vec<(f64,f64)>> field, None for every
existing call site (unchanged behavior, verified by regression test).
When Some, sample_at/sample_velocity_at/sample_acceleration_at map each
joint's own local time through its own (offset, duration) window instead
of sharing a single global timeline — the piece a staggered/whip-style
swing needs, without touching any of the swing planners that don't use it.
EOF
)"
```

---

### Task 5c: Staggered ("whip") fixed swing shape, with live GUI comparison

**Files:**
- Modify: `src/robot/motion/fixed_swing.rs` — add `SwingShapeStrategy` (`Synchronized`, `Staggered`), `DEFAULT_SWING_SHAPE_STRATEGY`, and change `plan_fixed_swing` to take a `shape: SwingShapeStrategy` parameter; add the staggered trajectory builder with its own torque/speed feasibility search.
- Modify: `src/robot/motion/planner.rs` — update `Planner::plan_fixed_swing`'s signature to match.
- Modify: `src/sim/physics/world.rs` — `try_fixed_swing_dictionary` passes a new `fixed_swing_shape_strategy` field (mirroring `fixed_swing_impact_strategy` from Task 3b) instead of a hardcoded call.
- Modify: `src/real/fixed_swing_worker.rs` — update its `Planner::plan_fixed_swing` call to pass `DEFAULT_SWING_SHAPE_STRATEGY` (matching how it already passes `DEFAULT_IMPACT_TIME_STRATEGY`).
- Modify: `src/sim/session/controls.rs`, `src/sim/session/session.rs`, `src/sim/gui/viewer/panel_ui_state.rs`, `src/sim/gui/viewer/panel.rs` — the same 4-hop GUI plumbing pattern as Task 3b's impact-time selector, for a second, independent live comparison toggle.

**Interfaces:**
- Consumes: `Trajectory::with_phase_offsets` (Task 5b), `Arm::required_torque_with_rotor` (already exists, public — `src/robot/arm.rs`), `Trajectory::{peak_joint_speed, sample_at, sample_velocity_at, sample_acceleration_at}` (already exist, public — the last three now offset-aware per Task 5b).
- Produces: `plan_fixed_swing(arm: &Arm, rail_x: f64, shape: SwingShapeStrategy) -> Result<Trajectory, DomainError>` (signature change from Tasks 1/2/4 — this task updates both existing call sites in the same commit), `Planner::fixed_swing_shape_strategy` equivalents.

**Why a bespoke feasibility check instead of reusing `physics.rs`'s `peak_torque_utilization`:** that function (and `kinematic_limit_violation`) samples `Trajectory::joint_segments()` using **one shared local time for every joint** (`segments[i].sample(local_t)` with the same `local_t` for all `i`) — correct only when every joint shares one timeline, which is exactly what this task's `Staggered` shape does NOT do. Reusing it on a staggered trajectory would silently sample the wrong instant for offset joints. Instead, this task samples through `Trajectory::sample_at`/`sample_velocity_at`/`sample_acceleration_at` (Task 5b made these offset-aware) and calls `Arm::required_torque_with_rotor` directly — both already `pub`, no visibility changes to `physics.rs` needed, and correct for any trajectory shape.

- [ ] **Step 1: Write the failing tests**

Add to `src/robot/motion/fixed_swing.rs`'s `#[cfg(test)] mod tests`:

```rust
    #[test]
    fn synchronized_shape_matches_the_original_move_to_fastest_behavior() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let via_shape =
            plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Synchronized).expect("sync");
        assert!(via_shape.joint_phase_offsets.is_none());
        for (actual, expected) in via_shape.start.values.iter().zip(fixed_swing_start_joints().values) {
            assert!((actual - expected).abs() < 1e-9);
        }
    }

    #[test]
    fn staggered_shape_sets_distinct_per_joint_windows_and_stays_feasible() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let staggered =
            plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Staggered).expect("staggered");
        let offsets = staggered
            .joint_phase_offsets
            .clone()
            .expect("staggered trajectory must set phase offsets");
        assert_eq!(offsets.len(), 4);
        // 근위(j0/j1)가 원위(j3)보다 먼저 시작해야 한다 — 채찍 순서 확인.
        assert!(offsets[0].0 <= offsets[3].0, "j0 시작 {} > j3 시작 {}", offsets[0].0, offsets[3].0);
        assert!(offsets[1].0 <= offsets[3].0, "j1 시작 {} > j3 시작 {}", offsets[1].0, offsets[3].0);
        // 각 관절 구간은 궤적 전체 시간 안에 들어와야 한다.
        for (index, (offset, duration)) in offsets.iter().enumerate() {
            assert!(*offset >= 0.0, "joint {index} offset={offset}");
            assert!(
                offset + duration <= staggered.duration_secs + 1e-6,
                "joint {index}: {offset}+{duration} > {}",
                staggered.duration_secs
            );
        }
    }

    #[test]
    fn staggered_shape_reaches_a_higher_peak_racket_speed_than_synchronized() {
        // 이 테스트가 곧 이 기능의 존재 이유다: 채찍형이 동기화형보다
        // 라켓 중심 최고 속력을 실제로 더 내야 한다.
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let sync =
            plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Synchronized).expect("sync");
        let staggered =
            plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Staggered).expect("staggered");

        let peak_speed = |trajectory: &Trajectory| -> f64 {
            const SAMPLES: usize = 80;
            let step = trajectory.duration_secs / SAMPLES as f64;
            let mut best = 0.0_f64;
            for index in 0..=SAMPLES {
                let t = step * index as f64;
                let dt = (step * 0.5).max(1e-6);
                let before = (t - dt).max(0.0);
                let after = (t + dt).min(trajectory.duration_secs);
                let p0 = robot
                    .arm
                    .forward_kinematics_with_rail(rail_x, &trajectory.sample_at(before))
                    .expect("fk")
                    .position
                    .coords;
                let p1 = robot
                    .arm
                    .forward_kinematics_with_rail(rail_x, &trajectory.sample_at(after))
                    .expect("fk")
                    .position
                    .coords;
                let speed = (p1 - p0).norm() / (after - before).max(1e-9);
                best = best.max(speed);
            }
            return best;
        };

        let sync_peak = peak_speed(&sync);
        let staggered_peak = peak_speed(&staggered);
        assert!(
            staggered_peak > sync_peak,
            "채찍형 최고속={staggered_peak} 동기화형 최고속={sync_peak} — 개선이 없음"
        );
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p pingpong-bot --lib fixed_swing 2>&1 | tail -40`
Expected: compile error — `SwingShapeStrategy`/the new `plan_fixed_swing` signature don't exist yet.

- [ ] **Step 3: Implement `SwingShapeStrategy` and the staggered builder**

In `src/robot/motion/fixed_swing.rs`, add:

```rust
/// 고정 스윙의 관절 타이밍 모양 — 사용자가 GUI에서 실시간 비교할 두 선택지.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwingShapeStrategy {
    /// 4관절이 같은 시간축을 공유하는 단일 quintic(기존 `move_to_fastest`).
    /// 라켓 방향은 이미 정확하지만(실측 96-100%가 전진 방향), 근위·원위
    /// 관절이 동시에 각자의 한계까지만 움직여 라켓 최고 속력이 낮다
    /// (2026-08-03 실측: 0.879 m/s).
    Synchronized,
    /// 근위(j0/j1)가 먼저 움직이기 시작해 멈추고, 원위(j2/j3)가 그 뒤에
    /// 시작해 임팩트 순간 "스냅"되는 채찍형 — 각 관절은 여전히 정지→정지
    /// quintic이지만 구간이 서로 어긋나 겹친다(채찍/골프 스윙의 kinetic
    /// chain과 같은 원리).
    Staggered,
}

/// 사용자가 GUI에서 두 전략을 비교하는 동안의 기본값 — 고친 쪽(Staggered)을
/// 기본으로 둔다.
pub const DEFAULT_SWING_SHAPE_STRATEGY: SwingShapeStrategy = SwingShapeStrategy::Staggered;

/// 근위→원위 순서로 어긋난 구간 — 궤적 전체 시간에 대한 분수 `(시작, 끝)`.
/// j0/j1이 먼저 시작해 먼저 끝나고, j2가 그 위에 걸쳐 움직이다가, j3가
/// 가장 늦게 시작해 임팩트 순간 스냅한다. 실측으로 조정 가능한 시작값
/// (`fixed_swing_impact_time_secs`처럼 이후 실측 기반 재조정 여지가 있다).
const STAGGERED_PHASE_FRACTIONS: [(f64, f64); 4] = [
    (0.0, 0.55),  // j0 yaw
    (0.0, 0.65),  // j1 shoulder
    (0.20, 0.65), // j2 elbow (0.20~0.85)
    (0.45, 0.55), // j3 wrist (0.45~1.00)
];
```

Change `plan_fixed_swing`'s signature and implementation:

```rust
/// 레일 `rail_x`에 고정한 채, IK 없이 시작→끝 관절각을 모터 한계(속도·가속·
/// 토크) 100%로 잇는 quintic — `shape`로 관절 타이밍을 고른다.
pub fn plan_fixed_swing(
    arm: &Arm,
    rail_x: f64,
    shape: SwingShapeStrategy,
) -> Result<Trajectory, DomainError> {
    return match shape {
        SwingShapeStrategy::Synchronized => {
            let start = Pose::new(rail_x, fixed_swing_start_joints());
            Planner::move_to_fastest(arm, &start, fixed_swing_end_joints(), rail_x)
        }
        SwingShapeStrategy::Staggered => plan_staggered_fixed_swing(arm, rail_x),
    };
}

/// [`SwingShapeStrategy::Staggered`] 빌더 — 동기화형(`move_to_fastest`)이 낸
/// 실현가능 소요 시간을 기준선으로 잡고, 그 위에 [`STAGGERED_PHASE_FRACTIONS`]
/// 비율로 관절별 구간을 어긋나게 둔 뒤, 이 모양 자체의 속도·토크 실현가능성을
/// (동기화형과 별개로) 확인한다 — 같은 각도 변화를 더 짧은 자기 구간에
/// 눌러넣으므로 관절별 각속도가 동기화형보다 커져, 기준선이 실현 가능했다고
/// 이 모양도 자동으로 실현 가능한 건 아니다. 안 되면 기준 소요 시간을 늘려
/// 재시도한다(`move_to_fastest`/`plan_return_to_center`와 같은 성장 탐색 정신).
fn plan_staggered_fixed_swing(arm: &Arm, rail_x: f64) -> Result<Trajectory, DomainError> {
    let baseline = {
        let start = Pose::new(rail_x, fixed_swing_start_joints());
        Planner::move_to_fastest(arm, &start, fixed_swing_end_joints(), rail_x)?
    };
    let start_joints = fixed_swing_start_joints();
    let end_joints = fixed_swing_end_joints();
    let n = start_joints.values.len();

    let mut duration = baseline.duration_secs;
    const MAX_DURATION_SECS: f64 = 3.0;
    const GROWTH: f64 = 1.2;
    let mut last_error: Option<DomainError> = None;
    while duration <= MAX_DURATION_SECS {
        let offsets: Vec<(f64, f64)> = STAGGERED_PHASE_FRACTIONS
            .iter()
            .take(n)
            .map(|(start_fraction, end_fraction)| {
                let offset = start_fraction * duration;
                (offset, (end_fraction - start_fraction) * duration)
            })
            .collect();
        let candidate = Trajectory::new(
            start_joints.clone(),
            end_joints.clone(),
            vec![0.0; n],
            vec![0.0; n],
            duration,
            Rail::fixed(rail_x),
        )
        .with_phase_offsets(offsets);

        match staggered_feasibility(arm, &candidate) {
            Ok(()) => return Ok(candidate),
            Err(error) => {
                last_error = Some(error);
                duration *= GROWTH;
            }
        }
    }
    return Err(last_error.unwrap_or(DomainError::InfeasibleSwing(
        crate::error::SwingPlanError::InverseKinematicsNoSolution {
            target_x: rail_x,
            target_y: 0.0,
            target_z: 0.0,
        },
    )));
}

/// `candidate`가 관절 속도·토크 한계 안인지 직접 샘플링으로 확인한다 —
/// `physics.rs`의 `peak_torque_utilization`/`kinematic_limit_violation`은
/// 관절마다 **같은** 로컬 시간을 공유한다고 가정해 위상이 어긋난 이
/// 궤적에는 안 맞는다(모듈 문서 참고). `Trajectory::sample_at` 계열은
/// (Task 5b에서) 위상 오프셋을 이미 반영하므로, 이 함수는 그것만으로 검사한다.
fn staggered_feasibility(arm: &Arm, candidate: &Trajectory) -> Result<(), DomainError> {
    if candidate.peak_joint_speed() > arm.max_joint_speed {
        return Err(DomainError::InfeasibleSwing(
            crate::error::SwingPlanError::TrajectoryExceedsLimits {
                rail_end_x: candidate.rail.end,
                violated: "관절 속도",
            },
        ));
    }
    if arm.joint_torque_limits.iter().all(|limit| !limit.is_finite()) {
        return Ok(());
    }
    const SAMPLES: usize = 40;
    let mut worst = 0.0_f64;
    for index in 0..=SAMPLES {
        let t = candidate.duration_secs * index as f64 / SAMPLES as f64;
        let q = candidate.sample_at(t);
        let qd = candidate.sample_velocity_at(t);
        let qdd = candidate.sample_acceleration_at(t);
        let Some(torques) = arm.required_torque_with_rotor(&q.values, &qd, &qdd) else {
            continue;
        };
        for (torque, &limit) in torques.iter().zip(arm.joint_torque_limits.iter()) {
            if limit.is_finite() && limit > 0.0 {
                worst = worst.max(torque.abs() / limit);
            }
        }
    }
    if worst > 1.0 {
        return Err(DomainError::InfeasibleSwing(
            crate::error::SwingPlanError::TrajectoryExceedsTorque {
                rail_end_x: candidate.rail.end,
                utilization: worst,
            },
        ));
    }
    return Ok(());
}
```

Add `use super::rail::Rail;` and `use crate::error::DomainError;`/`SwingPlanError` imports as needed at the top of `fixed_swing.rs` if not already present (check the existing import list first — `Rail`/`DomainError` may already be imported from Task 1).

In `src/robot/motion/mod.rs`, add `SwingShapeStrategy`, `DEFAULT_SWING_SHAPE_STRATEGY` to the `fixed_swing` re-export list.

In `src/robot/motion/planner.rs`, update the existing `Planner::plan_fixed_swing` wrapper's signature to match:

```rust
    /// [`super::fixed_swing::plan_fixed_swing`].
    pub fn plan_fixed_swing(
        arm: &Arm,
        rail_x: f64,
        shape: super::fixed_swing::SwingShapeStrategy,
    ) -> Result<Trajectory, DomainError> {
        return super::fixed_swing::plan_fixed_swing(arm, rail_x, shape);
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p pingpong-bot --lib fixed_swing 2>&1 | tail -60`
Expected: all tests pass, including `staggered_shape_reaches_a_higher_peak_racket_speed_than_synchronized` — this is the test that actually proves the fix works. If it fails (staggered peak ≤ synchronized peak), the `STAGGERED_PHASE_FRACTIONS` values need adjusting (try widening the overlap between j2/j3's windows and j0/j1's, or shifting j3's start earlier) — iterate on the constant, not the test.

- [ ] **Step 5: Update the two existing call sites**

In `src/sim/physics/world.rs`'s `try_fixed_swing_dictionary` (Task 2/3b), add a field next to `fixed_swing_impact_strategy`:

```rust
    fixed_swing_impact_strategy: motion::ImpactTimeStrategy,
    /// 고정 스윙의 관절 타이밍 모양 — GUI에서 두 전략을 실시간 비교한다.
    fixed_swing_shape_strategy: motion::SwingShapeStrategy,
```

Initialize next to `fixed_swing_impact_strategy: motion::DEFAULT_IMPACT_TIME_STRATEGY,`:

```rust
            fixed_swing_shape_strategy: motion::DEFAULT_SWING_SHAPE_STRATEGY,
```

Setter/getter next to the impact-strategy ones:

```rust
    pub fn set_fixed_swing_shape_strategy(&mut self, strategy: motion::SwingShapeStrategy) {
        self.fixed_swing_shape_strategy = strategy;
    }

    pub fn fixed_swing_shape_strategy(&self) -> motion::SwingShapeStrategy {
        return self.fixed_swing_shape_strategy;
    }
```

Update the `plan_fixed_swing` call inside `try_fixed_swing_dictionary`:

```rust
        let Ok(trajectory) = motion::Planner::plan_fixed_swing(
            &self.arm,
            target_rail_x,
            self.fixed_swing_shape_strategy,
        ) else {
            return;
        };
```

In `src/real/fixed_swing_worker.rs`, update its `Planner::plan_fixed_swing` call:

```rust
            let Ok(trajectory) = Planner::plan_fixed_swing(
                &arm,
                rail_x,
                pingpong_bot::robot::motion::DEFAULT_SWING_SHAPE_STRATEGY,
            ) else {
                continue;
            };
```

- [ ] **Step 6: Wire the second GUI comparison selector**

Repeat Task 3b's exact 4-hop pattern (Step 8 of that task) a second time, for `fixed_swing_shape_strategy` instead of `fixed_swing_impact_strategy`: add the field to `SimRuntimeControls` (`src/sim/session/controls.rs`) with default `motion::DEFAULT_SWING_SHAPE_STRATEGY`; extend the same tuple/sync block in `src/sim/session/session.rs` to also read and apply it; add the field to `PanelUiState` (`src/sim/gui/viewer/panel_ui_state.rs`); add a second `ui.radio_value` pair in `src/sim/gui/viewer/panel.rs`, directly below the impact-time selector added in Task 3b:

```rust
    ui.horizontal(|ui| {
        ui.label("스윙 모양:");
        ui.radio_value(
            &mut ui_state.fixed_swing_shape_strategy,
            crate::robot::motion::SwingShapeStrategy::Synchronized,
            "동기화형(기존)",
        );
        ui.radio_value(
            &mut ui_state.fixed_swing_shape_strategy,
            crate::robot::motion::SwingShapeStrategy::Staggered,
            "채찍형(신규)",
        );
    });
```

and the corresponding sync-back line in `draw()` next to `ctrl.fixed_swing_impact_strategy = ...`:

```rust
        ctrl.fixed_swing_shape_strategy = ui_state.fixed_swing_shape_strategy;
```

- [ ] **Step 7: Build and run the full test suite**

Run: `cargo build -p pingpong-bot 2>&1 | tail -30`
Expected: clean build.

Run: `cargo test -p pingpong-bot --lib 2>&1 | tail -20`
Expected: the same 7 pre-existing failures, plus all new tests passing, no new failures.

- [ ] **Step 8: Measure and report the actual improvement**

Write a short-lived diagnostic (or reuse the `staggered_shape_reaches_a_higher_peak_racket_speed_than_synchronized` test's own numbers via `--nocapture` on a temporary `println!`/`panic!`, then remove it before committing) to report the actual peak-speed numbers for both shapes on `defaults::robot()` at `rail.default_x()`. Include these numbers in your report — this is the direct answer to "did the fix work," parallel to Task 3b's duration/impact-time numbers.

- [ ] **Step 9: Manual GUI comparison (best-effort, report what you can and cannot verify)**

Same caveat as prior GUI tasks: launch briefly to confirm no panic in either radio-button state; you cannot click/interact with the window yourself. Report plainly that the live "does it feel like more force" comparison needs the user's own hands-on check.

- [ ] **Step 10: Commit**

```bash
git add src/robot/motion/fixed_swing.rs src/robot/motion/mod.rs src/robot/motion/planner.rs \
        src/sim/physics/world.rs src/real/fixed_swing_worker.rs \
        src/sim/session/controls.rs src/sim/session/session.rs \
        src/sim/gui/viewer/panel_ui_state.rs src/sim/gui/viewer/panel.rs
git commit -m "$(cat <<'EOF'
feat(robot): add a staggered (whip-style) fixed swing shape

The synchronized quintic shape (all 4 joints sharing one timeline) had
the ball-return direction almost exactly right (96-100% forward-aligned,
measured) but a low peak racket speed (0.879 m/s) despite large joint
excursions, because every joint hit its own speed limit at the same
instant instead of building sequential momentum. Add
SwingShapeStrategy::Staggered: proximal joints (yaw/shoulder) move first
and stop, distal joints (elbow/wrist) start later and snap through near
impact — a kinetic-chain profile, built on Task 5b's per-joint phase
offsets. Own torque/speed feasibility search (not physics.rs's
shared-local-time checks, which assume no phase offsets exist). Both
shapes are compared live via a new GUI selector, alongside Task 3b's
impact-time selector.
EOF
)"
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
        // Task 5c: 관절 타이밍 모양(Synchronized/Staggered)이 추가돼
        // plan_fixed_swing이 세 번째 인자를 받는다 — 기본값(Staggered)을 쓴다.
        Kind::FixedSwing => motion::Planner::plan_fixed_swing(
            arm,
            start.rail_x,
            pingpong_bot::robot::motion::DEFAULT_SWING_SHAPE_STRATEGY,
        )
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
