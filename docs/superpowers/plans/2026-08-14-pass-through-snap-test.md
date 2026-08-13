# Pass-Through Snap Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone real-hardware diagnostic tool that drives j0/j1/j2 toward an overshoot target (past the ball's real contact point) while j3 does an active backswing-then-snap timed to finish exactly at the estimated contact instant — all four joints moving concurrently — so the motion can be physically observed and tuned before any production integration decision.

**Architecture:** A new `tools/pass_through_snap_test` binary crate (same pattern as `tools/measure_joint_speed`), split into pure/testable planning logic (`geometry.rs`, `wrist_motion.rs`, `plan.rs` — all unit-testable without hardware, using `pingpong_bot::defaults::robot()`'s software arm model) and a thin hardware-glue layer (`run.rs`/`main.rs`/`args.rs`) that reads the real joint state, builds the plan, confirms with the user, streams it over `DynamixelBus`, and reports measured results. No changes to the production planner or control loop.

**Tech Stack:** Rust, cargo test / cargo build / cargo clippy, `pingpong-bot` library with `features = ["real"]`.

**Spec:** `docs/superpowers/specs/2026-08-14-pass-through-snap-test-design.md`

## Global Constraints

- Do not modify `src/robot/motion/physics.rs`, `src/robot/motion/trajectory.rs`, or `src/real/control_worker.rs` — this tool is additive and self-contained.
- Reuse existing public library primitives (`RampCruiseSegment`, `QuadraticSegment`, `DelayedQuadraticSegment`, `Arm::inverse_pose_at_fixed_rail_best_normal`, `Arm::wrist_joint_index`, `Arm::joint_limit`) rather than duplicating their internal math.
- The tool targets exactly the 4-DOF competition arm (`pingpong_bot::defaults::robot()`); joint indices 0/1/2 are hardcoded as j0(yaw)/j1(shoulder)/j2(elbow) per existing convention (`POWER_SWEEP_JOINT_INDICES` in `physics.rs`), and the wrist index comes from `arm.wrist_joint_index()`.
- Any infeasibility (timing, reach) must be reported in plain terms (what was required vs. what was available) and must stop the tool **before** any hardware command is sent — never partially command an infeasible motion.
- Follow the existing tool safety pattern: require a typed `y` confirmation, printing a full summary of the computed motion, before commanding real hardware.

---

### Task 1: Scaffold the crate + overshoot geometry

**Files:**
- Create: `tools/pass_through_snap_test/Cargo.toml`
- Create: `tools/pass_through_snap_test/src/args.rs`
- Create: `tools/pass_through_snap_test/src/main.rs`
- Create: `tools/pass_through_snap_test/src/geometry.rs`

**Interfaces:**
- Produces: `pub fn overshoot_target(target: pingpong_bot::Point3, overshoot_m: f64) -> (pingpong_bot::Point3, nalgebra::Vector3<f64>)` — returns `(overshoot_position, push_direction)`. Task 3 (`plan.rs`) calls this.
- Produces: `pub struct Args` (clap `Parser`) with fields: `dxl_port: Option<String>`, `target_x/target_y/target_z: f64`, `overshoot_m: f64` (default 0.05), `total_duration_secs: f64`, `impact_time_secs: f64`, `wrist_cocked_deg: f64`, `backswing_duration_secs: f64`, `ramp_secs: f64` (default 0.060), `snap_velocity_margin: f64` (default 0.85), `poll_hz: f64` (default 200.0). Tasks 3 and 4 consume these fields.

- [ ] **Step 1: Create the crate scaffold**

`tools/pass_through_snap_test/Cargo.toml`:

```toml
[package]
name = "pass-through-snap-test"
edition.workspace = true
version.workspace = true

[[bin]]
name = "pass_through_snap_test"
path = "src/main.rs"

[dependencies]
anyhow = "1.0.103"
clap = { version = "4.6.1", features = ["derive"] }
nalgebra = "0.35.0"
pingpong-bot = { path = "../..", default-features = false, features = ["real"] }
```

`tools/pass_through_snap_test/src/args.rs`:

```rust
//! CLI 인자.

use clap::Parser;

#[derive(Parser)]
#[command(
    name = "pass_through_snap_test",
    about = "j0/j1/j2를 임팩트 지점 너머(overshoot)로 밀면서, j3는 접었다가(backswing) 임팩트 시각에 맞춰 한계까지 스냅하는 동작을 실기에서 격리 테스트한다"
)]
pub struct Args {
    /// Dynamixel 포트 오버라이드 (`DynamixelConfig::default().port`보다 우선).
    #[arg(long)]
    pub dxl_port: Option<String>,

    /// 목표 접촉점(라켓 중심이 임팩트 순간 있어야 할 위치) [m], 월드 프레임.
    #[arg(long, allow_hyphen_values = true)]
    pub target_x: f64,
    #[arg(long, allow_hyphen_values = true)]
    pub target_y: f64,
    #[arg(long, allow_hyphen_values = true)]
    pub target_z: f64,

    /// 목표 접촉점 너머로 IK 목표를 얼마나 더 밀어둘지 [m].
    #[arg(long, default_value_t = 0.05)]
    pub overshoot_m: f64,

    /// j0/j1/j2가 overshoot 목표에 도달하는 전체 시간 [s].
    #[arg(long)]
    pub total_duration_secs: f64,

    /// 라켓이 실제 목표 접촉점을 지나는(공을 맞히는) 추정 시각 [s] — `total_duration_secs`보다 작아야 한다.
    #[arg(long)]
    pub impact_time_secs: f64,

    /// 손목이 접히는 목표 각도 [deg] (절대각).
    #[arg(long, allow_hyphen_values = true)]
    pub wrist_cocked_deg: f64,

    /// 손목이 접힌 각도까지 도달하는 시간 [s].
    #[arg(long)]
    pub backswing_duration_secs: f64,

    /// j0/j2가 정지에서 관절 속도 상한까지 가속하는 데 쓰는 시간 [s].
    #[arg(long, default_value_t = 0.060)]
    pub ramp_secs: f64,

    /// 손목 스냅이 노리는 속도를 관절 속도 상한의 이 비율까지로 제한한다 [무차원].
    #[arg(long, default_value_t = 0.85)]
    pub snap_velocity_margin: f64,

    /// 스트리밍 주기 [Hz].
    #[arg(long, default_value_t = 200.0)]
    pub poll_hz: f64,
}
```

`tools/pass_through_snap_test/src/main.rs`:

```rust
//! j0/j1/j2 overshoot + j3 backswing-스냅 격리 테스트 — 독립 실행형.

mod args;
mod geometry;
mod plan;
mod run;
mod wrist_motion;

use anyhow::Result;
use clap::Parser;

use args::Args;

fn main() -> Result<()> {
    let args = Args::parse();
    return run::run(&args);
}
```

Create placeholder `tools/pass_through_snap_test/src/plan.rs`, `tools/pass_through_snap_test/src/wrist_motion.rs`, `tools/pass_through_snap_test/src/run.rs` with just `//! TODO(task N)` doc comments for now, so `main.rs`'s `mod` declarations resolve — Tasks 2-4 fill them in. (This crate will not compile end-to-end until Task 4 is done; that's expected and fine — Tasks 1-2's tests run via `cargo test -p pass-through-snap-test --lib` scoped to individual files is not possible in Rust, so instead each task's tests are verified via the full crate once enough of it exists to compile. See the note in Task 2.)

- [ ] **Step 2: Write the failing test for overshoot geometry**

Create `tools/pass_through_snap_test/src/geometry.rs`:

```rust
//! Overshoot 목표 위치·방향 계산.

use nalgebra::Vector3;
use pingpong_bot::Point3;
use pingpong_bot::constants::table::{OPPONENT_HALF_CENTER_Y, WIDTH_X};

/// 목표 접촉점에서 상대 탁구대 중앙을 향하는 수평 단위벡터(`push_direction`)와,
/// 그 방향으로 `overshoot_m`만큼 더 나아간 지점(`overshoot_position`)을 계산한다.
/// `physics.rs`의 `ball_alignment_geometry`와 같은 방향 공식이지만, 공/라켓
/// 두께 오프셋 없이 목표점을 그대로 라켓 중심으로 쓴다(이 도구는 모션의
/// 모양을 보는 것이 목적이라 접촉 기하를 간략화했다).
pub fn overshoot_target(target: Point3, overshoot_m: f64) -> (Point3, Vector3<f64>) {
    let toward_opponent_center =
        Vector3::new(WIDTH_X * 0.5 - target.x, OPPONENT_HALF_CENTER_Y - target.y, 0.0);
    let push_direction = if toward_opponent_center.norm_squared() > 1e-12 {
        toward_opponent_center.normalize()
    } else {
        Vector3::y()
    };
    let overshoot_position = Point3::from(target.coords + push_direction * overshoot_m);
    return (overshoot_position, push_direction);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overshoot_position_moves_along_push_direction_by_the_requested_distance() {
        let target = Point3::new(WIDTH_X * 0.5, 1.0, 0.9);
        let (overshoot_position, push_direction) = overshoot_target(target, 0.05);
        assert!((push_direction.norm() - 1.0).abs() < 1e-9);
        let moved = overshoot_position - target;
        assert!((moved.norm() - 0.05).abs() < 1e-6);
        assert!(
            (moved.normalize() - push_direction).norm() < 1e-6,
            "overshoot should move exactly along push_direction"
        );
    }

    #[test]
    fn push_direction_points_toward_the_opponent_half() {
        // 목표점이 상대 탁구대 중앙(y가 더 큰 쪽)보다 가까운 쪽에 있다고 가정하면,
        // push_direction의 y 성분은 양수여야 한다(상대편 쪽으로 민다).
        let target = Point3::new(WIDTH_X * 0.5, 0.2, 0.9);
        let (_, push_direction) = overshoot_target(target, 0.05);
        assert!(
            push_direction.y > 0.0,
            "push_direction should point toward the opponent half: {push_direction:?}"
        );
    }

    #[test]
    fn zero_overshoot_leaves_the_position_unchanged() {
        let target = Point3::new(WIDTH_X * 0.3, 0.8, 0.95);
        let (overshoot_position, _) = overshoot_target(target, 0.0);
        assert!((overshoot_position - target).norm() < 1e-9);
    }
}
```

- [ ] **Step 3: Attempt to run the test and confirm the crate does not yet build (expected)**

Run: `cargo test -p pass-through-snap-test 2>&1 | tail -30`
Expected: build errors from the placeholder `plan.rs`/`wrist_motion.rs`/`run.rs` files (empty modules referenced by `main.rs` but not yet implemented enough to satisfy `mod` resolution, or simply "unused" warnings if left as empty files — either way, this is expected at this point). This step is just confirming the failure mode is the placeholders, not a mistake in `geometry.rs` itself. If the only errors are about the other three files being empty/unused, proceed to Step 4; if there's an error inside `geometry.rs` itself, fix that first.

- [ ] **Step 4: Make the placeholders minimally valid so `geometry.rs`'s tests can run in isolation**

Give each placeholder file trivial valid Rust so the crate compiles far enough for `cargo test` to reach `geometry.rs`'s tests:

`tools/pass_through_snap_test/src/wrist_motion.rs`:
```rust
//! 손목(j3) 접기(backswing) → 대기 → 스냅 3단계 모션. (Task 2에서 구현)
```

`tools/pass_through_snap_test/src/plan.rs`:
```rust
//! j0~j3 전체 모션 계획. (Task 3에서 구현)
```

`tools/pass_through_snap_test/src/run.rs`:
```rust
//! 실기 연결·확인·스트리밍·리포트. (Task 4에서 구현)

use anyhow::Result;

use crate::args::Args;

pub fn run(_args: &Args) -> Result<()> {
    anyhow::bail!("not yet implemented (Task 4)");
}
```

Run: `cargo test -p pass-through-snap-test 2>&1 | tail -40`
Expected: PASS for all of `geometry.rs`'s 3 tests (the crate now compiles: `main.rs`'s `run::run` exists even though it just bails, and `wrist_motion`/`plan` are valid empty modules).

- [ ] **Step 5: Commit**

```bash
git add tools/pass_through_snap_test/
git commit -m "feat(tools): scaffold pass_through_snap_test and add overshoot geometry"
```

---

### Task 2: Wrist backswing-then-snap motion

**Files:**
- Modify: `tools/pass_through_snap_test/src/wrist_motion.rs`

**Interfaces:**
- Consumes: `pingpong_bot::robot::JointLimit` (fields `min`/`max`), `pingpong_bot::robot::motion::quadratic_segment::{QuadraticSegment, DelayedQuadraticSegment}` (both already `pub`, reached via full path since not re-exported at `motion::` root).
- Produces: `pub fn snap_target(current: f64, cocked: f64, limit: JointLimit) -> f64`, `pub fn snap_duration(cocked: f64, target: f64, max_joint_speed: f64, margin: f64) -> f64`, `pub struct WristMotion` with `pub fn try_new(current: f64, cocked: f64, limit: JointLimit, backswing_secs: f64, impact_time_secs: f64, total_duration_secs: f64, max_joint_speed: f64, snap_velocity_margin: f64) -> Result<Self, String>`, `pub fn sample(&self, t: f64) -> (f64, f64, f64)`, `pub fn snap_target_angle(&self) -> f64`, `pub fn peak_speed(&self, samples: usize) -> f64`. Task 3 (`plan.rs`) constructs and samples `WristMotion`.

- [ ] **Step 1: Write the failing tests**

Replace `tools/pass_through_snap_test/src/wrist_motion.rs` with:

```rust
//! 손목(j3) 접기(backswing) → 대기 → 스냅 3단계 모션.
//!
//! 접기 구간(A)과 대기+스냅 구간(B)의 경계에서 위치는 반드시 이어지지만
//! (둘 다 `cocked` 각도에서 만난다), 속도는 이어지지 않을 수 있다 — A는
//! 정지에서 출발해 `cocked`에서 끝나는 등가속(끝 속도가 일반적으로
//! 0이 아니다)이고, B는 `cocked`에서 정지 상태로 대기를 시작하기 때문이다.
//! 이 도구는 정확한 부드러움보다 물리적으로 느껴보는 것이 목적이라, 이
//! 불연속은 실기 서보의 위치 추종(PID)이 짧은 지연으로 흡수할 것으로
//! 보고 의도적으로 받아들인다.

use pingpong_bot::robot::JointLimit;
use pingpong_bot::robot::motion::quadratic_segment::{DelayedQuadraticSegment, QuadraticSegment};

/// 목표(스냅) 각도 — 접힌 자세(`cocked`)의 반대편 한계를 고른다. `cocked`가
/// `current`보다 작으면(min 쪽으로 접었으면) 반대쪽인 `limit.max`가 목표다.
pub fn snap_target(current: f64, cocked: f64, limit: JointLimit) -> f64 {
    if cocked < current {
        return limit.max;
    }
    return limit.min;
}

/// 목표 각속도(관절 속도 상한 × `margin`)로 스냅을 끝내는 데 걸리는 최소 시간 —
/// 정지에서 등가속으로 `|target - cocked|`를 덮는 데 걸리는 시간의 절반 공식
/// (`2·Δq / v_target`, `v_target`이 등가속 평균 속도의 2배라는 사실에서 유도).
pub fn snap_duration(cocked: f64, target: f64, max_joint_speed: f64, margin: f64) -> f64 {
    let target_speed = (max_joint_speed * margin).max(f64::EPSILON);
    return 2.0 * (target - cocked).abs() / target_speed;
}

/// 손목 3단계 모션: 접기(A, `[0, backswing_secs]`) → 대기+스냅(B,
/// `[backswing_secs, impact_time_secs]`) → 스냅 뒤 유지(C,
/// `[impact_time_secs, total_duration_secs]`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WristMotion {
    current: f64,
    cocked: f64,
    target: f64,
    backswing_secs: f64,
    hold_secs: f64,
    phase_b_duration: f64,
    impact_time_secs: f64,
    total_duration_secs: f64,
}

impl WristMotion {
    /// 스냅이 `impact_time_secs` 전에 다 끝나는지 확인하며 만든다 — 안 끝나면
    /// 부족한 시간을 담은 에러 문자열을 반환한다.
    #[allow(clippy::too_many_arguments)]
    pub fn try_new(
        current: f64,
        cocked: f64,
        limit: JointLimit,
        backswing_secs: f64,
        impact_time_secs: f64,
        total_duration_secs: f64,
        max_joint_speed: f64,
        snap_velocity_margin: f64,
    ) -> Result<Self, String> {
        let target = snap_target(current, cocked, limit);
        let duration = snap_duration(cocked, target, max_joint_speed, snap_velocity_margin);
        let phase_b_duration = impact_time_secs - backswing_secs;
        let hold_secs = phase_b_duration - duration;
        if hold_secs < 0.0 {
            return Err(format!(
                "wrist snap needs {duration:.4}s but only {phase_b_duration:.4}s is available \
                 between backswing end ({backswing_secs:.4}s) and impact ({impact_time_secs:.4}s) \
                 -- shorten backswing_duration_secs or push impact_time_secs later"
            ));
        }
        return Ok(Self {
            current,
            cocked,
            target,
            backswing_secs,
            hold_secs,
            phase_b_duration,
            impact_time_secs,
            total_duration_secs,
        });
    }

    pub fn snap_target_angle(&self) -> f64 {
        return self.target;
    }

    /// `t`[s]에서 (각도, 각속도, 각가속도)를 샘플한다. `[0, total_duration_secs]`
    /// 밖은 clamp.
    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        let t = t.clamp(0.0, self.total_duration_secs);
        if t <= self.backswing_secs {
            return QuadraticSegment::new(self.current, 0.0, self.cocked, self.backswing_secs)
                .sample(t);
        }
        if t <= self.impact_time_secs {
            return DelayedQuadraticSegment::new(
                self.cocked,
                self.target,
                self.phase_b_duration,
                self.hold_secs,
            )
            .sample(t - self.backswing_secs);
        }
        return (self.target, 0.0, 0.0);
    }

    /// 전 구간 최대 |각속도| [rad/s] — 표시용.
    pub fn peak_speed(&self, samples: usize) -> f64 {
        let n = samples.max(2);
        let mut peak = 0.0_f64;
        for i in 0..=n {
            let t = self.total_duration_secs * (i as f64) / (n as f64);
            peak = peak.max(self.sample(t).1.abs());
        }
        return peak;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limit() -> JointLimit {
        return JointLimit::new(-1.5, 1.5);
    }

    #[test]
    fn snap_target_picks_the_limit_opposite_the_cocked_side() {
        assert_eq!(snap_target(0.0, -0.5, limit()), 1.5);
        assert_eq!(snap_target(0.0, 0.5, limit()), -1.5);
    }

    #[test]
    fn snap_duration_matches_hand_computed_formula() {
        // target_speed = 5.0*0.8=4.0, |target-cocked|=2.0 -> duration = 2*2.0/4.0 = 1.0
        let duration = snap_duration(-1.0, 1.0, 5.0, 0.8);
        assert!((duration - 1.0).abs() < 1e-9, "duration={duration}");
    }

    #[test]
    fn try_new_rejects_when_snap_does_not_fit_before_impact() {
        // cocked=-1.0, target=1.5 (limit.max), |Δ|=2.5, at max_joint_speed=5.0*margin=1.0
        // -> target_speed=5.0, duration=2*2.5/5.0=1.0s. backswing=0.05s, impact_time=0.10s
        // -> phase_b_duration=0.05s < duration(1.0s) -> infeasible.
        let result = WristMotion::try_new(0.0, -1.0, limit(), 0.05, 0.10, 0.20, 5.0, 1.0);
        assert!(result.is_err(), "expected infeasible, got {result:?}");
    }

    fn feasible_motion() -> WristMotion {
        // cocked=-0.5, current=0.0 -> target=limit.max=1.5, |Δ|=2.0.
        // margin*max_speed = 5.0*0.8=4.0 -> duration=2*2.0/4.0=1.0s.
        // backswing=0.2s, impact_time=1.5s -> phase_b_duration=1.3s, hold=0.3s >= 0. OK.
        return WristMotion::try_new(0.0, -0.5, limit(), 0.2, 1.5, 2.0, 5.0, 0.8)
            .expect("feasible by construction");
    }

    #[test]
    fn sample_reaches_cocked_angle_at_the_backswing_boundary() {
        let motion = feasible_motion();
        let (angle, _, _) = motion.sample(0.2);
        assert!((angle - -0.5).abs() < 1e-6, "angle={angle}");
    }

    #[test]
    fn sample_reaches_snap_target_at_impact_time() {
        let motion = feasible_motion();
        let (angle, _, _) = motion.sample(1.5);
        assert!((angle - 1.5).abs() < 1e-6, "angle={angle}");
    }

    #[test]
    fn sample_holds_target_after_impact_time() {
        let motion = feasible_motion();
        let (angle, velocity, _) = motion.sample(2.0);
        assert!((angle - 1.5).abs() < 1e-6, "angle={angle}");
        assert!(velocity.abs() < 1e-9, "velocity={velocity}");
    }

    #[test]
    fn position_is_continuous_across_the_backswing_boundary() {
        let motion = feasible_motion();
        let dt = 1e-6;
        let before = motion.sample(0.2 - dt).0;
        let after = motion.sample(0.2 + dt).0;
        assert!((before - after).abs() < 1e-3, "before={before} after={after}");
    }

    #[test]
    fn peak_speed_is_finite_and_positive_for_a_feasible_motion() {
        let motion = feasible_motion();
        let peak = motion.peak_speed(50);
        assert!(peak.is_finite());
        assert!(peak > 0.0);
    }
}
```

- [ ] **Step 2: Run to verify the tests fail first (before this step's implementation existed, i.e. verify the test file compiles and the assertions are meaningful)**

Since this replaces the placeholder in one shot, instead verify correctness by temporarily breaking one assertion (e.g. change the expected value in `snap_duration_matches_hand_computed_formula` to `2.0` instead of `1.0`) and confirming it fails, then revert. This substitutes for a true red step since the implementation was written together with the tests above.

Run: `cargo test -p pass-through-snap-test wrist_motion -- --nocapture 2>&1 | tail -20`
Expected after the deliberate break: FAIL on that one test. After reverting the deliberate break: all pass (see Step 3).

- [ ] **Step 3: Run all wrist_motion tests and verify they pass**

Run: `cargo test -p pass-through-snap-test wrist_motion 2>&1 | tail -20`
Expected: all 7 tests PASS.

- [ ] **Step 4: Commit**

```bash
git add tools/pass_through_snap_test/src/wrist_motion.rs
git commit -m "feat(tools): add wrist backswing-then-snap motion to pass_through_snap_test"
```

---

### Task 3: Full 4-joint plan (pure, testable with the software arm model)

**Files:**
- Modify: `tools/pass_through_snap_test/src/plan.rs`

**Interfaces:**
- Consumes: `geometry::overshoot_target` (Task 1), `wrist_motion::WristMotion` (Task 2), `pingpong_bot::robot::motion::{quadratic_segment::QuadraticSegment, ramp_cruise_segment::RampCruiseSegment}`, `pingpong_bot::robot::{Arm, IkSearch, Joints, Pose}`, `pingpong_bot::Point3`.
- Produces: `pub struct SwingPlan` with `pub fn build(arm: &Arm, current: &Joints, target: Point3, overshoot_m: f64, total_duration_secs: f64, impact_time_secs: f64, wrist_cocked_rad: f64, backswing_secs: f64, ramp_secs: f64, snap_velocity_margin: f64) -> Result<Self, String>` and `pub fn sample(&self, t: f64) -> Joints`. Task 4 (`run.rs`) calls `build` and `sample`.

- [ ] **Step 1: Write the failing tests**

Replace `tools/pass_through_snap_test/src/plan.rs` with:

```rust
//! j0~j3 전체 모션 계획 — overshoot 목표로 가는 j0/j1/j2와 backswing-스냅
//! j3를 하나의 시간축에 묶는다. 하드웨어 접근이 없어 실기 없이도
//! `pingpong_bot::defaults::robot()`의 소프트웨어 팔 모델로 테스트할 수 있다.

use pingpong_bot::Point3;
use pingpong_bot::robot::motion::quadratic_segment::QuadraticSegment;
use pingpong_bot::robot::motion::ramp_cruise_segment::RampCruiseSegment;
use pingpong_bot::robot::{Arm, IkSearch, Joints, Pose};

use crate::geometry::overshoot_target;
use crate::wrist_motion::WristMotion;

/// j0·j2 인덱스 — 이 팔에서 토크가 가장 큰 두 관절, 생산 코드의
/// `POWER_SWEEP_JOINT_INDICES`와 같은 관례.
const POWER_JOINT_INDICES: [usize; 2] = [0, 2];
/// 어깨(j1) 인덱스 — IK가 요구하는 만큼만 따라가는 수동 관절.
const PASSIVE_JOINT_INDEX: usize = 1;

#[derive(Debug, Clone, Copy, PartialEq)]
enum JointProfile {
    RampCruise(RampCruiseSegment),
    Quadratic(QuadraticSegment),
}

impl JointProfile {
    fn sample(&self, t: f64) -> (f64, f64, f64) {
        return match self {
            JointProfile::RampCruise(segment) => segment.sample(t),
            JointProfile::Quadratic(segment) => segment.sample(t),
        };
    }
}

pub struct SwingPlan {
    /// `PASSIVE_JOINT_INDEX`를 포함해 j0~j2 전부(손목 제외) 담는다 — 인덱스로
    /// 직접 접근한다.
    arm_profiles: Vec<JointProfile>,
    wrist_index: usize,
    wrist: WristMotion,
    overshoot_joints: Joints,
    total_duration_secs: f64,
}

impl SwingPlan {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        arm: &Arm,
        current: &Joints,
        target: Point3,
        overshoot_m: f64,
        total_duration_secs: f64,
        impact_time_secs: f64,
        wrist_cocked_rad: f64,
        backswing_secs: f64,
        ramp_secs: f64,
        snap_velocity_margin: f64,
    ) -> Result<Self, String> {
        if impact_time_secs >= total_duration_secs {
            return Err(format!(
                "impact_time_secs ({impact_time_secs:.4}) must be less than total_duration_secs \
                 ({total_duration_secs:.4}) -- the overshoot target must be reached after the \
                 real contact instant"
            ));
        }
        let rail_x = arm.rail.as_ref().map_or(0.0, |rail| rail.default_x());
        let (overshoot_position, push_direction) = overshoot_target(target, overshoot_m);
        let hint = Pose::new(rail_x, current.clone());
        let (overshoot_pose, _) = arm
            .inverse_pose_at_fixed_rail_best_normal(
                rail_x,
                overshoot_position,
                push_direction,
                &hint,
                IkSearch::Global,
            )
            .map_err(|error| format!("overshoot target unreachable: {error:?}"))?;
        let overshoot_joints = overshoot_pose.joints;

        let ramp_accel = arm.max_joint_speed / ramp_secs.max(f64::EPSILON);
        let mut arm_profiles = Vec::with_capacity(3);
        for index in 0..3 {
            if POWER_JOINT_INDICES.contains(&index) {
                let segment = RampCruiseSegment::new(
                    current.values[index],
                    overshoot_joints.values[index],
                    total_duration_secs,
                    ramp_accel,
                )
                .ok_or_else(|| {
                    format!(
                        "j{index} cannot reach its overshoot target ({:.4} -> {:.4}) within \
                         total_duration_secs={total_duration_secs:.4} even at the joint speed \
                         ceiling -- shorten overshoot_m or lengthen total_duration_secs",
                        current.values[index], overshoot_joints.values[index]
                    )
                })?;
                arm_profiles.push(JointProfile::RampCruise(segment));
            } else {
                debug_assert_eq!(index, PASSIVE_JOINT_INDEX);
                let segment = QuadraticSegment::new(
                    current.values[index],
                    0.0,
                    overshoot_joints.values[index],
                    total_duration_secs,
                );
                arm_profiles.push(JointProfile::Quadratic(segment));
            }
        }

        let wrist_index = arm
            .wrist_joint_index()
            .ok_or_else(|| "arm has no wrist joint".to_string())?;
        let limit = arm
            .joint_limit(wrist_index)
            .ok_or_else(|| "wrist joint has no configured limit".to_string())?;
        let wrist = WristMotion::try_new(
            current.values[wrist_index],
            wrist_cocked_rad,
            limit,
            backswing_secs,
            impact_time_secs,
            total_duration_secs,
            arm.max_joint_speed,
            snap_velocity_margin,
        )?;

        return Ok(Self {
            arm_profiles,
            wrist_index,
            wrist,
            overshoot_joints,
            total_duration_secs,
        });
    }

    pub fn overshoot_joints(&self) -> &Joints {
        return &self.overshoot_joints;
    }

    pub fn wrist_snap_target_angle(&self) -> f64 {
        return self.wrist.snap_target_angle();
    }

    pub fn wrist_peak_speed(&self, samples: usize) -> f64 {
        return self.wrist.peak_speed(samples);
    }

    pub fn total_duration_secs(&self) -> f64 {
        return self.total_duration_secs;
    }

    /// `t`[s]에서 4관절 전체 각도를 샘플한다.
    pub fn sample(&self, t: f64) -> Joints {
        let mut values = vec![0.0; 4];
        for index in 0..3 {
            values[index] = self.arm_profiles[index].sample(t).0;
        }
        values[self.wrist_index] = self.wrist.sample(t).0;
        return Joints { values };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::constants::table;

    fn sample_target() -> Point3 {
        return Point3::new(table::WIDTH_X * 0.5, 0.3, 0.95);
    }

    #[test]
    fn build_succeeds_for_a_reasonable_center_table_target() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            0.30,
            0.20,
            arm.joint_limit(3).expect("wrist limit").min * 0.5,
            0.05,
            0.060,
            0.85,
        );
        assert!(plan.is_ok(), "expected feasible plan, got {plan:?}");
    }

    #[test]
    fn build_rejects_when_impact_time_is_not_before_total_duration() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            0.20,
            0.20,
            -0.2,
            0.05,
            0.060,
            0.85,
        );
        assert!(plan.is_err());
    }

    #[test]
    fn build_rejects_when_ramp_cruise_is_infeasible() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        // 아주 짧은 총 시간 + 큰 overshoot로 j0/j2 도달 불가능을 강제한다.
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.50,
            0.02,
            0.01,
            -0.2,
            0.005,
            0.060,
            0.85,
        );
        assert!(plan.is_err());
    }

    #[test]
    fn build_rejects_when_wrist_snap_does_not_fit() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let wrist_limit = arm.joint_limit(3).expect("wrist limit");
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            0.30,
            0.11,
            wrist_limit.min * 0.9,
            0.10,
            0.060,
            0.85,
        );
        assert!(plan.is_err());
    }

    #[test]
    fn sample_at_zero_matches_current_joints() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            0.30,
            0.20,
            arm.joint_limit(3).expect("wrist limit").min * 0.5,
            0.05,
            0.060,
            0.85,
        )
        .expect("feasible plan");
        let sampled = plan.sample(0.0);
        for index in 0..4 {
            assert!(
                (sampled.values[index] - current.values[index]).abs() < 1e-6,
                "joint {index} mismatch at t=0: {} vs {}",
                sampled.values[index],
                current.values[index]
            );
        }
    }

    #[test]
    fn sample_at_total_duration_reaches_overshoot_joints_for_arm_joints() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            0.30,
            0.20,
            arm.joint_limit(3).expect("wrist limit").min * 0.5,
            0.05,
            0.060,
            0.85,
        )
        .expect("feasible plan");
        let sampled = plan.sample(0.30);
        let overshoot = plan.overshoot_joints();
        for index in 0..3 {
            assert!(
                (sampled.values[index] - overshoot.values[index]).abs() < 1e-6,
                "joint {index} should reach its overshoot target at total_duration_secs"
            );
        }
    }

    #[test]
    fn sample_wrist_reaches_snap_target_at_impact_time() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            0.30,
            0.20,
            arm.joint_limit(3).expect("wrist limit").min * 0.5,
            0.05,
            0.060,
            0.85,
        )
        .expect("feasible plan");
        let sampled = plan.sample(0.20);
        assert!(
            (sampled.values[3] - plan.wrist_snap_target_angle()).abs() < 1e-6,
            "wrist should reach its snap target exactly at impact_time_secs"
        );
    }
}
```

- [ ] **Step 2: Run the tests**

Run: `cargo test -p pass-through-snap-test plan:: 2>&1 | tail -40`
Expected: all 7 tests PASS. If `build_succeeds_for_a_reasonable_center_table_target` or the two `sample_*` tests fail with an IK or ramp-cruise error, the specific numeric literals chosen for `sample_target()`/`wrist_cocked_rad`/timings may not be feasible for the real arm model — adjust the magnitude of `wrist_cocked_rad` (currently `wrist_limit.min * 0.5`, a moderate cock) or `overshoot_m` slightly and re-run; do not weaken the assertions themselves.

- [ ] **Step 3: Commit**

```bash
git add tools/pass_through_snap_test/src/plan.rs
git commit -m "feat(tools): add full 4-joint SwingPlan to pass_through_snap_test"
```

---

### Task 4: Hardware execution (confirm, stream, report)

**Files:**
- Modify: `tools/pass_through_snap_test/src/run.rs`

**Interfaces:**
- Consumes: `plan::SwingPlan::{build, sample, overshoot_joints, wrist_snap_target_angle, wrist_peak_speed, total_duration_secs}` (Task 3), `pingpong_bot::hardware::dynamixel::{DynamixelBus, DynamixelConfig}`, `args::Args` (Task 1).
- Produces: `pub fn run(args: &Args) -> anyhow::Result<()>` — called from `main.rs` (already wired in Task 1).

- [ ] **Step 1: Implement the hardware glue**

Replace `tools/pass_through_snap_test/src/run.rs` with:

```rust
//! 실기 연결·확인·스트리밍·리포트.

use std::io::{self, Write};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use pingpong_bot::Point3;
use pingpong_bot::hardware::dynamixel::{DynamixelBus, DynamixelConfig};
use pingpong_bot::robot::Joints;

use crate::args::Args;
use crate::plan::SwingPlan;

pub fn run(args: &Args) -> Result<()> {
    let config = build_config(args);
    println!("포트={} — pass-through/wrist-snap 격리 테스트 시작", config.port);
    let mut bus = DynamixelBus::open(config).context("Dynamixel 버스 열기 실패")?;

    let active = pingpong_bot::defaults::robot().context("소프트웨어 팔 모델 로드 실패")?;
    let arm = &active.arm;

    let current = bus.read_joints().context("현재 관절각 읽기 실패")?;
    let target = Point3::new(args.target_x, args.target_y, args.target_z);

    let plan = SwingPlan::build(
        arm,
        &current,
        target,
        args.overshoot_m,
        args.total_duration_secs,
        args.impact_time_secs,
        args.wrist_cocked_deg.to_radians(),
        args.backswing_duration_secs,
        args.ramp_secs,
        args.snap_velocity_margin,
    )
    .map_err(|error| anyhow::anyhow!("계획 실패: {error}"))?;

    print_summary(&current, &plan);
    confirm()?;

    let samples = stream_and_record(&mut bus, &plan, args.poll_hz)?;
    report(&samples, &current, args.impact_time_secs);
    return Ok(());
}

fn build_config(args: &Args) -> DynamixelConfig {
    let mut config = DynamixelConfig::default();
    if let Some(port) = &args.dxl_port {
        config.port = port.clone();
    }
    return config;
}

fn print_summary(current: &Joints, plan: &SwingPlan) {
    let overshoot = plan.overshoot_joints();
    println!("\n=== 계획 ===");
    for index in 0..4 {
        println!(
            "  j{index}: {:.2}° -> {:.2}°",
            current.values[index].to_degrees(),
            overshoot.values[index].to_degrees()
        );
    }
    println!(
        "  손목 스냅 목표각={:.2}°, 전 구간 첨두 각속도(참고용)={:.4} rad/s",
        plan.wrist_snap_target_angle().to_degrees(),
        plan.wrist_peak_speed(50)
    );
    println!("  총 소요 시간={:.3}s", plan.total_duration_secs());
}

fn confirm() -> Result<()> {
    println!("\n경고: 위 계획대로 4관절을 실제로 동시에 움직입니다.");
    println!("주변에 팔이 부딪힐 물체·사람이 없는지 확인하세요.");
    print!("계속하려면 y 를 입력하고 Enter, 취소하려면 다른 키를 입력하세요: ");
    io::stdout().flush().ok();
    let mut input = String::new();
    io::stdin()
        .read_line(&mut input)
        .context("확인 입력 읽기 실패")?;
    if input.trim().eq_ignore_ascii_case("y") {
        return Ok(());
    }
    bail!("사용자가 취소함 — 이동 명령을 보내지 않고 종료합니다");
}

fn stream_and_record(
    bus: &mut DynamixelBus,
    plan: &SwingPlan,
    poll_hz: f64,
) -> Result<Vec<(f64, Joints)>> {
    let poll_period = Duration::from_secs_f64(1.0 / poll_hz.max(1.0));
    let total_duration = Duration::from_secs_f64(plan.total_duration_secs());
    let start = Instant::now();
    let mut samples = Vec::new();
    loop {
        let elapsed = start.elapsed();
        let target = plan.sample(elapsed.as_secs_f64());
        bus.write_joints(&target).context("스트리밍 명령 실패")?;
        let measured = bus.read_joints().context("스트리밍 중 관절각 읽기 실패")?;
        samples.push((elapsed.as_secs_f64(), measured));
        if elapsed >= total_duration {
            break;
        }
        thread::sleep(poll_period);
    }
    return Ok(samples);
}

fn report(samples: &[(f64, Joints)], start: &Joints, impact_time_secs: f64) {
    println!("\n=== 실측 결과 ===");
    let mut peak_speed = [0.0_f64; 4];
    for window in samples.windows(2) {
        let (t0, q0) = &window[0];
        let (t1, q1) = &window[1];
        let dt = t1 - t0;
        if dt > 1e-6 {
            for index in 0..4 {
                let speed = (q1.values[index] - q0.values[index]).abs() / dt;
                peak_speed[index] = peak_speed[index].max(speed);
            }
        }
    }
    for index in 0..4 {
        println!(
            "  j{index}: 시작={:.2}° 첨두 각속도={:.4} rad/s",
            start.values[index].to_degrees(),
            peak_speed[index]
        );
    }

    let closest = samples
        .iter()
        .min_by(|(t_a, _), (t_b, _)| (t_a - impact_time_secs).abs().total_cmp(&(t_b - impact_time_secs).abs()));
    if let Some((t, joints)) = closest {
        println!(
            "\n임팩트 추정 시각({impact_time_secs:.3}s)에 가장 가까운 실측 표본(t={t:.3}s):"
        );
        for (index, value) in joints.values.iter().enumerate() {
            println!("  j{index}={:.2}°", value.to_degrees());
        }
    }
}
```

- [ ] **Step 2: Build the full crate**

Run: `cargo build -p pass-through-snap-test 2>&1 | tail -40`
Expected: succeeds. Fix any type/borrow errors (e.g. `DynamixelBus`/`Joints`/`Point3` import paths) against the actual library signatures if the compiler disagrees with this plan's assumptions — the library is the source of truth.

- [ ] **Step 3: Run the full test suite for the crate**

Run: `cargo test -p pass-through-snap-test 2>&1 | tail -40`
Expected: all tests from Tasks 1-3 still PASS (this task added no new pure-logic tests, since `run.rs` is hardware glue with no automated test — see the spec's Testing Strategy section).

- [ ] **Step 4: Commit**

```bash
git add tools/pass_through_snap_test/src/run.rs
git commit -m "feat(tools): wire up hardware streaming and reporting for pass_through_snap_test"
```

---

### Task 5: Workspace-wide verification

**Files:** none — verification only.

- [ ] **Step 1: Confirm the new crate is registered in the workspace**

Run: `cargo metadata --no-deps --format-version 1 2>/dev/null | python3 -c "import json,sys; d=json.load(sys.stdin); print([p['name'] for p in d['packages'] if 'pass-through' in p['name']])"`
Expected: `['pass-through-snap-test']`.

- [ ] **Step 2: Run the full crate's tests one more time in isolation**

Run: `cargo test -p pass-through-snap-test 2>&1 | tail -10`
Expected: all tests PASS (17 total across geometry.rs, wrist_motion.rs, plan.rs).

- [ ] **Step 3: Run clippy on the new crate**

Run: `cargo clippy -p pass-through-snap-test 2>&1 | grep -v needless_return | grep -E "^(warning|error)"`
Expected: no output beyond the same pre-existing lint categories already accepted elsewhere in this codebase (see the earlier `measure_joint_speed` tool's precedent — this project does not gate on `clippy -D warnings`). If a genuinely new warning category appears in this crate's own files, fix it.

- [ ] **Step 4: Confirm the main library/binary are untouched**

Run: `cargo test --lib 2>&1 | tail -12` and `cargo test --bin pingpong-bot 2>&1 | tail -8`
Expected: identical pass/fail counts to the established baseline before this plan (7 pre-existing lib failures, 2 pre-existing bin failures, none new) — this plan touches no files under `src/`.

- [ ] **Step 5: No commit needed for this task** — verification only.
