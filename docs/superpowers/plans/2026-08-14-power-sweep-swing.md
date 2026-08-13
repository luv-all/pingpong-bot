# Power-Sweep Swing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the shared-duration push swing with a "power sweep" where j0 (yaw) and j2 (elbow) — the arm's highest-torque joints — drive the impact using a ramp-then-cruise velocity profile that sustains near-ceiling speed across a window (not just an instant), j3 (wrist) stays cocked and snaps in a short burst right before impact, and j1 (shoulder) remains a passive follower.

**Architecture:** Two new one-DOF joint-space primitives (`RampCruiseSegment`, `DelayedQuadraticSegment`) plug into `Trajectory` via a new per-joint profile vector and a new constructor (`with_power_sweep`), so existing quintic/quadratic swings are untouched. A new top-level planner function in `physics.rs` mirrors the existing `plan_fixed_joint_swing_quadratic*` structure (same IK-once, same push-distance bisection) but assembles the new per-joint profiles instead of one shared profile.

**Tech Stack:** Rust, cargo test / cargo build / cargo clippy.

**Spec:** `docs/superpowers/specs/2026-08-14-power-sweep-swing-design.md`

## Global Constraints

- `arm.max_joint_speed` is the joint-speed ceiling (uniform across joints today) — reuse it, do not introduce a new per-joint speed limit.
- `POWER_SWEEP_JOINT_INDICES = [0, 2]` (j0 yaw, j2 elbow) — do not hardcode joint count assumptions beyond the existing `joint_count < 4` guard already present in the sibling functions.
- New constants live in `src/defaults/motion.rs` next to the existing `FIXED_JOINT_SWING_*` constants, same naming/doc-comment style (Korean doc comments, matching the file's existing convention).
- Do not modify `plan_fixed_joint_swing` / `plan_fixed_joint_swing_quadratic` or their existing tests — this is a new, additive swing variant.
- Every new public/`pub(crate)` item needs the same terse Korean-doc-comment style already used throughout `physics.rs` / `trajectory.rs` / `quadratic_segment.rs`.

---

### Task 1: `RampCruiseSegment` primitive

**Files:**
- Create: `src/robot/motion/ramp_cruise_segment.rs`
- Modify: `src/robot/motion/mod.rs` (register the module)
- Test: same file, `#[cfg(test)] mod tests` block at the bottom (matches `quadratic_segment.rs` convention)

**Interfaces:**
- Produces: `pub struct RampCruiseSegment` with `pub fn new(q0: f64, qf: f64, duration: f64, accel: f64) -> Option<Self>`, `pub fn sample(&self, t: f64) -> (f64, f64, f64)`, `pub fn max_speed(&self, samples: usize) -> f64`, `pub fn max_acceleration(&self, samples: usize) -> f64`. Signature/return shape matches `QuadraticSegment` so it can be wrapped the same way.

- [ ] **Step 1: Write the failing tests**

Create `src/robot/motion/ramp_cruise_segment.rs`:

```rust
//! 관절 1축 "가속 후 정속 유지"(ramp-then-cruise) 세그먼트.
//!
//! [`super::quadratic_segment::QuadraticSegment`]는 전 구간 등가속이라 목표
//! 도달 순간(t=T)에만 순간적으로 첨두 속도에 닿는다 — 공 도착 시각 예측이
//! 조금만 어긋나도 라켓이 아직 다 가속하지 못한 상태로 맞을 수 있다. 이
//! 세그먼트는 먼저 고정 가속도로 목표 속도(`v_peak`)까지 가속한 뒤, 남은
//! 시간 동안 그 속도를 그대로 유지(등속 "순항")한다 — 임팩트 시각 T 앞에서
//! 첨두 속도가 유지되는 구간을 만들어 타이밍 오차에 강건하게 한다.
//!
//! `(Δq, T, a)`가 주어지면 `v_peak`는 유일하게 정해진다:
//! `Δq = v_peak·T − v_peak²/(2a)`. 이 이차식의 물리적으로 유효한(더 작은)
//! 해가 `v_peak = a·T − √((a·T)² − 2a·Δq)`다. `|Δq|`가 `a`와 `T`만으로 전
//! 구간을 가속해도 못 미치는 거리(`0.5·a·T²`)보다 크면 해가 없다(`None`).

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RampCruiseSegment {
    q0: f64,
    accel: f64,
    ramp_duration: f64,
    v_peak: f64,
    duration: f64,
}

impl RampCruiseSegment {
    pub fn new(q0: f64, qf: f64, duration: f64, accel: f64) -> Option<Self> {
        todo!()
    }

    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        todo!()
    }

    pub fn max_speed(&self, _samples: usize) -> f64 {
        todo!()
    }

    pub fn max_acceleration(&self, _samples: usize) -> f64 {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reaches_position_and_peak_velocity_boundary_at_duration() {
        // a=10, T=1.0, Δq=4.0 → v_peak=10-sqrt(100-80)=10-sqrt(20)≈5.5279
        let segment = RampCruiseSegment::new(0.0, 4.0, 1.0, 10.0).expect("feasible");
        let (q_end, v_end, _) = segment.sample(1.0);
        assert!((q_end - 4.0).abs() < 1e-9);
        let expected_v_peak = 10.0 - (100.0_f64 - 80.0).sqrt();
        assert!((v_end - expected_v_peak).abs() < 1e-9);
    }

    #[test]
    fn holds_cruise_speed_before_the_end_not_only_at_the_instant() {
        // Sustaining speed across a window is the whole point: velocity at
        // t=duration-0.1 should already equal the peak, not be ramping up.
        let segment = RampCruiseSegment::new(0.0, 4.0, 1.0, 10.0).expect("feasible");
        let v_near_end = segment.sample(0.9).1;
        let v_at_end = segment.sample(1.0).1;
        assert!((v_near_end - v_at_end).abs() < 1e-9, "not cruising: v(0.9)={v_near_end} v(1.0)={v_at_end}");
    }

    #[test]
    fn ramp_phase_is_constant_acceleration_from_rest() {
        let segment = RampCruiseSegment::new(0.0, 4.0, 1.0, 10.0).expect("feasible");
        let (_, v0, a0) = segment.sample(0.0);
        assert!(v0.abs() < 1e-9, "should start at rest");
        assert!((a0 - 10.0).abs() < 1e-9, "should start accelerating at full accel");
    }

    #[test]
    fn degenerates_to_plain_quadratic_at_the_reachability_boundary() {
        // Δq = 0.5*a*T^2 exactly ⇒ v_peak = a*T, ramp fills the whole duration
        // (no cruise phase) — same shape as a plain constant-acceleration
        // segment for the whole duration.
        let (a, t) = (10.0_f64, 1.0_f64);
        let delta = 0.5 * a * t * t;
        let segment = RampCruiseSegment::new(0.0, delta, t, a).expect("boundary is feasible");
        let (_, v_mid, a_mid) = segment.sample(0.5);
        assert!((v_mid - a * 0.5).abs() < 1e-6, "still accelerating at t=0.5: v={v_mid}");
        assert!((a_mid - a).abs() < 1e-6);
    }

    #[test]
    fn returns_none_when_distance_unreachable_in_time() {
        // Even accelerating the whole 1.0s at a=10 only reaches 5.0.
        assert!(RampCruiseSegment::new(0.0, 5.001, 1.0, 10.0).is_none());
    }

    #[test]
    fn handles_negative_direction_symmetrically() {
        let segment = RampCruiseSegment::new(0.0, -4.0, 1.0, 10.0).expect("feasible");
        let (q_end, v_end, _) = segment.sample(1.0);
        assert!((q_end - -4.0).abs() < 1e-9);
        assert!(v_end < 0.0, "velocity should point toward negative target");
    }

    #[test]
    fn max_speed_and_acceleration_report_the_solved_peaks() {
        let segment = RampCruiseSegment::new(0.0, 4.0, 1.0, 10.0).expect("feasible");
        let expected_v_peak = 10.0 - (100.0_f64 - 80.0).sqrt();
        assert!((segment.max_speed(24) - expected_v_peak).abs() < 1e-9);
        assert!((segment.max_acceleration(24) - 10.0).abs() < 1e-9);
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib ramp_cruise_segment`
Expected: compile error or panic from `todo!()` (module not yet registered will also fail — register it now, see Step 3).

- [ ] **Step 3: Register the module and implement**

In `src/robot/motion/mod.rs`, add (alphabetically between `quintic_segment` and `rail`):

```rust
pub mod ramp_cruise_segment;
```

Replace the `todo!()` bodies in `src/robot/motion/ramp_cruise_segment.rs`:

```rust
impl RampCruiseSegment {
    /// `q0`에서 정지 상태로 출발해 가속도 `accel`로 `v_peak`까지 가속한 뒤
    /// 남은 시간 동안 `v_peak`로 순항해 `duration` 뒤 `qf`에 도달한다.
    /// `|qf-q0|`가 그 시간 안에 전 구간을 가속해도 못 미치는 거리
    /// (`0.5·accel·duration²`)보다 크면 `None`.
    pub fn new(q0: f64, qf: f64, duration: f64, accel: f64) -> Option<Self> {
        let t = duration.max(f64::EPSILON);
        let a = accel.abs().max(f64::EPSILON);
        let delta = qf - q0;
        let max_reach = 0.5 * a * t * t;
        if delta.abs() > max_reach + 1e-9 {
            return None;
        }
        let sign = if delta < 0.0 { -1.0 } else { 1.0 };
        let magnitude = delta.abs();
        let discriminant = ((a * t) * (a * t) - 2.0 * a * magnitude).max(0.0);
        let v_peak_magnitude = a * t - discriminant.sqrt();
        let ramp_duration = (v_peak_magnitude / a).min(t);
        return Some(Self {
            q0,
            accel: sign * a,
            ramp_duration,
            v_peak: sign * v_peak_magnitude,
            duration: t,
        });
    }

    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        let t = t.clamp(0.0, self.duration);
        if t <= self.ramp_duration {
            let q = self.q0 + 0.5 * self.accel * t * t;
            let qd = self.accel * t;
            return (q, qd, self.accel);
        }
        let ramp_end_q = self.q0 + 0.5 * self.accel * self.ramp_duration * self.ramp_duration;
        let cruise_t = t - self.ramp_duration;
        let q = ramp_end_q + self.v_peak * cruise_t;
        return (q, self.v_peak, 0.0);
    }

    pub fn max_speed(&self, _samples: usize) -> f64 {
        return self.v_peak.abs();
    }

    pub fn max_acceleration(&self, _samples: usize) -> f64 {
        return self.accel.abs();
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib ramp_cruise_segment`
Expected: all 7 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/robot/motion/ramp_cruise_segment.rs src/robot/motion/mod.rs
git commit -m "feat(motion): add ramp-then-cruise joint segment primitive"
```

---

### Task 2: `DelayedQuadraticSegment` primitive (wrist snap)

**Files:**
- Modify: `src/robot/motion/quadratic_segment.rs`
- Test: same file, extend existing `#[cfg(test)] mod tests` block

**Interfaces:**
- Consumes: `QuadraticSegment::new(q0, v0, qf, duration) -> Self`, `QuadraticSegment::sample(&self, t: f64) -> (f64, f64, f64)` (both already exist).
- Produces: `pub struct DelayedQuadraticSegment` with `pub fn new(q0: f64, qf: f64, duration: f64, delay: f64) -> Self`, `pub fn sample(&self, t: f64) -> (f64, f64, f64)`, `pub fn max_speed(&self, samples: usize) -> f64`, `pub fn max_acceleration(&self, samples: usize) -> f64`.

- [ ] **Step 1: Write the failing tests**

Add to `src/robot/motion/quadratic_segment.rs`, inside the existing `#[cfg(test)] mod tests` block (after the last existing test):

```rust
    #[test]
    fn delayed_segment_holds_start_value_during_the_delay() {
        let segment = DelayedQuadraticSegment::new(0.2, 1.0, 0.4, 0.3);
        let (q_hold, v_hold, a_hold) = segment.sample(0.15);
        assert!((q_hold - 0.2).abs() < 1e-9, "should not have moved yet: q={q_hold}");
        assert!(v_hold.abs() < 1e-9);
        assert!(a_hold.abs() < 1e-9);
    }

    #[test]
    fn delayed_segment_reaches_target_exactly_at_duration() {
        let segment = DelayedQuadraticSegment::new(0.2, 1.0, 0.4, 0.3);
        let (q_end, _, _) = segment.sample(0.4);
        assert!((q_end - 1.0).abs() < 1e-9);
    }

    #[test]
    fn delayed_segment_moves_with_nonzero_velocity_during_the_burst() {
        let segment = DelayedQuadraticSegment::new(0.2, 1.0, 0.4, 0.3);
        let v_mid_burst = segment.sample(0.35).1;
        assert!(v_mid_burst.abs() > 1e-6, "should be moving during the burst window");
    }

    #[test]
    fn delayed_segment_with_zero_delay_matches_plain_quadratic_segment() {
        let delayed = DelayedQuadraticSegment::new(0.0, 1.0, 0.4, 0.0);
        let plain = QuadraticSegment::new(0.0, 0.0, 1.0, 0.4);
        for step in 0..=10 {
            let t = 0.4 * f64::from(step) / 10.0;
            let (q_d, v_d, a_d) = delayed.sample(t);
            let (q_p, v_p, a_p) = plain.sample(t);
            assert!((q_d - q_p).abs() < 1e-9, "position mismatch at t={t}");
            assert!((v_d - v_p).abs() < 1e-9, "velocity mismatch at t={t}");
            assert!((a_d - a_p).abs() < 1e-9, "acceleration mismatch at t={t}");
        }
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib quadratic_segment`
Expected: FAIL with "cannot find type `DelayedQuadraticSegment`".

- [ ] **Step 3: Implement**

Add above the existing `#[cfg(test)]` line in `src/robot/motion/quadratic_segment.rs`:

```rust
/// `delay`만큼 시작값에 정지해 있다가 나머지 시간 동안 [`QuadraticSegment`]로
/// 목표에 도달하는 래퍼 — 손목(j3)처럼 접힌 자세를 유지하다 임팩트 직전에만
/// 등가속으로 스냅하는 관절에 쓴다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DelayedQuadraticSegment {
    hold_value: f64,
    delay: f64,
    inner: QuadraticSegment,
}

impl DelayedQuadraticSegment {
    /// `q0`에서 `delay`초 동안 정지한 뒤, 나머지 `duration - delay`초 동안
    /// 등가속(정지에서 출발)으로 `qf`에 도달한다. `delay`는 `[0, duration]`로
    /// 클램프한다.
    pub fn new(q0: f64, qf: f64, duration: f64, delay: f64) -> Self {
        let delay = delay.clamp(0.0, duration.max(0.0));
        let inner = QuadraticSegment::new(q0, 0.0, qf, duration - delay);
        return Self {
            hold_value: q0,
            delay,
            inner,
        };
    }

    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        if t <= self.delay {
            return (self.hold_value, 0.0, 0.0);
        }
        return self.inner.sample(t - self.delay);
    }

    pub fn max_speed(&self, samples: usize) -> f64 {
        return self.inner.max_speed(samples);
    }

    pub fn max_acceleration(&self, samples: usize) -> f64 {
        return self.inner.max_acceleration(samples);
    }
}

```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib quadratic_segment`
Expected: all tests (existing + 4 new) PASS.

- [ ] **Step 5: Commit**

```bash
git add src/robot/motion/quadratic_segment.rs
git commit -m "feat(motion): add delayed-burst quadratic segment for wrist snap"
```

---

### Task 3: `Trajectory` per-joint power-sweep profiles

**Files:**
- Modify: `src/robot/motion/trajectory.rs`
- Test: same file, extend existing `#[cfg(test)] mod tests` block

**Interfaces:**
- Consumes: `RampCruiseSegment` (Task 1), `DelayedQuadraticSegment` (Task 2), existing `QuadraticSegment`, `QuinticSegment`.
- Produces: `pub(crate) enum PreImpactJointProfile { Quadratic, DelayedQuadratic { delay: f64 }, RampCruise { accel: f64 } }` and `pub fn Trajectory::with_power_sweep(start: Joints, impact: Joints, end: Joints, profiles: Vec<PreImpactJointProfile>, follow_through_velocity: Vec<f64>, impact_time_secs: f64, duration_secs: f64, rail: Rail, follow_through_rail_x: f64, follow_through_rail_velocity: f64) -> Self`. Later tasks (physics.rs) construct `PreImpactJointProfile` values and call this constructor.

- [ ] **Step 1: Write the failing tests**

Add to `src/robot/motion/trajectory.rs`, inside the existing `#[cfg(test)] mod tests` block:

```rust
    #[test]
    fn power_sweep_ramp_cruise_joint_reaches_target_and_sustains_speed_before_it() {
        let trajectory = Trajectory::with_power_sweep(
            Joints::from_slice(&[0.0]),
            Joints::from_slice(&[4.0]),
            Joints::from_slice(&[4.2]),
            vec![PreImpactJointProfile::RampCruise { accel: 10.0 }],
            vec![0.0],
            1.0,
            1.12,
            Rail::fixed(0.3),
            0.3,
            0.0,
        );
        let impact = trajectory.sample_at(1.0);
        assert!((impact.values[0] - 4.0).abs() < 1e-6);
        let v_near_impact = trajectory.sample_velocity_at(0.9)[0];
        let v_at_impact = trajectory.sample_velocity_at(1.0)[0];
        assert!(
            (v_near_impact - v_at_impact).abs() < 1e-3,
            "should be cruising before impact, not still ramping: v(0.9)={v_near_impact} v(1.0)={v_at_impact}"
        );
    }

    #[test]
    fn power_sweep_delayed_joint_holds_then_snaps_exactly_at_impact() {
        let trajectory = Trajectory::with_power_sweep(
            Joints::from_slice(&[-0.5]),
            Joints::from_slice(&[0.3]),
            Joints::from_slice(&[0.3]),
            vec![PreImpactJointProfile::DelayedQuadratic { delay: 0.8 }],
            vec![0.0],
            1.0,
            1.0,
            Rail::fixed(0.3),
            0.3,
            0.0,
        );
        let held = trajectory.sample_at(0.4);
        assert!((held.values[0] - -0.5).abs() < 1e-9, "wrist should still be cocked: {held:?}");
        let impact = trajectory.sample_at(1.0);
        assert!((impact.values[0] - 0.3).abs() < 1e-6);
    }

    #[test]
    fn power_sweep_mixes_profiles_per_joint_independently() {
        let trajectory = Trajectory::with_power_sweep(
            Joints::from_slice(&[0.0, -0.5]),
            Joints::from_slice(&[4.0, 0.3]),
            Joints::from_slice(&[4.2, 0.3]),
            vec![
                PreImpactJointProfile::RampCruise { accel: 10.0 },
                PreImpactJointProfile::DelayedQuadratic { delay: 0.8 },
            ],
            vec![0.0, 0.0],
            1.0,
            1.12,
            Rail::fixed(0.3),
            0.3,
            0.0,
        );
        let mid = trajectory.sample_at(0.4);
        assert!(mid.values[0] > 0.5, "j0 should already be moving: {mid:?}");
        assert!((mid.values[1] - -0.5).abs() < 1e-9, "j1 should still be held: {mid:?}");
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib trajectory`
Expected: FAIL with "cannot find type `PreImpactJointProfile`" / "no function `with_power_sweep`".

- [ ] **Step 3: Implement**

In `src/robot/motion/trajectory.rs`, update the import line:

```rust
use super::quadratic_segment::{DelayedQuadraticSegment, QuadraticSegment};
use super::quintic_segment::QuinticSegment;
use super::ramp_cruise_segment::RampCruiseSegment;
use super::rail::Rail;
```

Add the new enum near `PreImpactSegment`:

```rust
/// 관절별 타격-전 프로파일 — [`Trajectory::with_power_sweep`] 전용.
/// 항상 정지(v0=0)에서 출발한다는 전제라 quintic은 대상이 아니다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum PreImpactJointProfile {
    /// 전 구간 등가속(정지 출발) — [`QuadraticSegment`]와 동일.
    Quadratic,
    /// `delay`초 정지 후 등가속 스냅 — 손목처럼 접힌 자세를 유지하다 임팩트
    /// 직전에만 움직이는 관절.
    DelayedQuadratic { delay: f64 },
    /// 가속도 `accel`로 첨두속도까지 가속한 뒤 순항 — j0·j2처럼 임팩트
    /// 앞에서 첨두속도를 유지해야 하는 관절.
    RampCruise { accel: f64 },
}

impl PreImpactJointProfile {
    /// `pub(crate)`인 이유: `physics.rs`의 파워 스윙 플래너가 임팩트 속도를
    /// 미리 뽑아 보기 위해 직접 호출한다.
    pub(crate) fn build(&self, q0: f64, qf: f64, duration: f64) -> PreImpactSegment {
        return match *self {
            PreImpactJointProfile::Quadratic => {
                PreImpactSegment::Quadratic(QuadraticSegment::new(q0, 0.0, qf, duration))
            }
            PreImpactJointProfile::DelayedQuadratic { delay } => PreImpactSegment::DelayedQuadratic(
                DelayedQuadraticSegment::new(q0, qf, duration, delay),
            ),
            PreImpactJointProfile::RampCruise { accel } => {
                match RampCruiseSegment::new(q0, qf, duration, accel) {
                    Some(segment) => PreImpactSegment::RampCruise(segment),
                    // 호출자(physics.rs)가 미리 `RampCruiseSegment::new`로 실현
                    // 가능성을 검증하므로 여기 도달하면 안 되지만, Trajectory
                    // 자체는 실패하지 않는 기존 관례를 지키기 위한 방어적 대체.
                    None => PreImpactSegment::Quadratic(QuadraticSegment::new(q0, 0.0, qf, duration)),
                }
            }
        };
    }
}
```

Extend `PreImpactSegment`:

```rust
pub(crate) enum PreImpactSegment {
    Quintic(QuinticSegment),
    Quadratic(QuadraticSegment),
    DelayedQuadratic(DelayedQuadraticSegment),
    RampCruise(RampCruiseSegment),
}

impl PreImpactSegment {
    pub(crate) fn sample(&self, t: f64) -> (f64, f64, f64) {
        return match self {
            PreImpactSegment::Quintic(segment) => segment.sample(t),
            PreImpactSegment::Quadratic(segment) => segment.sample(t),
            PreImpactSegment::DelayedQuadratic(segment) => segment.sample(t),
            PreImpactSegment::RampCruise(segment) => segment.sample(t),
        };
    }

    fn max_speed(&self, samples: usize) -> f64 {
        return match self {
            PreImpactSegment::Quintic(segment) => segment.max_speed(samples),
            PreImpactSegment::Quadratic(segment) => segment.max_speed(samples),
            PreImpactSegment::DelayedQuadratic(segment) => segment.max_speed(samples),
            PreImpactSegment::RampCruise(segment) => segment.max_speed(samples),
        };
    }

    fn max_acceleration(&self, samples: usize) -> f64 {
        return match self {
            PreImpactSegment::Quintic(segment) => segment.max_acceleration(samples),
            PreImpactSegment::Quadratic(segment) => segment.max_acceleration(samples),
            PreImpactSegment::DelayedQuadratic(segment) => segment.max_acceleration(samples),
            PreImpactSegment::RampCruise(segment) => segment.max_acceleration(samples),
        };
    }
}
```

Add the field to `Trajectory` (next to `pre_impact_profile`):

```rust
    pre_impact_profile: SegmentProfile,
    /// 관절별 타격-전 프로파일 — 비어 있으면(기본) `pre_impact_profile`(전역)을
    /// 쓴다. [`Trajectory::with_power_sweep`]만 채운다.
    pre_impact_profiles: Vec<PreImpactJointProfile>,
```

Update the three existing constructors (`new`, `with_follow_through`, `with_quadratic_push`) to initialize the new field — add `pre_impact_profiles: Vec::new(),` to each of their `Self { ... }` literals (right after `pre_impact_profile: SegmentProfile::...,`).

Add the new constructor after `with_quadratic_push`:

```rust
    /// 관절마다 다른 타격-전 프로파일(정지 출발 등가속 / 지연 스냅 /
    /// 가속-순항)을 섞어 쓰는 "파워 스윙" 궤적을 만든다. 세 프로파일 모두
    /// `start_velocity=0`을 전제한다.
    #[allow(clippy::too_many_arguments)]
    pub fn with_power_sweep(
        start: Joints,
        impact: Joints,
        end: Joints,
        profiles: Vec<PreImpactJointProfile>,
        follow_through_velocity: Vec<f64>,
        impact_time_secs: f64,
        duration_secs: f64,
        rail: Rail,
        follow_through_rail_x: f64,
        follow_through_rail_velocity: f64,
    ) -> Self {
        let n = impact.values.len();
        assert_eq!(start.values.len(), n, "start joint count");
        assert_eq!(profiles.len(), n, "profile count");
        let start_velocity = vec![0.0; n];
        let mut end_velocity = Vec::with_capacity(n);
        let mut impact_acceleration = Vec::with_capacity(n);
        for i in 0..n {
            let segment = profiles[i].build(start.values[i], impact.values[i], impact_time_secs);
            let (_, velocity, acceleration) = segment.sample(impact_time_secs);
            end_velocity.push(velocity);
            impact_acceleration.push(acceleration);
        }
        return Self {
            start,
            end: impact,
            follow_through: end,
            start_velocity,
            end_velocity,
            follow_through_velocity,
            impact_acceleration,
            impact_time_secs,
            duration_secs,
            rail,
            follow_through_rail_x,
            follow_through_rail_velocity,
            pre_impact_profile: SegmentProfile::Quadratic,
            pre_impact_profiles: profiles,
        };
    }
```

Update `pre_impact_segments()` to consult per-joint profiles first:

```rust
    fn pre_impact_segments(&self) -> Vec<PreImpactSegment> {
        let n = self.start.values.len();
        assert_eq!(self.end.values.len(), n, "impact joint count");
        assert_eq!(self.start_velocity.len(), n, "start velocity count");
        assert_eq!(self.end_velocity.len(), n, "impact velocity count");
        if self.pre_impact_profiles.len() == n {
            return (0..n)
                .map(|i| {
                    self.pre_impact_profiles[i].build(
                        self.start.values[i],
                        self.end.values[i],
                        self.impact_time_secs,
                    )
                })
                .collect();
        }
        let mut segments = Vec::with_capacity(n);
        for i in 0..n {
            let impact_accel = self.impact_acceleration.get(i).copied().unwrap_or(0.0);
            segments.push(match self.pre_impact_profile {
                SegmentProfile::Quintic => PreImpactSegment::Quintic(QuinticSegment::new(
                    self.start.values[i],
                    self.end.values[i],
                    self.start_velocity[i],
                    self.end_velocity[i],
                    0.0,
                    impact_accel,
                    self.impact_time_secs,
                )),
                SegmentProfile::Quadratic => PreImpactSegment::Quadratic(QuadraticSegment::new(
                    self.start.values[i],
                    self.start_velocity[i],
                    self.end.values[i],
                    self.impact_time_secs,
                )),
            });
        }
        return segments;
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib trajectory`
Expected: all tests (existing + 3 new) PASS.

- [ ] **Step 5: Run the full trajectory + quadratic_segment + ramp_cruise_segment suites to check nothing regressed**

Run: `cargo test --lib motion::`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add src/robot/motion/trajectory.rs
git commit -m "feat(motion): add per-joint power-sweep profiles to Trajectory"
```

---

### Task 4: New tunable constants

**Files:**
- Modify: `src/defaults/motion.rs`

**Interfaces:**
- Produces: `pub const FIXED_JOINT_SWING_RAMP_SECS: f64`, `pub const FIXED_JOINT_SWING_CRUISE_SECS: f64`, `pub const FIXED_JOINT_SWING_SNAP_DURATION_SECS: f64`.

- [ ] **Step 1: Add the constants**

In `src/defaults/motion.rs`, after the existing `FIXED_JOINT_SWING_FOLLOW_THROUGH_SECS` constant (around line 115):

```rust
/// 파워 스윙에서 j0·j2가 정지에서 관절 속도 상한까지 가속하는 데 쓰는
/// 시간 [s]. `arm.max_joint_speed / FIXED_JOINT_SWING_RAMP_SECS`가 이 관절들의
/// 가속도로 쓰인다.
pub const FIXED_JOINT_SWING_RAMP_SECS: f64 = 0.060;
/// 가속 뒤 첨두속도를 그대로 유지(순항)하는 시간 [s] — 공 도착 시각 예측
/// 오차를 흡수하는 창이다. `FIXED_JOINT_SWING_RAMP_SECS`와 합이 파워 스윙의
/// 전체 타격-전 시간이 된다.
pub const FIXED_JOINT_SWING_CRUISE_SECS: f64 = 0.060;
/// 손목(j3)이 접힌 자세로 대기하다 등가속 스냅으로 목표각까지 움직이는
/// 시간 [s] — 파워 스윙 전체 시간의 마지막 구간이다.
pub const FIXED_JOINT_SWING_SNAP_DURATION_SECS: f64 = 0.050;
```

- [ ] **Step 2: Verify it compiles**

Run: `cargo build --lib`
Expected: succeeds (unused-constant warnings are fine at this point — they'll be consumed in Task 5).

- [ ] **Step 3: Commit**

```bash
git add src/defaults/motion.rs
git commit -m "feat(motion): add power-sweep timing constants"
```

---

### Task 5: `plan_fixed_joint_swing_power_sweep` in `physics.rs`

**Files:**
- Modify: `src/robot/motion/physics.rs`
- Test: same file, extend existing `#[cfg(test)] mod tests` block

**Interfaces:**
- Consumes: `Trajectory::with_power_sweep`, `PreImpactJointProfile` (Task 3, need to import — see below), `RampCruiseSegment::new` (Task 1, for the pre-check), `FIXED_JOINT_SWING_RAMP_SECS`/`FIXED_JOINT_SWING_CRUISE_SECS`/`FIXED_JOINT_SWING_SNAP_DURATION_SECS` (Task 4), `arm.wrist_joint_index() -> Option<usize>`, `arm.max_joint_speed: f64`, `arm.inverse_pose_at_fixed_rail_best_normal(...)` (already used by sibling functions), `evaluate_trajectory_feasibility` (already exists in this file).
- Produces: `pub fn plan_fixed_joint_swing_power_sweep(arm: &Arm, start: &robot::Pose) -> Result<FixedJointSwing, DomainError>` and `pub fn plan_fixed_joint_swing_power_sweep_from_alignment(arm: &Arm, start: &robot::Pose, aligned: &robot::Pose) -> Result<FixedJointSwing, DomainError>`.

`PreImpactJointProfile` and `PreImpactSegment` are currently `pub(crate)` inside `trajectory.rs` — confirm `PreImpactJointProfile` is visible from `physics.rs` (same crate, `pub(crate)` items are visible crate-wide) before importing it.

- [ ] **Step 1: Write the failing tests**

Add to `src/robot/motion/physics.rs`, inside the existing `#[cfg(test)] mod tests` block (after `fixed_joint_swing_quadratic_pushes_forward_with_constant_acceleration`, i.e. after line ~2358):

```rust
    #[test]
    fn fixed_joint_swing_power_sweep_j0_j2_sustain_ceiling_speed_before_impact() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let home = robot::Pose::new(
            arm.rail.as_ref().map_or(0.0, |rail| rail.default_x()),
            robot::Joints::from_slice(&crate::defaults::READY_JOINTS_4DOF),
        );
        let planned =
            plan_fixed_joint_swing_power_sweep(arm, &home).expect("power sweep swing");
        let trajectory = planned.trajectory;
        for &index in &[0usize, 2usize] {
            let v_end = trajectory.sample_velocity_at(trajectory.impact_time_secs)[index];
            let cruise_probe = trajectory.impact_time_secs - FIXED_JOINT_SWING_CRUISE_SECS * 0.5;
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
        let planned =
            plan_fixed_joint_swing_power_sweep(arm, &start).expect("power sweep swing");
        let trajectory = planned.trajectory;
        let wrist_index = arm.wrist_joint_index().expect("4dof arm has a wrist");
        let hold_end =
            (trajectory.impact_time_secs - FIXED_JOINT_SWING_SNAP_DURATION_SECS).max(0.0);
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

    #[test]
    fn fixed_joint_swing_power_sweep_stays_within_joint_limits() {
        let active = crate::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let rail_x = arm.rail.as_ref().map_or(0.0, |rail| rail.default_x());
        let mut joints = arm.default_joints.clone();
        joints.values[3] = arm.joint_limit(3).expect("q3 limit").min;
        let start = robot::Pose::new(rail_x, joints);
        let planned = plan_fixed_joint_swing_power_sweep(arm, &start)
            .expect("power sweep from wrist limit");
        assert!(arm.joints_in_limits(&planned.trajectory.end));
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib physics::tests::fixed_joint_swing_power_sweep`
Expected: FAIL with "cannot find function `plan_fixed_joint_swing_power_sweep`".

- [ ] **Step 3: Implement**

Update the import block at the top of `src/robot/motion/physics.rs`:

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
use crate::error::{DomainError, SwingPlanError};
use crate::robot::Arm;
use crate::robot::motion::Impact;
use crate::robot::motion::Prediction;
use crate::robot::{self, Joints};

use super::impact_candidate::{ImpactCandidate, best_impact_candidate};
use super::impact_target::{impact_target_from_candidate, solve_impact_target};
use super::planned_intercept::PlannedIntercept;
use super::quadratic_segment::QuadraticSegment;
use super::quintic_segment::QuinticSegment;
use super::ramp_cruise_segment::RampCruiseSegment;
use super::rail::Rail;
use super::trajectory::{PreImpactJointProfile, Trajectory};
```

Add after `plan_fixed_joint_swing_quadratic_to_pose` (after line ~1274, before the `/// 정지 → 정지로...` comment):

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
/// - j3(손목)는 [`FIXED_JOINT_SWING_SNAP_DURATION_SECS`] 전까지 접힌 자세로
///   대기하다 마지막 구간에서만 등가속 스냅으로 목표각에 도달한다.
pub fn plan_fixed_joint_swing_power_sweep(
    arm: &Arm,
    start: &robot::Pose,
) -> Result<FixedJointSwing, DomainError> {
    return plan_fixed_joint_swing_power_sweep_from_alignment(arm, start, start);
}

/// [`plan_fixed_joint_swing_power_sweep`]의 정렬-기준 버전 —
/// [`plan_fixed_joint_swing_quadratic_from_alignment`]와 같은 이유로 실측
/// 대신 마지막 절대 정렬 자세를 밀기 기준으로 쓴다.
pub fn plan_fixed_joint_swing_power_sweep_from_alignment(
    arm: &Arm,
    start: &robot::Pose,
    aligned: &robot::Pose,
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
    );
}

/// j0·j2 인덱스 — 이 팔에서 토크가 가장 큰 두 관절(이중/단일 MX-64).
const POWER_SWEEP_JOINT_INDICES: [usize; 2] = [0, 2];

fn plan_fixed_joint_swing_power_sweep_to_pose(
    arm: &Arm,
    start: &robot::Pose,
    target_position: Point3,
    target_normal: Vector3<f64>,
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

    let impact_time = FIXED_JOINT_SWING_RAMP_SECS + FIXED_JOINT_SWING_CRUISE_SECS;
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
        let delay = (impact_time - FIXED_JOINT_SWING_SNAP_DURATION_SECS).max(0.0);
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

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib physics::tests::fixed_joint_swing_power_sweep`
Expected: all 3 new tests PASS.

- [ ] **Step 5: Run the full physics test module to check nothing regressed**

Run: `cargo test --lib physics::`
Expected: all PASS (existing quintic/quadratic swing tests untouched and still green).

- [ ] **Step 6: Commit**

```bash
git add src/robot/motion/physics.rs src/robot/motion/trajectory.rs
git commit -m "feat(motion): add power-sweep swing planner driven by j0/j2"
```

---

### Task 6: Expose via `Planner` and wire into the real control loop

**Files:**
- Modify: `src/robot/motion/planner.rs`
- Modify: `src/real/control_worker.rs`

**Interfaces:**
- Consumes: `physics::plan_fixed_joint_swing_power_sweep`, `physics::plan_fixed_joint_swing_power_sweep_from_alignment` (Task 5).
- Produces: `Planner::fixed_joint_swing_power_sweep(arm: &Arm, start: &robot::Pose) -> Result<physics::FixedJointSwing, DomainError>`, `Planner::fixed_joint_swing_power_sweep_from_alignment(arm: &Arm, start: &robot::Pose, aligned: &robot::Pose) -> Result<physics::FixedJointSwing, DomainError>`.

- [ ] **Step 1: Add the `Planner` wrappers**

In `src/robot/motion/planner.rs`, after `fixed_joint_swing_quadratic_from_alignment` (around line 202):

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

- [ ] **Step 2: Verify it compiles**

Run: `cargo build --lib`
Expected: succeeds.

- [ ] **Step 3: Wire into `control_worker.rs`**

In `src/real/control_worker.rs`, at the call site around line 471 (`match Planner::fixed_joint_swing_quadratic_from_alignment(`), change the function name to `Planner::fixed_joint_swing_power_sweep_from_alignment`. The surrounding `match` arms, argument list, and everything downstream (`planned.trajectory`, `hardware.command_joints(swing)`, logging) stays exactly as-is — `FixedJointSwing` has the same shape regardless of which planner produced it.

- [ ] **Step 4: Verify it compiles**

Run: `cargo build --lib`
Expected: succeeds.

- [ ] **Step 5: Run the real-hardware control_worker test suite**

Run: `cargo test --lib real::control_worker`
Expected: all PASS. If any test asserted specifics about the old quadratic swing's timing/shape (e.g. asserted `impact_time_secs` equals the old fixed constant, or asserted constant acceleration throughout), it will need updating to match the power-sweep shape — read the failure message, and if it's asserting on swing *shape* rather than *outcome* (racket position, hardware commands sent, rail behavior), update the assertion to match the new profile rather than reverting the wiring.

- [ ] **Step 6: Commit**

```bash
git add src/robot/motion/planner.rs src/real/control_worker.rs
git commit -m "feat(real): switch commit swing to the j0/j2 power sweep"
```

---

### Task 7: Full-suite verification and cleanup

**Files:** none new — verification only, plus one doc-comment fix.

- [ ] **Step 1: Fix the stale doc comment on `Planner::fixed_joint_swing`**

`src/robot/motion/planner.rs` line 168 currently reads:

```rust
    /// 접힌 정렬 자세에서 j0·j1·j2 전진 푸시와 j3 손목 스냅을 합성한다.
    pub fn fixed_joint_swing(
```

This describes the *new* power-sweep behavior (which now has its own accurate doc comment from Task 6 Step 1), not what `fixed_joint_swing` (wrapping the quintic `plan_fixed_joint_swing`) actually does. Replace just that one doc-comment line with:

```rust
    /// 접힌 정렬 자세에서 별도 백스윙 없이 j0~j3로 라켓을 바로 민다 —
    /// 관절 배분은 IK가 정하며 특정 관절의 역할을 강제하지 않는다.
    pub fn fixed_joint_swing(
```

- [ ] **Step 2: Run the full test suite**

Run: `cargo test --lib`
Expected: all tests PASS, zero failures.

- [ ] **Step 3: Run clippy**

Run: `cargo clippy --lib -- -D warnings`
Expected: no warnings. Fix anything flagged (likely candidates: the `#[allow(clippy::too_many_arguments)]` already applied to the new constructors/functions mirrors existing sibling functions, so this should be clean).

- [ ] **Step 4: Manual sanity check of the numbers**

Run `cargo test --lib fixed_joint_swing_power_sweep -- --nocapture` and inspect: does `peak_joint_speed()` for the power-sweep trajectory come out higher than the equivalent quadratic-swing test (`fixed_joint_swing_quadratic_finds_faster_push_than_fixed_ladder`, which asserts `> 4.5`)? If the power-sweep numbers are surprisingly low (e.g. `RampCruiseSegment::new` returning `None` for j0/j2 at the default `FIXED_JOINT_SWING_RAMP_SECS`/`FIXED_JOINT_SWING_CRUISE_SECS` and falling through the bisection to a much shorter push every time), the two constants in Task 4 are probably too aggressive for this arm's actual reach — note it, but do not silently retune without checking with the user first, since these are physical tuning constants that affect real hardware behavior.

- [ ] **Step 5: Commit**

```bash
git add src/robot/motion/planner.rs
git commit -m "docs(motion): fix stale swing doc comment after power-sweep wiring"
```
