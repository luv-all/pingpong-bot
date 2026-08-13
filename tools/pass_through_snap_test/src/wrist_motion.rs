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
