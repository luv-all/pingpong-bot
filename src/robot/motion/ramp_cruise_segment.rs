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
        assert!(
            (v_near_end - v_at_end).abs() < 1e-9,
            "not cruising: v(0.9)={v_near_end} v(1.0)={v_at_end}"
        );
    }

    #[test]
    fn ramp_phase_is_constant_acceleration_from_rest() {
        let segment = RampCruiseSegment::new(0.0, 4.0, 1.0, 10.0).expect("feasible");
        let (_, v0, a0) = segment.sample(0.0);
        assert!(v0.abs() < 1e-9, "should start at rest");
        assert!(
            (a0 - 10.0).abs() < 1e-9,
            "should start accelerating at full accel"
        );
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
        assert!(
            (v_mid - a * 0.5).abs() < 1e-6,
            "still accelerating at t=0.5: v={v_mid}"
        );
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
