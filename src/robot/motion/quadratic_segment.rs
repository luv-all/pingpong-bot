//! 관절 1축 quadratic(등가속) 세그먼트.
//!
//! [`super::quintic_segment::QuinticSegment`]와 달리 자유도가 3개(`q0,v0,a`)뿐이라
//! `(Δq, v0, duration)` 세 값을 주면 가속도 `a`가 유일하게 정해진다 — 목표
//! 종료 속도를 별도로 지정할 자유도가 없다(정해진 값을 그대로 받는다).
//! 시작 가속도가 항상 `a`(quintic의 `a0=0` ease-in과 달리 t=0에서부터 전 구간
//! 등가속)라 첨두 가속도가 평균 가속도와 같다 — 같은 `(v0,vf,duration)`을
//! 만족하는 어떤 매끄러운 프로파일보다 첨두 가속도가 낮다(평균이 곧 첨두).

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadraticSegment {
    q0: f64,
    v0: f64,
    a: f64,
    duration: f64,
}

impl QuadraticSegment {
    /// `q0`에서 속도 `v0`로 출발해 `duration` 뒤 `qf`에 도달하는 등가속을 푼다.
    pub fn new(q0: f64, v0: f64, qf: f64, duration: f64) -> Self {
        let t = duration.max(f64::EPSILON);
        let a = 2.0 * (qf - q0 - v0 * t) / (t * t);
        return Self {
            q0,
            v0,
            a,
            duration: t,
        };
    }

    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        let t = t.clamp(0.0, self.duration);
        let q = self.q0 + self.v0 * t + 0.5 * self.a * t * t;
        let qd = self.v0 + self.a * t;
        let qdd = self.a;
        return (q, qd, qdd);
    }

    pub fn max_speed(&self, samples: usize) -> f64 {
        let n = samples.max(2);
        let mut peak = 0.0_f64;
        for i in 0..=n {
            let t = self.duration * (i as f64) / (n as f64);
            peak = peak.max(self.sample(t).1.abs());
        }
        return peak;
    }

    pub fn max_acceleration(&self, _samples: usize) -> f64 {
        return self.a.abs();
    }
}

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
        if t < self.delay {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reaches_position_and_velocity_boundary_at_duration() {
        let segment = QuadraticSegment::new(0.1, 0.0, 0.8, 0.4);
        let (q0, v0, _) = segment.sample(0.0);
        let (qf, _, _) = segment.sample(0.4);
        assert!((q0 - 0.1).abs() < 1e-9);
        assert!((v0 - 0.0).abs() < 1e-9);
        assert!((qf - 0.8).abs() < 1e-9);
    }

    #[test]
    fn acceleration_is_constant_across_the_whole_segment() {
        let segment = QuadraticSegment::new(0.0, 0.0, 0.1, 0.2);
        let a_start = segment.sample(0.0).2;
        let a_mid = segment.sample(0.1).2;
        let a_end = segment.sample(0.2).2;
        assert!((a_start - a_mid).abs() < 1e-12);
        assert!((a_mid - a_end).abs() < 1e-12);
    }

    #[test]
    fn matches_kinematic_constant_acceleration_formula_from_rest() {
        // v0=0이면 a = 2*(qf-q0)/T^2, v(T) = a*T.
        let (q0, qf, duration) = (0.0, 0.1, 0.2);
        let segment = QuadraticSegment::new(q0, 0.0, qf, duration);
        let expected_a = 2.0 * (qf - q0) / (duration * duration);
        let (_, vf, af) = segment.sample(duration);
        assert!((af - expected_a).abs() < 1e-9);
        assert!((vf - expected_a * duration).abs() < 1e-9);
    }

    #[test]
    fn home_is_the_vertex_when_starting_from_rest() {
        // v0=0이고 a>0이면 t=0이 q(t) 포물선의 최솟값(꼭짓점)이어야 한다 —
        // 즉 위치가 시작점에서 단조 증가해야 한다.
        let segment = QuadraticSegment::new(0.0, 0.0, 0.2, 0.3);
        let mut previous = segment.sample(0.0).0;
        for i in 1..=10 {
            let t = 0.3 * i as f64 / 10.0;
            let q = segment.sample(t).0;
            assert!(q >= previous - 1e-12, "position decreased at t={t}");
            previous = q;
        }
    }

    #[test]
    fn max_speed_and_acceleration_are_finite_for_typical_push() {
        let segment = QuadraticSegment::new(0.0, 0.0, 0.05, 0.2);
        assert!(segment.max_speed(24).is_finite());
        assert!(segment.max_acceleration(24).is_finite());
        assert!(segment.max_speed(24) > 0.0);
    }

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
}
