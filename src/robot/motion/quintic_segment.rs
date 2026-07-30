//! 관절 1축 quintic 세그먼트.

use nalgebra::{Matrix3, Vector3 as NaVector3};

/// 관절 1축 quintic - 위치/속도/가속도 경계 모두 지정 가능.
///
/// `a0=0.0, af=0.0`으로 호출하면 예전(시작/끝 가속 0 고정) 동작과
/// 바이트 단위로 동일하다 — `tests::zero_acceleration_boundary_matches_legacy_shape`
/// 참고. 임팩트 knot(스윙 궤적의 타격-전/팔로스루 접합점)에 0이 아닌 공유
/// 가속도를 넣을 수 있게 하려고 일반화했다(사용자 보고 "타격 순간 멈춤"
/// 증상 — `.omc/plans/2026-07-31-nonzero-impact-knot-acceleration.md`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuinticSegment {
    q0: f64,
    duration: f64,
    c1: f64,
    c2: f64,
    c3: f64,
    c4: f64,
    c5: f64,
}

impl QuinticSegment {
    #[allow(clippy::too_many_arguments)]
    pub fn new(q0: f64, qf: f64, v0: f64, vf: f64, a0: f64, af: f64, duration: f64) -> Self {
        let t = duration.max(f64::EPSILON);
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;

        let a = Matrix3::new(
            t3,
            t4,
            t5,
            3.0 * t2,
            4.0 * t3,
            5.0 * t4,
            6.0 * t,
            12.0 * t2,
            20.0 * t3,
        );
        let b = NaVector3::new(qf - q0 - v0 * t - 0.5 * a0 * t2, vf - v0 - a0 * t, af - a0);
        let coeffs = a.lu().solve(&b).unwrap_or(NaVector3::zeros());

        return Self {
            q0,
            duration: t,
            c1: v0,
            c2: 0.5 * a0,
            c3: coeffs.x,
            c4: coeffs.y,
            c5: coeffs.z,
        };
    }

    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        let t = t.clamp(0.0, self.duration);
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;

        let q = self.q0 + self.c1 * t + self.c2 * t2 + self.c3 * t3 + self.c4 * t4 + self.c5 * t5;
        let qd = self.c1
            + 2.0 * self.c2 * t
            + 3.0 * self.c3 * t2
            + 4.0 * self.c4 * t3
            + 5.0 * self.c5 * t4;
        let qdd = 2.0 * self.c2 + 6.0 * self.c3 * t + 12.0 * self.c4 * t2 + 20.0 * self.c5 * t3;
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

    pub fn max_acceleration(&self, samples: usize) -> f64 {
        let n = samples.max(2);
        let mut peak = 0.0_f64;
        for i in 0..=n {
            let t = self.duration * (i as f64) / (n as f64);
            peak = peak.max(self.sample(t).2.abs());
        }
        return peak;
    }

    /// 세그먼트 전 구간 저크(jerk, q'''(t)) 제곱의 적분 `∫₀ᵀ q'''(t)² dt`.
    ///
    /// `q'''(t) = 6·c3 + 24·c4·t + 60·c5·t²`이고 이 계수들은 경계조건(위치/
    /// 속도/가속도)의 아핀 함수이므로, 공유 knot 가속도 하나를 변수로 두고
    /// 두 세그먼트의 저크 비용 합을 구하면 그 변수에 대해 **정확히 2차식**이
    /// 된다 — [`jerk_minimizing_knot_acceleration`]이 이 사실을 쓴다.
    pub fn jerk_cost(&self) -> f64 {
        let t = self.duration;
        let a = 6.0 * self.c3;
        let b = 24.0 * self.c4;
        let c = 60.0 * self.c5;
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;
        return a * a * t
            + a * b * t2
            + (2.0 * a * c + b * b) * t3 / 3.0
            + b * c * t4 / 2.0
            + c * c * t5 / 5.0;
    }

    /// 타격-전(`q0,v0 → q1,v1`, 소요 `t1`)/팔로스루(`q1,v1 → q2,v2`, 소요
    /// `t2`) 두 세그먼트가 임팩트 knot에서 공유하는 가속도를, 두 세그먼트의
    /// 저크 비용 합을 최소화하는 값으로 구한다.
    ///
    /// [`jerk_cost`]가 knot 가속도 `a`의 정확한 2차식이므로, 세 점(`-h, 0, h`)
    /// 을 표본화해 2차식 정점을 그대로 복원한다(수치 탐색이 아니라 정확한
    /// 대수 공식) — `h`(`PROBE_ACCEL`)는 표본 간격일 뿐 결과에 영향을 주지
    /// 않는다. 곡률이 0에 가까우면(구간이 극히 짧아 사실상 선형) 예전 동작과
    /// 동일한 `0.0`으로 폴백한다. 상세:
    /// `.omc/plans/2026-07-31-nonzero-impact-knot-acceleration.md`.
    #[allow(clippy::too_many_arguments)]
    pub fn jerk_minimizing_knot_acceleration(
        q0: f64,
        v0: f64,
        q1: f64,
        v1: f64,
        t1: f64,
        q2: f64,
        v2: f64,
        t2: f64,
    ) -> f64 {
        const PROBE_ACCEL: f64 = 100.0;
        let total_jerk = |a: f64| -> f64 {
            let pre = QuinticSegment::new(q0, q1, v0, v1, 0.0, a, t1);
            let post = QuinticSegment::new(q1, q2, v1, v2, a, 0.0, t2);
            return pre.jerk_cost() + post.jerk_cost();
        };
        let y0 = total_jerk(-PROBE_ACCEL);
        let y1 = total_jerk(0.0);
        let y2 = total_jerk(PROBE_ACCEL);
        let curvature = y0 + y2 - 2.0 * y1;
        if curvature.abs() < 1e-9 {
            return 0.0;
        }
        return (y0 - y2) * PROBE_ACCEL / (2.0 * curvature);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quintic_hits_position_and_velocity_endpoints() {
        let segment = QuinticSegment::new(0.1, 0.8, 0.0, 0.5, 0.0, 0.0, 0.4);
        let (q0, v0, a0) = segment.sample(0.0);
        let (qf, vf, af) = segment.sample(segment.duration);
        assert!((q0 - 0.1).abs() < 1e-9);
        assert!((v0 - 0.0).abs() < 1e-6);
        assert!(a0.abs() < 1e-6);
        assert!((qf - 0.8).abs() < 1e-6);
        assert!((vf - 0.5).abs() < 1e-5);
        assert!(af.abs() < 1e-4);
    }

    #[test]
    fn nonzero_boundary_accelerations_are_reached() {
        let segment = QuinticSegment::new(0.0, 1.0, 0.0, 0.0, 2.0, -3.0, 0.5);
        let (q0, v0, a0) = segment.sample(0.0);
        let (qf, vf, af) = segment.sample(segment.duration);
        assert!((q0 - 0.0).abs() < 1e-9);
        assert!((v0 - 0.0).abs() < 1e-9);
        assert!((a0 - 2.0).abs() < 1e-6);
        assert!((qf - 1.0).abs() < 1e-6);
        assert!((vf - 0.0).abs() < 1e-5);
        assert!((af - (-3.0)).abs() < 1e-4);
    }

    /// 예전(시작/끝 가속 0 고정) 구현을 독립적으로 재구현해, 새 일반화
    /// 구현이 `a0=0.0, af=0.0`일 때 바이트 단위로 동일한 결과를 내는지
    /// 확인한다 — `.omc/plans/2026-07-31-nonzero-impact-knot-acceleration.md`
    /// Acceptance Criteria #1의 회귀 테스트.
    fn legacy_sample(q0: f64, qf: f64, v0: f64, vf: f64, duration: f64, t: f64) -> (f64, f64, f64) {
        let d = duration.max(f64::EPSILON);
        let d2 = d * d;
        let d3 = d2 * d;
        let d4 = d3 * d;
        let d5 = d4 * d;
        let a = Matrix3::new(
            d3,
            d4,
            d5,
            3.0 * d2,
            4.0 * d3,
            5.0 * d4,
            6.0 * d,
            12.0 * d2,
            20.0 * d3,
        );
        let b = NaVector3::new(qf - q0 - v0 * d, vf - v0, 0.0);
        let coeffs = a.lu().solve(&b).unwrap_or(NaVector3::zeros());
        let (c3, c4, c5) = (coeffs.x, coeffs.y, coeffs.z);
        let t = t.clamp(0.0, d);
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;
        let q = q0 + v0 * t + c3 * t3 + c4 * t4 + c5 * t5;
        let qd = v0 + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
        let qdd = 6.0 * c3 * t + 12.0 * c4 * t2 + 20.0 * c5 * t3;
        return (q, qd, qdd);
    }

    #[test]
    fn zero_acceleration_boundary_matches_legacy_shape() {
        let cases = [
            (0.1, 0.8, 0.0, 0.5, 0.4),
            (-1.2, 0.3, -0.5, 1.5, 0.25),
            (0.0, 0.0, 2.0, -2.0, 0.6),
            (3.0, -1.0, 0.0, 0.0, 0.15),
        ];
        for (q0, qf, v0, vf, duration) in cases {
            let segment = QuinticSegment::new(q0, qf, v0, vf, 0.0, 0.0, duration);
            for k in 0..=8 {
                let t = duration * k as f64 / 8.0;
                let (q, qd, qdd) = segment.sample(t);
                let (lq, lqd, lqdd) = legacy_sample(q0, qf, v0, vf, duration, t);
                assert!((q - lq).abs() < 1e-12, "q mismatch at t={t}");
                assert!((qd - lqd).abs() < 1e-12, "qd mismatch at t={t}");
                assert!((qdd - lqdd).abs() < 1e-12, "qdd mismatch at t={t}");
            }
        }
    }

    #[test]
    fn jerk_cost_is_zero_for_constant_velocity_motion() {
        // v0 == vf 이고 qf - q0 == v0 * duration이면 quintic 계수(c2..c5)가
        // 전부 0인 순수 등속 운동 — 저크가 항상 0이어야 한다.
        let segment = QuinticSegment::new(0.0, 2.0, 2.0, 2.0, 0.0, 0.0, 1.0);
        assert!(segment.jerk_cost() < 1e-12, "{}", segment.jerk_cost());
    }

    #[test]
    fn jerk_minimizing_knot_acceleration_is_a_true_minimum() {
        let (q0, v0, q1, v1, t1) = (0.0_f64, 0.0_f64, 1.0_f64, 0.8_f64, 0.3_f64);
        let (q2, v2, t2) = (1.3_f64, 0.0_f64, 0.2_f64);
        let cost_at = |knot: f64| -> f64 {
            let pre = QuinticSegment::new(q0, q1, v0, v1, 0.0, knot, t1);
            let post = QuinticSegment::new(q1, q2, v1, v2, knot, 0.0, t2);
            return pre.jerk_cost() + post.jerk_cost();
        };
        let a = QuinticSegment::jerk_minimizing_knot_acceleration(q0, v0, q1, v1, t1, q2, v2, t2);

        // 정점이면 도함수(중심차분)가 0에 가까워야 한다.
        let h = 1e-3;
        let slope = (cost_at(a + h) - cost_at(a - h)) / (2.0 * h);
        assert!(slope.abs() < 1e-2, "slope={slope} at a={a}");

        // 비용은 정확한 2차식(볼록)이므로, 이 정점이 예전 값(0.0)이나 멀리
        // 떨어진 값보다 비용이 낮거나 같아야 한다 — 전역 최솟값 확인.
        assert!(cost_at(a) <= cost_at(0.0) + 1e-9);
        assert!(cost_at(a) <= cost_at(a + 10.0) + 1e-9);
        assert!(cost_at(a) <= cost_at(a - 10.0) + 1e-9);
    }

    #[test]
    fn jerk_minimizing_knot_acceleration_stays_finite_across_representative_cases() {
        // 방어적 폴백(곡률≈0)이 정확히 언제 발동하는지는 손으로 구성하기
        // 어렵지만(경계 조건에 대한 아핀식이라 우연한 상쇄가 아니면 잘 안
        // 생김), 최소한 대표적인 짧은/긴 구간 조합에서 NaN·무한대로 새지
        // 않는지는 확인한다.
        let cases = [
            (0.0, 0.0, 1.0, 0.8, 0.3, 1.3, 0.0, 0.2),
            (0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.05),
            (-1.0, 2.0, 0.5, -1.0, 0.15, 0.2, 0.0, 0.35),
        ];
        for (q0, v0, q1, v1, t1, q2, v2, t2) in cases {
            let a =
                QuinticSegment::jerk_minimizing_knot_acceleration(q0, v0, q1, v1, t1, q2, v2, t2);
            assert!(a.is_finite(), "a={a} for case starting at q0={q0}");
        }
    }
}
