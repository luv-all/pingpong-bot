//! 관절 quintic 궤적 (plan §7.5).

use nalgebra::{Matrix3, Vector3 as NaVector3};

use crate::types::{Joints, RailMotion, SwingTrajectory};

/// 관절 1축 quintic — 위치·속도 경계, 시작/끝 가속 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuinticSegment {
    q0: f64,
    duration: f64,
    c1: f64,
    c3: f64,
    c4: f64,
    c5: f64,
}

impl QuinticSegment {
    pub fn new(q0: f64, qf: f64, v0: f64, vf: f64, duration: f64) -> Self {
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
        let b = NaVector3::new(qf - q0 - v0 * t, vf - v0, 0.0);
        let coeffs = a.lu().solve(&b).unwrap_or(NaVector3::zeros());

        return Self {
            q0,
            duration: t,
            c1: v0,
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

        let q = self.q0 + self.c1 * t + self.c3 * t3 + self.c4 * t4 + self.c5 * t5;
        let qd = self.c1 + 3.0 * self.c3 * t2 + 4.0 * self.c4 * t3 + 5.0 * self.c5 * t4;
        let qdd = 6.0 * self.c3 * t + 12.0 * self.c4 * t2 + 20.0 * self.c5 * t3;
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
}

impl SwingTrajectory {
    /// quintic 세그먼트를 만든다.
    pub fn new(
        start: Joints,
        end: Joints,
        start_velocity: Vec<f64>,
        end_velocity: Vec<f64>,
        duration_secs: f64,
        rail: RailMotion,
    ) -> Self {
        return Self {
            start,
            end,
            start_velocity,
            end_velocity,
            duration_secs,
            rail,
        };
    }

    /// 임팩트 시점 목표 관절각 (하위 호환).
    pub fn goal_joints(&self) -> &Joints {
        return &self.end;
    }

    fn segments(&self) -> Vec<QuinticSegment> {
        let n = self
            .start
            .values
            .len()
            .min(self.end.values.len())
            .min(self.start_velocity.len())
            .min(self.end_velocity.len());
        let mut segments = Vec::with_capacity(n);
        for i in 0..n {
            segments.push(QuinticSegment::new(
                self.start.values[i],
                self.end.values[i],
                self.start_velocity[i],
                self.end_velocity[i],
                self.duration_secs,
            ));
        }
        return segments;
    }

    fn rail_segment(&self) -> QuinticSegment {
        return QuinticSegment::new(
            self.rail.start,
            self.rail.end,
            self.rail.start_velocity,
            self.rail.end_velocity,
            self.duration_secs,
        );
    }

    /// `t` [s]에서 레일 x [m]를 샘플한다.
    pub fn sample_rail_at(&self, t: f64) -> f64 {
        return self.rail_segment().sample(t).0;
    }

    /// 궤적 전 구간 최대 레일 속도 [m/s].
    pub fn peak_rail_speed(&self) -> f64 {
        return self.rail_segment().max_speed(24);
    }

    /// `t` [s]에서 관절각을 샘플한다.
    pub fn sample_at(&self, t: f64) -> Joints {
        let values = self
            .segments()
            .into_iter()
            .map(|segment| segment.sample(t).0)
            .collect();
        return Joints { values };
    }

    /// `t` [s]에서 관절 각속도 [rad/s].
    pub fn sample_velocities_at(&self, t: f64) -> Vec<f64> {
        return self
            .segments()
            .into_iter()
            .map(|segment| segment.sample(t).1)
            .collect();
    }

    /// 궤적 전 구간 최대 관절 각속도 [rad/s].
    pub fn peak_joint_speed(&self) -> f64 {
        return self
            .segments()
            .iter()
            .map(|segment| segment.max_speed(24))
            .fold(0.0_f64, f64::max);
    }

    /// 궤적 전 구간 최대 관절 각가속도 [rad/s²].
    pub fn peak_joint_acceleration(&self) -> f64 {
        return self
            .segments()
            .iter()
            .map(|segment| segment.max_acceleration(24))
            .fold(0.0_f64, f64::max);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quintic_hits_position_and_velocity_endpoints() {
        let segment = QuinticSegment::new(0.1, 0.8, 0.0, 0.5, 0.4);
        let (q0, v0, a0) = segment.sample(0.0);
        let (qf, vf, af) = segment.sample(segment.duration);
        assert!((q0 - 0.1).abs() < 1e-9);
        assert!((v0 - 0.0).abs() < 1e-6);
        assert!(a0.abs() < 1e-6);
        assert!((qf - 0.8).abs() < 1e-6);
        assert!((vf - 0.5).abs() < 1e-5);
        assert!(af.abs() < 1e-4);
    }
}
