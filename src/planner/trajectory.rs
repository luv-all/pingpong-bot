//! 관절 quintic 궤적.

use nalgebra::{Matrix3, Vector3 as NaVector3};

use crate::robot::Joints;

/// quintic 스윙에 딸린 리니어 X 이동.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RailMotion {
    pub start: f64,
    pub end: f64,
    pub start_velocity: f64,
    pub end_velocity: f64,
}

impl RailMotion {
    pub const fn fixed(x: f64) -> Self {
        return Self {
            start: x,
            end: x,
            start_velocity: 0.0,
            end_velocity: 0.0,
        };
    }
}

impl Default for RailMotion {
    fn default() -> Self {
        return Self::fixed(0.0);
    }
}

/// 하드웨어에 넘기는 quintic 스윙 궤적.
#[derive(Debug, Clone, PartialEq)]
pub struct SwingTrajectory {
    pub start: Joints,
    /// 임팩트 knot 관절각.
    pub end: Joints,
    /// 팔로스루 종료 관절각.
    pub follow_through: Joints,
    pub start_velocity: Vec<f64>,
    /// 임팩트 knot 관절 속도.
    pub end_velocity: Vec<f64>,
    /// 팔로스루 종료 관절 속도.
    pub follow_through_velocity: Vec<f64>,
    pub impact_time_secs: f64,
    pub duration_secs: f64,
    /// 시작→임팩트 레일 운동.
    pub rail: RailMotion,
    pub follow_through_rail_x: f64,
    pub follow_through_rail_velocity: f64,
}

/// 관절 1축 quintic - 위치/속도 경계, 시작/끝 가속 0.
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
            follow_through: end.clone(),
            end,
            start_velocity,
            follow_through_velocity: end_velocity.clone(),
            end_velocity,
            impact_time_secs: duration_secs,
            duration_secs,
            follow_through_rail_x: rail.end,
            follow_through_rail_velocity: rail.end_velocity,
            rail,
        };
    }

    #[allow(clippy::too_many_arguments)]
    pub fn with_follow_through(
        start: Joints,
        impact: Joints,
        end: Joints,
        start_velocity: Vec<f64>,
        impact_velocity: Vec<f64>,
        end_velocity: Vec<f64>,
        impact_time_secs: f64,
        duration_secs: f64,
        rail: RailMotion,
        follow_through_rail_x: f64,
        follow_through_rail_velocity: f64,
    ) -> Self {
        return Self {
            start,
            end: impact,
            follow_through: end,
            start_velocity,
            end_velocity: impact_velocity,
            follow_through_velocity: end_velocity,
            impact_time_secs,
            duration_secs,
            rail,
            follow_through_rail_x,
            follow_through_rail_velocity,
        };
    }

    /// 임팩트 시점 목표 관절각.
    pub fn goal_joints(&self) -> &Joints {
        return &self.end;
    }

    pub fn impact_joints(&self) -> &Joints {
        return &self.end;
    }

    pub fn end_joints(&self) -> &Joints {
        return &self.follow_through;
    }

    fn pre_impact_segments(&self) -> Vec<QuinticSegment> {
        let n = self.start.values.len();
        assert_eq!(self.end.values.len(), n, "impact joint count");
        assert_eq!(self.start_velocity.len(), n, "start velocity count");
        assert_eq!(self.end_velocity.len(), n, "impact velocity count");
        let mut segments = Vec::with_capacity(n);
        for i in 0..n {
            segments.push(QuinticSegment::new(
                self.start.values[i],
                self.end.values[i],
                self.start_velocity[i],
                self.end_velocity[i],
                self.impact_time_secs,
            ));
        }
        return segments;
    }

    fn follow_through_segments(&self) -> Vec<QuinticSegment> {
        let n = self.end.values.len();
        assert_eq!(self.follow_through.values.len(), n, "end joint count");
        assert_eq!(self.end_velocity.len(), n, "impact velocity count");
        assert_eq!(self.follow_through_velocity.len(), n, "end velocity count");
        let duration = (self.duration_secs - self.impact_time_secs).max(f64::EPSILON);
        let mut segments = Vec::with_capacity(n);
        for i in 0..n {
            segments.push(QuinticSegment::new(
                self.end.values[i],
                self.follow_through.values[i],
                self.end_velocity[i],
                self.follow_through_velocity[i],
                duration,
            ));
        }
        return segments;
    }

    fn pre_impact_rail_segment(&self) -> QuinticSegment {
        return QuinticSegment::new(
            self.rail.start,
            self.rail.end,
            self.rail.start_velocity,
            self.rail.end_velocity,
            self.impact_time_secs,
        );
    }

    fn follow_through_rail_segment(&self) -> QuinticSegment {
        return QuinticSegment::new(
            self.rail.end,
            self.follow_through_rail_x,
            self.rail.end_velocity,
            self.follow_through_rail_velocity,
            (self.duration_secs - self.impact_time_secs).max(f64::EPSILON),
        );
    }

    /// `t` [s]에서 레일 x [m]를 샘플한다.
    pub fn sample_rail_at(&self, t: f64) -> f64 {
        if t <= self.impact_time_secs || self.duration_secs <= self.impact_time_secs {
            return self.pre_impact_rail_segment().sample(t).0;
        }
        return self
            .follow_through_rail_segment()
            .sample(t - self.impact_time_secs)
            .0;
    }

    /// 궤적 전 구간 최대 레일 속도 [m/s].
    pub fn peak_rail_speed(&self) -> f64 {
        return self
            .pre_impact_rail_segment()
            .max_speed(24)
            .max(self.follow_through_rail_segment().max_speed(24));
    }

    /// `t` [s]에서 관절각을 샘플한다.
    pub fn sample_at(&self, t: f64) -> Joints {
        let values = if t <= self.impact_time_secs || self.duration_secs <= self.impact_time_secs {
            self.pre_impact_segments()
                .into_iter()
                .map(|segment| segment.sample(t).0)
                .collect()
        } else {
            self.follow_through_segments()
                .into_iter()
                .map(|segment| segment.sample(t - self.impact_time_secs).0)
                .collect()
        };
        return Joints { values };
    }

    /// 임팩트 전/후 per-joint quintic 세그먼트 `(pre, post)`를 한 번에 만든다.
    ///
    /// Newton-Euler 토크 샘플링처럼 궤적을 여러 시점에서 반복 평가할 때, 매
    /// 샘플마다 세그먼트를 재구성(관절당 3x3 LU)하지 않고 한 번 만들어 두고
    /// `QuinticSegment::sample`로 `(각, 각속도, 각가속도)`를 뽑도록 노출한다.
    /// 임팩트 시점 판정은 `t <= impact_time_secs`면 `pre`, 아니면 로컬 시간
    /// `t - impact_time_secs`로 `post`를 쓴다.
    pub fn joint_segments(&self) -> (Vec<QuinticSegment>, Vec<QuinticSegment>) {
        return (self.pre_impact_segments(), self.follow_through_segments());
    }

    /// 궤적 전 구간 최대 관절 각속도 [rad/s].
    pub fn peak_joint_speed(&self) -> f64 {
        let pre = self
            .pre_impact_segments()
            .iter()
            .map(|segment| segment.max_speed(24))
            .fold(0.0_f64, f64::max);
        let post = self
            .follow_through_segments()
            .iter()
            .map(|segment| segment.max_speed(24))
            .fold(0.0_f64, f64::max);
        return pre.max(post);
    }

    /// 궤적 전 구간 최대 관절 각가속도 [rad/s^2].
    pub fn peak_joint_acceleration(&self) -> f64 {
        let pre = self
            .pre_impact_segments()
            .iter()
            .map(|segment| segment.max_acceleration(24))
            .fold(0.0_f64, f64::max);
        let post = self
            .follow_through_segments()
            .iter()
            .map(|segment| segment.max_acceleration(24))
            .fold(0.0_f64, f64::max);
        return pre.max(post);
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

    #[test]
    fn swing_trajectory_is_continuous_through_internal_impact_knot() {
        let trajectory = SwingTrajectory::with_follow_through(
            Joints::from_slice(&[0.0]),
            Joints::from_slice(&[1.0]),
            Joints::from_slice(&[1.08]),
            vec![0.0],
            vec![0.8],
            vec![0.0],
            0.40,
            0.50,
            RailMotion {
                start: 0.2,
                end: 0.5,
                start_velocity: 0.0,
                end_velocity: 0.1,
            },
            0.51,
            0.0,
        );
        let impact = trajectory.sample_at(trajectory.impact_time_secs);
        let end = trajectory.sample_at(trajectory.duration_secs);
        assert!((impact.values[0] - 1.0).abs() < 1e-6);
        assert!((end.values[0] - 1.08).abs() < 1e-6);

        let dt = 1e-5;
        let before = trajectory
            .sample_at(trajectory.impact_time_secs - dt)
            .values[0];
        let after = trajectory
            .sample_at(trajectory.impact_time_secs + dt)
            .values[0];
        let velocity = (after - before) / (2.0 * dt);
        assert!((velocity - 0.8).abs() < 1e-3);
        assert!((trajectory.sample_rail_at(trajectory.duration_secs) - 0.51).abs() < 1e-6);
    }
}
