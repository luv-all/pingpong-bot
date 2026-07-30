//! 하드웨어에 넘기는 quintic 스윙 궤적.

use crate::robot::Joints;

use super::quintic_segment::QuinticSegment;
use super::rail::Rail;

/// 하드웨어에 넘기는 quintic 스윙 궤적.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
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
    pub rail: Rail,
    pub follow_through_rail_x: f64,
    pub follow_through_rail_velocity: f64,
}

impl Trajectory {
    /// quintic 세그먼트를 만든다.
    pub fn new(
        start: Joints,
        end: Joints,
        start_velocity: Vec<f64>,
        end_velocity: Vec<f64>,
        duration_secs: f64,
        rail: Rail,
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
        rail: Rail,
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

    /// `t` [s]에서 관절 각속도 [rad/s]를 샘플한다.
    pub fn sample_velocity_at(&self, t: f64) -> Vec<f64> {
        if t <= self.impact_time_secs || self.duration_secs <= self.impact_time_secs {
            return self
                .pre_impact_segments()
                .into_iter()
                .map(|segment| segment.sample(t).1)
                .collect();
        }
        return self
            .follow_through_segments()
            .into_iter()
            .map(|segment| segment.sample(t - self.impact_time_secs).1)
            .collect();
    }

    /// `t` [s]에서 관절 각가속도 [rad/s²]를 샘플한다.
    pub fn sample_acceleration_at(&self, t: f64) -> Vec<f64> {
        if t <= self.impact_time_secs || self.duration_secs <= self.impact_time_secs {
            return self
                .pre_impact_segments()
                .into_iter()
                .map(|segment| segment.sample(t).2)
                .collect();
        }
        return self
            .follow_through_segments()
            .into_iter()
            .map(|segment| segment.sample(t - self.impact_time_secs).2)
            .collect();
    }

    /// 임팩트 전/후 per-joint quintic 세그먼트 `(pre, post)`를 한 번에 만든다.
    ///
    /// Newton-Euler 토크 샘플링처럼 궤적을 여러 시점에서 반복 평가할 때, 매
    /// 샘플마다 세그먼트를 재구성(관절당 3x3 LU)하지 않고 한 번 만들어 두고
    /// `QuinticSegment::sample`로 `(각, 각속도, 각가속도)`를 뽑도록 노출한다.
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
        return self
            .peak_joint_accelerations()
            .into_iter()
            .fold(0.0_f64, f64::max);
    }

    /// 관절별 전 구간 peak |q̇| [rad/s].
    ///
    /// [`peak_joint_speed`]는 전 관절 최댓값 하나만 주므로 "어느 관절이
    /// 병목인가"를 알 수 없다. 관절마다 모터가 달라 속도 한계도 다르므로
    /// (MX-64R 63 rpm / MX-28T 55 rpm) 관절별 비교가 필요하다.
    ///
    /// [`peak_joint_speed`]: Self::peak_joint_speed
    pub fn peak_joint_speeds(&self) -> Vec<f64> {
        let pre = self.pre_impact_segments();
        let post = self.follow_through_segments();
        let n = pre.len().max(post.len());
        let mut out = vec![0.0_f64; n];
        for (index, segment) in pre.iter().enumerate() {
            out[index] = out[index].max(segment.max_speed(24));
        }
        for (index, segment) in post.iter().enumerate() {
            out[index] = out[index].max(segment.max_speed(24));
        }
        return out;
    }

    /// 관절별 전 구간 peak |α| [rad/s^2].
    pub fn peak_joint_accelerations(&self) -> Vec<f64> {
        let pre = self.pre_impact_segments();
        let post = self.follow_through_segments();
        let n = pre.len().max(post.len());
        let mut out = vec![0.0_f64; n];
        for (index, segment) in pre.iter().enumerate() {
            out[index] = out[index].max(segment.max_acceleration(24));
        }
        for (index, segment) in post.iter().enumerate() {
            out[index] = out[index].max(segment.max_acceleration(24));
        }
        return out;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::Joints;

    #[test]
    fn swing_trajectory_is_continuous_through_internal_impact_knot() {
        let trajectory = Trajectory::with_follow_through(
            Joints::from_slice(&[0.0]),
            Joints::from_slice(&[1.0]),
            Joints::from_slice(&[1.08]),
            vec![0.0],
            vec![0.8],
            vec![0.0],
            0.40,
            0.50,
            Rail {
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
