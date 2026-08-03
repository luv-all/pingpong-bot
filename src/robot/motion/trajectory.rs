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
    /// 임팩트 knot 관절 가속도 — 타격-전 세그먼트의 끝과 팔로스루 세그먼트의
    /// 시작이 공유하는 값(연속성). `0.0`이면 예전(타격 순간 가속도 강제 0)
    /// 동작과 동일하다. `Trajectory::new`(단순 점대점 이동)는 항상 `0.0`.
    /// 상세: `.omc/plans/2026-07-31-nonzero-impact-knot-acceleration.md`.
    pub impact_acceleration: Vec<f64>,
    pub impact_time_secs: f64,
    pub duration_secs: f64,
    /// 시작→임팩트 레일 운동.
    pub rail: Rail,
    pub follow_through_rail_x: f64,
    pub follow_through_rail_velocity: f64,
    /// 관절별 `(로컬 시작 오프셋 [s], 로컬 구간 길이 [s])` — pre-impact 구간에만
    /// 적용된다. `None`이면 모든 관절이 `impact_time_secs`를 그대로 공유한다
    /// (기존 동작, 이 필드가 없던 시절과 동일). `Some`이면 관절 i는 전역 시간
    /// `[offset, offset+duration]` 구간에서만 움직이고, 그 밖에서는 시작/끝
    /// 값에 정지한다 — 근위→원위 순서로 어긋난 채찍형 스윙
    /// ([`crate::robot::motion::fixed_swing`])에 쓰인다. 팔로스루 구간은
    /// 영향받지 않는다.
    pub joint_phase_offsets: Option<Vec<(f64, f64)>>,
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
        let n = end_velocity.len();
        return Self {
            start,
            follow_through: end.clone(),
            end,
            start_velocity,
            follow_through_velocity: end_velocity.clone(),
            end_velocity,
            impact_acceleration: vec![0.0; n],
            impact_time_secs: duration_secs,
            duration_secs,
            follow_through_rail_x: rail.end,
            follow_through_rail_velocity: rail.end_velocity,
            rail,
            joint_phase_offsets: None,
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
        impact_acceleration: Vec<f64>,
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
            impact_acceleration,
            impact_time_secs,
            duration_secs,
            rail,
            follow_through_rail_x,
            follow_through_rail_velocity,
            joint_phase_offsets: None,
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

    fn follow_through_segments(&self) -> Vec<QuinticSegment> {
        let n = self.end.values.len();
        assert_eq!(self.follow_through.values.len(), n, "end joint count");
        assert_eq!(self.end_velocity.len(), n, "impact velocity count");
        assert_eq!(self.follow_through_velocity.len(), n, "end velocity count");
        let duration = (self.duration_secs - self.impact_time_secs).max(f64::EPSILON);
        let mut segments = Vec::with_capacity(n);
        for i in 0..n {
            let impact_accel = self.impact_acceleration.get(i).copied().unwrap_or(0.0);
            segments.push(QuinticSegment::new(
                self.end.values[i],
                self.follow_through.values[i],
                self.end_velocity[i],
                self.follow_through_velocity[i],
                impact_accel,
                0.0,
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
            0.0,
            0.0,
            self.impact_time_secs,
        );
    }

    fn follow_through_rail_segment(&self) -> QuinticSegment {
        return QuinticSegment::new(
            self.rail.end,
            self.follow_through_rail_x,
            self.rail.end_velocity,
            self.follow_through_rail_velocity,
            0.0,
            0.0,
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

    /// 궤적 전 구간 최대 레일 가속도 [m/s²].
    ///
    /// [`Self::peak_rail_speed`]의 가속도 짝. 실기 AXL 스테이지는
    /// `RAIL_ACCEL_M_S2`로 제한되는데 `kinematic_limit_violation`은 레일
    /// **속도**만 검사해서, 플래너가 레일이 실제로 못 내는 궤적을 통과시킨다
    /// (WP5에서 발견, WP2a에서 계량).
    pub fn peak_rail_acceleration(&self) -> f64 {
        return self
            .pre_impact_rail_segment()
            .max_acceleration(24)
            .max(self.follow_through_rail_segment().max_acceleration(24));
    }

    /// `t` [s]에서 관절각을 샘플한다.
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

    /// `t` [s]에서 관절 각속도 [rad/s]를 샘플한다.
    pub fn sample_velocity_at(&self, t: f64) -> Vec<f64> {
        if t <= self.impact_time_secs || self.duration_secs <= self.impact_time_secs {
            return self
                .pre_impact_segments()
                .into_iter()
                .enumerate()
                .map(|(i, segment)| segment.sample(self.pre_impact_local_time(i, t)).1)
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
                .enumerate()
                .map(|(i, segment)| segment.sample(self.pre_impact_local_time(i, t)).2)
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

    /// `impact_acceleration`이 0이 아니면 타격-전 세그먼트의 끝과 팔로스루
    /// 세그먼트의 시작이 그 값을 그대로 공유해 knot에서 가속도가 연속이어야
    /// 한다(예전처럼 항상 0으로 꺾이지 않아야 함) —
    /// `.omc/plans/2026-07-31-nonzero-impact-knot-acceleration.md`.
    #[test]
    fn nonzero_impact_acceleration_is_continuous_through_the_knot() {
        let trajectory = Trajectory::with_follow_through(
            Joints::from_slice(&[0.0]),
            Joints::from_slice(&[1.0]),
            Joints::from_slice(&[1.08]),
            vec![0.0],
            vec![0.8],
            vec![0.0],
            vec![3.5],
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
        let just_before = trajectory.sample_acceleration_at(trajectory.impact_time_secs - 1e-9)[0];
        let just_after = trajectory.sample_acceleration_at(trajectory.impact_time_secs + 1e-9)[0];
        assert!((just_before - 3.5).abs() < 1e-3, "before={just_before}");
        assert!((just_after - 3.5).abs() < 1e-3, "after={just_after}");
    }

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
}
