//! 하드웨어에 넘기는 스윙 궤적 — 타격-전 구간은 quintic 또는 quadratic(등가속),
//! 팔로스루는 항상 quintic.

use crate::robot::Joints;

use super::quadratic_segment::{DelayedQuadraticSegment, QuadraticSegment};
use super::quintic_segment::QuinticSegment;
use super::ramp_cruise_segment::RampCruiseSegment;
use super::rail::Rail;

/// 타격-전 구간에 쓸 세그먼트 모양.
///
/// [`Trajectory::new`]/[`Trajectory::with_follow_through`]는 항상 `Quintic`을
/// 쓰고([`QuinticSegment`]가 `a0=0` ease-in), [`Trajectory::with_quadratic_push`]만
/// `Quadratic`을 쓴다 — 휴지 자세(t=0, v=0)가 등가속 포물선의 꼭짓점이 되는
/// 프로파일로, quintic의 ease-in이 강제하는 "느린 시작"이 없다.
#[derive(Debug, Clone, Copy, PartialEq)]
enum SegmentProfile {
    Quintic,
    Quadratic,
}

/// 타격-전 세그먼트를 한 축으로 통일해 다루기 위한 래퍼.
///
/// `pub(crate)`인 이유: [`Trajectory::joint_segments`]가 크레이트 내부(토크
/// 샘플링, `physics.rs`)에 노출한다.
#[derive(Debug, Clone, Copy, PartialEq)]
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

/// 하드웨어에 넘기는 스윙 궤적.
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
    pre_impact_profile: SegmentProfile,
    /// 관절별 타격-전 프로파일 — 비어 있으면(기본) `pre_impact_profile`(전역)을
    /// 쓴다. [`Trajectory::with_power_sweep`]만 채운다.
    pre_impact_profiles: Vec<PreImpactJointProfile>,
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
            pre_impact_profile: SegmentProfile::Quintic,
            pre_impact_profiles: Vec::new(),
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
            pre_impact_profile: SegmentProfile::Quintic,
            pre_impact_profiles: Vec::new(),
        };
    }

    /// 타격-전 구간을 등가속(quadratic)으로 만든다 — `start_velocity`(보통
    /// 0)에서 출발해 `impact_time_secs` 뒤 `impact` 관절각에 도달하는 유일한
    /// 등가속을 관절마다 풀고, 그 시점의 속도·가속도를 그대로 knot 경계값으로
    /// 써서 팔로스루(quintic)가 이어받는다. quintic 버전과 달리 임팩트
    /// 속도는 별도로 지정하지 않는다 — `(Δq, T)`가 정해지면 유도값이다.
    #[allow(clippy::too_many_arguments)]
    pub fn with_quadratic_push(
        start: Joints,
        impact: Joints,
        end: Joints,
        start_velocity: Vec<f64>,
        follow_through_velocity: Vec<f64>,
        impact_time_secs: f64,
        duration_secs: f64,
        rail: Rail,
        follow_through_rail_x: f64,
        follow_through_rail_velocity: f64,
    ) -> Self {
        let n = impact.values.len();
        assert_eq!(start.values.len(), n, "start joint count");
        assert_eq!(start_velocity.len(), n, "start velocity count");
        let mut end_velocity = Vec::with_capacity(n);
        let mut impact_acceleration = Vec::with_capacity(n);
        for i in 0..n {
            let segment = QuadraticSegment::new(
                start.values[i],
                start_velocity[i],
                impact.values[i],
                impact_time_secs,
            );
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
            pre_impact_profiles: Vec::new(),
        };
    }

    /// 관절마다 다른 타격-전 프로파일(정지 출발 등가속 / 지연 스냅 /
    /// 가속-순항)을 섞어 쓰는 "파워 스윙" 궤적을 만든다. 세 프로파일 모두
    /// `start_velocity=0`을 전제한다.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn with_power_sweep(
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
    pub(crate) fn joint_segments(&self) -> (Vec<PreImpactSegment>, Vec<QuinticSegment>) {
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

    /// [`Trajectory::with_quadratic_push`]는 타격-전 구간을 등가속(quadratic)
    /// 세그먼트로 만든다 — 시작 속도 0에서 출발해 목표 관절각까지 일정
    /// 가속도로 밀고, 그 지점에서의 속도·가속도를 팔로스루(quintic) 시작
    /// 경계조건으로 그대로 넘겨받아야(knot에서 위치·속도·가속도 모두 연속)
    /// 한다.
    #[test]
    fn quadratic_push_trajectory_reaches_targets_and_is_continuous_through_the_knot() {
        let trajectory = Trajectory::with_quadratic_push(
            Joints::from_slice(&[0.0]),
            Joints::from_slice(&[1.0]),
            Joints::from_slice(&[1.08]),
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

        let start = trajectory.sample_at(0.0);
        let impact = trajectory.sample_at(trajectory.impact_time_secs);
        let end = trajectory.sample_at(trajectory.duration_secs);
        assert!((start.values[0] - 0.0).abs() < 1e-9);
        assert!((impact.values[0] - 1.0).abs() < 1e-6);
        assert!((end.values[0] - 1.08).abs() < 1e-6);

        // 등가속이므로 v(T) = 2*Δq/T (v0=0).
        let expected_impact_velocity = 2.0 * 1.0 / 0.40;
        let impact_velocity = trajectory.sample_velocity_at(trajectory.impact_time_secs)[0];
        assert!(
            (impact_velocity - expected_impact_velocity).abs() < 1e-6,
            "impact_velocity={impact_velocity}"
        );

        // knot에서 위치/속도/가속도가 모두 연속이어야 한다(팔로스루가 같은
        // 값에서 출발).
        let dt = 1e-6;
        let vel_before = trajectory.sample_velocity_at(trajectory.impact_time_secs - dt)[0];
        let vel_after = trajectory.sample_velocity_at(trajectory.impact_time_secs + dt)[0];
        assert!(
            (vel_before - vel_after).abs() < 1e-3,
            "velocity discontinuous at knot: before={vel_before} after={vel_after}"
        );
        let accel_before = trajectory.sample_acceleration_at(trajectory.impact_time_secs - 1e-9)[0];
        let accel_after = trajectory.sample_acceleration_at(trajectory.impact_time_secs + 1e-9)[0];
        assert!(
            (accel_before - accel_after).abs() < 1e-3,
            "acceleration discontinuous at knot: before={accel_before} after={accel_after}"
        );

        // 등가속 구간 전체에서 가속도가 상수여야 한다(quintic ease-in과의 핵심 차이).
        let accel_start = trajectory.sample_acceleration_at(0.0)[0];
        let accel_mid = trajectory.sample_acceleration_at(0.20)[0];
        assert!(
            (accel_start - accel_mid).abs() < 1e-6,
            "start={accel_start} mid={accel_mid}"
        );
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
        assert!(
            (held.values[0] - -0.5).abs() < 1e-9,
            "wrist should still be cocked: {held:?}"
        );
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
        assert!(
            (mid.values[1] - -0.5).abs() < 1e-9,
            "j1 should still be held: {mid:?}"
        );
    }
}
