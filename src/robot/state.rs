//! 런타임 관절 상태 - sim/real encoder 읽기가 같은 타입을 채운다.

use super::{Arm, RacketPose};
use crate::Joints;

/// 런타임 관절 상태 - sim `RobotState`/real encoder 읽기가 같은 타입을 채운다.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotState {
    /// 리니어 레일 x [m]
    rail_x: f64,
    /// 리니어 목표 x [m]
    rail_target: f64,
    /// 현재 관절각
    angles: Joints,
    /// 추종 목표 관절각 (궤적 없을 때)
    targets: Joints,
    /// quintic 스윙 재생
    active_swing: Option<SwingPlayback>,
}

#[derive(Debug, Clone, PartialEq)]
struct SwingPlayback {
    trajectory: crate::SwingTrajectory,
    elapsed: f64,
    /// `advance_swing_torque_limited`용 관절 각속도 [rad/s].
    joint_vel: Vec<f64>,
}

impl RobotState {
    /// 초기 관절각/레일 x로 상태를 만든다.
    pub fn new(initial: Joints, rail_x: f64) -> Self {
        return Self {
            rail_x,
            rail_target: rail_x,
            targets: initial.clone(),
            angles: initial,
            active_swing: None,
        };
    }

    /// 리니어 레일 x [m].
    pub fn rail_x(&self) -> f64 {
        return self.rail_x;
    }

    /// 스윙 궤적 재생 중인지.
    pub fn is_swinging(&self) -> bool {
        return self.active_swing.is_some();
    }

    /// 현재 관절각.
    pub fn joints(&self) -> &Joints {
        return &self.angles;
    }

    /// 목표 관절각.
    pub fn targets(&self) -> &Joints {
        return &self.targets;
    }

    /// 목표 관절각을 직접 설정한다.
    pub fn set_targets(&mut self, targets: Joints) {
        self.targets = targets;
    }

    /// quintic 스윙 궤적을 시작한다 (이미 스윙 중이면 무시).
    pub fn begin_swing(&mut self, trajectory: crate::SwingTrajectory) {
        if self.active_swing.is_some() {
            return;
        }
        self.replace_swing(trajectory);
    }

    /// 스윙을 현재 포즈 기준 새 궤적으로 교체한다 (elapsed=0).
    pub fn replace_swing(&mut self, trajectory: crate::SwingTrajectory) {
        self.targets = trajectory.end_joints().clone();
        self.rail_target = trajectory.follow_through_rail_x;
        self.active_swing = Some(SwingPlayback {
            trajectory,
            elapsed: 0.0,
            joint_vel: Vec::new(),
        });
    }

    /// 진행 중 스윙을 취소한다 (다음 공 발사 전).
    pub fn cancel_swing(&mut self) {
        self.active_swing = None;
    }

    /// quintic 궤적을 `dt`만큼 진행한다. 완료 시 `true`.
    ///
    /// 계획된 임팩트·팔로스루 knot를 사후 clamp 없이 그대로 재생한다.
    /// 토크 포화 추종은 [`Self::advance_swing_torque_limited`] / Rapier [`crate::sim::ArmMultibody`].
    pub fn advance_swing(&mut self, _arm: &Arm, dt: f64) -> bool {
        let Some(playback) = &mut self.active_swing else {
            return false;
        };
        playback.elapsed += dt;
        let t = playback.elapsed.min(playback.trajectory.duration_secs);
        let sampled = playback.trajectory.sample_at(t);
        self.rail_x = playback.trajectory.sample_rail_at(t);
        self.angles = sampled;
        if playback.elapsed >= playback.trajectory.duration_secs {
            self.active_swing = None;
            return true;
        }
        return false;
    }

    /// 토크 한도(`τ_max/I`)로 목표 샘플을 추종한다 — 듀얼 yaw vs 단일 비교용.
    ///
    /// 관절 속도 상태 `ω`를 두고 `|α| ≤ τ_max/I`로 적분한다. 위치만 클램프하면
    /// 궤적 초반에 포화되지 않아 듀얼/단일이 같아 보인다.
    pub fn advance_swing_torque_limited(&mut self, _arm: &Arm, dt: f64) -> bool {
        let Some(playback) = &mut self.active_swing else {
            return false;
        };
        if playback.joint_vel.is_empty() {
            playback.joint_vel = vec![0.0; self.angles.values.len()];
        }
        playback.elapsed += dt;
        let t = playback.elapsed.min(playback.trajectory.duration_secs);
        let desired = playback.trajectory.sample_at(t);
        let desired_vel = playback.trajectory.sample_velocity_at(t);
        self.rail_x = playback.trajectory.sample_rail_at(t);

        let control = crate::tunables::current().control;
        let inertia = control.joint_inertia.max(1e-9);
        let n = self.angles.values.len().min(desired.values.len());
        for i in 0..n {
            let a_max = control.max_joint_torques.get(i).copied().unwrap_or(6.0) / inertia;
            let omega = playback.joint_vel[i];
            let omega_des = desired_vel.get(i).copied().unwrap_or(0.0);
            // PD-ish velocity chase, then saturate α by torque budget.
            let alpha_cmd = ((omega_des - omega) / dt.max(1e-9)
                + 40.0 * (desired.values[i] - self.angles.values[i]))
                .clamp(-a_max, a_max);
            let omega_next = omega + alpha_cmd * dt;
            self.angles.values[i] += 0.5 * (omega + omega_next) * dt;
            playback.joint_vel[i] = omega_next;
        }

        if playback.elapsed >= playback.trajectory.duration_secs {
            self.angles = desired;
            self.active_swing = None;
            return true;
        }
        return false;
    }

    /// 목표 관절각을 `max_speed` [rad/s]로 추종한다 (궤적 없을 때 폴백).
    ///
    /// 스윙(타격이든 복귀든)이 끝나는 순간 중앙 포즈(관절 `default_joints`,
    /// 레일 `default_x` = 테이블 폭 중앙)가 아니면 곧바로 복귀 궤적을 이어서
    /// 시작한다 — 실물 로봇은 모터 토크 한계 때문에 끝에서 끝으로 급하게 못
    /// 움직이므로, 매번 중앙으로 되돌아온 상태에서 다음 스윙을 시작해야 한다.
    pub fn step_toward_targets(&mut self, arm: &Arm, dt: f64) {
        if self.active_swing.is_some() {
            let finished = self.advance_swing(arm, dt);
            if finished && !self.is_at_center(arm) {
                let start = crate::RobotPose::new(self.rail_x, self.angles.clone());
                if let Ok(trajectory) = crate::plan_return_to_center(arm, &start) {
                    self.replace_swing(trajectory);
                }
            }
            return;
        }
        if let Some(rail) = &arm.rail {
            let diff = self.rail_target - self.rail_x;
            let step = (rail.max_speed * dt).min(diff.abs());
            self.rail_x += diff.signum() * step;
        }
        let n = self.angles.values.len().min(self.targets.values.len());
        for i in 0..n {
            let raw_diff = self.targets.values[i] - self.angles.values[i];
            let diff = if arm.joint_limit(i).is_none() {
                (raw_diff + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI
            } else {
                raw_diff
            };
            let step = (arm.max_joint_speed * dt).min(diff.abs());
            self.angles.values[i] += diff.signum() * step;
        }
        self.angles = crate::planner::collision::clamp_above_table(arm, self.rail_x, &self.angles);
    }

    /// 레일·관절이 이미 중앙 포즈(`Arm::default_joints`, `LinearRail::default_x`
    /// = 테이블 폭 중앙) 근처인지. `LinearRail::home_x`(레일 원점, x=0)는
    /// 부팅 시 "대기 위치"일 뿐 여기서 말하는 중앙이 아니다.
    fn is_at_center(&self, arm: &Arm) -> bool {
        const RAIL_EPSILON_M: f64 = 1e-3;
        const JOINT_EPSILON_RAD: f64 = 1e-3;

        let rail_center = arm.rail.as_ref().map_or(self.rail_x, |rail| rail.default_x());
        if (self.rail_x - rail_center).abs() > RAIL_EPSILON_M {
            return false;
        }
        return self
            .angles
            .values
            .iter()
            .zip(arm.default_joints.values.iter())
            .all(|(actual, center)| (actual - center).abs() <= JOINT_EPSILON_RAD);
    }

    /// 현재 관절각으로 FK 라켓 자세를 계산한다.
    pub fn racket_pose(&self, arm: &Arm) -> Option<RacketPose> {
        if arm.rail.is_some() {
            return arm.forward_kinematics_with_rail(self.rail_x, &self.angles);
        }
        return arm.forward_kinematics(&self.angles);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entry::{competition_tunables, install_competition_tunables};
    use crate::tunables::{ControlParams, EstimatorParams, ImpactParams, Tunables, install};
    use crate::{RailMotion, SwingTrajectory};

    #[test]
    fn playback_targets_and_reaches_follow_through_end() {
        install_competition_tunables();
        let arm = crate::entry::competition_arm().expect("arm");
        let start = arm.initial_state();
        let mut impact = start.joints().clone();
        impact.values[0] += 0.01;
        let mut end = impact.clone();
        end.values[0] += 0.01;
        let trajectory = SwingTrajectory::with_follow_through(
            start.joints().clone(),
            impact,
            end.clone(),
            vec![0.0; end.values.len()],
            vec![0.2; end.values.len()],
            vec![0.0; end.values.len()],
            0.20,
            0.26,
            RailMotion::fixed(start.rail_x()),
            start.rail_x(),
            0.0,
        );
        let mut state = start;
        state.replace_swing(trajectory);
        assert_eq!(state.targets, end);
        assert!(state.advance_swing(&arm, 0.26));
        for (actual, expected) in state.joints().values.iter().zip(end.values) {
            assert!((actual - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn dual_yaw_torque_tracks_farther_than_single() {
        install_competition_tunables();
        let arm = crate::entry::competition_arm().expect("arm");
        let start = arm.initial_state();
        let mut impact = start.joints().clone();
        impact.values[0] += 0.5;
        let end = impact.clone();
        let trajectory = SwingTrajectory::with_follow_through(
            start.joints().clone(),
            impact,
            end,
            vec![0.0; 4],
            vec![3.0; 4],
            vec![0.0; 4],
            0.05,
            0.08,
            RailMotion::fixed(start.rail_x()),
            start.rail_x(),
            0.0,
        );

        let mut dual = start.clone();
        dual.replace_swing(trajectory.clone());
        for _ in 0..8 {
            dual.advance_swing_torque_limited(&arm, 0.005);
        }
        let dual_q0 = dual.joints().values[0].abs();

        install(Tunables {
            control: ControlParams {
                max_joint_torques: [6.0, 6.0, 6.0, 6.0],
                ..competition_tunables().control
            },
            impact: ImpactParams {
                ..competition_tunables().impact
            },
            estimator: EstimatorParams {
                ..competition_tunables().estimator
            },
        });
        let mut single = start;
        single.replace_swing(trajectory);
        for _ in 0..8 {
            single.advance_swing_torque_limited(&arm, 0.005);
        }
        let single_q0 = single.joints().values[0].abs();
        assert!(
            dual_q0 > single_q0 + 1e-4,
            "τ0=12 should outpace τ0=6: dual={dual_q0} single={single_q0}"
        );
        install_competition_tunables();
    }
}
