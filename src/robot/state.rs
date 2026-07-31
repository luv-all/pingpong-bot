//! 런타임 관절 상태 - sim/real encoder 읽기가 같은 타입을 채운다.

use super::playback_trajectory::PlaybackTrajectory;
use super::swing_playback::SwingPlayback;
use super::{Arm, RacketPose};
use crate::robot::Joints;
use crate::robot::motion;

/// 런타임 관절 상태 - sim/real encoder 읽기가 같은 타입을 채운다.
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    /// 리니어 레일 x [m]
    rail_x: f64,
    /// 리니어 목표 x [m]
    rail_target: f64,
    /// 리니어 레일 현재 속도 [m/s] — `RAIL_ACCEL_M_S2` 가속 제한 적분 상태.
    ///
    /// 속도만 제한하면(예전 동작) 레일이 한 틱 만에 정지→최고속으로 뛰는데,
    /// 실기 AXL 스테이지는 `RAIL_ACCEL_M_S2`로 가속·감속한다.
    rail_vel: f64,
    /// 현재 관절각
    angles: Joints,
    /// 추종 목표 관절각 (궤적 없을 때)
    targets: Joints,
    /// 스윙 재생(quintic 또는 순수 토크 bang-bang)
    active_swing: Option<SwingPlayback>,
    /// 위치 이동 후 이어서 돌아갈 출발 포즈.
    return_pose_after_motion: Option<crate::robot::Pose>,
    /// 스윙 종료 후 `plan_return_to_center` 자동 복귀 (메인 sim 기본 on, jog off).
    auto_return_to_center: bool,
}

impl State {
    /// 초기 관절각/레일 x로 상태를 만든다.
    pub fn new(initial: Joints, rail_x: f64) -> Self {
        return Self {
            rail_x,
            rail_target: rail_x,
            rail_vel: 0.0,
            targets: initial.clone(),
            angles: initial,
            active_swing: None,
            return_pose_after_motion: None,
            auto_return_to_center: true,
        };
    }

    /// 스윙이 끝나면 테이블 중앙으로 자동 복귀할지 (메인 랠리 sim용).
    pub fn set_auto_return_to_center(&mut self, enabled: bool) {
        self.auto_return_to_center = enabled;
    }

    pub fn auto_return_to_center(&self) -> bool {
        return self.auto_return_to_center;
    }

    /// 스윙 취소 후 관절·레일을 즉시 스냅 (플래그 유지).
    pub fn snap_to_pose(&mut self, pose: crate::robot::Pose) {
        self.active_swing = None;
        self.return_pose_after_motion = None;
        self.rail_x = pose.rail_x;
        self.rail_target = pose.rail_x;
        self.rail_vel = 0.0;
        self.angles = pose.joints.clone();
        self.targets = pose.joints;
    }

    /// 스윙 재생 중이면 `(elapsed, q, qd, qdd)` — RNEA HUD용.
    ///
    /// bang-bang은 각가속도를 직접 노출하지 않아 `qdd=0`으로 둔다
    /// (중력·코리올리스만 보이는 근사).
    pub fn active_swing_sample(&self) -> Option<(f64, Vec<f64>, Vec<f64>, Vec<f64>)> {
        let playback = self.active_swing.as_ref()?;
        let t = playback.elapsed.min(playback.trajectory.duration_secs());
        let q = playback.trajectory.sample_at(t).values;
        let qd = playback.trajectory.sample_velocity_at(t);
        let qdd = match &playback.trajectory {
            PlaybackTrajectory::Quintic(trajectory) => trajectory.sample_acceleration_at(t),
            PlaybackTrajectory::BangBang(_) => vec![0.0; q.len()],
        };
        return Some((playback.elapsed, q, qd, qdd));
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

    /// 리니어 레일 목표 x [m]를 직접 설정한다.
    ///
    /// `set_targets`의 레일 짝. 보간은 하지 않는다 — [`Self::advance_rail`]이
    /// 설정된 목표를 향해 `rail.max_speed`·`RAIL_ACCEL_M_S2`로 접근한다.
    pub fn set_rail_target(&mut self, rail_x: f64) {
        self.rail_target = rail_x;
    }

    /// quintic 스윙 궤적을 시작한다 (이미 스윙 중이면 무시).
    pub fn begin_swing(&mut self, trajectory: motion::Trajectory) {
        if self.active_swing.is_some() {
            return;
        }
        self.replace_swing(trajectory);
    }

    /// 스윙을 현재 포즈 기준 새 quintic 궤적으로 교체한다 (elapsed=0).
    pub fn replace_swing(&mut self, trajectory: motion::Trajectory) {
        self.replace_playback(PlaybackTrajectory::Quintic(trajectory), 0.0);
    }

    /// 목표 위치 이동을 시작하고, 완료 후 출발 포즈로 복귀한다.
    pub fn replace_motion_and_return(
        &mut self,
        trajectory: motion::Trajectory,
        return_pose: crate::robot::Pose,
    ) {
        // 정밀 예측으로 진행 중 궤적을 교체해도 복귀점은 재계획 순간의
        // 자세가 아니라 공을 받기 전 출발 자세여야 한다.
        if self.return_pose_after_motion.is_none() {
            self.return_pose_after_motion = Some(return_pose);
        }
        self.replace_swing(trajectory);
    }

    /// 스윙을 현재 포즈 기준 새 순수 토크 bang-bang 궤적으로 교체한다
    /// (elapsed=0) - GUI "bang-bang swing" 토글이 켜졌을 때 `replace_swing`
    /// 대신 쓴다.
    pub fn replace_bang_bang_swing(&mut self, trajectory: motion::bang_bang::Trajectory) {
        self.replace_playback(PlaybackTrajectory::BangBang(trajectory), 0.0);
    }

    /// `replace_bang_bang_swing`과 같지만 재생을 `elapsed`[s] 지점부터
    /// 시작한다 — 백그라운드 워커에서 계획을 받아오는 동안 흐른 sim 시간을
    /// 보정하기 위함(`sim::physics::bang_bang_worker`). 계획은 "요청한
    /// 순간부터 `Tg` 안에 도달"을 가정하는데, 계산이 끝나 커밋되는 시점은
    /// 그보다 늦으므로 `elapsed=0`으로 재생을 시작하면 이미 지나간 시간만큼
    /// 뒤로 밀려 실제 공 도착보다 늦게 움직이는 것처럼 보인다.
    pub fn replace_bang_bang_swing_at(
        &mut self,
        trajectory: motion::bang_bang::Trajectory,
        elapsed: f64,
    ) {
        self.replace_playback(PlaybackTrajectory::BangBang(trajectory), elapsed);
    }

    fn replace_playback(&mut self, trajectory: PlaybackTrajectory, elapsed: f64) {
        let elapsed = elapsed.clamp(0.0, trajectory.duration_secs());
        self.targets = trajectory.sample_at(elapsed);
        self.angles = self.targets.clone();
        self.rail_target = trajectory.follow_through_rail_x();
        self.rail_x = trajectory.sample_rail_at(elapsed);
        // 궤적 재생 중에는 레일 위치를 궤적이 직접 준다 — 슬루 적분 상태를
        // 남겨두면 재생이 끝난 뒤 낡은 속도로 튄다.
        self.rail_vel = 0.0;
        self.active_swing = Some(SwingPlayback {
            trajectory,
            elapsed,
            joint_vel: Vec::new(),
        });
    }

    /// 재생 중인 quintic 스윙 궤적 (없거나 bang-bang이면 `None`).
    pub fn active_trajectory(&self) -> Option<&motion::Trajectory> {
        return match self.active_swing.as_ref().map(|s| &s.trajectory) {
            Some(PlaybackTrajectory::Quintic(trajectory)) => Some(trajectory),
            _ => None,
        };
    }

    /// 진행 중 스윙을 취소한다 (다음 공 발사 전).
    pub fn cancel_swing(&mut self) {
        self.active_swing = None;
        self.return_pose_after_motion = None;
    }

    /// 시뮬 폐루프: 궤적·레일 명령만 갱신한다. 측정 관절각은 건드리지 않는다.
    pub fn step_commands(&mut self, arm: &Arm, dt: f64) {
        if self.active_swing.is_some() {
            let finished = self.advance_swing_commands(dt);
            if finished && let Some(return_pose) = self.return_pose_after_motion.take() {
                self.start_return_to_pose(arm, return_pose);
            } else if finished && self.auto_return_to_center && !self.is_at_center(arm) {
                let start = crate::robot::Pose::new(self.rail_x, self.angles.clone());
                if let Ok(trajectory) = motion::Planner::return_to_center(arm, &start) {
                    self.replace_swing(trajectory);
                }
            }
            return;
        }
        self.advance_rail(arm, dt);
    }

    /// 레일을 `rail_target` 쪽으로 한 틱 옮긴다 — 속도 한계(`rail.max_speed`)와
    /// **가속도 한계**([`RAIL_ACCEL_M_S2`](crate::defaults::motion::RAIL_ACCEL_M_S2))를
    /// 모두 지키는 사다리꼴 프로파일 근사.
    ///
    /// 남은 거리 `d`에서 `a`로 정확히 멈출 수 있는 속도는 `√(2a|d|)`다. 목표
    /// 속도를 `min(max_speed, √(2a|d|))`로 잡으면 가속 구간·순항 구간·감속
    /// 구간이 자동으로 나오고 오버슛 없이 정지한다.
    ///
    /// 왜 필요한가: 속도 제한만 있던 예전 코드는 정지 상태에서 한 틱 만에
    /// `max_speed`로 뛰었다 — 실기 AXL 리니어 스테이지가 못 하는 동작이라
    /// sim이 실기보다 낙관적인 coarse 추종을 보여줬다.
    fn advance_rail(&mut self, arm: &Arm, dt: f64) {
        let Some(rail) = &arm.rail else {
            return;
        };
        let diff = self.rail_target - self.rail_x;
        if diff == 0.0 && self.rail_vel == 0.0 {
            return;
        }
        let accel = crate::defaults::motion::RAIL_ACCEL_M_S2;
        let brake_speed = (2.0 * accel * diff.abs()).sqrt();
        let desired_vel = diff.signum() * rail.max_speed.min(brake_speed);
        self.rail_vel += (desired_vel - self.rail_vel).clamp(-accel * dt, accel * dt);
        let step = self.rail_vel * dt;
        if step.abs() >= diff.abs() {
            self.rail_x = self.rail_target;
            self.rail_vel = 0.0;
        } else {
            self.rail_x += step;
        }
    }

    /// coarse(commit 전) 선추종 목표를 `goal` 쪽으로 **rate-limit** 해서 갱신한다.
    ///
    /// [`Self::set_targets`]와 달리 한 틱에 `arm.max_joint_speed · dt` 이상은
    /// 못 움직인다. sim의 Rapier 위치-PD 모터는 `targets`를 그대로 스텝 입력으로
    /// 받고 `motor_max_force`(토크 한계)로만 눌리므로, 목표를 매 틱 통째로
    /// 바꾸면 실기에 없는 무제한 스텝 입력이 된다.
    ///
    /// 갱신된 목표는 [`clamp_above_table`](crate::robot::collision::clamp_above_table)로
    /// 테이블 위로 클램프한다 — 명령 자세 자체가 테이블을 파고들지 않게 한다.
    pub fn slew_targets_toward(&mut self, arm: &Arm, goal: &Joints, dt: f64) {
        let n = self.targets.values.len().min(goal.values.len());
        for i in 0..n {
            let raw_diff = goal.values[i] - self.targets.values[i];
            let diff = if arm.joint_limit(i).is_none() {
                (raw_diff + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI
            } else {
                raw_diff
            };
            let step = (arm.max_joint_speed * dt).min(diff.abs());
            self.targets.values[i] += diff.signum() * step;
        }
        self.targets = crate::robot::collision::clamp_above_table(arm, self.rail_x, &self.targets);
    }

    /// 스윙 시계만 진행하고 `targets`·`rail_x`를 샘플한다 (각도 덮어쓰기 없음).
    fn advance_swing_commands(&mut self, dt: f64) -> bool {
        let Some(playback) = &mut self.active_swing else {
            return false;
        };
        playback.elapsed += dt;
        let duration = playback.trajectory.duration_secs();
        let t = playback.elapsed.min(duration);
        self.targets = playback.trajectory.sample_at(t);
        self.rail_x = playback.trajectory.sample_rail_at(t);
        if playback.elapsed >= duration {
            self.active_swing = None;
            return true;
        }
        return false;
    }

    /// 물리 다물체에서 읽은 관절각을 반영한다.
    pub fn set_measured_joints(&mut self, joints: Joints) {
        self.angles = joints;
    }

    /// 재생 중인 스윙(quintic 또는 bang-bang)을 `dt`만큼 진행한다. 완료 시 `true`.
    ///
    /// 계획된 궤적을 사후 clamp 없이 그대로 재생한다.
    /// 시뮬 폐루프는 [`Self::step_commands`] + 다물체 측정을 쓴다.
    /// 토크 포화 추종은 [`Self::advance_swing_torque_limited`] / Rapier [`crate::sim::physics::ArmMultibody`].
    pub fn advance_swing(&mut self, _arm: &Arm, dt: f64) -> bool {
        let Some(playback) = &mut self.active_swing else {
            return false;
        };
        playback.elapsed += dt;
        let duration = playback.trajectory.duration_secs();
        let t = playback.elapsed.min(duration);
        let sampled = playback.trajectory.sample_at(t);
        self.rail_x = playback.trajectory.sample_rail_at(t);
        self.angles = sampled;
        if playback.elapsed >= duration {
            self.active_swing = None;
            return true;
        }
        return false;
    }

    /// 토크 한도(`τ_max/I`)로 목표 샘플을 추종한다 — 듀얼 yaw vs 단일 비교용.
    ///
    /// 관절 속도 상태 `ω`를 두고 `|α| ≤ τ_max/I`로 적분한다. 위치만 클램프하면
    /// 궤적 초반에 포화되지 않아 듀얼/단일이 같아 보인다.
    ///
    /// **시뮬 런타임은 이 경로를 쓰지 않는다** — `sim/physics/world.rs`는
    /// [`Self::step_commands`](Rapier 다물체 측정 폐루프)로 간다. 이 메서드는
    /// `dual_yaw_torque_tracks_farther_than_single` 단위테스트에서만 호출되는,
    /// 듀얼 yaw 모터 토크 예산이 단일보다 실제로 유리하다는 설계 가정을 검증하는
    /// 독립 회귀테스트다 — 프로덕션 경로에 연결이 빠진 게 아니라 의도된 상태.
    pub fn advance_swing_torque_limited(
        &mut self,
        _arm: &Arm,
        dt: f64,
        control: &crate::defaults::ControlParams,
    ) -> bool {
        let Some(playback) = &mut self.active_swing else {
            return false;
        };
        if playback.joint_vel.is_empty() {
            playback.joint_vel = vec![0.0; self.angles.values.len()];
        }
        playback.elapsed += dt;
        let duration = playback.trajectory.duration_secs();
        let t = playback.elapsed.min(duration);
        let desired = playback.trajectory.sample_at(t);
        let desired_vel = playback.trajectory.sample_velocity_at(t);
        self.rail_x = playback.trajectory.sample_rail_at(t);

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

        if playback.elapsed >= duration {
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
            if finished && let Some(return_pose) = self.return_pose_after_motion.take() {
                self.start_return_to_pose(arm, return_pose);
            } else if finished && self.auto_return_to_center && !self.is_at_center(arm) {
                let start = crate::robot::Pose::new(self.rail_x, self.angles.clone());
                if let Ok(trajectory) = motion::Planner::return_to_center(arm, &start) {
                    self.replace_swing(trajectory);
                }
            }
            return;
        }
        self.advance_rail(arm, dt);
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
        self.angles = crate::robot::collision::clamp_above_table(arm, self.rail_x, &self.angles);
    }

    fn start_return_to_pose(&mut self, arm: &Arm, return_pose: crate::robot::Pose) {
        let start = crate::robot::Pose::new(self.rail_x, self.angles.clone());
        if let Ok(trajectory) =
            motion::Planner::move_to(arm, &start, return_pose.joints, return_pose.rail_x)
        {
            // 복귀 궤적 완료 후에는 다시 자동 복귀를 시작하지 않는다.
            self.replace_swing(trajectory);
        }
    }

    /// 레일·관절이 이미 중앙 포즈(`Arm::default_joints`, `LinearRail::default_x`
    /// = 테이블 폭 중앙) 근처인지. `LinearRail::home_x`(레일 원점, x=0)는
    /// 부팅 시 "대기 위치"일 뿐 여기서 말하는 중앙이 아니다.
    fn is_at_center(&self, arm: &Arm) -> bool {
        const RAIL_EPSILON_M: f64 = 1e-3;
        const JOINT_EPSILON_RAD: f64 = 1e-3;

        let rail_center = arm
            .rail
            .as_ref()
            .map_or(self.rail_x, |rail| rail.default_x());
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
    use crate::defaults::ControlParams;
    use crate::robot::motion;

    #[test]
    fn playback_targets_and_reaches_follow_through_end() {
        let arm = crate::defaults::primitive_4dof().expect("arm").arm;
        let start = arm.initial_state();
        let mut impact = start.joints().clone();
        impact.values[0] += 0.01;
        let mut end = impact.clone();
        end.values[0] += 0.01;
        let trajectory = motion::Trajectory::with_follow_through(
            start.joints().clone(),
            impact,
            end.clone(),
            vec![0.0; end.values.len()],
            vec![0.2; end.values.len()],
            vec![0.0; end.values.len()],
            vec![0.0; end.values.len()],
            0.20,
            0.26,
            motion::Rail::fixed(start.rail_x()),
            start.rail_x(),
            0.0,
        );
        let mut state = start;
        let start_joints = state.joints().clone();
        state.replace_swing(trajectory);
        assert_eq!(state.targets, start_joints);
        assert!(state.advance_swing(&arm, 0.26));
        for (actual, expected) in state.joints().values.iter().zip(end.values) {
            assert!((actual - expected).abs() < 1e-12);
        }
    }

    /// 레일은 정지 상태에서 한 틱 만에 `max_speed`로 뛰지 못한다 —
    /// `RAIL_ACCEL_M_S2`가 실제로 걸리는지 잠근다(WP5 회귀 가드).
    ///
    /// 참고: `RAIL_MAX_SPEED=5.0`에 도달하려면 `v²/2a = 1.04 m`가 필요한데
    /// 레일 전장은 `table::WIDTH_X=1.525 m`다 — 실제 프로파일은 사다리꼴이
    /// 아니라 **삼각형**이고 순항 구간이 없다. 그래서 "max_speed에 도달하는가"가
    /// 아니라 "가속 한계를 지키며 오버슛 없이 도착하는가"로 판정한다.
    #[test]
    fn rail_slew_obeys_acceleration_limit_from_rest() {
        const DT: f64 = 1.0 / 1000.0;
        let arm = crate::defaults::primitive_4dof().expect("arm").arm;
        let rail = *arm.rail.as_ref().expect("rail");
        let mut state = arm.initial_state();
        let start = state.rail_x();
        let goal = rail.x_max;
        state.set_rail_target(goal);

        let accel = crate::defaults::motion::RAIL_ACCEL_M_S2;
        state.step_commands(&arm, DT);
        let first = (state.rail_x() - start).abs();
        assert!(
            first <= accel * DT * DT * 1.001,
            "첫 틱 이동 {first} > a·dt² = {} — 가속 제한이 안 걸렸다",
            accel * DT * DT
        );
        assert!(
            first < rail.max_speed * DT * 0.5,
            "첫 틱에 예전(속도만 제한) 스텝 {}에 근접했다: {first}",
            rail.max_speed * DT
        );

        // 매 틱 속도 증가분이 a·dt를 넘지 않고, max_speed도 안 넘고,
        // 오버슛 없이 목표에 도착해야 한다.
        let mut prev_speed = first / DT;
        let mut arrived = None;
        for step in 0..5_000 {
            let before = state.rail_x();
            state.step_commands(&arm, DT);
            let speed = (state.rail_x() - before).abs() / DT;
            assert!(
                speed <= prev_speed + accel * DT * 1.001,
                "틱 {step}에서 속도가 {prev_speed} → {speed} 로 뛰었다 (한계 a·dt={})",
                accel * DT
            );
            assert!(speed <= rail.max_speed * 1.001, "max_speed 초과: {speed}");
            prev_speed = speed;
            if (state.rail_x() - goal).abs() < 1e-9 {
                arrived = Some(step);
                break;
            }
        }
        assert!(arrived.is_some(), "레일이 목표에 도착하지 못했다");
        assert!(
            state.rail_x() <= goal + 1e-9,
            "오버슛: {} > {goal}",
            state.rail_x()
        );
    }

    /// coarse 선추종 목표는 `max_joint_speed`로 슬루된다 — `set_targets`처럼
    /// 한 틱에 통째로 점프하면 Rapier 모터에 무제한 스텝 입력이 된다(WP5).
    #[test]
    fn coarse_slew_limits_target_step_to_max_joint_speed() {
        const DT: f64 = 1.0 / 1000.0;
        let arm = crate::defaults::primitive_4dof().expect("arm").arm;
        let mut state = arm.initial_state();
        let before = state.targets().clone();
        let mut goal = before.clone();
        for value in goal.values.iter_mut() {
            *value += 1.0;
        }

        state.slew_targets_toward(&arm, &goal, DT);

        let cap = arm.max_joint_speed * DT;
        for (i, (after, start)) in state
            .targets()
            .values
            .iter()
            .zip(before.values.iter())
            .enumerate()
        {
            let moved = (after - start).abs();
            assert!(
                moved <= cap * 1.001,
                "q{i} 목표가 한 틱에 {moved} rad 움직였다 (상한 {cap})"
            );
        }
        assert!(
            state.targets().values != before.values,
            "슬루가 목표를 전혀 못 옮겼다"
        );
    }

    #[test]
    fn dual_yaw_torque_tracks_farther_than_single() {
        let arm = crate::defaults::primitive_4dof().expect("arm").arm;
        let start = arm.initial_state();
        let mut impact = start.joints().clone();
        impact.values[0] += 0.5;
        let end = impact.clone();
        let trajectory = motion::Trajectory::with_follow_through(
            start.joints().clone(),
            impact,
            end,
            vec![0.0; 4],
            vec![3.0; 4],
            vec![0.0; 4],
            vec![0.0; 4],
            0.05,
            0.08,
            motion::Rail::fixed(start.rail_x()),
            start.rail_x(),
            0.0,
        );

        let dual_ctrl = ControlParams::default();
        let mut dual = start.clone();
        dual.replace_swing(trajectory.clone());
        for _ in 0..8 {
            dual.advance_swing_torque_limited(&arm, 0.005, &dual_ctrl);
        }
        let dual_q0 = dual.joints().values[0].abs();

        let single_ctrl = ControlParams {
            max_joint_torques: [3.0, 3.0, 1.25, 1.25],
            ..ControlParams::default()
        };
        let mut single = start;
        single.replace_swing(trajectory);
        for _ in 0..8 {
            single.advance_swing_torque_limited(&arm, 0.005, &single_ctrl);
        }
        let single_q0 = single.joints().values[0].abs();
        assert!(
            dual_q0 > single_q0 + 1e-4,
            "τ0=6 should outpace τ0=3: dual={dual_q0} single={single_q0}"
        );
    }
}
