//! 시뮬 디버그용 월드 스냅샷.

use crate::defaults;
use crate::error::SwingPlanError;
use crate::robot::Arm;
use crate::robot::Joints;
use crate::robot::collision::{robot_obbs, table_penetration};
use crate::robot::motion;

use super::commit_phase::CommitPhase;
use super::obb::DebugObb;

const TRUTH_ARC_CAP: usize = 160;
const ARC_SAMPLE_MAX: usize = 48;
const GHOST_SAMPLES: usize = 32;

/// 실패·한계·궤적 오버레이용 데이터.
#[derive(Debug, Clone, Default)]
pub struct SimDebugSnapshot {
    pub last_fail: Option<SwingPlanError>,
    pub last_fail_text: Option<String>,
    pub unreachable_xyz: Option<[f64; 3]>,
    pub committed_racket_path: Vec<[f64; 3]>,
    pub predicted_arc: Vec<[f64; 3]>,
    pub truth_arc: Vec<[f64; 3]>,
    pub table_pen_depth: f64,
    /// 관통 중인 OBB (하이라이트용)
    pub penetrating_obbs: Vec<DebugObb>,
    pub joint_at_limit: Vec<bool>,
    pub torque_over: Vec<bool>,
    /// commit 궤적 관절별 peak |τ| [N·m]
    pub torque_peak_nm: Vec<f64>,
    /// 재생 중 현재 |τ| [N·m]
    pub torque_now_nm: Vec<f64>,
    pub accel_over: bool,
    pub net_gate_ok: Option<bool>,
    pub commit_phase: CommitPhase,
    pub omega: [f64; 3],
    /// 관절별 `|commit 시점 자세 → 임팩트 자세|` (WP12, 타격 전 이동량).
    pub pre_impact_travel: Vec<f64>,
    /// 관절별 `|임팩트 자세 → 팔로스루 종료 자세|` (WP12, 타격 후 이동량).
    pub follow_through_travel: Vec<f64>,
    /// 관절별 임팩트 순간 명령 각속도 [rad/s] (부호 있음, WP12).
    pub impact_velocity: Vec<f64>,
    /// 관절별 궤적 전 구간(타격 전+후) peak |q̇| [rad/s] (WP12, 정규화 기준).
    pub peak_joint_speed_overall: Vec<f64>,
    /// 관절별 임팩트 knot 공유 명령 각가속도 [rad/s²] (부호 있음).
    /// `0.0`이면 예전(타격 순간 가속도 강제 0) 동작과 동일 — 이 값이 0이
    /// 아니게 만드는 게 바로 이 수정의 목적이다. 상세:
    /// `.omc/plans/2026-07-31-nonzero-impact-knot-acceleration.md`.
    pub impact_acceleration: Vec<f64>,
    /// `set_torque_now`용 RNEA 스크래치 — 매 틱(최대 1kHz) 재할당을 피하려고
    /// 재사용한다 (`src/robot/motion/physics.rs`의 스크래치 재사용 패턴과 동일).
    torque_scratch: crate::robot::dynamics::RneaScratch,
}

impl SimDebugSnapshot {
    /// 새 발사 시 실패·궤적 오버레이를 비운다.
    pub fn reset_for_new_flight(&mut self) {
        self.last_fail = None;
        self.last_fail_text = None;
        self.unreachable_xyz = None;
        self.committed_racket_path.clear();
        self.predicted_arc.clear();
        self.truth_arc.clear();
        self.table_pen_depth = 0.0;
        self.penetrating_obbs.clear();
        self.joint_at_limit.clear();
        self.torque_over.clear();
        self.torque_peak_nm.clear();
        self.torque_now_nm.clear();
        self.accel_over = false;
        self.net_gate_ok = None;
        self.commit_phase = CommitPhase::Idle;
        self.omega = [0.0; 3];
        self.pre_impact_travel.clear();
        self.follow_through_travel.clear();
        self.impact_velocity.clear();
        self.peak_joint_speed_overall.clear();
        self.impact_acceleration.clear();
    }

    pub fn record_fail(&mut self, err: &SwingPlanError) {
        self.last_fail = Some(err.clone());
        self.last_fail_text = Some(err.to_string());
        self.unreachable_xyz = err.target_xyz();
    }

    pub fn record_abandon_text(&mut self, reason: &str) {
        self.last_fail_text = Some(reason.to_string());
        self.commit_phase = CommitPhase::Abandoned;
    }

    pub fn clear_fail_on_success(&mut self) {
        self.last_fail = None;
        self.last_fail_text = None;
        self.unreachable_xyz = None;
        self.commit_phase = CommitPhase::Committed;
    }

    pub fn set_committed_path(&mut self, arm: &Arm, trajectory: &motion::Trajectory) {
        self.committed_racket_path = sample_racket_path(arm, trajectory, GHOST_SAMPLES);
        // WP12: 커밋 시점 자세 → 임팩트 자세(타격 전), 임팩트 자세 → 팔로스루
        // 종료 자세(타격 후)의 관절별 이동량. `COARSE_TRACK_JOINT_FRACTION`이
        // 타격 전 구간의 Δq를 얼마나 남겨두는지(=사용자 보고 "타격 순간 멈춤"
        // 증상의 직접 지표) 재는 용도 — world.rs의 상수 doc comment 참고.
        self.pre_impact_travel = trajectory
            .start
            .values
            .iter()
            .zip(trajectory.end.values.iter())
            .map(|(a, b)| (b - a).abs())
            .collect();
        self.follow_through_travel = trajectory
            .end
            .values
            .iter()
            .zip(trajectory.follow_through.values.iter())
            .map(|(a, b)| (b - a).abs())
            .collect();
        self.impact_velocity = trajectory.end_velocity.clone();
        self.peak_joint_speed_overall = trajectory.peak_joint_speeds();
        self.impact_acceleration = trajectory.impact_acceleration.clone();
        let control = defaults::ControlParams::default();
        let duration = trajectory.duration_secs.max(f64::EPSILON);
        let samples = ((duration / 0.005).ceil() as usize).max(24);
        let n = arm.joint_count();
        // aggregated_inertials는 primitive·URDF 프리셋 모두 필수로 채워져
        // required_torque가 항상 RNEA로 계산된다 — 관성 데이터가 없어 스칼라
        // 근사(joint_inertia * alpha)로 폴백하던 옛 경로는 이제 없다.
        let mut peaks = vec![0.0_f64; n];
        for k in 0..=samples {
            let t = duration * k as f64 / samples as f64;
            let q = trajectory.sample_at(t);
            let qd = trajectory.sample_velocity_at(t);
            let qdd = trajectory.sample_acceleration_at(t);
            // 플래너 게이트(`peak_torque_utilization`)와 같은 모델을 봐야
            // HUD의 "τ 초과" 표시가 커밋 판정과 일치한다 — 회전자 반사관성 포함(WP8).
            if let Some(tau) = arm.required_torque_with_rotor(&q.values, &qd, &qdd) {
                for i in 0..n {
                    peaks[i] = f64::max(peaks[i], tau[i].abs());
                }
            }
        }
        self.torque_peak_nm = peaks.clone();
        self.torque_over = peaks
            .iter()
            .enumerate()
            .map(|(i, &peak)| {
                let limit = control.max_joint_torques.get(i).copied().unwrap_or(0.0);
                peak > limit + 1e-6
            })
            .collect();
        self.accel_over = trajectory.peak_joint_acceleration() > control.max_joint_accel + 1e-6;
    }

    /// 스윙 재생 중이면 궤적 샘플, 아니면 현재 자세(중력)로 τ를 채운다.
    ///
    /// 매 물리 틱(최대 1kHz) 호출되므로 스크래치·출력 버퍼를 재사용해
    /// (`required_joint_torques_into`) 힙 할당을 피한다 — 길이가 안 맞으면
    /// `required_torque`처럼 조용히 스킵(기존 값 유지).
    ///
    /// **여기만 회전자 반사관성을 일부러 뺀다**(WP8). 이 값은
    /// `SimWorld::drive_arm_motors`가 Rapier 관절 모터의 `motor_max_force`
    /// 예산으로 그대로 쓰는데, Rapier 다물체에는 회전자 관성이 아예 없어서
    /// (URDF `<inertial>`에만 의존) 시뮬 팔은 실물보다 가볍다. 가벼운 몸체에
    /// 실물 기준 토크 예산을 주면 시뮬이 실물보다 빨라져 sim-to-real 격차가
    /// 오히려 벌어진다. 제대로 된 해법은 Rapier 관절에도 armature를 넣는
    /// 것이며 별도 과제다 (`docs/measure-physics.md` 참고).
    pub fn set_torque_now(&mut self, arm: &Arm, q: &[f64], qd: &[f64], qdd: &[f64]) {
        let n = arm.joint_count();
        if q.len() != n || qd.len() != n || qdd.len() != n {
            return;
        }
        let joints = Joints::from_slice(q);
        crate::robot::dynamics::required_joint_torques_into(
            arm,
            &joints,
            qd,
            qdd,
            &mut self.torque_scratch,
            &mut self.torque_now_nm,
        );
    }

    /// 매 스텝: 관절·관통·ω·진실/예측 탄도.
    pub fn refresh_runtime(
        &mut self,
        arm: &Arm,
        rail_x: f64,
        joints: &Joints,
        ball_pos: nalgebra::Vector3<f64>,
        ball_vel: nalgebra::Vector3<f64>,
        omega: nalgebra::Vector3<f64>,
        in_flight: bool,
        physics: &defaults::PhysicsParams,
        hit_plane_y: f64,
    ) {
        self.omega = [omega.x, omega.y, omega.z];
        self.table_pen_depth = table_penetration(arm, rail_x, joints);
        self.penetrating_obbs = robot_obbs(arm, rail_x, joints)
            .into_iter()
            .filter(|obb| obb.table_penetration() > 1e-3)
            .map(|obb| DebugObb::from(&obb))
            .collect();
        self.joint_at_limit = joints
            .values
            .iter()
            .enumerate()
            .map(|(i, &q)| {
                let Some(limit) = arm.joint_limit(i) else {
                    return false;
                };
                const EPS: f64 = 1e-3;
                return q <= limit.min + EPS || q >= limit.max - EPS;
            })
            .collect();

        if in_flight {
            self.push_truth(ball_pos);
            self.net_gate_ok = Some(crate::physics::Kinematics::clears_net(
                ball_pos, ball_vel, omega, physics,
            ));
            self.predicted_arc = sample_predicted_arc(
                ball_pos,
                ball_vel,
                omega,
                physics,
                hit_plane_y,
                ARC_SAMPLE_MAX,
            );
        } else {
            self.net_gate_ok = None;
            if self.commit_phase != CommitPhase::Committed
                && self.commit_phase != CommitPhase::Abandoned
            {
                self.commit_phase = CommitPhase::Idle;
            }
        }
    }

    fn push_truth(&mut self, pos: nalgebra::Vector3<f64>) {
        self.truth_arc.push([pos.x, pos.y, pos.z]);
        if self.truth_arc.len() > TRUTH_ARC_CAP {
            let drop = self.truth_arc.len() - TRUTH_ARC_CAP;
            self.truth_arc.drain(0..drop);
        }
    }
}

fn sample_racket_path(arm: &Arm, trajectory: &motion::Trajectory, samples: usize) -> Vec<[f64; 3]> {
    let n = samples.max(2);
    let mut out = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = trajectory.duration_secs * i as f64 / n as f64;
        let joints = trajectory.sample_at(t);
        let rail = trajectory.sample_rail_at(t);
        if let Some(pose) = arm.forward_kinematics_with_rail(rail, &joints) {
            let p = pose.position.coords;
            out.push([p.x, p.y, p.z]);
        }
    }
    return out;
}

fn sample_predicted_arc(
    mut pos: nalgebra::Vector3<f64>,
    mut vel: nalgebra::Vector3<f64>,
    mut omega: nalgebra::Vector3<f64>,
    physics: &defaults::PhysicsParams,
    plane_y: f64,
    max_samples: usize,
) -> Vec<[f64; 3]> {
    let est = defaults::EstimatorParams::default();
    let mut out = Vec::with_capacity(max_samples);
    out.push([pos.x, pos.y, pos.z]);
    let mut t = 0.0;
    while out.len() < max_samples && t < est.max_lead {
        let (next_pos, next_vel, next_omega) =
            crate::physics::Kinematics::step(pos, vel, omega, est.integrate_dt, physics);
        pos = next_pos;
        vel = next_vel;
        omega = next_omega;
        t += est.integrate_dt;
        out.push([pos.x, pos.y, pos.z]);
        if pos.y <= plane_y || pos.z < 0.2 {
            break;
        }
    }
    return out;
}
