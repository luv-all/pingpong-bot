//! 시뮬 디버그용 월드 스냅샷 (뷰어·Status가 읽음).

use crate::SwingPlanError;
use crate::defaults;
use crate::estimator::ballistics;
use crate::planner::collision::{OrientedBox, robot_obbs, table_penetration};
use crate::{Arm, Joints, SwingTrajectory};

/// 스윙 commit 게이트 단계.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CommitPhase {
    #[default]
    Idle,
    WaitMidcourt,
    WaitWindow,
    InWindow,
    TooLate,
    Committed,
    Abandoned,
}

impl CommitPhase {
    pub fn as_str(self) -> &'static str {
        return match self {
            Self::Idle => "idle",
            Self::WaitMidcourt => "wait midcourt",
            Self::WaitWindow => "wait window",
            Self::InWindow => "in window",
            Self::TooLate => "too late",
            Self::Committed => "committed",
            Self::Abandoned => "abandoned",
        };
    }

    /// Status 창용 한글 설명.
    pub fn label_ko(self) -> &'static str {
        return match self {
            Self::Idle => "대기",
            Self::WaitMidcourt => "상대 코트 — 대기 중",
            Self::WaitWindow => "아직 이름 — 창 대기",
            Self::InWindow => "스윙 결정 가능",
            Self::TooLate => "너무 늦음",
            Self::Committed => "스윙 확정",
            Self::Abandoned => "이번 공 포기",
        };
    }
}

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
    pub accel_over: bool,
    pub net_gate_ok: Option<bool>,
    pub commit_phase: CommitPhase,
    pub omega: [f64; 3],
}

/// 뷰어용 OBB (중심·half extents·축).
#[derive(Debug, Clone, Copy)]
pub struct DebugObb {
    pub center: [f64; 3],
    pub half_extents: [f64; 3],
    /// 열-우선 9개: axes.column(0), column(1), column(2)
    pub axes: [[f64; 3]; 3],
}

impl From<&OrientedBox> for DebugObb {
    fn from(obb: &OrientedBox) -> Self {
        return Self {
            center: [obb.center.x, obb.center.y, obb.center.z],
            half_extents: [obb.half_extents.x, obb.half_extents.y, obb.half_extents.z],
            axes: [
                [
                    obb.axes.column(0).x,
                    obb.axes.column(0).y,
                    obb.axes.column(0).z,
                ],
                [
                    obb.axes.column(1).x,
                    obb.axes.column(1).y,
                    obb.axes.column(1).z,
                ],
                [
                    obb.axes.column(2).x,
                    obb.axes.column(2).y,
                    obb.axes.column(2).z,
                ],
            ],
        };
    }
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
        self.accel_over = false;
        self.net_gate_ok = None;
        self.commit_phase = CommitPhase::Idle;
        self.omega = [0.0; 3];
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

    pub fn set_committed_path(&mut self, arm: &Arm, trajectory: &SwingTrajectory) {
        self.committed_racket_path = sample_racket_path(arm, trajectory, GHOST_SAMPLES);
        let control = defaults::control();
        let peaks = trajectory.peak_joint_accelerations();
        self.torque_over = peaks
            .iter()
            .enumerate()
            .map(|(i, &alpha)| {
                let limit = control.max_joint_torques.get(i).copied().unwrap_or(0.0);
                control.joint_inertia * alpha > limit + 1e-6
            })
            .collect();
        self.accel_over = trajectory.peak_joint_acceleration() > control.max_joint_accel + 1e-6;
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
            self.net_gate_ok = Some(ballistics::clears_net_gate(
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

fn sample_racket_path(arm: &Arm, trajectory: &SwingTrajectory, samples: usize) -> Vec<[f64; 3]> {
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
    omega: nalgebra::Vector3<f64>,
    physics: &defaults::PhysicsParams,
    plane_y: f64,
    max_samples: usize,
) -> Vec<[f64; 3]> {
    let est = defaults::estimator();
    let mut out = Vec::with_capacity(max_samples);
    out.push([pos.x, pos.y, pos.z]);
    let mut t = 0.0;
    while out.len() < max_samples && t < est.max_lead {
        let (next_pos, next_vel) =
            ballistics::semi_implicit_euler(pos, vel, omega, est.integrate_dt, physics);
        pos = next_pos;
        vel = next_vel;
        t += est.integrate_dt;
        out.push([pos.x, pos.y, pos.z]);
        if pos.y <= plane_y || pos.z < 0.2 {
            break;
        }
    }
    return out;
}
