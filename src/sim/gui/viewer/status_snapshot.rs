//! 패널 Status 창 스냅샷.

use crate::defaults;
use crate::estimator::Prediction;
use crate::sim::gui::debug::CommitPhase;
use crate::sim::physics;
use crate::sim::physics::world::SimWorld;

/// 상태 표시용 스냅샷 — world 락을 메인 스레드에서 잡지 않기 위함.
#[derive(Clone, Debug)]
pub struct StatusSnapshot {
    pub ball_state: physics::BallState,
    pub sim_time: f64,
    pub ball_pos: (f32, f32, f32),
    pub ball_vel: (f32, f32, f32),
    pub joints: Vec<String>,
    /// hit plane 예측 (디버그)
    pub debug_prediction: Option<Prediction>,
    pub swing_committed: bool,
    pub swing_abandoned: bool,
    pub last_fail_text: Option<String>,
    pub unreachable_xyz: Option<[f64; 3]>,
    pub commit_phase: CommitPhase,
    pub table_pen_depth: f64,
    pub torque_over: Vec<bool>,
    pub torque_peak_nm: Vec<f64>,
    pub torque_now_nm: Vec<f64>,
    pub accel_over: bool,
    pub joint_at_limit: Vec<bool>,
    pub omega: [f64; 3],
    pub net_gate_ok: Option<bool>,
    /// 관절 월드 원점 [m] (앵커 HUD)
    pub joint_world: Vec<[f32; 3]>,
    pub joint_q: Vec<f64>,
    pub joint_q_min: Vec<Option<f64>>,
    pub joint_q_max: Vec<Option<f64>>,
    pub torque_limit_nm: Vec<f64>,
}

impl StatusSnapshot {
    /// 월드에서 한 프레임 분량의 상태를 읽는다.
    pub fn from_world(world: &SimWorld) -> Self {
        let bp = world.ball_position();
        let bv = world.ball_velocity();
        let snap = world.debug_snap();
        let arm = world.arm();
        let joints = world.robot().joints();
        let rail_x = world.robot().rail_x();
        let joint_world = arm
            .joint_origins_world(rail_x, joints)
            .unwrap_or_default()
            .into_iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        let control = defaults::ControlParams::default();
        let joint_q = joints.values.clone();
        let joint_q_min: Vec<Option<f64>> = (0..arm.joint_count())
            .map(|i| arm.joint_limit(i).map(|l| l.min))
            .collect();
        let joint_q_max: Vec<Option<f64>> = (0..arm.joint_count())
            .map(|i| arm.joint_limit(i).map(|l| l.max))
            .collect();
        let torque_limit_nm: Vec<f64> = (0..arm.joint_count())
            .map(|i| control.max_joint_torques.get(i).copied().unwrap_or(6.0))
            .collect();
        return Self {
            ball_state: world.ball_state,
            sim_time: world.sim_time,
            ball_pos: (bp.x, bp.y, bp.z),
            ball_vel: (bv.x, bv.y, bv.z),
            joints: world
                .robot()
                .joints()
                .values
                .iter()
                .map(|v| format!("{v:.2}"))
                .collect(),
            debug_prediction: world.debug_prediction().cloned(),
            swing_committed: world.swing_committed(),
            swing_abandoned: world.swing_abandoned(),
            last_fail_text: snap.last_fail_text.clone(),
            unreachable_xyz: snap.unreachable_xyz,
            commit_phase: snap.commit_phase,
            table_pen_depth: snap.table_pen_depth,
            torque_over: snap.torque_over.clone(),
            torque_peak_nm: snap.torque_peak_nm.clone(),
            torque_now_nm: snap.torque_now_nm.clone(),
            accel_over: snap.accel_over,
            joint_at_limit: snap.joint_at_limit.clone(),
            omega: snap.omega,
            net_gate_ok: snap.net_gate_ok,
            joint_world,
            joint_q,
            joint_q_min,
            joint_q_max,
            torque_limit_nm,
        };
    }
}
