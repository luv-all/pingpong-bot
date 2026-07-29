//! egui 입력 초안.

use pingpong_bot::constants::table;

use super::kind::MotionKind;

/// egui 입력 초안.
#[derive(Debug, Clone)]
pub struct MotionDraft {
    pub kind: MotionKind,
    pub joint_index: usize,
    pub joint_deg: f64,
    pub angles_deg: [f64; 4],
    pub rail_x: f64,
    /// IK / Pose / Swing 공통: 현재 라켓 위치 대비 Δ(좌우, 전후, 높이) [m].
    pub reach_dxyz: [f64; 3],
    /// Pose / Swing: 현재 법선 기준 기울기 [deg].
    pub tilt_pitch_deg: f64,
    pub tilt_yaw_deg: f64,
    pub swing_speed: f64,
    /// AimBall / SwingBall: 공 도달 월드 좌표 [m].
    pub arrival_xyz: [f64; 3],
    /// SwingBall: 공 입사 속도 [m/s].
    pub ball_vin: [f64; 3],
}

impl Default for MotionDraft {
    fn default() -> Self {
        return Self {
            kind: MotionKind::Joint,
            joint_index: 0,
            joint_deg: 0.0,
            angles_deg: [0.0; 4],
            rail_x: 0.0,
            reach_dxyz: [0.0; 3],
            tilt_pitch_deg: 0.0,
            tilt_yaw_deg: 0.0,
            swing_speed: 1.5,
            arrival_xyz: [
                table::WIDTH_X * 0.5,
                table::DEFAULT_HIT_PLANE_Y,
                table::SURFACE_Z + 0.18,
            ],
            ball_vin: [0.0, -6.0, -1.5],
        };
    }
}
