//! egui 입력 초안.

use pingpong_bot::sim::launch;

use super::kind::Kind;

/// egui 입력 초안.
#[derive(Debug, Clone)]
pub struct Draft {
    pub kind: Kind,
    pub joint_index: usize,
    pub joint_deg: f64,
    pub angles_deg: [f64; 4],
    pub rail_x: f64,
    /// IK / Pose: 현재 라켓 위치 대비 Δ(좌우, 전후, 높이) [m].
    pub reach_dxyz: [f64; 3],
    /// Pose: 현재 법선 기준 기울기 [deg].
    pub tilt_pitch_deg: f64,
    pub tilt_yaw_deg: f64,
    /// Swing: 슈터 설정 (sim controls와 동기화된다).
    pub shooter: launch::Settings,
}

impl Default for Draft {
    fn default() -> Self {
        return Self {
            kind: Kind::Joint,
            joint_index: 0,
            joint_deg: 0.0,
            angles_deg: [0.0; 4],
            rail_x: 0.0,
            reach_dxyz: [0.0; 3],
            tilt_pitch_deg: 0.0,
            tilt_yaw_deg: 0.0,
            shooter: launch::Settings::default(),
        };
    }
}
