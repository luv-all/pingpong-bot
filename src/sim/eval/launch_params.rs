//! Eval 전용 발사 설정.

/// Eval 전용 발사 설정 — Shooter 패널과 분리 (실기 리모컨 대응).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaunchParams {
    /// 발사 속도 [m/s].
    pub speed_mps: f64,
    /// 좌·우 존 yaw 절대값 [deg] (중앙은 0).
    pub side_yaw_deg: f64,
}

impl Default for LaunchParams {
    fn default() -> Self {
        return Self {
            speed_mps: 6.0,
            // 테이블 1/3 바깥쪽을 겨냥하는 대략값 (±).
            side_yaw_deg: 10.0,
        };
    }
}
