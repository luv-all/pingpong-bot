//! sim 실행 설정.

/// sim 실행 설정.
#[derive(Debug, Clone, Copy)]
pub struct SimSessionConfig {
    /// 물리 적분 주파수 [Hz] — 공 CCD용 (plan §9)
    pub physics_hz: f64,
    /// 가상 카메라 프레임률 [Hz]
    pub frame_hz: f64,
    /// 1.0 = 실시간, 10.0 = 10배속
    pub time_scale: f64,
    /// sim 가상 카메라 대수
    pub camera_count: u8,
}

impl Default for SimSessionConfig {
    fn default() -> Self {
        return Self {
            physics_hz: 1000.0,
            frame_hz: 120.0,
            time_scale: 1.0,
            camera_count: 3,
        };
    }
}
