//! ChArUco 보정 결과 메타.

/// 보정 결과 메타 (로그용).
#[derive(Debug, Clone)]
pub struct Report {
    pub rms: f64,
    pub frames_used: usize,
    pub frames_total: usize,
}
