//! 한 프레임 ChArUco 검출 결과.

/// 한 프레임 ChArUco 검출 + 오버레이 (인터랙티브 calib용).
#[derive(Debug, Clone)]
pub struct FrameDetect {
    /// 보정에 쓸 만한 코너 수 (≥ [`super::MIN_CHARUCO_CORNERS`])
    pub corners: usize,
    pub markers: usize,
    pub ok: bool,
}
