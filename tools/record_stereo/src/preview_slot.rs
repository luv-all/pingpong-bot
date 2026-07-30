//! UI용 최신 프레임 슬롯.

use anyhow::Result;
use opencv::core::Mat;
use opencv::prelude::*;

pub struct PreviewSlot {
    pub left: Mat,
    pub right: Mat,
    /// 우리 루프가 쌍을 처리한 속도.
    pub grab_fps: f64,
    /// 카메라가 실제로 프레임을 내주는 속도 (`ThreadedCapture` 내부 grab 스레드).
    ///
    /// `grab_fps`와 나란히 봐야 병목이 갈린다 — 둘이 같으면 카메라 공급이 한계고,
    /// `capture_fps`가 더 높으면 우리 루프가 못 따라가는 것이다.
    pub capture_fps: (f64, f64),
    pub ring_secs: f64,
    pub ring_pairs: usize,
}

impl PreviewSlot {
    pub fn try_clone(&self) -> Result<Self> {
        return Ok(Self {
            left: self
                .left
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone left: {e}"))?,
            right: self
                .right
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone right: {e}"))?,
            grab_fps: self.grab_fps,
            capture_fps: self.capture_fps,
            ring_secs: self.ring_secs,
            ring_pairs: self.ring_pairs,
        });
    }
}
