//! UI용 최신 프레임 슬롯.

use anyhow::Result;
use opencv::core::Mat;
use opencv::prelude::*;

pub struct PreviewSlot {
    pub left: Mat,
    pub right: Mat,
    pub grab_fps: f64,
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
            ring_secs: self.ring_secs,
            ring_pairs: self.ring_pairs,
        });
    }
}
