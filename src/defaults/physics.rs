//! 공–테이블 물리 계수.

use anyhow::{Result, ensure};

/// 해석된 물리 계수 (항상 concrete 값).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhysicsParams {
    /// 반발 e
    pub restitution: f64,
    /// 접선 마찰 mu
    pub friction: f64,
    /// 이차 항력 k
    pub drag: f64,
}

impl PhysicsParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            (0.0..=1.0).contains(&self.restitution),
            "restitution in 0..=1"
        );
        ensure!((0.0..=1.0).contains(&self.friction), "friction in 0..=1");
        ensure!(self.drag >= 0.0, "drag >= 0");
        return Ok(());
    }
}

pub fn physics() -> PhysicsParams {
    return PhysicsParams {
        restitution: 0.85,
        friction: 0.15,
        drag: 0.0,
    };
}
