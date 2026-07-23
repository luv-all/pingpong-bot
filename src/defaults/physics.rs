//! 공–테이블·네트 물리 계수.

use anyhow::{Result, ensure};

/// 해석된 물리 계수 (항상 concrete 값).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhysicsParams {
    /// 테이블·공 반발 e (공–테이블 접촉).
    pub restitution: f64,
    /// 테이블 접선 마찰 mu.
    pub friction: f64,
    /// 공 collider 접선 마찰 (테이블 `friction`과 Average combine).
    pub ball_friction: f64,
    /// 네트 반발 e.
    pub net_restitution: f64,
    /// 이차 항력 k — `a -= k |v| v`. Rapier 기본에는 항력 없음 → 0.
    pub drag: f64,
    /// Magnus `k_m` — `a += k_m (ω × v)`. plan §6 Model C.
    ///
    /// 대략 `C_m ρ R³ / m` (C_m≈1, ρ≈1.2, R=0.02, m=0.0027 → ≈0.0036).
    pub magnus: f64,
}

impl PhysicsParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            (0.0..=1.0).contains(&self.restitution),
            "restitution in 0..=1"
        );
        ensure!((0.0..=1.0).contains(&self.friction), "friction in 0..=1");
        ensure!(
            (0.0..=1.0).contains(&self.ball_friction),
            "ball_friction in 0..=1"
        );
        ensure!(
            (0.0..=1.0).contains(&self.net_restitution),
            "net_restitution in 0..=1"
        );
        ensure!(self.drag >= 0.0, "drag >= 0");
        ensure!(self.magnus >= 0.0, "magnus >= 0");
        return Ok(());
    }
}

pub fn physics() -> PhysicsParams {
    return PhysicsParams {
        // ITTF 테이블: 30 cm 낙하 → ~23 cm 반발 → e≈√(23/30)≈0.88.
        // (강판 규격 305→240–260 mm면 0.89–0.92. 목재 테이블은 약간 낮다.)
        restitution: 0.88,
        // Rapier 테이블과 동일 SSOT. 예전 하드코딩 0.4를 유지해 바운스
        // 접선 감쇠·랠리 회귀를 보존한다 (0.15면 랜덤 샷 그리드가 깨짐).
        friction: 0.4,
        ball_friction: 0.2,
        net_restitution: 0.3,
        drag: 0.0,
        // C_m ρ R³ / m ≈ 1.2 * (0.02)^3 / 0.0027
        magnus: 0.00356,
    };
}
