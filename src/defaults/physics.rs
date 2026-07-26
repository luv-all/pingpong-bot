//! 공–테이블·네트 물리 계수.

use anyhow::{Result, ensure};

/// 해석된 물리 계수 (항상 concrete 값).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhysicsParams {
    /// 테이블·공 반발 e (공–테이블 접촉).
    pub restitution: f64,
    /// 테이블 접선 마찰 — 예측기 바운스 커널 μ (`table_ball_mu`).
    pub friction: f64,
    /// 공 collider 접선 마찰 (라켓·테이블과 Rapier Average).
    /// 라켓 접촉을 보존하려고 테이블 `friction`과 다를 수 있음.
    /// 테이블–공 Rapier 실효 μ는 `rapier_table_ball_mu` (현 Average≈0.3).
    pub ball_friction: f64,
    /// 네트 반발 e — 실체 콜라이더(비-sensor). `Min` combine으로 공 e와 합쳐
    /// soft/죽은 튕김. 0에 가까울수록 네트에 맞고 힘없이 떨어진다.
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
        // 예측기 바운스 μ = friction (0.4). Rapier 테이블–공은
        // Average(friction, ball_friction)≈0.3 — 갭은 `rapier_table_ball_mu` 테스트로
        // 고정. 재료/Max combine 정렬은 시뮬 그리드 재튜닝과 함께 (스펙 E3b 후속).
        friction: 0.4,
        ball_friction: 0.2,
        // soft 네트: 거의 죽어서 떨어지되 살짝 튕김 (sensor 관통 대신).
        net_restitution: 0.05,
        drag: 0.0,
        // C_m ρ R³ / m ≈ 1.2 * (0.02)^3 / 0.0027
        magnus: 0.00356,
    };
}
