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

impl Default for PhysicsParams {
    fn default() -> Self {
        return Self {
            // 실측 (2026-08-12, `measure-restitution --clip fly_45..53`, 9클립 12바운스
            // 창 회귀 — `src/physics/measure/traj_measure.rs`): 클립별 e 중앙값 0.718,
            // 범위 0.107~0.829(fly_46 0.107 하나가 낮은 이상치, 중앙값엔 거의 영향 없음).
            // 예전 ITTF 규격값(30cm 낙하→23cm 반발, e≈0.88)을 대체 — 이 테이블·공으로 잰
            // 적이 없던 값이었다.
            restitution: 0.72,
            // 아직 미실측 — 그대로 둔다. 같은 실측 세션에서 접선 비율(`friction_from_
            // tangential_speeds`)로 재봤더니 0.13~0.90으로 전혀 안 모였는데, 그 공식이
            // 스핀을 안 본다(폐기된 `(1-μ)v_t` 커널 전제) — 지금 `table_bounce`(Coulomb)는
            // 접선 임펄스가 스핀에도 걸려서, 서브마다 다른 스핀이 그대로 "마찰 산포"로
            // 새어 들어간다. 스핀을 같이 풀기 전엔(다음 단계) 이 비율 하나로 μ를 못 잡는다.
            friction: 0.4,
            ball_friction: 0.2,
            net_restitution: 0.05,
            drag: 0.0,
            // C_m ρ R³ / m ≈ 1.2 * (0.02)^3 / 0.0027
            magnus: 0.00356,
        };
    }
}
