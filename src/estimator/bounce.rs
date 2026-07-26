//! 테이블–공 바운스 SSOT.
//!
//! 법선: \(v_n'=-e v_n\). 접선: \(v_t'=(1-\mu)v_t\). \(\omega'=\omega\).
//!
//! μ 기본값은 `physics.friction` (예측기 레거시). Rapier 테이블–공은
//! Average(`friction`,`ball_friction`)이라 실효값이 다를 수 있음 — 재료/combine
//! 정렬은 회귀 그리드와 함께 올린다 (`table_ball_mu` vs `rapier_table_ball_mu`).

use nalgebra::Vector3;

use crate::defaults::PhysicsParams;

/// 예측기 바운스 커널이 쓰는 μ.
#[inline]
pub fn table_ball_mu(physics: &PhysicsParams) -> f64 {
    return physics.friction;
}

/// Rapier 기본 Average combine의 테이블–공 실효 μ.
#[inline]
pub fn rapier_table_ball_mu(physics: &PhysicsParams) -> f64 {
    return 0.5 * (physics.friction + physics.ball_friction);
}

pub fn table_bounce(
    v: Vector3<f64>,
    omega: Vector3<f64>,
    physics: &PhysicsParams,
) -> (Vector3<f64>, Vector3<f64>) {
    let mu = table_ball_mu(physics).clamp(0.0, 1.0);
    let e = physics.restitution.clamp(0.0, 1.0);
    let tang_scale = 1.0 - mu;
    let v_out = Vector3::new(v.x * tang_scale, v.y * tang_scale, -v.z * e);
    return (v_out, omega);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::defaults;

    #[test]
    fn bounce_matches_legacy_friction_formula() {
        let p = defaults::physics();
        let v = Vector3::new(1.0, -2.0, -3.0);
        let w = Vector3::new(0.1, -0.2, 0.3);
        let (v2, w2) = table_bounce(v, w, &p);
        assert!((v2.z - (-v.z * p.restitution)).abs() < 1e-12);
        assert!((v2.x - v.x * (1.0 - p.friction)).abs() < 1e-12);
        assert!((v2.y - v.y * (1.0 - p.friction)).abs() < 1e-12);
        assert!((w2 - w).norm() < 1e-15);
    }

    #[test]
    fn documents_rapier_average_gap_until_material_align() {
        let p = defaults::physics();
        assert!((table_ball_mu(&p) - 0.4).abs() < 1e-15);
        assert!((rapier_table_ball_mu(&p) - 0.3).abs() < 1e-15);
    }
}
