//! 테이블–공 바운스 SSOT.
//!
//! 법선: \(v_n'=-e\,v_n\).
//!
//! 접선: Coulomb. 공 아래 접촉점의 슬립 속도 \(v_c = v_t + \omega\times(-R\hat z)\)를
//! 마찰 임펄스로 줄인다. 미끄러지면 \(|J_t| = \mu J_n\), 그 전에 슬립이 0이 되면
//! (구름 전이) \(|J_t| = |v_c| / (1/m + R^2/I)\). 스핀은 \(\omega' = \omega + (r\times J_t)/I\).
//!
//! 예전 커널은 \(v_t'=(1-\mu)v_t\) + \(\omega'=\omega\)였다. 이 형태는 법선
//! 임펄스와 무관하게 접선을 일정 비율로 깎아서, 입사각이 얕을수록(= 실제
//! 랠리 대부분) Rapier보다 접선속도를 과하게 줄인다. 2026-07-27 실측:
//! 커널 \(v_y'=-3.90\) vs Rapier \(-4.68\) → 0.2 s 예측에서 임팩트 높이가
//! 105 mm 어긋나 라켓이 공 위를 지나갔다 (eval 30발 중 16발 헛스윙).

use nalgebra::Vector3;

use crate::constants::ball;
use crate::defaults::PhysicsParams;

/// 테이블–공 접촉의 실효 마찰계수.
///
/// Rapier가 테이블 collider `friction`과 공 collider `ball_friction`을 기본
/// Average combine으로 합치므로, 예측 커널도 같은 값을 써야 예측 임팩트와
/// 시뮬 임팩트가 어긋나지 않는다.
#[inline]
pub fn table_ball_mu(physics: &PhysicsParams) -> f64 {
    return rapier_table_ball_mu(physics);
}

/// Rapier 기본 Average combine의 테이블–공 실효 μ.
#[inline]
pub fn rapier_table_ball_mu(physics: &PhysicsParams) -> f64 {
    return 0.5 * (physics.friction + physics.ball_friction);
}

/// 접촉점 슬립을 0으로 만드는(구름 전이) 접선 임펄스 크기 계수.
///
/// \(\Delta v_c = J_t\,(1/m + R^2/I)\) 이므로 \(J_{stick} = |v_c| / (1/m + R^2/I)\).
#[inline]
fn stick_impulse_per_slip() -> f64 {
    return 1.0 / (1.0 / ball::MASS + ball::RADIUS * ball::RADIUS / ball::SHELL_INERTIA);
}

pub fn table_bounce(
    v: Vector3<f64>,
    omega: Vector3<f64>,
    physics: &PhysicsParams,
) -> (Vector3<f64>, Vector3<f64>) {
    let e = physics.restitution.clamp(0.0, 1.0);
    if v.z >= 0.0 {
        return (v, omega);
    }
    let v_out_z = -v.z * e;

    // 공 아래 극점(테이블 접촉점)의 속도 중 접선 성분.
    let arm = Vector3::new(0.0, 0.0, -ball::RADIUS);
    let contact = v + omega.cross(&arm);
    let slip = Vector3::new(contact.x, contact.y, 0.0);
    let slip_speed = slip.norm();
    if slip_speed < 1e-9 {
        return (Vector3::new(v.x, v.y, v_out_z), omega);
    }

    let mu = table_ball_mu(physics).max(0.0);
    let normal_impulse = ball::MASS * (1.0 + e) * v.z.abs();
    let magnitude = (mu * normal_impulse).min(slip_speed * stick_impulse_per_slip());
    let impulse = slip * (-magnitude / slip_speed);

    let v_out = Vector3::new(
        v.x + impulse.x / ball::MASS,
        v.y + impulse.y / ball::MASS,
        v_out_z,
    );
    let omega_out = omega + arm.cross(&impulse) / ball::SHELL_INERTIA;
    return (v_out, omega_out);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::defaults;

    #[test]
    fn normal_uses_restitution() {
        let p = defaults::physics();
        let v = Vector3::new(0.0, -6.5, -3.0);
        let (v2, _) = table_bounce(v, Vector3::zeros(), &p);
        assert!((v2.z - (-v.z * p.restitution)).abs() < 1e-12);
    }

    #[test]
    fn shallow_impact_slips_and_keeps_most_tangential_speed() {
        // 얕은 입사(접선 ≫ 법선): 마찰 임펄스가 μ·J_n으로 제한되어
        // 접선속도는 조금만 줄어든다. 예전 (1-μ) 모델은 40%를 깎았다.
        let p = defaults::physics();
        let v = Vector3::new(0.0, -6.5, -3.0);
        let (v2, w2) = table_bounce(v, Vector3::zeros(), &p);
        let mu = table_ball_mu(&p);
        let expected_dv = mu * (1.0 + p.restitution) * v.z.abs();
        assert!(
            (v2.y - (v.y + expected_dv)).abs() < 1e-9,
            "slip 접선: {} vs {}",
            v2.y,
            v.y + expected_dv
        );
        assert!(
            v2.y < v.y * (1.0 - mu),
            "예전 (1-μ) 모델보다 접선을 덜 깎아야 함"
        );
        // 뒤에서 앞으로 미끄러지면 톱스핀(+x)이 생긴다.
        assert!(w2.x > 0.0, "슬립이 스핀을 만들어야 함: {}", w2.x);
    }

    #[test]
    fn steep_impact_reaches_rolling_and_caps_tangential_loss() {
        // 가파른 입사: μ·J_n이 충분히 커서 접촉점 슬립이 0이 된다(구름).
        let p = defaults::physics();
        let v = Vector3::new(0.0, -2.0, -6.0);
        let (v2, w2) = table_bounce(v, Vector3::zeros(), &p);
        let contact_after = v2.y + (-ball::RADIUS) * -w2.x;
        assert!(
            contact_after.abs() < 1e-9,
            "구름 전이면 접촉점 슬립이 0: {contact_after}"
        );
    }

    #[test]
    fn slipping_tangential_change_is_spin_independent_but_spin_is_not() {
        // 완전 슬립이면 접선 임펄스 = μ·J_n 으로 고정 — 슬립 크기(=스핀)와 무관.
        // 이게 Coulomb의 서명이고, 예전 (1-μ)v_t 모델과 갈리는 지점이다.
        let p = defaults::physics();
        let v = Vector3::new(0.0, -6.0, -3.0);
        let (none_v, none_w) = table_bounce(v, Vector3::zeros(), &p);
        let (back_v, back_w) = table_bounce(v, Vector3::new(-40.0, 0.0, 0.0), &p);
        assert!(
            (back_v.y - none_v.y).abs() < 1e-12,
            "슬립 중 Δv_t는 스핀 무관"
        );
        assert!(
            (back_w.x - (none_w.x - 40.0)).abs() < 1e-9,
            "출사 ω는 입사 ω만큼 이동: {} vs {}",
            back_w.x,
            none_w.x - 40.0
        );
    }

    #[test]
    fn strong_topspin_reverses_friction_and_speeds_ball_up() {
        // 접촉점이 진행방향으로 미끄러질 만큼 톱스핀이 강하면 마찰이
        // 공을 오히려 밀어준다 (v_y가 더 음수 = 더 빠름).
        let p = defaults::physics();
        let v = Vector3::new(0.0, -6.0, -3.0);
        let none = table_bounce(v, Vector3::zeros(), &p).0;
        let spun = table_bounce(v, Vector3::new(400.0, 0.0, 0.0), &p).0;
        assert!(
            spun.y < none.y,
            "강한 톱스핀 {} 은 무회전 {} 보다 빨라야 함",
            spun.y,
            none.y
        );
    }

    #[test]
    fn kernel_mu_matches_rapier_effective_mu() {
        let p = defaults::physics();
        assert!((table_ball_mu(&p) - rapier_table_ball_mu(&p)).abs() < 1e-15);
        assert!((table_ball_mu(&p) - 0.3).abs() < 1e-15);
    }
}
