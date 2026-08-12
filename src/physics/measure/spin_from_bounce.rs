//! 바운스 전후 속도로 되튐 후 스핀을 닫힌식으로 구한다.
//!
//! `table_bounce`(`src/physics/bounce.rs`)의 접선 임펄스는 슬립(미끄럼) 구간에서
//! `μ·J_n`으로 고정된다 — 슬립 **방향**만 보이고 크기(=ω)는 안 보인다는 뜻이다
//! (`slipping_tangential_change_is_spin_independent_but_spin_is_not` 테스트가 이미
//! 증명). 그래서 이 함수는 **구름 전이가 일어난 바운스에서만** 유일해를 낸다 — 그
//! 경우엔 출사 접촉점 슬립이 정확히 0이라 `v_out`만으로 `ω_out`이 정해진다. 슬립으로
//! 끝난 바운스는 `None`을 낸다 — 호출 쪽이 사전값(`ASSUMED_SPIN`)으로 접어야 한다.

use nalgebra::Vector3;

use crate::constants::ball;
use crate::defaults::PhysicsParams;
use crate::physics::Kinematics;

/// 구름 판정 여유 — 잡음으로 살짝 넘친 슬립까지 구름으로 오판하지 않게.
const ROLL_MARGIN: f64 = 0.85;
/// 그래도 나오면 못 믿는 크기 [rad/s] — 실측 최대가 153(2026-08-12, fly_45~53
/// 반발 역산)이라 2배 여유. `vision::fit::refine_spin`의 `SPIN_MAX`와 같은 값·같은
/// 근거를 여기도 쓴다.
///
/// 왜 필요한가: 롤/슬립 판정 자체가 `physics.restitution`·`friction`을 쓰는데, 라이브
/// 파이프라인은 이 값을 실측 반발계수(0.72)가 아니라 스핀 미보정을 흡수하려고 올린
/// 값(`vision::fit::RESTITUTION≈0.86`)으로 쓴다 — cap이 커진 만큼 실제로는 슬립인
/// 바운스도 롤로 오판할 여지가 생긴다. 실측(2026-08-12, 새 물리 적용 후 fly_46)으로
/// 확인됨: 판정은 롤로 통과했는데 나온 값이 311 rad/s — 대수적으로는 유일해였지만
/// 물리적으로 말이 안 된다. 판정 여유(`ROLL_MARGIN`)만으론 못 거른다 — 크기 자체를
/// 한 번 더 본다.
const MAX_PLAUSIBLE_SPIN: f64 = 300.0;

/// 바운스 전후 속도(v_in: 바운스 직전, v_out: 바운스 직후)로 되튐 후 ω를 구한다.
///
/// 구름이 아니면(=슬립으로 끝났으면) `None`. `ω_z`는 접촉점 속도에 안 들어가 구조적으로
/// 안 보이므로 항상 0으로 둔다 (`table_bounce`의 기존 문서화된 한계와 같다).
///
/// `v_in`/`v_out`는 접촉 프레임 인접 2점차가 아니라 넓은 창 회귀로 구한 값이어야 한다 —
/// 2점차는 가장 노이즈 심한 접촉 프레임 하나에 전부 의존해 e·μ 추정이 요동친다
/// (`TrajAnalysis::windowed_velocity` 참고).
pub fn spin_after_bounce_if_rolling(
    v_in: Vector3<f64>,
    v_out: Vector3<f64>,
    physics: &PhysicsParams,
) -> Option<Vector3<f64>> {
    if v_in.z >= 0.0 {
        return None; // 바운스가 아니다 (아직 안 내려오는 중).
    }
    let e = physics.restitution.clamp(0.0, 1.0);
    let mu = Kinematics::table_ball_mu(physics).max(0.0);
    let normal_impulse = ball::MASS * (1.0 + e) * v_in.z.abs();
    let cap = mu * normal_impulse;
    // 접선 임펄스 = m·Δv_t — 슬립이면 이 크기가 cap 근처에 붙는다.
    let measured = ball::MASS * (Vector3::new(v_out.x, v_out.y, 0.0) - Vector3::new(v_in.x, v_in.y, 0.0)).norm();
    if cap < 1e-9 || measured >= cap * ROLL_MARGIN {
        return None;
    }
    // 구름: 출사 접촉점 슬립이 정확히 0 → v + ω×(0,0,-R) = 0 을 풀면 ω_out이 v_out만으로 나온다.
    let omega = Vector3::new(-v_out.y / ball::RADIUS, v_out.x / ball::RADIUS, 0.0);
    if omega.norm() > MAX_PLAUSIBLE_SPIN {
        return None; // 대수적으로는 유일해라도 물리적으로 말이 안 되면 안 믿는다.
    }
    return Some(omega);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovers_exact_omega_out_when_bounce_reaches_rolling() {
        let p = PhysicsParams::default();
        // steep_impact_reaches_rolling_and_caps_tangential_loss(bounce.rs)와 같은 케이스.
        let v_in = Vector3::new(0.0, -2.0, -6.0);
        let (v_out, w_out_true) = Kinematics::bounce_on_table(v_in, Vector3::zeros(), &p);
        let solved =
            spin_after_bounce_if_rolling(v_in, v_out, &p).expect("가파른 입사는 구름으로 잡혀야 함");
        assert!(
            (solved - w_out_true).norm() < 1e-6,
            "solved={solved:?} true={w_out_true:?}"
        );
    }

    #[test]
    fn returns_none_when_bounce_is_sliding() {
        let p = PhysicsParams::default();
        // shallow_impact_slips_and_keeps_most_tangential_speed(bounce.rs)와 같은 케이스.
        let v_in = Vector3::new(0.0, -6.5, -3.0);
        let (v_out, _) = Kinematics::bounce_on_table(v_in, Vector3::zeros(), &p);
        assert!(spin_after_bounce_if_rolling(v_in, v_out, &p).is_none());
    }

    #[test]
    fn not_a_bounce_if_v_in_is_already_rising() {
        let p = PhysicsParams::default();
        let v_in = Vector3::new(0.0, -3.0, 2.0); // z >= 0 — 이미 올라가는 중, 바운스 아님.
        assert!(spin_after_bounce_if_rolling(v_in, Vector3::new(0.0, -3.0, 1.0), &p).is_none());
    }
}
