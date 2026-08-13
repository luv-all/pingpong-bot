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
/// 구름 해가 출사 속도와 어긋나도 되는 배수 — 크기 상한이 아니라 **일관성** 검사다.
///
/// 원래 여기엔 절대 상한(300 rad/s)이 있었는데 **종류가 틀린 값과 비교하고 있었다**.
/// 구름 조건은 `ω_x = -v_y/R`이고 R=20mm라, v_y가 -6.2 m/s면 ω_x=311이 물리적으로
/// **강제된다** — 이상한 값이 아니라 구름의 정의다. 300으로 막으면 `300·R = 6 m/s`
/// 위로 굴러 나가는 공을 전부 배제하는데, 사람 서브는 대부분 그 위다(실측 2026-08-13:
/// 롤 판정을 통과한 유일한 바운스가 |ω|=312로 여기 잘려 나갔다 — 발동률 0/9의 한 축).
/// 근거로 삼았던 "실측 최대 153 rad/s"는 **입사 샷 스핀**이고 이 함수가 내는 건
/// **바운스 후 구름 ω**다. 단위만 같고 물리량이 다르다.
///
/// 대신 구름 해가 자기 입력과 모순되지 않는지만 본다: 구름이면 `|ω|·R`이 곧 접촉점
/// 기준 접선 속력이므로 `|v_out|`과 같은 규모여야 한다. 여기서 크게 벗어나면 v_out
/// 자체가 잡음이라는 뜻이라 못 믿는다.
const MAX_ROLL_SPEED_RATIO: f64 = 1.5;

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
    // 구름이면 |ω|·R == 면내 |v_out| 이 항등적으로 성립한다. 성립을 확인하는 게 아니라
    // (항등식이라 늘 성립한다) v_out의 **수직 성분까지 포함한** 크기와 견줘 본다 —
    // 잡음으로 v_out이 통째로 부풀면 여기서 걸린다.
    let tangential = omega.norm() * ball::RADIUS;
    if tangential > v_out.norm() * MAX_ROLL_SPEED_RATIO {
        return None;
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
