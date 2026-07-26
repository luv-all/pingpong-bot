//! Rapier 관절 위치 모터 게인 — **시뮬 전용**.
//!
//! 실물 경로(`hardware::dynamixel`)는 이 값을 참조하지 않는다. 실물에는 Goal
//! Position + Goal Current(RNEA τ)만 나가고, 위치 루프는 MX-64 내부 PID가
//! 돈다. 여기 값은 그 내부 루프를 Rapier 안에서 흉내 내는 모델 파라미터다.
//!
//! 아직 실측으로 보정되지 않았다 — `docs/measure-physics.md`의 "모터 위치
//! 루프" 절 참고.

use anyhow::{Result, ensure};

/// 관절 위치 모터 PD 게인.
///
/// Rapier는 매 스텝 `τ = k(q_target − q) − d·q̇` 를 내고 `motor_max_force`
/// (= RNEA τ)로 클램프한다. 두 항이 **같은 토크 예산을 나눠 쓰므로**, d가
/// 크면 주어진 추종 오차로 낼 수 있는 관절 속도가 그만큼 낮아진다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimMotorParams {
    /// 위치 게인 k.
    pub position_stiffness: f64,
    /// 속도 게인 d. 임계감쇠는 `2√(k·I)` (관절 유효 관성 I).
    pub position_damping: f64,
}

impl SimMotorParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.position_stiffness > 0.0, "position_stiffness > 0");
        ensure!(self.position_damping >= 0.0, "position_damping >= 0");
        return Ok(());
    }

    /// 관절 유효 관성 `inertia`에 대한 감쇠비 ζ = d / (2√(k·I)).
    ///
    /// ζ≈1이 임계감쇠. ζ≫1이면 과감쇠라 스윙이 느려지고, ζ≪1이면 진동한다.
    pub fn damping_ratio(&self, inertia: f64) -> f64 {
        let critical = 2.0 * (self.position_stiffness * inertia).sqrt();
        if critical <= f64::EPSILON {
            return f64::INFINITY;
        }
        return self.position_damping / critical;
    }
}

pub fn sim_motor() -> SimMotorParams {
    return SimMotorParams {
        position_stiffness: 5_000.0,
        // 링크 질량 0.04~0.08 kg → 관절 유효 관성 I≈5e-3~1.5e-2. 임계감쇠
        // 2√(k·I)는 10~17이다. 이전 값 200은 ζ≈12~20의 과감쇠라 라켓이 명령
        // 속도의 28%밖에 못 따라갔고 리턴이 네트를 못 넘었다 (스윙 중 관절
        // 추종오차 0.05 rad → 10에서 0.01 rad).
        position_damping: 10.0,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_validate() {
        sim_motor().validate().expect("sim_motor");
    }

    /// 감쇠는 임계감쇠 대역(과감쇠도 진동도 아님)에 있어야 한다.
    ///
    /// 이 팔의 관절 유효 관성 범위에서 ζ를 확인한다. 게인을 만질 때 20배
    /// 과감쇠(옛 200) 같은 값으로 되돌아가지 않게 막는 가드다.
    #[test]
    fn damping_is_near_critical_for_this_arm() {
        let motor = sim_motor();
        // 링크 질량 0.04~0.08 kg 기준 관절 유효 관성 하·상한.
        for inertia in [5.0e-3, 1.5e-2] {
            let zeta = motor.damping_ratio(inertia);
            assert!(
                (0.4..=1.6).contains(&zeta),
                "ζ={zeta:.2} (I={inertia}) — 임계감쇠 대역을 벗어남"
            );
        }
    }

    #[test]
    fn old_overdamped_value_would_be_rejected() {
        let old = SimMotorParams {
            position_stiffness: 5_000.0,
            position_damping: 200.0,
        };
        assert!(old.damping_ratio(5.0e-3) > 10.0);
    }
}
