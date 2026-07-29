//! Rapier 관절 위치 모터 게인 — **시뮬 전용**.
//!
//! 실물 경로(`hardware::dynamixel`)는 이 값을 참조하지 않는다. 실물에는 Goal
//! Position + Goal Current(RNEA τ)만 나가고, 위치 루프는 MX-64 내부 PID가
//! 돈다. 여기 값은 그 내부 루프를 Rapier 안에서 흉내 내는 모델 파라미터다.
//!
//! 아직 실측으로 보정되지 않았다 — `docs/measure-physics.md`의 "모터 위치
//! 루프" 절 참고.

use anyhow::{Result, ensure};

/// 이 팔의 자유도 — 게인 배열 길이. `joint_torque_limits_4dof_array`와 짝.
pub const SIM_MOTOR_JOINTS: usize = 4;

/// 4-dof 관절별 유효 관성 I_i [kg·m^2] — `robot::dynamics::mass_matrix` 대각.
///
/// 휴지 자세(`READY_JOINTS_4DOF`)와 대표 스윙 임팩트 자세
/// (`SAMPLE_IMPACT_JOINTS`) 두 곳에서 `M(q)`의 대각 `M[i][i]`를 재고, 관절별로
/// 더 큰(= 더 센 게인이 필요한) 쪽을 골랐다. RNEA 질량 행렬이라 하위 링크와
/// 라켓의 **반사 관성이 이미 들어 있다** — 링크 하나의 국소 질량만 보던 옛
/// 추정(I≈5e-3~1.5e-2)이 base/shoulder를 크게 과소평가했다.
///
/// | 관절 | 휴지 | 임팩트 | 채택 |
/// |------|------|--------|------|
/// | 0 yaw      | 3.373e-2 | 2.337e-2 | 3.373e-2 |
/// | 1 shoulder | 1.617e-2 | 1.195e-2 | 1.617e-2 |
/// | 2 elbow    | 1.406e-2 | 1.429e-2 | 1.429e-2 |
/// | 3 wrist    | 2.196e-3 | 2.196e-3 | 2.196e-3 |
///
/// 자세에 따라 변하는 값이라 이 두 자세는 근사다(시뮬 전용 가정). 실제 팔
/// 모델과 어긋나면 `inertia_matches_mass_matrix_diagonal` 테스트가 잡는다.
pub const JOINT_EFFECTIVE_INERTIA_4DOF: [f64; SIM_MOTOR_JOINTS] =
    [3.373e-2, 1.617e-2, 1.429e-2, 2.196e-3];

/// 위치 루프 목표 고유진동수 ω_n [rad/s] — 관절 전체 공통.
///
/// Rapier 모터 토크는 `motor_max_force`(관절별 τ 한계)로 클램프돼서 스윙
/// 대부분을 **포화 상태**로 돈다. 포화 구간에서는 τ의 부호만 의미가 있어
/// `k`·`d`의 절대 크기가 아니라 **비 `d/k = 2ζ/ω_n`** 만 남고, 임팩트 시점
/// 추종 오차가 `(2ζ/ω_n)·q̇` 로 붙는다. 즉 빠른 관절이 더 뒤처진다.
///
/// 옛 균일 게인 `(k, d) = (5000, 10)`은 `d/k = 2e-3` → ζ=1 환산 ω_n=1000과
/// 정확히 같았다. `607790e`가 임팩트 속도 부담을 base/shoulder로 옮긴 뒤
/// base가 가장 빨라졌고(q̇≈2.7 rad/s), 그래서 base만 눈에 띄게 늦었다.
/// ω_n을 2000으로 올려 그 오차를 절반 이하로 줄인다 — Rapier 실측(dt=1ms,
/// 대표 스윙 임팩트 시점 |q−q_cmd|):
///
/// | ω_n | yaw | shoulder | elbow | wrist |
/// |-----|-----|----------|-------|-------|
/// | 1000 (옛 균일 게인) | 1.73 mrad | 0.00 | 0.19 | 0.06 |
/// | 1500 | 0.87 mrad | 0.00 | 0.02 | 0.03 |
/// | **2000** | **0.43 mrad** | 0.00 | 0.07 | 0.02 |
/// | 5000 | 0.37 mrad | 0.00 | 0.21 | 0.00 |
///
/// 3000 이상은 더 나아지지 않고 수치 채터만 남아서 2000에서 멈춘다.
pub const SIM_MOTOR_BANDWIDTH_RAD_S: f64 = 2_000.0;

/// ζ=1(임계감쇠)·공통 ω_n에서 관절별 `(k_i, d_i)`.
///
/// `k_i = ω_n^2·I_i`, `d_i = 2·ω_n·I_i` — 관성이 큰 관절(base/shoulder)이
/// 그만큼 센 게인을 받아 모든 관절의 위치 루프가 같은 대역폭으로 돈다.
const fn per_joint_gains() -> ([f64; SIM_MOTOR_JOINTS], [f64; SIM_MOTOR_JOINTS]) {
    let w = SIM_MOTOR_BANDWIDTH_RAD_S;
    let mut stiffness = [0.0; SIM_MOTOR_JOINTS];
    let mut damping = [0.0; SIM_MOTOR_JOINTS];
    let mut i = 0;
    while i < SIM_MOTOR_JOINTS {
        let inertia = JOINT_EFFECTIVE_INERTIA_4DOF[i];
        stiffness[i] = w * w * inertia;
        damping[i] = 2.0 * w * inertia;
        i += 1;
    }
    return (stiffness, damping);
}

/// 관절 위치 모터 PD 게인 — **관절별**.
///
/// Rapier는 매 스텝 `τ = k(q_target − q) − d·q̇` 를 내고 `motor_max_force`
/// (= RNEA τ)로 클램프한다. 두 항이 **같은 토크 예산을 나눠 쓰므로**, d가
/// 크면 주어진 추종 오차로 낼 수 있는 관절 속도가 그만큼 낮아진다.
///
/// 관절마다 유효 관성이 15배까지 차이 나서(base 3.4e-2 vs wrist 2.2e-3) 하나의
/// `(k, d)` 쌍으로는 어느 한 관절에만 맞는 대역폭이 나온다 —
/// [`JOINT_EFFECTIVE_INERTIA_4DOF`] 참고.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimMotorParams {
    /// 관절별 위치 게인 k_i.
    pub position_stiffness: [f64; SIM_MOTOR_JOINTS],
    /// 관절별 속도 게인 d_i. 임계감쇠는 `2√(k_i·I_i)`.
    pub position_damping: [f64; SIM_MOTOR_JOINTS],
}

impl SimMotorParams {
    pub fn validate(&self) -> Result<()> {
        for (joint, &stiffness) in self.position_stiffness.iter().enumerate() {
            ensure!(stiffness > 0.0, "position_stiffness[{joint}] > 0");
        }
        for (joint, &damping) in self.position_damping.iter().enumerate() {
            ensure!(damping >= 0.0, "position_damping[{joint}] >= 0");
        }
        return Ok(());
    }

    /// 관절 `joint`의 위치 게인 — 배열보다 관절이 많으면 마지막(말단) 값.
    pub fn stiffness_at(&self, joint: usize) -> f64 {
        return self.position_stiffness[joint.min(SIM_MOTOR_JOINTS - 1)];
    }

    /// 관절 `joint`의 속도 게인 — 배열보다 관절이 많으면 마지막(말단) 값.
    pub fn damping_at(&self, joint: usize) -> f64 {
        return self.position_damping[joint.min(SIM_MOTOR_JOINTS - 1)];
    }

    /// 관절 `joint`의 유효 관성 `inertia`에 대한 감쇠비 ζ = d_i / (2√(k_i·I)).
    ///
    /// ζ≈1이 임계감쇠. ζ≫1이면 과감쇠라 스윙이 느려지고, ζ≪1이면 진동한다.
    pub fn damping_ratio(&self, joint: usize, inertia: f64) -> f64 {
        let critical = 2.0 * (self.stiffness_at(joint) * inertia).sqrt();
        if critical <= f64::EPSILON {
            return f64::INFINITY;
        }
        return self.damping_at(joint) / critical;
    }
}

impl Default for SimMotorParams {
    fn default() -> Self {
        let (position_stiffness, position_damping) = per_joint_gains();
        return Self {
            position_stiffness,
            position_damping,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::Joints;
    use crate::robot::dynamics::mass_matrix;

    /// 대표 스윙 임팩트 자세 — `plan_swing`(time_to_impact 0.45 s, 임팩트
    /// y=0.18 / z=면+0.18, 입사 −7 m/s)이 실제로 낸 임팩트 관절각.
    /// 매번 궤적을 계획하지 않고 그 자세만 고정해 관성을 잰다.
    const SAMPLE_IMPACT_JOINTS: [f64; 4] = [0.2390, 0.0, 0.6616, -1.0631];

    fn representative_inertias() -> [f64; 4] {
        let arm = crate::defaults::shared_robot().arm.clone();
        let poses = [
            arm.default_joints.values.clone(),
            SAMPLE_IMPACT_JOINTS.to_vec(),
        ];
        let mut worst = [0.0_f64; 4];
        for pose in poses {
            let m = mass_matrix(&arm, &Joints::from_slice(&pose));
            for (joint, slot) in worst.iter_mut().enumerate() {
                *slot = slot.max(m[(joint, joint)]);
            }
        }
        return worst;
    }

    #[test]
    fn defaults_validate() {
        SimMotorParams::default().validate().expect("sim_motor");
    }

    /// 하드코딩한 [`JOINT_EFFECTIVE_INERTIA_4DOF`]가 실제 팔의 질량 행렬
    /// 대각과 여전히 맞는지 — 링크 관성/CAD가 바뀌면 게인도 다시 뽑아야 한다.
    #[test]
    fn inertia_matches_mass_matrix_diagonal() {
        let measured = representative_inertias();
        for (joint, &expected) in JOINT_EFFECTIVE_INERTIA_4DOF.iter().enumerate() {
            let actual = measured[joint];
            let drift = (actual - expected).abs() / expected;
            assert!(
                drift < 0.05,
                "joint {joint}: 상수 I={expected:.4e} vs mass_matrix I={actual:.4e} \
                 (drift {:.1}%) — 게인 상수를 다시 뽑아야 함",
                drift * 100.0
            );
        }
    }

    /// 감쇠는 관절마다 임계감쇠 대역(과감쇠도 진동도 아님)에 있어야 한다.
    ///
    /// 관절별 **실제 반사 관성**(`mass_matrix` 대각, 휴지·임팩트 자세 중
    /// 보수적인 쪽)으로 ζ를 확인한다. 옛 테스트는 링크 하나의 국소 질량에서
    /// 짐작한 평평한 5e-3~1.5e-2 구간을 모든 관절에 똑같이 썼다 — 그 값으로는
    /// base(3.4e-2)가 사실 ζ≈0.39로 대역 밖이었는데도 통과했다. 게인을 만질
    /// 때 20배 과감쇠(옛 200) 같은 값으로 되돌아가지 않게 막는 가드다.
    #[test]
    fn damping_is_near_critical_for_this_arm() {
        let motor = SimMotorParams::default();
        for (joint, &inertia) in representative_inertias().iter().enumerate() {
            let zeta = motor.damping_ratio(joint, inertia);
            assert!(
                (0.4..=1.6).contains(&zeta),
                "joint {joint}: ζ={zeta:.2} (I={inertia:.4e}) — 임계감쇠 대역을 벗어남"
            );
        }
    }

    /// 옛 평평한 게인은 관절별로 ζ가 4배 흩어졌다 — base는 과소감쇠,
    /// wrist는 과감쇠. 관절별 게인이 그걸 없앤다.
    #[test]
    fn flat_gains_would_spread_damping_ratio_across_joints() {
        let inertias = representative_inertias();
        let flat = SimMotorParams {
            position_stiffness: [5_000.0; SIM_MOTOR_JOINTS],
            position_damping: [10.0; SIM_MOTOR_JOINTS],
        };
        let flat_zeta: Vec<f64> = (0..SIM_MOTOR_JOINTS)
            .map(|joint| flat.damping_ratio(joint, inertias[joint]))
            .collect();
        let spread = flat_zeta.iter().copied().fold(f64::MIN, f64::max)
            / flat_zeta.iter().copied().fold(f64::MAX, f64::min);
        assert!(spread > 3.0, "옛 평평한 게인 ζ 분산 {spread:.1}배: {flat_zeta:?}");

        let tuned = SimMotorParams::default();
        let tuned_zeta: Vec<f64> = (0..SIM_MOTOR_JOINTS)
            .map(|joint| tuned.damping_ratio(joint, inertias[joint]))
            .collect();
        let tuned_spread = tuned_zeta.iter().copied().fold(f64::MIN, f64::max)
            / tuned_zeta.iter().copied().fold(f64::MAX, f64::min);
        assert!(
            tuned_spread < 1.1,
            "관절별 게인은 ζ가 고르게 나와야: {tuned_zeta:?}"
        );
    }

    #[test]
    fn old_overdamped_value_would_be_rejected() {
        let old = SimMotorParams {
            position_stiffness: [5_000.0; SIM_MOTOR_JOINTS],
            position_damping: [200.0; SIM_MOTOR_JOINTS],
        };
        assert!(old.damping_ratio(0, 5.0e-3) > 10.0);
    }

    /// 관절별 게인은 관성 비에 그대로 비례해야 한다 (공통 ω_n·ζ=1).
    #[test]
    fn stiffness_tracks_reflected_inertia() {
        let motor = SimMotorParams::default();
        let w = SIM_MOTOR_BANDWIDTH_RAD_S;
        for (joint, &inertia) in JOINT_EFFECTIVE_INERTIA_4DOF.iter().enumerate() {
            assert!((motor.stiffness_at(joint) - w * w * inertia).abs() < 1e-6);
            assert!((motor.damping_at(joint) - 2.0 * w * inertia).abs() < 1e-9);
        }
        // base는 wrist보다 관성이 15배 크니 게인도 그만큼 세야 한다.
        assert!(motor.stiffness_at(0) > motor.stiffness_at(3) * 10.0);
    }

    /// 배열보다 관절이 많은 팔이 와도 마지막 값으로 폴백한다.
    #[test]
    fn gain_lookup_saturates_past_last_joint() {
        let motor = SimMotorParams::default();
        assert_eq!(motor.stiffness_at(9), motor.stiffness_at(3));
        assert_eq!(motor.damping_at(9), motor.damping_at(3));
    }
}
