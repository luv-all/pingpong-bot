//! Dynamixel 연속 구동 한계 (엔지니어링 가정).
//!
//! datasheet stall/RPM은 [`crate::constants::dynamixel`].

use crate::constants::dynamixel::{
    MX28_GEAR_RATIO, MX28_NO_LOAD_SPEED_RPM, MX28_STALL_TORQUE_NM, MX64_GEAR_RATIO,
    MX64_STALL_TORQUE_NM, rev_min_to_rad_s,
};

/// 짧은 위치 차단 동작에서 사용할 stall 토크 비율.
///
/// 실기 드라이버의 PWM/Current limit가 이미 최대 출력으로 설정되므로 sim과
/// 플래너도 같은 100%를 사용한다. 연속 운전 열 한계가 아니라 한 공을 받은 뒤
/// 원위치로 복귀하는 짧은 동작 기준이다.
pub const CONTINUOUS_TORQUE_DERATE: f64 = 1.0;

/// MX-64 회전자(+기어박스) 관성 [kg·m^2], **회전자축 기준**.
///
/// 출처: Rhoban BAM(`https://github.com/Rhoban/bam`,
/// `bam/params/mx64/m1.json`)이 진자 테스트벤치 로그로 식별한 출력축 기준
/// 겉보기 관성(armature) `J_m = 1.195e-2 kg·m^2`를 감속비 제곱으로 나눈 값
/// (`J_r = J_m / N^2 = 1.195e-2 / 200^2 ≈ 2.99e-7`). 논문:
/// Duclusaud et al., "Extended Friction Models for the Physics Simulation of
/// Servo Actuators" (arXiv:2410.08650) §II-A — `J_m = N^2 J_r` 정의와
/// MX-64 식별 대상 명시.
///
/// **실측이 아니라 제3자 식별값**이다(Robotis는 회전자 관성을 공개하지
/// 않는다). 같은 저장소의 m1~m6 모델이 1.096e-2~1.227e-2로 흩어지므로
/// 값의 불확실도는 ±10% 정도로 본다.
pub const MX64_ROTOR_INERTIA_KG_M2: f64 = 3.0e-7;

/// MX-28 회전자(+기어박스) 관성 [kg·m^2], **회전자축 기준** — **추정치, 실측 필요**.
///
/// MX-28은 공개 식별 데이터가 없다. 같은 MX 계열(전부 Coreless(Maxon))
/// 두 실측점 — MX-64(`J_r`=2.99e-7, N=200, stall 6.0 N·m)과
/// MX-106(BAM `armature`=2.661e-2, N=225, stall 8.4 N·m → `J_r`=5.26e-7) —
/// 으로 회전자축 기준 stall 토크(`stall/N`) 대비 `J_r` 멱법칙을 세 방식으로
/// 외삽했다:
///
/// | 방법 | MX-28 `J_r` | 출력축 반사관성 |
/// |------|-------------|-----------------|
/// | MX-64↔MX-106 2점 적합 (지수 2.59) | 3.40e-8 | 1.27e-3 |
/// | 서보 질량비 적합 (72/126/153 g) | 5.85e-8 | 2.18e-3 |
/// | 기하 상사 `J ∝ T^(5/3)` | 7.4e-8 | 2.76e-3 |
///
/// 세 값의 기하평균(≈5.4e-8)을 채택한다. 실제 값은 1.3e-3~2.8e-3 kg·m^2
/// (출력축 기준) 범위 어딘가다. MX-28이 구동하는 elbow/wrist는 링크 관성이
/// 각각 1.43e-2 / 2.20e-3 kg·m^2 라 elbow는 이 불확실도에 거의 둔감하고,
/// wrist만 민감하다(±35%).
pub const MX28_ROTOR_INERTIA_KG_M2: f64 = 5.4e-8;

/// 출력축 기준 반사관성 `I_reflected = I_rotor · N^2` [kg·m^2].
///
/// 감속기 뒤에서 본 회전자 관성은 감속비의 **제곱**으로 증폭된다 — MX-64는
/// N=200이라 ×40000이다. 강체 링크만 보는 RNEA
/// ([`crate::robot::dynamics::required_joint_torques_into`])에는 이 항이
/// 없어서, 토크 실현 판단은
/// [`crate::robot::dynamics::required_joint_torques_with_rotor_into`]로 더한다.
pub const fn reflected_inertia(rotor_inertia: f64, gear_ratio: f64) -> f64 {
    return rotor_inertia * gear_ratio * gear_ratio;
}

/// 무부하 RPM 대비 실현 가능한 관절 속도 감쇠.
///
/// **0.5 → 0.8 (2026-07-31).** 옛 0.5는 근거가 없었다 — 바로 위
/// 속도와 토크는 물리적 성격이 다르다. 토크는 이번 짧은 위치 차단 동작에서 100%를
/// 쓰지만, 속도는 무부하 RPM을 그대로 도달 가능 속도로 보지 않고 0.8을 유지한다.
///
/// 두 논거가 갈린다:
///
/// - **속도-토크 선형**: 80% 속도에서는 20% stall 토크뿐이다. 실측 토크 이용률 0.82
///   (디레이트 한계 대비 = stall의 0.41)를 쓰려면 속도는 무부하의 ~59%여야 한다 → 0.59.
/// - **quintic 궤적**: 피크 속도는 **가속도가 0인 지점**에서 나온다. 그 순간 관성 토크는
///   0이고 중력 보상만 남으므로 토크 요구가 낮다 → 더 높은 값이 가능하다.
///
/// 후자를 택해 0.8로 둔다. 실기 클립 기준 요구 비율이 2.20~2.99 → 1.37~1.87로 내려간다
/// (여전히 1.0 초과라 이것만으로 칠 수 있게 되지는 않는다).
///
/// **위험이 비대칭이다.** 한계를 실제보다 높이면 플래너가 모터가 못 따라가는 궤적을
/// 통과시키고, 정직한 거부 대신 **조용한 빗나감**이 된다 — 추종 오차로 라켓이 늦게 도착하는데
/// 로그에는 성공으로 찍힌다. 그래서 실기에서 명령각 대비 실제각 오차를 같이 잰다
/// (`control_worker`의 추종 오차 로그). 오차가 커지는 지점이 실제 한계이고, 그 값을 근거로
/// 다시 박을 것.
pub const JOINT_SPEED_DERATE: f64 = 0.8;

/// 실기 4축 관절 속도 상한 [rad/s] — MX-28 무부하 × [`JOINT_SPEED_DERATE`].
pub const DYNAMIXEL_MAX_JOINT_SPEED_RAD_S: f64 =
    rev_min_to_rad_s(MX28_NO_LOAD_SPEED_RPM) * JOINT_SPEED_DERATE;

/// 4-dof 관절별 연속 토크 안전 한계 [N·m].
///
/// joint0=yaw=MX-64×2(듀얼), joint1=shoulder=MX-64, joint2/3=MX-28.
pub fn joint_torque_limits_4dof_array() -> [f64; 4] {
    return [
        2.0 * MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
        MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
        MX28_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
        MX28_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
    ];
}

pub fn joint_torque_limits_4dof() -> Vec<f64> {
    return joint_torque_limits_4dof_array().to_vec();
}

/// 4-dof 관절별 회전자 반사관성 [kg·m^2] — 출력(관절)축 기준.
///
/// 모터 매핑은 [`joint_torque_limits_4dof_array`]와 같은 SSOT
/// (`.omc/research/dynamixel-specs.md` §3): joint0=yaw=MX-64×2(듀얼),
/// joint1=shoulder=MX-64, joint2/3=elbow/wrist=MX-28.
///
/// yaw는 두 모터의 회전자가 같은 축에 기계적으로 결합돼 함께 돌기 때문에
/// 반사관성도 **더해진다**(토크 한계가 2배인 것과 같은 이유).
///
/// 참고 — 강체 링크 관성(`JOINT_EFFECTIVE_INERTIA_4DOF`) 대비 비중:
///
/// | 관절 | 링크 `M_ii` | 반사관성 | 증가율 |
/// |------|-------------|----------|--------|
/// | 0 yaw      | 3.373e-2 | 2.40e-2 | +71% |
/// | 1 shoulder | 1.617e-2 | 1.20e-2 | +74% |
/// | 2 elbow    | 1.429e-2 | 2.01e-3 | +14% |
/// | 3 wrist    | 2.196e-3 | 2.01e-3 | +92% |
pub fn joint_reflected_inertias_4dof_array() -> [f64; 4] {
    let mx64 = reflected_inertia(MX64_ROTOR_INERTIA_KG_M2, MX64_GEAR_RATIO);
    let mx28 = reflected_inertia(MX28_ROTOR_INERTIA_KG_M2, MX28_GEAR_RATIO);
    if std::env::var_os("WP8_DISABLE_REFLECTED_INERTIA").is_some() {
        return [0.0; 4];
    }
    return [2.0 * mx64, mx64, mx28, mx28];
}

pub fn joint_reflected_inertias_4dof() -> Vec<f64> {
    return joint_reflected_inertias_4dof_array().to_vec();
}
