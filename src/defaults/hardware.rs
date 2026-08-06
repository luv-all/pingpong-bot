//! Dynamixel · AXL 레일 벤치 배선 — [`Default`]가 앱 프리셋.

use crate::hardware::dynamixel::{DynamixelConfig, MirrorSlave};
use crate::hardware::rail::RailConfig;

use super::motion::RAIL_ACCEL_M_S2;

/// 실기 좌측 안전 마진 [m].
pub const RAIL_LEFT_END_MARGIN_M: f64 = 0.0100;
/// 실기 우측 안전 마진 [m].
pub const RAIL_RIGHT_END_MARGIN_M: f64 = 0.0705;
/// 실기에서 확인한 레일 좌표 범위 [m].
pub const RAIL_PHYSICAL_X_MIN_M: f64 = 0.0;
pub const RAIL_PHYSICAL_X_MAX_M: f64 = 1.41;
/// AXL 보드 실측 원점(보드 0.0m)에 대응하는 제어 좌표 [m].
///
/// 기하학적 원점 0.705m에 발사기 기준 오른쪽 정렬 보정 2.5cm를 더한
/// 기존 0.730m에, 2026-08-07 실물 레일이 +X 마진 밖에 서 있던 영점
/// 오류를 보정하려고 전체 레일 길이의 절반에서 +X 3cm를 되돌린
/// 0.675m를 더했다.
/// `reverse=true`에서 이 값을 늘리면 모든 실물 명령이 물리 -X
/// (보드 +) 방향으로 이동한다.
pub const RAIL_POSITIVE_X_TRIM_M: f64 = 0.030;
pub const RAIL_NEGATIVE_X_ZERO_SHIFT_M: f64 =
    (RAIL_PHYSICAL_X_MAX_M - RAIL_PHYSICAL_X_MIN_M) / 2.0 - RAIL_POSITIVE_X_TRIM_M;
pub const RAIL_BOARD_ZERO_DOMAIN_M: f64 = 0.7300 + RAIL_NEGATIVE_X_ZERO_SHIFT_M;
/// sim·real 공통 이동 범위 [m].
pub const RAIL_X_MIN_M: f64 = RAIL_PHYSICAL_X_MIN_M + RAIL_LEFT_END_MARGIN_M;
pub const RAIL_X_MAX_M: f64 = RAIL_PHYSICAL_X_MAX_M - RAIL_RIGHT_END_MARGIN_M;
/// 탁구대 실측 중앙 보정 위치 [m].
pub const RAIL_READY_X_M: f64 = 0.6750;
/// 손목 q3(ID 5) 실물 혼·라켓 장착 영점 보정 [rad].
///
/// 2026-08-06 시작 자세에서 Goal-Present는 5 tick으로 정상이었지만,
/// 모델은 라켓 면 +6.76°를 예측하는 반면 실물은 거의 0°(수직)이었다.
/// 원래 벤치 기준 +8°를 복원하도록 모터 목표를 -8° 더 돌린다.
pub const WRIST_JOINT_ZERO_OFFSET_RAD: f64 = -8.0_f64.to_radians();

/// 하단 듀얼 MX-64 q0(ID 1·2) 재조립 혼 영점 보정 [rad].
///
/// 2026-08-07 재조립 후 코드 준비 자세에서 하단 링크가 탁구대
/// 상판 기준 약 15°로, 모델 목표 59.8°보다 약 45° 앞으로 누웠다.
/// 논리 q0 목표는 그대로 유지하고 ID1 목표를 +45°, ID2 미러
/// 목표를 -45° 옮겨 기계 체결 오프셋을 좌표 변환에서 보정한다.
/// 준비 자세 q0=30.189°의 버스 목표는 기존 1705/2391tick에서
/// 약 2217/1879tick으로 바뀐다.
pub const BASE_JOINT_ZERO_OFFSET_RAD: f64 = 45.0_f64.to_radians();

/// 하단 듀얼 MX-64(ID 1·2) 실측 대칭 허용 오차 [tick].
///
/// 4096 tick/rev 기준 60 tick은 약 5.27°이다. 타격 후 하중·유격으로
/// 관측된 42 tick은 허용하되, 그보다 큰 체결·혼 어괋남은 계속 차단한다.
pub const MIRROR_ALIGNMENT_MAX_ERROR_TICKS: i32 = 60;
/// 대칭 오차 초과 시 추가로 재측정할 횟수.
pub const MIRROR_ALIGNMENT_RECOVERY_RETRIES: u32 = 3;
/// 대칭 오차 재측정 사이의 기계 정착 대기 [ms].
pub const MIRROR_ALIGNMENT_RECOVERY_DELAY_MS: u64 = 100;

impl Default for DynamixelConfig {
    /// 벤치 4-dof + yaw 미러(ID1↔ID2). 포트는 호출측/`--dxl-port`로 덮어쓴다.
    fn default() -> Self {
        return Self {
            port: "COM8".to_owned(),
            baudrate: 57_600,
            protocol_version: 2.0,
            motor_ids: vec![1, 3, 4, 5],
            ticks_per_revolution: 4096,
            zero_tick: 2048,
            addr_goal_position: 116,
            addr_torque_enable: 64,
            addr_present_position: 132,
            addr_profile_acceleration: 108,
            addr_profile_velocity: 112,
            addr_operating_mode: 11,
            // Position Control Mode — e-Manual MX-64/MX-28 Protocol 2.0.
            operating_mode: 3,
            addr_pwm_limit: 36,
            // 100% PWM (unit ≈ 0.113%). Position 모드의 실질 출력 천장.
            pwm_limit_max: 885,
            addr_current_limit: 38,
            // MX-64만 Current Limit 보유(기본·최대 1941). MX-28(ID 4·5)에는 레지스터 없음.
            current_limit_max_by_id: vec![(1, 1941), (2, 1941), (3, 1941)],
            profile_acceleration: 0,
            profile_velocity: 0,
            // 모터 구동 중 노이즈로 발생하는 일시적 timeout/checksum 오류를 흡수한다.
            comm_retries: 8,
            comm_retry_delay_ms: 30,
            stream_hz: 200.0,
            joint_signs: vec![-1, -1, 1, 1],
            // q0는 하단 듀얼 혼 재조립각, q3는 라켓 장착각을 보정한다.
            joint_offsets_rad: vec![
                BASE_JOINT_ZERO_OFFSET_RAD,
                0.0,
                0.0,
                WRIST_JOINT_ZERO_OFFSET_RAD,
            ],
            // 모터 절대각 한계 [deg] — `motor_deg = 180 + sign·joint_deg` (zero_tick 2048 = 180°).
            //
            // **URDF 관절 한계에 맞춘다 (2026-07-31).** 예전 값은 파이썬 매니퓰레이터에서
            // 그대로 가져온 것으로, 설계 문서가 "Python defaults are the SSOT **until measured
            // on bench**"라고 밝힌 미검증 값이었다. 그런데 URDF보다 좁아서, 플래너(URDF 한계로
            // 계획)가 낸 자세를 하드웨어가 **조용히 잘라** 팔이 엉뚱한 곳에 섰다 — dry-run
            // 클립에서 j3가 0.50 rad(28.8°) 어긋난 원인이다.
            //
            // | 관절 | URDF | 옛 모터 한계가 허용 | 새 값 |
            // |------|------|--------------------|-------|
            // | j0 | continuous | −40°~+90° | 절대 모터 안전 한계 90~220° 유지 |
            // | j1 | ±30° | −45°~+45° | 유지 (이미 URDF보다 넓다) |
            // | j2 | −115°~+85° | −88°~+50° | 65~265 (= URDF 전체) |
            // | j3 | ±120° | −60°~+40° | 60~300 (= URDF 전체) |
            //
            // 이제 **플래너가 권한을 갖는다** — URDF 한계·테이블 관통·토크·속도를 이미 검사하고,
            // 모터 클램프는 그 뒤의 최후 보호막으로 남는다. MX 시리즈는 Position 모드에서
            // 0~4095틱(360°)을 쓰므로 새 값은 전부 하드웨어 범위 안이다.
            //
            // j0은 URDF가 `continuous`라 기구적 한계(케이블·자기 간섭)를 모델이 말해주지 않는다.
            // 혼 영점을 보정해도 실기에 설정된 절대 안전 범위는 넓히지 않는다.
            motor_angle_limits_deg: vec![
                [90.0, 220.0],
                [135.0, 225.0],
                [65.0, 265.0],
                [60.0, 300.0],
            ],
            mirror_slaves: vec![MirrorSlave {
                master_id: 1,
                slave_id: 2,
            }],
            // 종료 시 토크를 끄면 팔이 그대로 주저앉는다 — 켠 채로 둔다.
            hold_torque_on_close: true,
        };
    }
}

impl Default for RailConfig {
    /// 벤치 AXL 레일. `dll_path`는 머신마다 `/--dll-path`로 덮어쓴다.
    fn default() -> Self {
        return Self {
            enabled: true,
            dll_path: std::path::PathBuf::from(
                "C:/Users/user/Downloads/Interfacing File/Interfacing File/Linear/LM_interface/src/lib/AXL.dll",
            ),
            axis: 0,
            irq_no: 7,
            pulses_per_meter: 250_000,
            reverse: true,
            board_zero_domain_m: RAIL_BOARD_ZERO_DOMAIN_M,
            x_min_m: RAIL_X_MIN_M,
            x_max_m: RAIL_X_MAX_M,
            vel: 11.25,
            accel: RAIL_ACCEL_M_S2,
            decel: RAIL_ACCEL_M_S2,
            min_vel: 0.001,
            max_vel: 11.25,
            pulse_out_method: 4,
            enc_input_method: 3,
            abs_rel_mode: 0,
            profile_mode: 3,
            accel_unit: 0,
            soft_limit_stop_mode: 0,
            soft_limit_selection: 0,
            inposition_use: 1,
            alarm_use: 0,
            limit_stop_mode: 0,
            pos_end_limit_level: 2,
            neg_end_limit_level: 2,
        };
    }
}
