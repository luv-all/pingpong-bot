//! Dynamixel · AXL 레일 벤치 배선 — [`Default`]가 앱 프리셋.

use crate::hardware::dynamixel::{DynamixelConfig, MirrorSlave};
use crate::hardware::rail::RailConfig;

use super::motion::RAIL_ACCEL_M_S2;

/// 실측 AXL 레일의 앱 좌표 범위 [m]. 로봇 IK와 실기 드라이버가 반드시 같은 값을 쓴다.
pub const RAIL_X_MIN_M: f64 = 0.0;
pub const RAIL_X_MAX_M: f64 = 1.41;

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
            comm_retries: 5,
            comm_retry_delay_ms: 20,
            stream_hz: 200.0,
            joint_signs: vec![-1, -1, 1, 1],
            joint_offsets_rad: vec![0.0; 4],
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
            // | j0 | continuous | −40°~+90° | 유지 (URDF에 한계가 없어 넓힐 근거가 없다) |
            // | j1 | ±30° | −45°~+45° | 유지 (이미 URDF보다 넓다) |
            // | j2 | −115°~+85° | −88°~+50° | 65~265 (= URDF 전체) |
            // | j3 | ±120° | −60°~+40° | 60~300 (= URDF 전체) |
            //
            // 이제 **플래너가 권한을 갖는다** — URDF 한계·테이블 관통·토크·속도를 이미 검사하고,
            // 모터 클램프는 그 뒤의 최후 보호막으로 남는다. MX 시리즈는 Position 모드에서
            // 0~4095틱(360°)을 쓰므로 새 값은 전부 하드웨어 범위 안이다.
            //
            // j0은 URDF가 `continuous`라 기구적 한계(케이블·자기 간섭)를 모델이 말해주지 않는다.
            // 근거 없이 넓히지 않고 그대로 둔다 — 벤치에서 실제 가동범위를 재면 갱신할 것.
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
            x_min_m: RAIL_X_MIN_M,
            x_max_m: RAIL_X_MAX_M,
            vel: 5.0,
            accel: RAIL_ACCEL_M_S2,
            decel: RAIL_ACCEL_M_S2,
            min_vel: 0.001,
            max_vel: 5.0,
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
