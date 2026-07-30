//! Dynamixel · AXL 레일 벤치 배선 — [`Default`]가 앱 프리셋.

use crate::hardware::dynamixel::{DynamixelConfig, MirrorSlave};
use crate::hardware::rail::RailConfig;

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
            addr_goal_current: 102,
            addr_operating_mode: 11,
            operating_mode_current_position: 5,
            nm_per_goal_current_unit: 1.0 / 1.4 / 1000.0 / 2.0,
            profile_acceleration: 20,
            profile_velocity: 80,
            comm_retries: 5,
            comm_retry_delay_ms: 20,
            stream_hz: 200.0,
            joint_signs: vec![-1, -1, 1, 1],
            joint_offsets_rad: vec![0.0; 4],
            motor_angle_limits_deg: vec![
                [90.0, 220.0],
                [135.0, 225.0],
                [92.0, 230.0],
                [120.0, 220.0],
            ],
            mirror_slaves: vec![MirrorSlave {
                master_id: 1,
                slave_id: 2,
            }],
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
            x_min_m: 0.0,
            x_max_m: 1.41,
            vel: 5.0,
            accel: 12.0,
            decel: 12.0,
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
