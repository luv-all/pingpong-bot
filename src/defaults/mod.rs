//! 앱 기본 배선·휴리스틱 SSOT.
//!
//! 도메인 타입에는 프리셋을 두지 않는다. **[`Default`] 조립은 여기만**.
//! 규격·datasheet(ITTF, CAD, G, B0332, DXL stall)는 [`crate::constants`].
//!
//! 패턴:
//! - `impl Default for camera::Params|Config|CliArgs` — 앱 프리셋
//! - `pub const` — clap `default_value_t`·임계값
//! - [`detector_for`] / [`robot`] — 조립이 `Result`이거나 파이프라인인 팩토리만 예외
//!
//! | 모듈 | Default |
//! |------|--------|
//! | [`physics`] | `PhysicsParams` |
//! | [`control`] | `ControlParams` |
//! | [`impact`] | `ImpactParams` |
//! | [`estimator`] | `EstimatorParams` |
//! | [`robot`] | URDF·primitive (`Result`) |
//! | [`vision`] | 튜너블 + 캐스케이드·picker·트리거 조립, colormask/calib 로드 |
//! | [`calib`] | Cam* / Charuco / Rig |
//! | [`hardware`] | DynamixelConfig / RailConfig |
//! | [`dxl_limits`] | derate·속도·토크 배열 |
//! | [`motion`] | InterceptWindow + 정렬·스윙·bang-bang consts |
//! | [`rail`] | 레일 좌표계·프레임 SSOT (영점·범위·마운트·모션) |
//! | [`sim`] | Settings + 랜덤/eval consts |
//! | [`sim_motor`] | `SimMotorParams` |
//!
//! 활성 로봇을 바꾸려면 [`robot`] 본문만 고친다.

pub mod calib;
mod control;
pub mod detector;
pub mod dxl_limits;
mod estimator;
mod hardware;
mod impact;
pub mod motion;
mod physics;
pub mod rail;
mod robot;
pub mod sim;
mod sim_motor;
pub mod vision;

pub use calib::{
    CHARUCO_MARKER_LENGTH_M, CHARUCO_SQUARE_LENGTH_M, CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y,
    CURRENT_RIG_CLIPS, DEFAULT_CALIBRATION_PATH, DEFAULT_CALIBRATION_PENDING_NAME,
    DEFAULT_CLIPS_DIR, DEFAULT_COLORMASK_PATH, DEFAULT_DATA_DIR, DEFAULT_FOV_Y_DEG,
    DEFAULT_STEREO_CAM_ROLES, DEFAULT_STREAM_BACKEND, DEFAULT_STREAM_FOURCC, DEFAULT_STREAM_FPS,
    DEFAULT_STREAM_HEIGHT, DEFAULT_STREAM_THREADED, DEFAULT_STREAM_WIDTH, LEFT_CAMERA_ID,
    LEFT_DEVICE, MAX_REPROJ_RMSE_PX, MIN_CHARUCO_CORNERS, RIGHT_CAMERA_ID, RIGHT_DEVICE,
    calibration_path, calibration_pending_path, colormask_path, ensure_parent_dir,
};
pub use control::ControlParams;
pub use dxl_limits::{
    CONTINUOUS_TORQUE_DERATE, DYNAMIXEL_MAX_JOINT_SPEED_RAD_S, JOINT_SPEED_DERATE,
    MX28_ROTOR_INERTIA_KG_M2, MX64_ROTOR_INERTIA_KG_M2, joint_reflected_inertias_4dof,
    joint_reflected_inertias_4dof_array, joint_torque_limits_4dof, joint_torque_limits_4dof_array,
    reflected_inertia,
};
pub use estimator::EstimatorParams;
pub use hardware::{BASE_JOINT_ZERO_OFFSET_RAD, WRIST_JOINT_ZERO_OFFSET_RAD};
pub use impact::ImpactParams;
pub use motion::{
    ALIGNMENT_CONTACT_BELOW_RACKET_CENTER_M, ALIGNMENT_MIN_UPWARD_TILT_DEG,
    ALIGNMENT_TARGET_HEIGHT_OFFSET_M, ALIGNMENT_TARGET_X_OFFSET_M, COARSE_TRACK_JOINT_FRACTION,
    FIRST_CONTROL_AFTER_DETECTION_SECS, FIXED_JOINT_PUSH_DISTANCE_M, FIXED_JOINT_PUSH_LIFT_M,
    FIXED_JOINT_SNAP_SPEED_RATIO,
    FIXED_JOINT_SWING_DURATION_SECS, FIXED_JOINT_SWING_LEAD_SECS, HOME_RETURN_SPEED_RATIO,
    JACOBIAN_DAMPING, JDOT_STEP, MAGNUS_OMEGA_MAX, MAX_INTERCEPT_SAMPLES, MAX_PLAN_TIME_SECS,
    MIN_TIME_TO_GO_SECS, PLAN_DT_SECS, POSITION_TOLERANCE_RAD_OR_M, POST_ALIGNMENT_HOLD_SECS,
    RACKET_DIRECTION_TOLERANCE_DEG, RACKET_SPEED_RATIO_TOLERANCE, RETURN_TO_CENTER_GROWTH,
    RETURN_TO_CENTER_MAX_SECS, RETURN_TO_CENTER_MIN_SECS, TIME_TO_GO_BIAS,
};
pub use physics::PhysicsParams;
pub use rail::{
    DEFAULT_RAIL_CALIBRATION_PATH, RAIL_ACCEL_M_S2, RAIL_BOARD_ZERO_DOMAIN_M, RAIL_BOTTOM_Z_M,
    RAIL_COORDINATE_POSITIVE_X_OFFSET_M, RAIL_HOMING_OVERTRAVEL_MARGIN_M,
    RAIL_HOMING_RETURN_VELOCITY_M_S, RAIL_HOMING_TIMEOUT_SECS, RAIL_HOMING_VELOCITY_M_S,
    RAIL_LEFT_END_MARGIN_M, RAIL_MAX_SPEED, RAIL_MOUNT_Z_M, RAIL_NEGATIVE_X_ZERO_SHIFT_M,
    RAIL_PHYSICAL_X_MAX_M, RAIL_PHYSICAL_X_MIN_M, RAIL_POSITIVE_X_TRIM_M, RAIL_PULSES_PER_METER,
    RAIL_READY_X_M, RAIL_RIGHT_END_MARGIN_M, RAIL_X_MAX_M, RAIL_X_MIN_M, rail_calibration_path,
    rail_frame,
};
pub use robot::{
    POST_HIT_READY_JOINTS_4DOF, READY_JOINTS_4DOF, primitive_4dof, primitive_4dof_with_mount,
    robot, shared_robot, urdf_4dof, urdf_test,
};
pub use sim::{
    EVAL_MAX_SCORE, EVAL_NET_PASSTHROUGH_RETRIES, EVAL_PASS_SCORE_EXCLUSIVE, EVAL_PITCH_JITTER_DEG,
    EVAL_RACKET_REHIT_MIN_STEPS, EVAL_SHOTS_PER_ZONE, EVAL_SPEED_JITTER_MPS, EVAL_TOTAL_SHOTS,
    EVAL_YAW_JITTER_DEG, RANDOM_SHOT_FIXED_MUZZLE_HEIGHT_Z_M, RANDOM_SHOT_FIXED_MUZZLE_INSET_Y_M,
    RANDOM_SHOT_FIXED_PITCH_DEG, RANDOM_SHOT_FIXED_ROLL_DEG, RANDOM_SHOT_FIXED_YAW_DEGS,
    RANDOM_SHOT_HEIGHT_MAX_M, RANDOM_SHOT_HEIGHT_MIN_M, RANDOM_SHOT_LATERAL_MAX_M,
    RANDOM_SHOT_LATERAL_MIN_M, RANDOM_SHOT_NET_GATE_MAX_TRIES, RANDOM_SHOT_PITCH_MAX_DEG,
    RANDOM_SHOT_PITCH_MIN_DEG, RANDOM_SHOT_ROLL_MAX_DEG, RANDOM_SHOT_ROLL_MIN_DEG,
    RANDOM_SHOT_SIDESPIN_MAX, RANDOM_SHOT_SIDESPIN_MIN, RANDOM_SHOT_SPEED_MAX_MPS,
    RANDOM_SHOT_SPEED_MIN_MPS, RANDOM_SHOT_TARGET_PADDING_M, RANDOM_SHOT_TOPSPIN_MAX,
    RANDOM_SHOT_TOPSPIN_MIN,
};
pub use sim_motor::{
    JOINT_EFFECTIVE_INERTIA_4DOF, SIM_MOTOR_BANDWIDTH_RAD_S, SIM_MOTOR_JOINTS, SimMotorParams,
};
pub use vision::{
    PIXEL_LOUPE_SRC_HALF, PIXEL_LOUPE_ZOOM, camera_params_for, colormask_for, detector_for,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera;
    use crate::defaults::colormask_for;
    use crate::hardware::dynamixel::DynamixelConfig;
    use crate::hardware::rail::RailConfig;
    use crate::robot::motion::InterceptWindow;

    #[test]
    fn presets_validate() {
        PhysicsParams::default().validate().unwrap();
        ControlParams::default().validate().unwrap();
        ImpactParams::default().validate().unwrap();
        EstimatorParams::default().validate().unwrap();
        InterceptWindow::default().validate().unwrap();
        DynamixelConfig::default().validate().unwrap();
        RailConfig::default().validate().unwrap();
        let rail_config = RailConfig::default();
        assert_eq!(rail_config.accel, RAIL_ACCEL_M_S2);
        assert_eq!(rail_config.decel, RAIL_ACCEL_M_S2);
        assert!((RAIL_ACCEL_M_S2 - 16.0).abs() < 1e-12);
        assert_eq!(rail_config.pulses_per_meter, RAIL_PULSES_PER_METER);
        assert_eq!(RAIL_PULSES_PER_METER, 240_385);
        let robot = robot().unwrap();
        let model_rail = robot.arm.rail.expect("기본 로봇 리니어 레일");
        assert_eq!(model_rail.x_min, rail_config.x_min_m);
        assert_eq!(model_rail.x_max, rail_config.x_max_m);
        assert!((model_rail.mount_z - RAIL_MOUNT_Z_M).abs() < 1e-12);
        assert!((model_rail.x_min - 0.0100).abs() < 1e-12);
        assert!((model_rail.x_max - 1.3395).abs() < 1e-12);
        assert!((model_rail.default_x() - 0.6750).abs() < 1e-12);
        assert!((rail_config.board_zero_domain_m - 1.3950).abs() < 1e-12);
        assert!((rail_config.domain_to_board_abs(0.0100) - 1.3850).abs() < 1e-12);
        assert!((rail_config.domain_to_board_abs(1.3395) - 0.0555).abs() < 1e-12);
        assert!((rail_config.domain_to_board_abs(0.6750) - 0.7200).abs() < 1e-12);
        assert!((ALIGNMENT_CONTACT_BELOW_RACKET_CENTER_M - 0.0200).abs() < 1e-12);
        assert!((RAIL_COORDINATE_POSITIVE_X_OFFSET_M - 0.0150).abs() < 1e-12);
        assert!((rail_frame().mount_x() - 0.09025).abs() < 1e-12);
        assert!((rail_frame().rail_bottom_z - RAIL_BOTTOM_Z_M).abs() < 1e-12);
        assert!((rail_frame().mount_z() - RAIL_MOUNT_Z_M).abs() < 1e-12);
        assert!((RAIL_BOTTOM_Z_M - 0.760).abs() < 1e-12);
        assert!((RAIL_MOUNT_Z_M - 0.815).abs() < 1e-12);
        assert!((ALIGNMENT_TARGET_X_OFFSET_M - 0.09475).abs() < 1e-12);
        assert!((JOINT_SPEED_DERATE - 0.95).abs() < 1e-12);
        assert!((rail_config.x_min_m - model_rail.x_min).abs() < 1e-12);
        assert!((rail_config.x_max_m - model_rail.x_max).abs() < 1e-12);
        let c = ControlParams::default();
        assert!((c.max_joint_torques[0] - 12.0).abs() < 1e-12);
        assert!((c.max_joint_torques[1] - 6.0).abs() < 1e-12);
        assert!((c.max_joint_torques[2] - 2.5).abs() < 1e-12);
        assert!((c.max_joint_torques[3] - 2.5).abs() < 1e-12);
        assert!((ImpactParams::default().max_return_speed - 6.0).abs() < 1e-12);
    }

    /// 커밋된 색 마스크가 읽히고 유효한가.
    ///
    /// `presets_validate` 와 나눠 둔다 — 이건 코드가 아니라 **리그 산출물**을 검사하므로,
    /// 카메라를 옮기면 정당하게 빨개진다. 그때 `tune-colormask` 로 다시 잡으라는 신호다.
    #[test]
    fn committed_colormask_is_valid() {
        for id in [camera::Id(0), camera::Id(1)] {
            let params = colormask_for(id).unwrap_or_else(|error| {
                panic!("cam{}: {error:#} — tune-colormask 로 다시 잡을 것", id.0)
            });
            params.validate().unwrap();
        }
    }

    #[test]
    fn shared_robot_is_4dof() {
        assert_eq!(shared_robot().arm.joint_count(), 4);
    }

    /// 커밋된 캘리브로 테이블 네 귀퉁이와 중앙이 화면 안에 드는지.
    ///
    /// 리그를 다시 캘리브했을 때 테이블이 화각을 벗어나는 배치를 여기서 잡는다. 벗어나면
    /// 그 구석의 공은 두 대가 같이 못 보고, 삼각측량이 안 되는 사각이 생긴다.
    #[test]
    fn committed_calibration_sees_the_whole_table() {
        use crate::Point3;
        use crate::constants::table;

        let z = table::SURFACE_Z;
        let probes = [
            ("center", table::WIDTH_X * 0.5, table::LENGTH_Y * 0.5),
            ("x0 y0", 0.0, 0.0),
            ("xW y0", table::WIDTH_X, 0.0),
            ("x0 yL", 0.0, table::LENGTH_Y),
            ("xW yL", table::WIDTH_X, table::LENGTH_Y),
        ];

        for id in [camera::Id(0), camera::Id(1)] {
            let cam = camera_params_for(id).unwrap();
            for (name, x, y) in probes {
                assert!(
                    cam.project_world(Point3::new(x, y, z)).is_some(),
                    "cam{}: 테이블 {name} 이 화각 밖이다",
                    id.0
                );
            }
        }
    }
}
