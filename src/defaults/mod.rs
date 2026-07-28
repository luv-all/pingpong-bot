//! 앱 기본 배선·휴리스틱 SSOT.
//!
//! 도메인 타입에는 프리셋을 두지 않는다. **[`Default`] 조립은 여기만**.
//! 규격·datasheet(ITTF, CAD, G, B0332, DXL stall)는 [`crate::constants`].
//!
//! 패턴:
//! - `impl Default for Params|Config|CliArgs` — 앱 프리셋
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
//! | [`vision`] | Scorer/Roi + [`detector_for`] |
//! | [`calib`] | Cam* / Charuco / Rig |
//! | [`hardware`] | DynamixelConfig / RailConfig |
//! | [`dxl_limits`] | derate·속도·토크 배열 |
//! | [`planner`] | InterceptWindow + bang-bang consts |
//! | [`sim`] | BallShooterSettings + 랜덤/eval consts |
//! | [`sim_motor`] | `SimMotorParams` |
//!
//! 활성 로봇을 바꾸려면 [`robot`] 본문만 고친다.

pub mod calib;
mod control;
pub mod dxl_limits;
mod estimator;
mod hardware;
mod impact;
mod physics;
pub mod planner;
mod robot;
pub mod sim;
mod sim_motor;
pub mod vision;

pub use calib::{
    CHARUCO_MARKER_LENGTH_M, CHARUCO_SQUARE_LENGTH_M, CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y,
    DEFAULT_CALIBRATION_PATH, DEFAULT_CALIBRATION_PENDING_NAME, DEFAULT_COLORMASK_PATH,
    DEFAULT_DATA_DIR, DEFAULT_FOV_Y_DEG, DEFAULT_STEREO_CAM_ROLES, DEFAULT_STREAM_BACKEND,
    DEFAULT_STREAM_FOURCC, DEFAULT_STREAM_FPS, DEFAULT_STREAM_HEIGHT, DEFAULT_STREAM_THREADED,
    DEFAULT_STREAM_WIDTH, LEFT_CAMERA_ID, LEFT_DEVICE, MAX_REPROJ_RMSE_PX, MIN_CHARUCO_CORNERS,
    RIGHT_CAMERA_ID, RIGHT_DEVICE, calibration_path, calibration_pending_path, colormask_path,
    ensure_parent_dir,
};
pub use control::ControlParams;
pub use dxl_limits::{
    CONTINUOUS_TORQUE_DERATE, DYNAMIXEL_MAX_JOINT_SPEED_RAD_S, JOINT_SPEED_DERATE,
    joint_torque_limits_4dof, joint_torque_limits_4dof_array,
};
pub use estimator::EstimatorParams;
pub use impact::ImpactParams;
pub use physics::PhysicsParams;
pub use planner::{
    JACOBIAN_DAMPING, JDOT_STEP, MAGNUS_OMEGA_MAX, MAX_INTERCEPT_SAMPLES, MAX_PLAN_TIME_SECS,
    MIN_TIME_TO_GO_SECS, PLAN_DT_SECS, POSITION_TOLERANCE_RAD_OR_M, RACKET_DIRECTION_TOLERANCE_DEG,
    RACKET_SPEED_RATIO_TOLERANCE, RAIL_ACCEL_M_S2, RETURN_TO_CENTER_GROWTH,
    RETURN_TO_CENTER_MAX_SECS, RETURN_TO_CENTER_MIN_SECS, TIME_TO_GO_BIAS,
};
pub use robot::{
    RAIL_MAX_SPEED, READY_JOINTS_4DOF, primitive_4dof, primitive_4dof_with_mount, rail_frame,
    robot, shared_robot, urdf_4dof, urdf_test,
};
pub use sim::{
    EVAL_MAX_SCORE, EVAL_NET_PASSTHROUGH_RETRIES, EVAL_PASS_SCORE_EXCLUSIVE, EVAL_PITCH_JITTER_DEG,
    EVAL_RACKET_REHIT_MIN_STEPS, EVAL_SHOTS_PER_ZONE, EVAL_SPEED_JITTER_MPS, EVAL_TOTAL_SHOTS,
    EVAL_YAW_JITTER_DEG, RANDOM_SHOT_HEIGHT_MAX_M, RANDOM_SHOT_HEIGHT_MIN_M,
    RANDOM_SHOT_LATERAL_MAX_M, RANDOM_SHOT_LATERAL_MIN_M, RANDOM_SHOT_NET_GATE_MAX_TRIES,
    RANDOM_SHOT_PITCH_MAX_DEG, RANDOM_SHOT_PITCH_MIN_DEG, RANDOM_SHOT_ROLL_MAX_DEG,
    RANDOM_SHOT_ROLL_MIN_DEG, RANDOM_SHOT_SIDESPIN_MAX, RANDOM_SHOT_SIDESPIN_MIN,
    RANDOM_SHOT_SPEED_MAX_MPS, RANDOM_SHOT_SPEED_MIN_MPS, RANDOM_SHOT_TARGET_PADDING_M,
    RANDOM_SHOT_TOPSPIN_MAX, RANDOM_SHOT_TOPSPIN_MIN,
};
pub use sim_motor::SimMotorParams;
pub use vision::{
    MOTION_DIFF_THRESH, MOTION_WEIGHT, PIXEL_LOUPE_SRC_HALF, PIXEL_LOUPE_ZOOM, camera_params_for,
    colormask_for, detector_for,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detector::{RoiParams, ScorerParams};
    use crate::hardware::dynamixel::DynamixelConfig;
    use crate::hardware::rail::RailConfig;
    use crate::planner::InterceptWindow;
    use crate::{CameraId, colormask_for};

    #[test]
    fn presets_validate() {
        PhysicsParams::default().validate().unwrap();
        ControlParams::default().validate().unwrap();
        ImpactParams::default().validate().unwrap();
        EstimatorParams::default().validate().unwrap();
        InterceptWindow::default().validate().unwrap();
        ScorerParams::default().validate().unwrap();
        colormask_for(CameraId(0)).unwrap().validate().unwrap();
        colormask_for(CameraId(1)).unwrap().validate().unwrap();
        RoiParams::default().validate().unwrap();
        DynamixelConfig::default().validate().unwrap();
        RailConfig::default().validate().unwrap();
        let c = ControlParams::default();
        assert!((c.max_joint_torques[0] - 6.0).abs() < 1e-12);
        assert!((c.max_joint_torques[1] - 3.0).abs() < 1e-12);
        assert!((c.max_joint_torques[2] - 1.25).abs() < 1e-12);
        assert!((c.max_joint_torques[3] - 1.25).abs() < 1e-12);
        assert!((ImpactParams::default().max_return_speed - 6.0).abs() < 1e-12);
    }

    #[test]
    fn shared_robot_is_4dof() {
        assert_eq!(shared_robot().arm.joint_count(), 4);
    }
}
