//! pingpong-bot 공용 라이브러리.
//!
//! 경연용 단일 애플리케이션 안에서 카메라·추정·로봇·시뮬레이션·계획을
//! 기능별 모듈로 나눈다.

pub mod camera;
pub mod clock;
pub mod constants;
pub mod detector;
pub mod error;
pub mod estimator;
pub mod geometry;
pub mod hardware;
pub mod physics_config;
pub mod pipeline;
pub mod planner;
pub mod robot;
pub mod sim;
pub mod telemetry;

pub mod ballistics {
    pub use crate::estimator::ballistics::*;
}

pub use camera::calibrate_charuco_draft;
pub use camera::{
    BallObservation, Calibration, CameraId, CameraParams, FrameSource, PixelPoint, SimCamera,
    SyntheticCamera, dlt_triangulate, sample_at, triangulate_projections, triangulate_synced,
};
pub use clock::{Clock, SimClock, SystemClock};
pub use detector::{PassthroughDetector, passthrough_detect};
pub use error::{DomainError, HwError, HwFailDetail, ObservationError, SwingPlanError};
pub use estimator::{
    BallEkf, Estimator, HitPlane, Prediction, drag_from_trajectory,
    friction_from_tangential_speeds, physics_coeffs_toml, predict_hit_plane,
    restitution_from_bounce_heights, restitution_from_normal_speeds,
};
pub use geometry::Point3;
#[cfg(all(windows, feature = "real"))]
pub use hardware::RealHardware;
pub use hardware::{Hardware, SimHardware};
pub use physics_config::{
    PhysicsConfig, PhysicsParams, load_physics_from_config, merge_physics_into_config,
};
pub use pipeline::{
    DEFAULT_ROBOT_ID, PipelineConfig, PipelineError, PipelineThread, ROBOTS, RobotEntry,
    competition_arm, find_robot, robot_ids_csv, run, shared_competition_arm,
};
pub use planner::{
    InterceptWindow, MAX_INTERCEPT_SAMPLES, OrientedBox, RailMotion, SwingTrajectory, accel,
    ball_past_midcourt_for_commit, clamp_above_table, in_swing_commit_window, plan_best_swing,
    plan_swing, rally_return_velocity, required_racket_velocity, robot_obbs, table_penetration,
    verify_impact_model,
};
pub use robot::rail::LinearRail;
pub use robot::{
    Arm, ArmBuildError, ArmBuilder, JointLimit, Joints, MountPreset, RacketPose, RobotBuildError,
    RobotBuilder, RobotPose, RobotState, SUPPORTED_FK_JOINTS, SerialChain, SerialChainError,
    SerialJoint, SimRobot, UrdfGeometry, UrdfLinkVisual, UrdfLoadError, UrdfRobot,
};
pub use sim::{
    BallAction, BallEvent, BallScript, BallShooterSettings, BallState, BallVec3, ShooterLayout,
    SimBallEstimator, SimRuntimeControls, SimSession, SimSessionConfig, SimWorld,
    new_shutdown_flag,
};
#[cfg(feature = "gui")]
pub use sim::{SimViewerOptions, run_sim_viewer};
pub use telemetry::{NoopTelemetry, Telemetry, TelemetryEvent, TracingTelemetry};
