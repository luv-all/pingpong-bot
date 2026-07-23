//! pingpong-bot 공용 라이브러리.
//!
//! 경연용 단일 애플리케이션 안에서 카메라·추정·로봇·시뮬레이션·계획을
//! 기능별 모듈로 나눈다.

pub mod camera;
pub mod constants;
pub mod detector;
pub mod defaults;
pub mod error;
pub mod estimator;
pub mod hardware;
pub mod pipeline;
pub mod planner;
pub mod robot;
pub mod sim;
pub mod telemetry;

/// 월드 좌표 점 [m] — `nalgebra::Point3<f64>`.
pub type Point3 = nalgebra::Point3<f64>;

pub use camera::{
    BallObservation, Calibration, CameraId, CameraParams, CharucoBoardSpec, CharucoCalibReport,
    CharucoFrameDetect, ExposureReadout, Frame, FrameSource, HintSource, ImageDirSource,
    MIN_CHARUCO_CORNERS, OpenCvCapture, PixelPoint, PreviewAction, SimCamera, calibrate_charuco,
    destroy_window, detect_and_draw_charuco, dlt_triangulate, draw_cam_label, draw_circle_px,
    draw_debug_lines, draw_help_lines, draw_world_velocity, hstack_bgr, sample_at, show_bgr,
    triangulate_projections, triangulate_synced, triangulate_views,
};
pub use defaults::{
    ControlParams, EstimatorParams, ImpactParams, PhysicsParams, arm, colormask, control, detector,
    dynamixel, estimator, impact, intercept, physics, primitive_4dof, rail, rail_frame, robot,
    scorer, shared_arm, shared_robot, urdf_4dof, urdf_test,
};
pub use detector::{
    BallDetector, Candidate, CandidateGenerator, ColorSpace, ColormaskDetector, ColormaskParams,
    ContourDetector, FuseDetector, IntoCandidateGenerators, MotionPrior, ParseColorSpaceError,
    RoiTrack, Scorer, ScorerParams, fuse, passthrough_detect, track, undistort_frame,
};
pub use error::{DomainError, HwError, ObservationError, SwingPlanError};
pub use estimator::{
    BallEkf, BounceEvent, Estimator, HitPlane, Prediction, RollEvent, TrajPoint, detect_bounces,
    detect_rolls, drag_from_trajectory, format_physics_for_defaults, friction_from_tangential_speeds,
    mean_bounce_e, mean_roll_mu, predict_hit_plane, restitution_from_bounce_heights,
    restitution_from_normal_speeds,
};
#[cfg(feature = "real")]
pub use hardware::RealHardware;
pub use hardware::{Hardware, SimHardware};
pub use pipeline::{CameraFeed, PipelineConfig, PipelineError, PipelineThread, run};
pub use planner::{
    InterceptWindow, MAX_INTERCEPT_SAMPLES, OrientedBox, RailMotion, SwingTrajectory, accel,
    ball_past_midcourt_for_commit, clamp_above_table, in_swing_commit_window, plan_best_swing,
    plan_return_to_center, plan_swing, rally_return_velocity, required_racket_velocity,
    robot_obbs, table_penetration, verify_impact_model,
};
pub use robot::{
    Arm, ArmBuildError, ArmBuilder, JointLimit, Joints, LinearRail, MountPreset, RacketPose,
    RailFrame, Robot, RobotBuildError, RobotBuilder, RobotPose, RobotState, SerialChain,
    SerialChainError, SerialJoint, UrdfGeometry, UrdfLinkVisual, UrdfLoadError, UrdfModel,
};
pub use sim::{
    BallAction, BallEvent, BallScript, BallShooterSettings, BallState, BallVec3, ShooterLayout,
    SimBallEstimator, SimRuntimeControls, SimSession, SimSessionConfig, SimWorld,
    new_shutdown_flag,
};
#[cfg(feature = "gui")]
pub use sim::{SimViewerOptions, run_sim_viewer};
pub use telemetry::{Telemetry, TelemetryEvent, TracingTelemetry};
