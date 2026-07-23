//! pingpong-bot 공용 라이브러리.
//!
//! 경연용 단일 애플리케이션 안에서 카메라·추정·로봇·시뮬레이션·계획을
//! 기능별 모듈로 나눈다.

pub mod camera;
pub mod clock;
pub mod config_resolve;
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

pub use camera::{
    BallObservation, Calibration, CameraId, CameraParams, CharucoBoardSpec, CharucoCalibReport,
    CharucoFrameDetect, ExposureReadout, Frame, FrameSource, HintSource, ImageDirSource,
    MIN_CHARUCO_CORNERS, OpenCvCapture, PixelPoint, PreviewAction, SimCamera, calibrate_charuco,
    destroy_window, detect_and_draw_charuco, dlt_triangulate, draw_cam_label, draw_circle_px,
    draw_debug_lines, draw_help_lines, draw_world_velocity, hstack_bgr, sample_at, show_bgr,
    triangulate_projections, triangulate_synced, triangulate_views,
};
pub use clock::Clock;
pub use config_resolve::{
    DEFAULT_CONFIG_PATH, calibration_path_from_config, resolve_calibration_path,
};
pub use detector::{
    Appearance, AppearanceParams, BallDetector, Candidate, CandidateGenerator, ColorSpace,
    ColormaskConfig, ColormaskDetector, ColormaskParams, ContourDetector, FuseDetector,
    IntoCandidateGenerators, MotionParams, MotionPrior, ParseAppearanceError,
    ParseColorSpaceError, RoiTrack, Scorer, ScorerParams, VisionCameraConfig, VisionConfig, fuse,
    fuse_from_vision, fuse_vision, load_vision_from_config, passthrough_detect, scorer_from_vision,
    track, track_vision, undistort_frame, vision_from_toml,
};
pub use error::{DomainError, HwError, ObservationError, SwingPlanError};
pub use estimator::{
    BallEkf, BounceEvent, Estimator, HitPlane, Prediction, RollEvent, TrajPoint, detect_bounces,
    detect_rolls, drag_from_trajectory, friction_from_tangential_speeds, mean_bounce_e,
    mean_roll_mu, physics_coeffs_toml, predict_hit_plane, restitution_from_bounce_heights,
    restitution_from_normal_speeds,
};
pub use geometry::Point3;
#[cfg(feature = "real")]
pub use hardware::RealHardware;
pub use hardware::{Hardware, SimHardware};
pub use physics_config::{
    PhysicsConfig, PhysicsParams, load_physics_from_config, merge_physics_into_config,
};
pub use pipeline::{
    CameraFeed, DEFAULT_ROBOT_ID, PipelineConfig, PipelineError, PipelineThread, ROBOTS,
    RobotEntry, find_robot, robot_ids_csv, run, shared_competition_arm,
};
pub use planner::{
    BangBangTrajectory, InterceptWindow, MAX_INTERCEPT_SAMPLES, OrientedBox, RailMotion,
    SwingFeasibility, SwingTrajectory, accel, ball_past_midcourt_for_commit, clamp_above_table,
    in_swing_commit_window, plan_bang_bang_swing, plan_best_swing, plan_coarse_track,
    plan_return_to_center, plan_swing, rally_return_velocity, required_racket_velocity,
    robot_obbs, swing_feasibility, table_penetration, verify_impact_model,
};
pub use robot::rail::LinearRail;
pub use robot::{
    Arm, ArmBuildError, ArmBuilder, JointLimit, Joints, LinkInertial, MountPreset, RacketPose,
    RobotBuildError, RobotBuilder, RobotPose, RobotState, SerialChain, SerialChainError,
    SerialJoint, SimRobot, UrdfGeometry, UrdfLinkVisual, UrdfLoadError, UrdfRobot,
};
pub use sim::{
    BallAction, BallEvent, BallScript, BallShooterSettings, BallState, BallVec3, ShooterLayout,
    SimBallEstimator, SimRuntimeControls, SimSession, SimSessionConfig, SimWorld,
    new_shutdown_flag,
};
#[cfg(feature = "gui")]
pub use sim::{SimViewerOptions, run_sim_viewer};
pub use telemetry::{Telemetry, TelemetryEvent, TracingTelemetry};
