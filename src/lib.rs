//! pingpong-bot 공용 라이브러리.
//!
//! 경연용 단일 애플리케이션 안에서 카메라·추정·로봇·시뮬레이션·계획을
//! 기능별 모듈로 나눈다.

pub mod camera;
pub mod constants;
pub mod defaults;
pub mod detector;
pub mod error;
pub mod estimator;
pub mod hardware;
pub mod logging;
pub mod pipeline;
pub mod planner;
pub mod robot;
pub mod sim;
pub mod telemetry;

/// 월드 좌표 점 [m] — `nalgebra::Point3<f64>`.
pub type Point3 = nalgebra::Point3<f64>;

pub use camera::{
    BallObservation, Calibration, CamCliArgs, CamRigConfig, CamStreamArgs, CameraId, CameraParams,
    CameraRole, CaptureBackend, CharucoBoardSpec, CharucoCalibReport, CharucoFrameDetect,
    DEFAULT_FOV_Y_DEG, DEFAULT_STREAM_FOURCC, DEFAULT_STREAM_FPS, DEFAULT_STREAM_HEIGHT,
    DEFAULT_STREAM_WIDTH, ExposureReadout, Frame, FrameSource, HintSource, ImageDirSource,
    MAX_REPROJ_RMSE_PX, MIN_CHARUCO_CORNERS, OpenCvCapture, PixelPickMouse, PixelPoint,
    PreviewAction, ResolvedCam, ShowBgrResult, SimCamera, StereoCamCliArgs, StereoPairCliArgs,
    StreamPreset, TABLE_LANDMARK_COUNT, TableLandmark, TablePnpResult, ThreadedCapture, arducam_b0332,
    arrow_delta, calibrate_charuco, calibrate_table_pnp, destroy_window, detect_and_draw_charuco,
    display_fit_bounds, dlt_triangulate, draw_cam_label, draw_circle_px, draw_debug_lines,
    draw_help_lines, draw_pixel_loupe, draw_world_grid, draw_world_velocity, ensure_reproj_below,
    ensure_reproj_ok, fit_bgr_downscale, hstack_bgr, parse_fourcc, resolve_cams, sample_at, show_bgr,
    table_landmark_mesh_edges, table_landmarks, triangulate_projections, triangulate_synced,
    triangulate_views, unscale_xy, upsert_camera, apply_grid_key, WorldGridParams,
};
pub use defaults::{
    ControlParams, DEFAULT_CALIBRATION_PATH, DEFAULT_CALIBRATION_PENDING_NAME,
    DEFAULT_COLORMASK_PATH, DEFAULT_DATA_DIR, EstimatorParams, ImpactParams, PhysicsParams,
    SimMotorParams, calibration_path, calibration_pending_path, camera_params_for, colormask_for,
    colormask_path, detector_for, ensure_parent_dir, primitive_4dof, rail_frame, robot, shared_robot,
    urdf_4dof, urdf_test,
};
pub use detector::{
    BallDetector, Candidate, CandidateGenerator, ColorContourCascade, ColorSpace, ColormaskBgr,
    ColormaskCam, ColormaskDetector, ColormaskParams, ColormaskSet, ContourDetector, FloorEdgeMask,
    FuseDetector, IntoCandidateGenerators, MotionPrior, ParseColorSpaceError, RoiParams, RoiTrack,
    Scorer, ScorerParams, SpatialGate, fuse, load_colormask_set, load_colormask_set_or_empty,
    passthrough_detect, save_colormask_set, scorer_params_from_calib, track, undistort_frame,
};
pub use error::{DomainError, HwError, ObservationError, SwingPlanError};
pub use estimator::{
    BallEkf, BounceEvent, Estimator, HitPlane, Prediction, RollEvent, TrajPoint, detect_bounces,
    detect_rolls, drag_from_trajectory, format_physics_for_defaults,
    friction_from_tangential_speeds, mean_bounce_e, mean_roll_mu, predict_hit_plane,
    restitution_from_bounce_heights, restitution_from_normal_speeds,
};
#[cfg(feature = "real")]
pub use hardware::RealHardware;
pub use hardware::dynamixel::DynamixelConfig;
pub use hardware::rail::RailConfig;
pub use hardware::{Hardware, SimHardware};
pub use logging::init_tracing;
pub use pipeline::{CameraFeed, PipelineConfig, PipelineError, PipelineThread, run};
pub use planner::{
    BangBangTrajectory, InterceptWindow, MAX_INTERCEPT_SAMPLES, OrientedBox,
    PlannedBangBangIntercept, RacketGuidanceScratch, RacketGuidanceStep, RailMotion,
    SwingFeasibility, SwingTrajectory, accel, aero_accel, ball_past_midcourt_for_commit,
    clamp_above_table, in_swing_commit_window, plan_bang_bang_swing, plan_best_swing,
    plan_coarse_track, plan_return_to_center, plan_swing, rally_return_velocity,
    required_racket_velocity, robot_obbs, step_racket_guidance, swing_feasibility,
    table_penetration, verify_impact_model,
};
pub use robot::{
    Arm, ArmBuildError, ArmBuilder, JointLimit, Joints, LinearRail, LinkInertial, MountPreset,
    RacketPose, RailFrame, Robot, RobotBuildError, RobotBuilder, RobotPose, RobotState,
    SerialChain, SerialChainError, SerialJoint, UrdfGeometry, UrdfLinkVisual, UrdfLoadError,
    UrdfModel, is_feasible, required_torque,
};
pub use sim::{
    BallShooterSettings, BallState, ShooterLayout, SimBallEstimator, SimRuntimeControls,
    SimSession, SimSessionConfig, SimWorld, new_shutdown_flag,
};
#[cfg(feature = "gui")]
pub use sim::{
    BallHandle, BallOnlyViewerOptions, BallVisual, RobotHandle, SceneHostOptions, SceneLayers,
    ShooterHandle, SimScene, SimSceneBuilder, SimViewerOptions, TableSceneOptions,
    build_table_scene, run_ball_only_viewer, run_scene_host, run_sim_viewer,
};
pub use telemetry::{Telemetry, TelemetryEvent, TracingTelemetry};
