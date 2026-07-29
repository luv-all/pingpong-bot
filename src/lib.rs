//! pingpong-bot 공용 라이브러리.
//!
//! 경연용 단일 애플리케이션 안에서 카메라·추정·로봇·시뮬레이션·계획을
//! 기능별 모듈로 나눈다.
//!
//! 도메인 타입은 모듈 경로로 쓴다 (`camera::Id`, `ball::Observation`).
//! 루트 `pub use`로 짧은 이름을 펼치지 않는다.

pub mod ball;
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
pub mod swing;
pub mod telemetry;

/// 월드 좌표 점 [m] — `nalgebra::Point3<f64>`.
pub type Point3 = nalgebra::Point3<f64>;

pub use camera::{
    Calibration, CamCliArgs, CamRigConfig, CamStreamArgs, CaptureBackend, Charuco,
    DEFAULT_CLIPS_DIR, DEFAULT_FOV_Y_DEG, DEFAULT_STREAM_FOURCC, DEFAULT_STREAM_FPS,
    DEFAULT_STREAM_HEIGHT, DEFAULT_STREAM_WIDTH, ExposureReadout, Frame, FrameSource, HintSource,
    ImageDirSource, MAX_REPROJ_RMSE_PX, MIN_CHARUCO_CORNERS, MonoOfflineArgs, OpenCvCapture,
    PixelPickMouse, Preview, PreviewAction, ResolvedCam, ResolvedStereoOffline, ShowBgrResult,
    SimCamera, StereoCamCliArgs, StereoClip, StereoOfflineArgs, StereoPairCliArgs, StreamPreset,
    TABLE_LANDMARK_COUNT, TablePnp, ThreadedCapture, Triangulate, WorldGridParams, arducam_b0332,
};
pub use defaults::{
    ControlParams, DEFAULT_CALIBRATION_PATH, DEFAULT_CALIBRATION_PENDING_NAME,
    DEFAULT_COLORMASK_PATH, DEFAULT_DATA_DIR, EstimatorParams, ImpactParams, PhysicsParams,
    SimMotorParams,
};
pub use detector::{
    AppearanceChain, AppearanceLayer, Candidate, CandidateGenerator, ColorSpace, ColormaskBgr,
    ColormaskCam, ColormaskDetector, ColormaskParams, ColormaskSet, ContourDetector, Detector,
    DetectorBuilder, FloorEdgeMask, MotionPrior, ParseColorSpaceError, RoiParams, RoiTrack, Scorer,
    ScorerParams,
};
pub use error::{DomainError, HwError, ObservationError, SwingPlanError};
pub use estimator::{Estimator, HitPlane, Prediction};
#[cfg(feature = "real")]
pub use hardware::RealHardware;
pub use hardware::dynamixel::DynamixelConfig;
pub use hardware::rail::RailConfig;
pub use hardware::{Hardware, SimHardware};
pub use pipeline::{CameraFeed, Pipeline, PipelineConfig, PipelineError, PipelineThread};
pub use planner::{Impact, InterceptWindow, MAX_INTERCEPT_SAMPLES, OrientedBox};
pub use robot::{
    Arm, ArmBuildError, ArmBuilder, JointLimit, Joints, LinearRail, LinkInertial, MountPreset,
    RacketPose, RailFrame, Robot, RobotBuildError, RobotBuilder, RobotPose, RobotState,
    SerialChain, SerialChainError, SerialJoint, UrdfGeometry, UrdfLinkVisual, UrdfLoadError,
    UrdfModel,
};
#[cfg(feature = "gui")]
pub use sim::{
    BallHandle, BallOnlyViewerOptions, BallVisual, RobotHandle, SceneHost, SceneHostOptions,
    SceneLayers, SceneUiDraw, SceneUiHook, ShooterHandle, SimScene, SimSceneBuilder, SimViewer,
    SimViewerOptions, TableSceneOptions,
};
pub use sim::{
    BallShooterSettings, BallState, EvalProtocol, ShooterLayout, SimBallEstimator,
    SimRuntimeControls, SimSession, SimSessionConfig, SimWorld,
};
pub use telemetry::{Telemetry, TelemetryEvent, TracingTelemetry};
