//! # pingpong-infra
//!
//! 비전(OpenCV SSOT)·sim·하드웨어 어댑터.
//! 경연 바이너리 기준: 비전은 domain 포트로 감싸지 않고 여기 (`vision`)에 둔다.
//!
//! - `sim` (기본): Rapier3d 디지털 트윈 + macOS 개발 (삼각측량 = nalgebra DLT 폴백)
//! - `opencv`: 시스템 OpenCV로 `triangulatePoints` / ChArUco
//! - `real`: Windows 전용 실물 하드웨어 (AXL은 `#[cfg(windows)]` 격리)

mod camera;
mod clock;
mod estimator;
mod hardware;
mod robot;
mod sim;
mod telemetry;
pub mod vision;

pub use camera::{SimCamera, SyntheticCamera};
pub use clock::{SimClock, SystemClock};
pub use estimator::SimBallEstimator;
pub use hardware::SimHardware;
pub use robot::{
    map_control_joints_or_truncate, map_control_joints_to_urdf, validate_control_to_urdf_map,
    MountPreset, RobotBuildError, RobotBuilder, SimRobot, UrdfGeometry, UrdfLinkVisual,
    UrdfLoadError, UrdfRobot,
};
pub use sim::{
    new_shutdown_flag, BallAction, BallEvent, BallScript, BallShooterSettings, BallState,
    BallVec3, SimRuntimeControls, SimSession, SimSessionConfig, SimWorld, ShooterLayout,
};
#[cfg(feature = "gui")]
pub use sim::{run_sim_viewer, SimViewerOptions};
pub use telemetry::{NoopTelemetry, TracingTelemetry};
pub use vision::{
    passthrough_detect, sample_at, triangulate_projections, triangulate_synced, Calibration,
    CameraParams, FrameSource, PassthroughDetector,
};

#[cfg(feature = "opencv")]
pub use vision::calibrate_charuco_draft;

#[cfg(all(windows, feature = "real"))]
pub use hardware::RealHardware;
