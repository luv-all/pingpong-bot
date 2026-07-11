//! # pingpong-infra
//!
//! 도메인 포트(trait)의 **어댑터** 구현.
//! OpenCV, Rapier, Dynamixel, AXL, Rerun 등은 여기에만 들어간다.
//!
//! - `sim` (기본): Rapier3d 디지털 트윈 + macOS 개발
//! - `real`: Windows 전용 실물 하드웨어 (AXL은 `#[cfg(windows)]` 격리)

mod clock;
mod detector;
mod robot_builder;
mod sim;
mod synthetic_camera;
mod telemetry;
mod urdf;

#[cfg(all(windows, feature = "real"))]
mod real_hardware;

pub use clock::{SimClock, SystemClock};
pub use detector::PassthroughDetector;
pub use robot_builder::{MountPreset, RobotBuildError, RobotBuilder, SimRobot};
pub use sim::{
    new_shutdown_flag, BallAction, BallEvent, BallScript, BallShooterSettings, BallState,
    BallVec3, SimBallEstimator, SimCamera, SimHardware, SimRuntimeControls, SimSession,
    SimSessionConfig, SimWorld, ShooterLayout,
};
#[cfg(feature = "gui")]
pub use sim::{run_sim_viewer, SimViewerOptions};
pub use synthetic_camera::SyntheticCamera;
pub use telemetry::{NoopTelemetry, TracingTelemetry};
pub use urdf::{UrdfGeometry, UrdfLinkVisual, UrdfLoadError, UrdfRobot};

#[cfg(all(windows, feature = "real"))]
pub use real_hardware::RealHardware;
