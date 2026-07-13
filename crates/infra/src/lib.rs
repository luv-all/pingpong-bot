//! # pingpong-infra
//!
//! 도메인 포트(trait)의 **어댑터** 구현.
//! OpenCV, Rapier, Dynamixel, AXL, Rerun 등은 여기에만 들어간다.
//!
//! - `sim` (기본): Rapier3d 디지털 트윈 + macOS 개발
//! - `real`: Windows 전용 실물 하드웨어 (AXL은 `#[cfg(windows)]` 격리)

mod camera;
mod clock;
mod detector;
mod estimator;
mod hardware;
mod robot;
mod sim;
mod telemetry;

pub use camera::{SimCamera, SyntheticCamera};
pub use clock::{SimClock, SystemClock};
pub use detector::PassthroughDetector;
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

#[cfg(all(windows, feature = "real"))]
pub use hardware::RealHardware;
