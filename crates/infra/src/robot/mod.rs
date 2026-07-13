//! sim·제어용 로봇 조립 및 URDF.

mod builder;
pub mod urdf;

pub use builder::{MountPreset, RobotBuildError, RobotBuilder, SimRobot};
pub use urdf::{
    map_control_joints_or_truncate, map_control_joints_to_urdf, validate_control_to_urdf_map,
    UrdfGeometry, UrdfLinkVisual, UrdfLoadError, UrdfRobot,
};
