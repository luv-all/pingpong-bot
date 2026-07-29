//! 로봇 팔 기구학.
//!
//! `Arm`은 sim/real이 같이 쓰는 불변 기하 모델이다. 부팅 때 [`Robot`]으로 조립하고
//! `robot.arm`으로 FK/IK·스윙 계획에 넘긴다. 공유 배선은 [`crate::defaults::shared_robot`].
//!
//! 조립은 [`build`] (`ArmBuilder` / `RobotBuilder`), 런타임 추종은 [`State`].

pub mod build;
pub mod dynamics;
pub mod rail;
pub mod serial;
pub mod state;
pub mod urdf;

mod arm;
mod joint_limit;
mod joints;
mod link_inertial;
mod playback_trajectory;
mod pose;
mod racket_pose;
mod swing_playback;

#[cfg(feature = "gui")]
pub mod handle;
#[cfg(feature = "gui")]
pub mod primitive_nodes;
#[cfg(feature = "gui")]
pub mod urdf_nodes;
#[cfg(feature = "gui")]
mod urdf_visual_node;
#[cfg(feature = "gui")]
pub mod visual;
#[cfg(feature = "gui")]
mod visual_geom;

#[cfg(test)]
mod tests;

pub use arm::Arm;
pub use build::{ArmBuildError, ArmBuilder, MountPreset, Robot, RobotBuildError, RobotBuilder};
pub use joint_limit::JointLimit;
pub use joints::Joints;
pub use link_inertial::LinkInertial;
pub use pose::Pose;
pub use racket_pose::RacketPose;
pub use rail::{LinearRail, RailFrame};
pub use serial::{SerialChain, SerialChainError, SerialJoint};
pub use state::State;
pub use urdf::{UrdfGeometry, UrdfLinkVisual, UrdfLoadError, UrdfModel};

#[cfg(feature = "gui")]
pub use handle::Handle;
#[cfg(feature = "gui")]
pub use primitive_nodes::PrimitiveNodes;
#[cfg(feature = "gui")]
pub use urdf_nodes::UrdfNodes;
#[cfg(feature = "gui")]
pub use visual::Visual;

/// 하위 호환: `robot::builder` / `robot::loader`
pub use build::builder;
pub use build::loader;
