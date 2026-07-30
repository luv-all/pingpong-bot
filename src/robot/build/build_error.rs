//! [`RobotBuilder::build`] 실패.

use thiserror::Error;

use crate::robot::urdf::UrdfLoadError;

/// [`RobotBuilder::build`] 실패.
#[derive(Debug, Error)]
pub enum RobotBuildError {
    #[error("URDF 경로가 지정되지 않았습니다")]
    MissingUrdfPath,
    #[error(transparent)]
    Urdf(#[from] UrdfLoadError),
    #[error("`Arm` 변환 실패: {reason}")]
    ArmConversion { reason: String },
}
