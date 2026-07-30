//! URDF 로드 실패.

use std::path::PathBuf;

use thiserror::Error;

/// URDF 로드 실패.
#[derive(Debug, Error)]
pub enum UrdfLoadError {
    #[error("URDF 파일 읽기 실패: {path}")]
    Read {
        path: PathBuf,
        #[source]
        source: urdf_rs::UrdfError,
    },
    #[error("엔드이펙터 link `{link}` 를 URDF에서 찾을 수 없습니다")]
    EndEffectorNotFound { link: String },
    #[error("link `{link}` 까지의 관절 체인을 구성할 수 없습니다")]
    ChainNotFound { link: String },
    #[error("actuated revolute 관절이 없습니다 (ee={ee_link})")]
    NoActuatedJoints { ee_link: String },
    #[error("`Arm` 변환 실패: {reason}")]
    ArmConversion { reason: String },
}
