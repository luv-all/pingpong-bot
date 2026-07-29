//! 관측/삼각측량 관련 오류.

use thiserror::Error;

use crate::camera;

#[derive(Debug, Clone, PartialEq, Error)]
pub enum ObservationError {
    /// 삼각측량에 필요한 카메라 수 부족
    #[error("삼각측량 카메라 부족 - {cameras_with_observation}/{required}대만 관측됨")]
    TriangulationInsufficient {
        cameras_with_observation: usize,
        required: usize,
    },
    /// 동기화 시각 보간에 필요한 앞뒤 관측 프레임 없음
    #[error("{camera_id} - 동기화 시각 보간용 앞뒤 프레임 없음")]
    InterpolationFailed { camera_id: camera::Id },
    /// Calibration에 해당 카메라가 없음
    #[error("{camera_id} - Calibration에 파라미터 없음")]
    MissingCalibration { camera_id: camera::Id },
    /// DLT가 유한한 3D 점을 내지 못함 (퇴화/수치 실패)
    #[error("DLT 삼각측량 실패 (퇴화 또는 비유한 해)")]
    TriangulationFailed,
}
