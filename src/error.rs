//! 도메인 전용 에러 타입.
//!
//! variant마다 왜 실패했는지 필드로 담아 로그/디버깅에 바로 쓸 수 있게 한다.

use std::fmt;

use crate::CameraId;

/// 도메인 계층 공통 에러.
#[derive(Debug, Clone, PartialEq)]
pub enum DomainError {
    /// 스윙 계획/실행 불가
    InfeasibleSwing(SwingPlanError),
    /// 관측/삼각측량 오류
    InvalidObservation(ObservationError),
}

/// 스윙 궤적을 만들거나 실행할 수 없는 이유.
#[derive(Debug, Clone, PartialEq)]
pub enum SwingPlanError {
    /// 역기구학 해가 없음 - 목표 위치가 팔 도달 범위 밖
    InverseKinematicsNoSolution {
        target_x: f64,
        target_y: f64,
        target_z: f64,
    },
    /// 임팩트 시각까지 남은 시간이 최소 스윙 소요 시간보다 짧음
    InsufficientTime {
        time_to_impact_secs: f64,
        min_swing_secs: f64,
    },
    /// 임팩트 모델상 원하는 리턴 속도를 만들 수 없음
    ReturnVelocityUnreachable {
        incoming_velocity: [f64; 3],
        outgoing_velocity: [f64; 3],
    },
}

/// 관측/삼각측량 관련 오류.
#[derive(Debug, Clone, PartialEq)]
pub enum ObservationError {
    /// 삼각측량에 필요한 카메라 수 부족
    TriangulationInsufficient {
        cameras_with_observation: usize,
        required: usize,
    },
    /// 동기화 시각 보간에 필요한 앞뒤 관측 프레임 없음
    InterpolationFailed { camera_id: CameraId },
    /// Calibration에 해당 카메라가 없음
    MissingCalibration { camera_id: CameraId },
    /// DLT가 유한한 3D 점을 내지 못함 (퇴화/수치 실패)
    TriangulationFailed,
}

/// 하드웨어 포트 오류.
#[derive(Debug, Clone, PartialEq)]
pub enum HwError {
    /// 스윙 명령 전송 실패
    CommandFailed {
        /// 궤적 소요 시간 [s]
        duration_secs: f64,
        /// 관절 축 수
        joint_count: usize,
    },
    /// 관절 읽기 실패
    ReadFailed,
    /// 하드웨어 설정 검증 실패
    InvalidConfig { reason: String },
}

impl fmt::Display for DomainError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return match self {
            Self::InfeasibleSwing(e) => write!(f, "스윙 궤적 불가: {e}"),
            Self::InvalidObservation(e) => write!(f, "관측값 오류: {e}"),
        };
    }
}

impl fmt::Display for SwingPlanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return match self {
            Self::InverseKinematicsNoSolution {
                target_x,
                target_y,
                target_z,
            } => write!(
                f,
                "역기구학 해 없음 - 목표 위치 ({target_x:.3}, {target_y:.3}, {target_z:.3}) m 가 도달 범위 밖"
            ),
            Self::InsufficientTime {
                time_to_impact_secs,
                min_swing_secs,
            } => write!(
                f,
                "임팩트까지 {time_to_impact_secs:.3}s 남음 - 최소 스윙 {min_swing_secs:.3}s 필요"
            ),
            Self::ReturnVelocityUnreachable {
                incoming_velocity,
                outgoing_velocity,
            } => write!(
                f,
                "목표 리턴 속도 불가 - 입사 [{:.2}, {:.2}, {:.2}] -> 목표 [{:.2}, {:.2}, {:.2}] m/s",
                incoming_velocity[0],
                incoming_velocity[1],
                incoming_velocity[2],
                outgoing_velocity[0],
                outgoing_velocity[1],
                outgoing_velocity[2]
            ),
        };
    }
}

impl fmt::Display for ObservationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return match self {
            Self::TriangulationInsufficient {
                cameras_with_observation,
                required,
            } => write!(
                f,
                "삼각측량 카메라 부족 - {cameras_with_observation}/{required}대만 관측됨"
            ),
            Self::InterpolationFailed { camera_id } => {
                write!(f, "{camera_id} - 동기화 시각 보간용 앞뒤 프레임 없음")
            }
            Self::MissingCalibration { camera_id } => {
                write!(f, "{camera_id} - Calibration에 파라미터 없음")
            }
            Self::TriangulationFailed => write!(f, "DLT 삼각측량 실패 (퇴화 또는 비유한 해)"),
        };
    }
}

impl fmt::Display for HwError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return match self {
            Self::CommandFailed {
                duration_secs,
                joint_count,
            } => write!(
                f,
                "하드웨어 명령 실패 ({duration_secs:.3}s, {joint_count}축): 통신/전송 오류"
            ),
            Self::ReadFailed => write!(f, "하드웨어 상태 읽기 실패: 통신/전송 오류"),
            Self::InvalidConfig { reason } => write!(f, "하드웨어 설정 오류: {reason}"),
        };
    }
}

impl std::error::Error for DomainError {}
impl std::error::Error for SwingPlanError {}
impl std::error::Error for ObservationError {}
impl std::error::Error for HwError {}
