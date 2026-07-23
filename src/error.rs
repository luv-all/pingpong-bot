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
    /// 목표 라켓속도를 관절속도로 역산한 결과가 특이점 근처처럼 관절
    /// 속도 한계를 크게 벗어남 - 이 IK 해로 스윙을 시도하면 quintic
    /// 균일 스케일다운(`fit_end_velocity`)이 다른 모든 관절까지
    /// 저속으로 뭉개버려 사실상 "임팩트"가 사라진다.
    NearSingularity {
        joint_index: usize,
        required_speed: f64,
        speed_limit: f64,
    },
    /// 임팩트 자세·목표속도 자체는 도달 가능한데, 거기까지 잇는 quintic
    /// 궤적(+ 팔로스루)이 중간에 관절 각도/속도 한계를 벗어남.
    ///
    /// 이전에는 이 실패가 `InverseKinematicsNoSolution`으로 보고돼
    /// "목표가 팔 도달 범위 밖"이라는 **사실과 다른** 메시지가 나갔다
    /// (2026-07-23). 실제로는 목표에 IK 해가 멀쩡히 있고 필요 관절속도도
    /// 한계의 60% 수준인데도 같은 메시지가 떠, 조사 방향이 리치/속도
    /// 재보정 쪽으로 잘못 유도됐다.
    TrajectoryExceedsLimits {
        rail_end_x: f64,
        /// 실제로 위반한 한계 이름 (관절 속도/각가속도/각도 범위, 레일 속도/범위).
        violated: &'static str,
    },
    /// 궤적이 관절 각도/속도 한계는 지키지만 **토크** 한계를 넘음.
    /// `utilization`은 최악 관절의 `|토크|/한계` 비율(>1이면 초과).
    TrajectoryExceedsTorque { rail_end_x: f64, utilization: f64 },
    /// 임팩트 자세는 도달 가능한데 궤적 중간에 테이블 등과 충돌한다.
    ///
    /// 위와 같은 이유로 별도 variant로 분리했다.
    TrajectoryCollides { rail_end_x: f64 },
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
            Self::NearSingularity {
                joint_index,
                required_speed,
                speed_limit,
            } => write!(
                f,
                "특이점 근처 IK 해 - 관절 {joint_index} 필요속도 {required_speed:.2} rad/s \
                 가 한계 {speed_limit:.2} rad/s를 크게 초과"
            ),
            Self::TrajectoryExceedsLimits {
                rail_end_x,
                violated,
            } => write!(
                f,
                "임팩트 자세는 도달 가능하나 quintic 궤적이 중간에 [{violated}] \
                 한계를 벗어남 (레일 끝 x={rail_end_x:.3} m)"
            ),
            Self::TrajectoryExceedsTorque {
                rail_end_x,
                utilization,
            } => write!(
                f,
                "임팩트 자세는 도달 가능하나 궤적이 토크 한계를 초과 \
                 (최악 관절 이용률 {:.0}%, 레일 끝 x={rail_end_x:.3} m)",
                utilization * 100.0
            ),
            Self::TrajectoryCollides { rail_end_x } => write!(
                f,
                "임팩트 자세는 도달 가능하나 궤적 중간에 충돌 발생 \
                 (레일 끝 x={rail_end_x:.3} m)"
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
