//! 도메인 전용 에러 타입.
//!
//! variant마다 왜 실패했는지 필드로 담아 로그/디버깅에 바로 쓸 수 있게 한다.

use thiserror::Error;

use crate::Id;

/// 도메인 계층 공통 에러.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum DomainError {
    /// 스윙 계획/실행 불가
    #[error("스윙 궤적 불가: {0}")]
    InfeasibleSwing(#[source] SwingPlanError),
    /// 관측/삼각측량 오류
    #[error("관측값 오류: {0}")]
    InvalidObservation(#[source] ObservationError),
}

/// 스윙 궤적을 만들거나 실행할 수 없는 이유.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum SwingPlanError {
    /// 역기구학 해가 없음 - 목표 위치가 팔 도달 범위 밖
    #[error(
        "역기구학 해 없음 - 목표 위치 ({target_x:.3}, {target_y:.3}, {target_z:.3}) m 가 도달 범위 밖"
    )]
    InverseKinematicsNoSolution {
        target_x: f64,
        target_y: f64,
        target_z: f64,
    },
    /// 임팩트 시각까지 남은 시간이 최소 스윙 소요 시간보다 짧음
    #[error("임팩트까지 {time_to_impact_secs:.3}s 남음 - 최소 스윙 {min_swing_secs:.3}s 필요")]
    InsufficientTime {
        time_to_impact_secs: f64,
        min_swing_secs: f64,
    },
    /// 임팩트 모델상 원하는 리턴 속도를 만들 수 없음
    #[error(
        "목표 리턴 속도 불가 - 입사 [{:.2}, {:.2}, {:.2}] -> 목표 [{:.2}, {:.2}, {:.2}] m/s",
        .incoming_velocity[0],
        .incoming_velocity[1],
        .incoming_velocity[2],
        .outgoing_velocity[0],
        .outgoing_velocity[1],
        .outgoing_velocity[2]
    )]
    ReturnVelocityUnreachable {
        incoming_velocity: [f64; 3],
        outgoing_velocity: [f64; 3],
    },
    /// 목표 라켓속도를 관절속도로 역산한 결과가 특이점 근처처럼 관절
    /// 속도 한계를 크게 벗어남 - 이 IK 해로 스윙을 시도하면 quintic
    /// 균일 스케일다운(`fit_end_velocity`)이 다른 모든 관절까지
    /// 저속으로 뭉개버려 사실상 "임팩트"가 사라진다.
    #[error(
        "특이점 근처 IK 해 - 관절 {joint_index} 필요속도 {required_speed:.2} rad/s \
         가 한계 {speed_limit:.2} rad/s를 크게 초과"
    )]
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
    #[error(
        "임팩트 자세는 도달 가능하나 quintic 궤적이 중간에 [{violated}] \
         한계를 벗어남 (레일 끝 x={rail_end_x:.3} m)"
    )]
    TrajectoryExceedsLimits {
        rail_end_x: f64,
        /// 실제로 위반한 한계 이름 (관절 속도/각가속도/각도 범위, 레일 속도/범위).
        violated: &'static str,
    },
    /// 궤적이 관절 각도/속도 한계는 지키지만 **토크** 한계를 넘음.
    /// `utilization`은 최악 관절의 `|토크|/한계` 비율(>1이면 초과).
    #[error(
        "임팩트 자세는 도달 가능하나 궤적이 토크 한계를 초과 \
         (최악 관절 이용률 {utilization_pct:.0}%, 레일 끝 x={rail_end_x:.3} m)",
        utilization_pct = .utilization * 100.0
    )]
    TrajectoryExceedsTorque { rail_end_x: f64, utilization: f64 },
    /// 임팩트/궤적 자세가 테이블을 관통
    #[error("테이블 관통 {depth:.3}m - 목표 ({target_x:.3}, {target_y:.3}, {target_z:.3}) m")]
    TablePenetration {
        target_x: f64,
        target_y: f64,
        target_z: f64,
        depth: f64,
    },
    /// 관절 속도·가속·토크 또는 레일 한계를 만족하는 궤적 없음
    #[error("관절/토크/레일 한계 - 목표 ({target_x:.3}, {target_y:.3}, {target_z:.3}) m")]
    JointOrTorqueLimit {
        target_x: f64,
        target_y: f64,
        target_z: f64,
    },
}

impl SwingPlanError {
    /// 재시도해도 안전 스윙이 안 나오는 하드 실패 (도달·리턴 불가).
    ///
    /// `InsufficientTime`만 false — 공이 더 가까워질 때까지 기다릴 수 있다.
    pub fn is_hard_unreachable(&self) -> bool {
        return !matches!(self, Self::InsufficientTime { .. });
    }

    /// 디버그 마커용 목표 좌표 (있으면).
    pub fn target_xyz(&self) -> Option<[f64; 3]> {
        return match self {
            Self::InverseKinematicsNoSolution {
                target_x,
                target_y,
                target_z,
            }
            | Self::TablePenetration {
                target_x,
                target_y,
                target_z,
                ..
            }
            | Self::JointOrTorqueLimit {
                target_x,
                target_y,
                target_z,
            } => Some([*target_x, *target_y, *target_z]),
            Self::InsufficientTime { .. }
            | Self::ReturnVelocityUnreachable { .. }
            | Self::NearSingularity { .. }
            | Self::TrajectoryExceedsLimits { .. }
            | Self::TrajectoryExceedsTorque { .. } => None,
        };
    }
}

/// 관측/삼각측량 관련 오류.
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
    InterpolationFailed { camera_id: Id },
    /// Calibration에 해당 카메라가 없음
    #[error("{camera_id} - Calibration에 파라미터 없음")]
    MissingCalibration { camera_id: Id },
    /// DLT가 유한한 3D 점을 내지 못함 (퇴화/수치 실패)
    #[error("DLT 삼각측량 실패 (퇴화 또는 비유한 해)")]
    TriangulationFailed,
}

/// 하드웨어 포트 오류.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum HwError {
    /// 스윙 명령 전송 실패
    #[error("하드웨어 명령 실패 ({duration_secs:.3}s, {joint_count}축): {reason}")]
    CommandFailed {
        /// 궤적 소요 시간 [s]
        duration_secs: f64,
        /// 관절 축 수
        joint_count: usize,
        /// 하위 원인 (시리얼/프로토콜/길이 불일치 등)
        reason: String,
    },
    /// 관절·레일 상태 읽기 실패
    #[error("하드웨어 상태 읽기 실패: {reason}")]
    ReadFailed {
        /// 하위 원인 (시리얼/프로토콜/뮤텍스 등)
        reason: String,
    },
    /// 하드웨어 설정 검증 실패
    #[error("하드웨어 설정 오류: {reason}")]
    InvalidConfig { reason: String },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hard_unreachable_skips_only_insufficient_time() {
        assert!(
            SwingPlanError::InverseKinematicsNoSolution {
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
            }
            .is_hard_unreachable()
        );
        assert!(
            SwingPlanError::ReturnVelocityUnreachable {
                incoming_velocity: [0.0; 3],
                outgoing_velocity: [0.0; 3],
            }
            .is_hard_unreachable()
        );
        assert!(
            !SwingPlanError::InsufficientTime {
                time_to_impact_secs: 0.05,
                min_swing_secs: 0.1,
            }
            .is_hard_unreachable()
        );
        assert!(
            SwingPlanError::TablePenetration {
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
                depth: 0.01,
            }
            .is_hard_unreachable()
        );
        assert!(
            SwingPlanError::JointOrTorqueLimit {
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
            }
            .is_hard_unreachable()
        );
    }

    #[test]
    fn hw_error_display_includes_reason() {
        let err = HwError::ReadFailed {
            reason: "Present Position sync_read 실패".into(),
        };
        assert_eq!(
            err.to_string(),
            "하드웨어 상태 읽기 실패: Present Position sync_read 실패"
        );
    }

    #[test]
    fn domain_error_source_chains_to_swing_plan() {
        use std::error::Error as _;
        let inner = SwingPlanError::InsufficientTime {
            time_to_impact_secs: 0.05,
            min_swing_secs: 0.1,
        };
        let err = DomainError::InfeasibleSwing(inner.clone());
        assert_eq!(
            err.source().map(ToString::to_string),
            Some(inner.to_string())
        );
    }
}
