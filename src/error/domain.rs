//! 도메인 계층 공통 에러.

use thiserror::Error;

use super::observation::ObservationError;
use super::swing_plan::SwingPlanError;

#[derive(Debug, Clone, PartialEq, Error)]
pub enum DomainError {
    /// 스윙 계획/실행 불가
    #[error("스윙 궤적 불가: {0}")]
    InfeasibleSwing(#[source] SwingPlanError),
    /// 관측/삼각측량 오류
    #[error("관측값 오류: {0}")]
    InvalidObservation(#[source] ObservationError),
}
