//! 하드웨어 포트 오류.

use thiserror::Error;

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
