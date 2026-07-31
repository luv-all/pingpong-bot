//! 파이프라인 실행 오류.

use super::PipelineThread;

/// 파이프라인 실행 오류.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PipelineError {
    ThreadPanicked { thread: PipelineThread },
    Configuration(String),
}

impl std::fmt::Display for PipelineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return match self {
            Self::ThreadPanicked { thread } => {
                write!(f, "파이프라인 {thread} 스레드가 패닉했습니다")
            }
            Self::Configuration(reason) => write!(f, "파이프라인 설정 오류: {reason}"),
        };
    }
}

impl std::error::Error for PipelineError {}
