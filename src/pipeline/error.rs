//! 파이프라인 실행 오류.

use super::PipelineThread;

/// 파이프라인 실행 오류.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineError {
    ThreadPanicked { thread: PipelineThread },
}

impl std::fmt::Display for PipelineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return match self {
            Self::ThreadPanicked { thread } => {
                write!(f, "파이프라인 {thread} 스레드가 패닉했습니다")
            }
        };
    }
}

impl std::error::Error for PipelineError {}
