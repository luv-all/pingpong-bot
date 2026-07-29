//! 파이프라인 스레드 역할.

/// 파이프라인 워커 스레드 역할.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineThread {
    Camera,
    Estimation,
    Control,
}

impl std::fmt::Display for PipelineThread {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return match self {
            Self::Camera => write!(f, "카메라"),
            Self::Estimation => write!(f, "추정"),
            Self::Control => write!(f, "제어"),
        };
    }
}
