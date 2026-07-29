use thiserror::Error;

/// 레일 설정 검증 실패.
#[derive(Debug, Error)]
pub enum RailConfigError {
    #[error("enabled=true일 때 dll_path는 비어 있으면 안 됩니다")]
    DllPathEmpty,
    #[error("enabled=true일 때 pulses_per_meter는 0보다 커야 합니다")]
    PulsesPerMeter,
    #[error("x_min_m은 x_max_m보다 작아야 합니다")]
    InvalidRange,
    #[error("motion 파라미터가 유효하지 않습니다")]
    MotionParams,
}
