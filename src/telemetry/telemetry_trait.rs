//! 시각화와 로깅 출력.

use super::event::TelemetryEvent;

pub trait Telemetry: Send + Sync {
    fn log(&self, event: TelemetryEvent);
}
