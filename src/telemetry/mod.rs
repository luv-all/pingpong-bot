//! 텔레메트리. 본선은 [`TracingTelemetry`].

use crate::estimator::Prediction;
use crate::planner::SwingTrajectory;
use tracing::{debug, info_span};

/// 텔레메트리 이벤트.
#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryEvent {
    Prediction(Prediction),
    SwingCommand(SwingTrajectory),
}

/// 시각화와 로깅 출력.
pub trait Telemetry: Send + Sync {
    fn log(&self, event: TelemetryEvent);
}

/// tracing 로그로 이벤트를 남기는 구현.
pub struct TracingTelemetry;

impl Telemetry for TracingTelemetry {
    fn log(&self, event: TelemetryEvent) {
        match event {
            TelemetryEvent::Prediction(prediction) => {
                debug!(
                    time_to_impact_secs = prediction.time_to_impact_secs,
                    x = prediction.impact_position.coords.x,
                    y = prediction.impact_position.coords.y,
                    "궤적 예측"
                );
            }
            TelemetryEvent::SwingCommand(trajectory) => {
                let _span = info_span!(
                    "swing_command",
                    duration_secs = trajectory.duration_secs,
                    rail_start = trajectory.rail.start,
                    rail_end = trajectory.rail.end,
                )
                .entered();
            }
        }
    }
}
