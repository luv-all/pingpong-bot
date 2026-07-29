//! tracing 로그로 이벤트를 남기는 구현.

use tracing::{debug, info_span};

use super::event::TelemetryEvent;
use super::telemetry_trait::Telemetry;

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
