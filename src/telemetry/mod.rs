//! 텔레메트리 어댑터.
//!
//! Phase 2 본선은 [`TracingTelemetry`]. Rerun 어댑터는 마일스톤 4.

use crate::camera::BallObservation;
use crate::estimator::Prediction;
use crate::planner::SwingTrajectory;
use tracing::{debug, info_span};

/// 텔레메트리 이벤트.
#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryEvent {
    BallObservation(BallObservation),
    Prediction(Prediction),
    SwingCommand(SwingTrajectory),
}

/// 시각화와 로깅 출력.
pub trait Telemetry: Send + Sync {
    fn log(&self, event: TelemetryEvent);
}

/// 텔레메트리를 무시하는 no-op 구현.
pub struct NoopTelemetry;

impl Telemetry for NoopTelemetry {
    fn log(&self, _event: TelemetryEvent) {}
}

/// tracing 로그로 이벤트를 남기는 구현.
pub struct TracingTelemetry;

impl Telemetry for TracingTelemetry {
    fn log(&self, event: TelemetryEvent) {
        match event {
            TelemetryEvent::BallObservation(observation) => {
                debug!(
                    camera_id = %observation.camera_id,
                    x = observation.pixel.x,
                    y = observation.pixel.y,
                    "공 관측"
                );
            }
            TelemetryEvent::Prediction(prediction) => {
                debug!(
                    time_to_impact_secs = prediction.time_to_impact_secs,
                    x = prediction.impact_position.v.x,
                    y = prediction.impact_position.v.y,
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
