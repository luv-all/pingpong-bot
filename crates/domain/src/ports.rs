//! 남는 포트 — 하드웨어·추정·시계·텔레메트리.
//!
//! 비전(캡처·검출)은 `pingpong_infra::vision` 구체 타입을 쓴다.

use std::time::Instant;

use crate::error::HwError;
use crate::types::{HitPlane, Point3, Prediction, RobotPose, SwingTrajectory, TelemetryEvent};

/// monotonic 시각. sim 에서는 시간 가속 가능.
pub trait Clock: Send {
    fn now(&self) -> Instant;
}

/// 공 상태 추정 + 타격 평면 예측.
pub trait Estimator: Send {
    fn update(&mut self, position: Point3, timestamp: Instant);
    fn predict_to(&self, plane: HitPlane) -> Option<Prediction>;
}

/// 로봇 팔 / 리니어 구동.
pub trait Hardware: Send {
    fn command(&mut self, trajectory: &SwingTrajectory) -> Result<(), HwError>;
    fn read_pose(&mut self) -> Result<RobotPose, HwError>;
    fn is_busy(&mut self) -> bool {
        return false;
    }
}

/// 시각화 / 로깅.
pub trait Telemetry: Send + Sync {
    fn log(&self, event: TelemetryEvent);
}
