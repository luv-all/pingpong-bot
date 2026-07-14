//! 헥사고날 포트.
//!
//! 카메라/검출/추정/하드웨어/텔레메트리 경계를 trait 으로 고정한다.
//! infra 가 구현하고 app/bin 이 조립한다.

use std::time::Instant;

use crate::error::HwError;
use crate::types::{
    CameraId, HitPlane, PixelPoint, Point3, Prediction, RobotPose, SwingTrajectory, TelemetryEvent,
};

/// monotonic 시각. sim 에서는 시간 가속 가능.
pub trait Clock: Send {
    fn now(&self) -> Instant;
}

/// 카메라 프레임 소스. 이미지 버퍼는 infra 안에 두고, 여기엔 픽셀만 나온다.
pub trait CameraSource: Send {
    /// (카메라, 검출 픽셀(없으면 None), 시각). 프레임 자체가 없으면 None.
    fn next(&mut self) -> Option<(CameraId, Option<PixelPoint>, Instant)>;
}

/// 프레임에서 공 픽셀을 검출한다.
///
/// sim 은 이미 픽셀을 넘기고, 실물은 이미지 버퍼를 어댑터 안에 두고 여기서 계산한다.
pub trait Detector: Send {
    fn detect(&mut self, hint: Option<PixelPoint>) -> Option<PixelPoint>;
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
