//! 헥사고날 **포트** 정의.
//!
//! 카메라·검출·추정·하드웨어·텔레메트리 등 변하는 경계를 trait으로 고정한다.
//! `infra` crate가 이 trait들을 구현(어댑터)하고, `app`/`bin`이 조립한다.

use std::time::Instant;

use crate::error::HwError;
use crate::types::{
    BallObservation, FrameRef, HitPlane, PixelPoint, Prediction, RobotPose, Roi, SwingTrajectory,
    TelemetryEvent,
};

/// monotonic 시각 제공. sim에서는 `SimClock`로 시간 가속 가능.
pub trait Clock: Send {
    /// 현재 시각을 반환한다.
    fn now(&self) -> Instant;
}

/// 카메라 프레임 소스. 프레임은 스레드 안에서만 사용하고 채널에는 `BallObservation`만 보낸다.
pub trait CameraSource: Send {
    /// 다음 프레임을 (카메라 ID, 프레임, 타임스탬프)로 반환한다.
    fn next(&mut self) -> Option<(crate::types::CameraId, FrameRef, Instant)>;
}

/// 2D 프레임에서 공 픽셀 좌표를 검출한다.
pub trait Detector: Send {
    /// ROI 내에서 공 픽셀을 검출한다.
    fn detect(&mut self, frame: FrameRef, roi: Option<Roi>) -> Option<PixelPoint>;
}

/// 공 상태(위치·속도) 추정 및 타격 평면까지 예측 (확장 칼만 필터).
pub trait Estimator: Send {
    /// 새 관측으로 내부 상태를 갱신한다.
    fn update(&mut self, observation: BallObservation);
    /// 타격 평면까지의 임팩트를 예측한다.
    fn predict_to(&self, plane: HitPlane) -> Option<Prediction>;
}

/// 로봇 팔·리니어 액추에이터 구동.
pub trait Hardware: Send {
    /// 스윙 궤적을 하드웨어에 전송한다.
    fn command(&mut self, trajectory: &SwingTrajectory) -> Result<(), HwError>;
    /// 현재 레일 x·관절각을 읽는다.
    fn read_pose(&mut self) -> Result<RobotPose, HwError>;
    /// 스윙 궤적 재생 중이면 true (sim 자동 스윙과 제어 루프 중복 방지).
    fn is_busy(&mut self) -> bool {
        return false;
    }
}

/// 시각화·로깅 (Rerun, tracing 등).
pub trait Telemetry: Send + Sync {
    /// 텔레메트리 이벤트를 기록한다.
    fn log(&self, event: TelemetryEvent);
}
