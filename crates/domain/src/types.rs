//! 공유 도메인 타입.
//!
//! 채널에 흐르는 건 픽셀 결과와 스윙/예측 값이다. 이미지 버퍼는 infra 안에 둔다.

use std::fmt;
use std::time::Instant;

use nalgebra::Vector3;

/// 월드 좌표 점 [m].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub v: Vector3<f64>,
}

impl Point3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        return Self {
            v: Vector3::new(x, y, z),
        };
    }

    pub fn from_vector(v: Vector3<f64>) -> Self {
        return Self { v };
    }
}

/// 이미지 픽셀 좌표.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PixelPoint {
    pub x: f64,
    pub y: f64,
}

impl PixelPoint {
    pub fn new(x: f64, y: f64) -> Self {
        return Self { x, y };
    }

    pub fn lerp(self, other: Self, w: f64) -> Self {
        return Self {
            x: self.x + (other.x - self.x) * w,
            y: self.y + (other.y - self.y) * w,
        };
    }
}

/// 카메라 식별자.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub struct CameraId(pub u8);

impl CameraId {
    pub const fn new(index: u8) -> Self {
        return Self(index);
    }

    pub fn index(self) -> u8 {
        return self.0;
    }
}

impl fmt::Display for CameraId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return write!(f, "카메라 {}번", self.0);
    }
}

/// 한 프레임에서 검출한 공.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallObservation {
    pub pixel: PixelPoint,
    pub camera_id: CameraId,
    pub timestamp: Instant,
}

/// 접수 평면. 월드 y [m] 하나.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HitPlane {
    pub y: f64,
}

/// EKF가 낸 임팩트 시점 공 상태. 제어 스윙 목표로도 쓴다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prediction {
    pub time_to_impact_secs: f64,
    pub impact_position: Point3,
    pub incoming_velocity: Vector3<f64>,
}

/// revolute 관절각 [rad].
#[derive(Debug, Clone, PartialEq)]
pub struct Joints {
    pub values: Vec<f64>,
}

impl Joints {
    pub fn from_slice(values: &[f64]) -> Self {
        return Self {
            values: values.to_vec(),
        };
    }
}

/// 레일 x + 팔 관절각 스냅샷 (`plan_swing` 입력).
#[derive(Debug, Clone, PartialEq)]
pub struct RobotPose {
    pub rail_x: f64,
    pub joints: Joints,
}

impl RobotPose {
    pub fn new(rail_x: f64, joints: Joints) -> Self {
        return Self { rail_x, joints };
    }
}

/// quintic 스윙에 딸린 리니어 X 이동.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RailMotion {
    pub start: f64,
    pub end: f64,
    pub start_velocity: f64,
    pub end_velocity: f64,
}

impl RailMotion {
    pub const fn fixed(x: f64) -> Self {
        return Self {
            start: x,
            end: x,
            start_velocity: 0.0,
            end_velocity: 0.0,
        };
    }
}

impl Default for RailMotion {
    fn default() -> Self {
        return Self::fixed(0.0);
    }
}

/// 하드웨어에 넘기는 quintic 스윙 궤적.
#[derive(Debug, Clone, PartialEq)]
pub struct SwingTrajectory {
    pub start: Joints,
    pub end: Joints,
    pub start_velocity: Vec<f64>,
    pub end_velocity: Vec<f64>,
    pub duration_secs: f64,
    pub rail: RailMotion,
}

/// 텔레메트리 이벤트.
#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryEvent {
    BallObservation(BallObservation),
    Prediction(Prediction),
    SwingCommand(SwingTrajectory),
}
