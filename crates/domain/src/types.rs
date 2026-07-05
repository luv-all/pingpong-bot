//! 공유 도메인 타입.
//!
//! - 좌표계: `Point3<World>` 등 프레임 마커로 혼동 방지
//! - 채널 메시지: `BallObservation`, `Target`, `SwingTrajectory` 등
//! - `FrameRef`: 프레임 행렬은 infra 경계 안에 두고, 채널에는 픽셀 결과만 흘린다 (plan §11.1)

use std::fmt;
use std::marker::PhantomData;
use std::time::Instant;

use nalgebra::Vector3;

// --- 좌표계 마커 (PhantomData로 프레임 구분) ---

/// 월드(전역) 좌표계 마커.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct World;

/// 카메라 좌표계. extrinsics는 `Calibration`이 `CameraId`로 조회한다.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CameraFrame;

/// 3D 점 — 프레임 마커 `F`로 좌표계를 구분한다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3<F> {
    /// 좌표 벡터 [m]
    pub v: Vector3<f64>,
    _frame: PhantomData<F>,
}

impl<F> Point3<F> {
    /// (x, y, z)로 점을 만든다.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        return Self {
            v: Vector3::new(x, y, z),
            _frame: PhantomData,
        };
    }

    /// 벡터에서 점을 만든다.
    pub fn from_vector(v: Vector3<f64>) -> Self {
        return Self {
            v,
            _frame: PhantomData,
        };
    }
}

/// 이미지 픽셀 좌표.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PixelPoint {
    /// 가로 [px]
    pub x: f64,
    /// 세로 [px]
    pub y: f64,
}

impl PixelPoint {
    /// (x, y) 픽셀을 만든다.
    pub fn new(x: f64, y: f64) -> Self {
        return Self { x, y };
    }

    /// 두 픽셀 사이를 선형 보간한다.
    pub fn lerp(self, other: Self, w: f64) -> Self {
        return Self {
            x: self.x + (other.x - self.x) * w,
            y: self.y + (other.y - self.y) * w,
        };
    }
}

/// 카메라 식별자 — 배치·대수는 실험 후 `Calibration`/설정으로 정한다.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CameraId(pub u8);

impl CameraId {
    /// 인덱스로 카메라 ID를 만든다.
    pub const fn new(index: u8) -> Self {
        return Self(index);
    }

    /// 0부터 시작하는 카메라 번호.
    pub fn index(self) -> u8 {
        return self.0;
    }
}

impl fmt::Display for CameraId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return write!(f, "카메라 {}번", self.0);
    }
}

/// 관심 영역 (ROI, Region of Interest) — 직전 공 위치 주변만 검출할 때 쓴다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Roi {
    /// 좌상단 x [px]
    pub x: i32,
    /// 좌상단 y [px]
    pub y: i32,
    /// 너비 [px]
    pub width: i32,
    /// 높이 [px]
    pub height: i32,
}

/// 검출 파이프라인용 프레임 핸들.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameRef {
    pub(crate) pixel: Option<PixelPoint>,
}

impl FrameRef {
    /// sim에서 검출된 픽셀이 있는 프레임.
    pub fn sim(pixel: PixelPoint) -> Self {
        return Self { pixel: Some(pixel) };
    }

    /// 검출 실패·시야 밖 프레임.
    pub fn empty() -> Self {
        return Self { pixel: None };
    }

    /// 프레임에 담긴 공 픽셀 (없으면 None).
    pub fn pixel(&self) -> Option<PixelPoint> {
        return self.pixel;
    }
}

/// 한 프레임에서 검출한 공 — 픽셀 좌표 + 어느 카메라 + 촬영 시각.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallObservation {
    /// 검출 픽셀
    pub pixel: PixelPoint,
    /// 촬영 카메라
    pub camera_id: CameraId,
    /// 촬영 시각
    pub timestamp: Instant,
}

/// 공이 도달할 접수 평면 (월드 y 좌표 [m]).
///
/// 슈터(+y)에서 날아온 공 궤적이 **y = const** 면을 지나는 시점·(x, z)를 예측한다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HitPlane {
    /// 접수 평면 y [m] (로봇 쪽 꼭짓점에서 테이블 길이 방향)
    pub y: f64,
}

/// EKF가 예측한 임팩트 시점의 공 상태.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prediction {
    /// 임팩트까지 남은 시간 [s]
    pub time_to_impact_secs: f64,
    /// 임팩트 위치 (월드 좌표)
    pub impact_position: Point3<World>,
    /// 임팩트 직전 공 속도 [m/s]
    pub incoming_velocity: Vector3<f64>,
}

/// 제어 스레드가 사용하는 스윙 목표.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Target {
    /// EKF 예측 결과
    pub prediction: Prediction,
}

/// revolute 관절각 묶음 [rad].
#[derive(Debug, Clone, PartialEq)]
pub struct Joints {
    /// 축 순서대로의 각도
    pub values: Vec<f64>,
}

impl Joints {
    /// 슬라이스에서 관절각을 만든다.
    pub fn from_slice(values: &[f64]) -> Self {
        return Self {
            values: values.to_vec(),
        };
    }
}

/// 하드웨어에 전달할 스윙 궤적.
#[derive(Debug, Clone, PartialEq)]
pub struct SwingTrajectory {
    /// 목표 관절각
    pub joints: Joints,
    /// 궤적 소요 시간 [s]
    pub duration_secs: f64,
}

/// 텔레메트리·로깅 이벤트 종류.
#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryEvent {
    /// 공 검출 관측
    BallObservation(BallObservation),
    /// 궤적 예측
    Prediction(Prediction),
    /// 스윙 명령
    SwingCommand(SwingTrajectory),
}

/// ChArUco 등으로 측정한 카메라 번들. `cameras[i]` ↔ `CameraId(i)`.
#[derive(Debug, Clone, PartialEq)]
pub struct Calibration {
    /// 등록된 카메라 목록
    pub cameras: Vec<CameraParams>,
}

/// 카메라 1대의 캘리브레이션 메타데이터.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraParams {
    /// 카메라 ID
    pub camera_id: CameraId,
    /// 설정용 표시 이름 (예: "공중-왼쪽"). 없으면 `카메라 N번`으로 표시.
    pub label: Option<String>,
}

impl Calibration {
    /// 등록된 카메라 대수.
    pub fn camera_count(&self) -> usize {
        return self.cameras.len();
    }

    /// 삼각측량 최소 카메라 수 (스테레오 2). 3대 이상이면 정확도 향상.
    pub fn min_cameras_for_triangulation(&self) -> usize {
        return 2;
    }

    /// ID로 카메라 파라미터를 조회한다.
    pub fn params(&self, camera_id: CameraId) -> Option<&CameraParams> {
        return self.cameras.iter().find(|c| c.camera_id == camera_id);
    }
}

impl Default for Calibration {
    fn default() -> Self {
        return Self {
            cameras: (0..3)
                .map(|i| CameraParams {
                    camera_id: CameraId(i),
                    label: None,
                })
                .collect(),
        };
    }
}
