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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
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

/// 리니어 X 위치·팔 관절각.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotPose {
    /// 레일 위 베이스 x [m]
    pub rail_x: f64,
    /// revolute 관절각
    pub joints: Joints,
}

impl RobotPose {
    /// 레일 x와 관절각으로 포즈를 만든다.
    pub fn new(rail_x: f64, joints: Joints) -> Self {
        return Self { rail_x, joints };
    }
}

/// quintic 스윙에 포함되는 리니어 X 이동.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RailMotion {
    /// 시작 x [m]
    pub start: f64,
    /// 끝 x [m]
    pub end: f64,
    /// 시작 속도 [m/s]
    pub start_velocity: f64,
    /// 끝 속도 [m/s]
    pub end_velocity: f64,
}

impl RailMotion {
    /// 고정 위치 (이동 없음).
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

/// 하드웨어에 전달할 스윙 궤적 (quintic, plan §7.5).
#[derive(Debug, Clone, PartialEq)]
pub struct SwingTrajectory {
    /// 스윙 시작 관절각
    pub start: Joints,
    /// 임팩트 시점 목표 관절각
    pub end: Joints,
    /// 시작 관절 각속도 [rad/s]
    pub start_velocity: Vec<f64>,
    /// 임팩트 시점 관절 각속도 [rad/s]
    pub end_velocity: Vec<f64>,
    /// 궤적 소요 시간 [s]
    pub duration_secs: f64,
    /// 리니어 레일 X 이동
    pub rail: RailMotion,
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
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Calibration {
    /// 등록된 카메라 목록
    pub cameras: Vec<CameraParams>,
}

/// 카메라 1대의 핀홀 캘리브레이션 (OpenCV 관례: +X 오른쪽, +Y 아래, +Z 전방).
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CameraParams {
    /// 카메라 ID
    pub camera_id: CameraId,
    /// 설정용 표시 이름 (예: "공중-왼쪽"). 없으면 `카메라 N번`으로 표시.
    pub label: Option<String>,
    /// 이미지 너비 [px]
    pub width: u32,
    /// 이미지 높이 [px]
    pub height: u32,
    /// 초점거리 x [px]
    pub fx: f64,
    /// 초점거리 y [px]
    pub fy: f64,
    /// 주점 x [px]
    pub cx: f64,
    /// 주점 y [px]
    pub cy: f64,
    /// 월드 → 카메라 회전 (행 = 카메라 축의 월드 방향)
    pub rotation: nalgebra::Matrix3<f64>,
    /// 월드 → 카메라 평행이동: `X_cam = R X_world + t`
    pub translation: Vector3<f64>,
}

impl CameraParams {
    /// sim 기본 배치: 테이블 주위 원호, 테이블 중앙을 바라봄.
    pub fn sim_layout(camera_id: CameraId, camera_count: u8) -> Self {
        let count = camera_count.max(1);
        let index = camera_id.index();
        let t = if count <= 1 {
            0.5
        } else {
            (f64::from(index) + 0.5) / f64::from(count)
        };
        let angle = std::f64::consts::FRAC_PI_2 + t * std::f64::consts::PI;
        let radius = 2.2;
        let height = 1.85;
        let table_center = Vector3::new(
            crate::constants::table::WIDTH_X * 0.5,
            crate::constants::table::LENGTH_Y * 0.5,
            crate::constants::table::SURFACE_Z,
        );
        let eye = table_center + Vector3::new(radius * angle.cos(), radius * angle.sin(), height);
        let width = 640_u32;
        let height_px = 480_u32;
        let fov_y = 55.0_f64.to_radians();
        return Self::look_at(
            camera_id,
            None,
            eye,
            table_center,
            Vector3::new(0.0, 0.0, 1.0),
            width,
            height_px,
            fov_y,
        );
    }

    /// eye → target look-at으로 핀홀 파라미터를 만든다.
    pub fn look_at(
        camera_id: CameraId,
        label: Option<String>,
        eye: Vector3<f64>,
        target: Vector3<f64>,
        world_up: Vector3<f64>,
        width: u32,
        height: u32,
        fov_y: f64,
    ) -> Self {
        let forward = (target - eye).normalize();
        let right = forward.cross(&world_up).normalize();
        let up = right.cross(&forward).normalize();
        // OpenCV: +Y down → camera Y = -up
        let rotation = nalgebra::Matrix3::from_rows(&[
            right.transpose(),
            (-up).transpose(),
            forward.transpose(),
        ]);
        let translation = -rotation * eye;
        let fy = (f64::from(height) * 0.5) / (fov_y * 0.5).tan();
        let fx = fy;
        let cx = f64::from(width) * 0.5;
        let cy = f64::from(height) * 0.5;
        return Self {
            camera_id,
            label,
            width,
            height,
            fx,
            fy,
            cx,
            cy,
            rotation,
            translation,
        };
    }

    /// `P = K [R|t]` (3×4).
    pub fn projection_matrix(&self) -> nalgebra::Matrix3x4<f64> {
        let k = nalgebra::Matrix3::new(self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0);
        let mut rt = nalgebra::Matrix3x4::zeros();
        rt.fixed_view_mut::<3, 3>(0, 0).copy_from(&self.rotation);
        rt.set_column(3, &self.translation);
        return k * rt;
    }

    /// 월드 점 → 픽셀. 카메라 뒤·이미지 밖이면 `None`.
    pub fn project_world(&self, point: Point3<World>) -> Option<PixelPoint> {
        let x_cam = self.rotation * point.v + self.translation;
        if x_cam.z <= 0.05 {
            return None;
        }
        let u = self.fx * (x_cam.x / x_cam.z) + self.cx;
        let v = self.fy * (x_cam.y / x_cam.z) + self.cy;
        if u < 0.0 || v < 0.0 || u >= f64::from(self.width) || v >= f64::from(self.height) {
            return None;
        }
        return Some(PixelPoint::new(u, v));
    }
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

    /// sim 기본 배치로 N대 Calibration을 만든다.
    pub fn sim(camera_count: u8) -> Self {
        let n = camera_count.max(2);
        return Self {
            cameras: (0..n)
                .map(|i| CameraParams::sim_layout(CameraId(i), n))
                .collect(),
        };
    }
}

impl Default for Calibration {
    fn default() -> Self {
        return Self::sim(3);
    }
}
