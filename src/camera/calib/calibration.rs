//! ChArUco 등 카메라 번들 캘리브레이션.

use crate::constants::table;
use crate::{CameraId, PixelPoint, Point3};
use nalgebra::Vector3;

/// ChArUco 등으로 측정한 카메라 번들. `cameras[i]` <-> `CameraId(i)`.
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
    /// OpenCV 왜곡 계수 (보통 5 또는 8). 빈 벡터 = 왜곡 없음.
    pub dist: Vec<f64>,
    /// 월드 -> 카메라 회전 (행 = 카메라 축의 월드 방향)
    pub rotation: nalgebra::Matrix3<f64>,
    /// 월드 -> 카메라 평행이동: `X_cam = R X_world + t`
    pub translation: Vector3<f64>,
}

impl CameraParams {
    /// sim 기본 배치: 테이블 주위 원호, 테이블 중앙을 바라봄.
    pub fn sim_layout(camera_id: CameraId, camera_count: u8) -> Self {
        let count = camera_count.max(1);
        let index = camera_id.0;
        let t = if count <= 1 {
            0.5
        } else {
            (f64::from(index) + 0.5) / f64::from(count)
        };
        let angle = std::f64::consts::FRAC_PI_2 + t * std::f64::consts::PI;
        let radius = 2.2;
        let height = 1.85;
        let table_center = Vector3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.5,
            table::SURFACE_Z,
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

    /// eye -> target look-at으로 핀홀 파라미터를 만든다.
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
        // OpenCV: +Y down -> camera Y = -up
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
            dist: Vec::new(),
            rotation,
            translation,
        };
    }

    /// 왜곡 계수가 있으면 true.
    pub fn has_distortion(&self) -> bool {
        return self.dist.iter().any(|c| c.abs() > f64::EPSILON);
    }

    /// `P = K [R|t]` (3x4).
    pub fn projection_matrix(&self) -> nalgebra::Matrix3x4<f64> {
        let k = nalgebra::Matrix3::new(self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0);
        let mut rt = nalgebra::Matrix3x4::zeros();
        rt.fixed_view_mut::<3, 3>(0, 0).copy_from(&self.rotation);
        rt.set_column(3, &self.translation);
        return k * rt;
    }

    /// 월드 점 -> 픽셀. 카메라 뒤/이미지 밖이면 `None`.
    pub fn project_world(&self, point: Point3) -> Option<PixelPoint> {
        let x_cam = self.rotation * point.coords + self.translation;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_params_include_empty_dist() {
        let cam = CameraParams::sim_layout(CameraId(0), 3);
        assert!(cam.dist.is_empty());
        assert!(!cam.has_distortion());
    }

    #[test]
    fn camera_params_serde_requires_dist() {
        let cam = CameraParams::sim_layout(CameraId(0), 1);
        let json = serde_json::to_string(&cam).expect("serialize");
        assert!(json.contains("\"dist\""));
        let back: CameraParams = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back.dist, cam.dist);

        let without_dist = r#"{
            "camera_id": 0,
            "label": null,
            "width": 640,
            "height": 480,
            "fx": 500.0,
            "fy": 500.0,
            "cx": 320.0,
            "cy": 240.0,
            "rotation": [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0],
            "translation": [0.0, 0.0, 0.0]
        }"#;
        assert!(serde_json::from_str::<CameraParams>(without_dist).is_err());
    }
}
