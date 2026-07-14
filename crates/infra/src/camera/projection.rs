//! 핀홀 카메라 투영 (sim 전용) — domain `CameraParams`와 동일 모델.

use pingpong_domain::{CameraId, CameraParams, PixelPoint, Point3};
use rapier3d::prelude::Vector;

/// 카메라 시야 — domain 캘리브의 얇은 래퍼.
#[derive(Debug, Clone)]
pub struct CameraView {
    /// 공유 핀홀 파라미터
    pub params: CameraParams,
}

impl CameraView {
    /// 카메라 대수에 따라 테이블 주위에 배치한다 (`CameraParams::sim_layout`).
    pub fn for_camera_index(index: u8, count: u8) -> Self {
        return Self {
            params: CameraParams::sim_layout(CameraId::new(index), count),
        };
    }

    /// 월드 좌표 [m] → 픽셀. 시야 밖·카메라 뒤면 `None`.
    pub fn project(&self, world: Vector) -> Option<PixelPoint> {
        let point = Point3::new(f64::from(world.x), f64::from(world.y), f64::from(world.z));
        return self.params.project_world(point);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_domain::constants::table;

    #[test]
    fn table_center_projects_near_image_center() {
        let view = CameraView::for_camera_index(1, 3);
        let pixel = view
            .project(Vector::new(
                (table::WIDTH_X * 0.5) as f32,
                (table::LENGTH_Y * 0.5) as f32,
                table::SURFACE_Z as f32,
            ))
            .expect("테이블 중앙");
        assert!((pixel.x - 320.0).abs() < 80.0);
        assert!((pixel.y - 240.0).abs() < 80.0);
    }
}
