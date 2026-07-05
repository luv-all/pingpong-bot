//! 핀홀 카메라 투영 (sim 전용).

use pingpong_domain::{constants::table, PixelPoint};
use rapier3d::prelude::Vector;

/// 카메라 시야 설정.
#[derive(Debug, Clone, Copy)]
pub struct CameraView {
    /// 카메라 위치 (월드)
    pub eye: Vector,
    /// 바라보는 점
    pub target: Vector,
    /// 위쪽 방향
    pub up: Vector,
    /// 이미지 너비 [px]
    pub width: u32,
    /// 이미지 높이 [px]
    pub height: u32,
    /// 수직 시야각 [rad]
    pub fov_y: f32,
}

impl CameraView {
    /// 카메라 대수에 따라 테이블 주위에 배치한다.
    pub fn for_camera_index(index: u8, count: u8) -> Self {
        let t = if count <= 1 {
            0.5
        } else {
            (f32::from(index) + 0.5) / f32::from(count)
        };
        let angle = std::f32::consts::FRAC_PI_2 + t * std::f32::consts::PI;
        let radius = 2.2;
        let height = 1.85;
        let table_center = Vector::new(
            (table::WIDTH_X * 0.5) as f32,
            (table::LENGTH_Y * 0.5) as f32,
            table::SURFACE_Z as f32,
        );

        return Self {
            eye: table_center
                + Vector::new(radius * angle.cos(), radius * angle.sin(), height),
            target: table_center,
            up: Vector::new(0.0, 0.0, 1.0),
            width: 640,
            height: 480,
            fov_y: 55.0_f32.to_radians(),
        };
    }

    /// 월드 좌표 [m] → 픽셀. 시야 밖·카메라 뒤면 `None`.
    pub fn project(&self, world: Vector) -> Option<PixelPoint> {
        let forward = (self.target - self.eye).normalize();
        let right = forward.cross(self.up).normalize();
        let up = right.cross(forward);

        let rel = world - self.eye;
        let x_cam = rel.dot(right);
        let y_cam = rel.dot(up);
        let z_cam = rel.dot(forward);

        if z_cam <= 0.05 {
            return None;
        }

        let half_fov = self.fov_y * 0.5;
        let tan_half = half_fov.tan();
        let scale = (f64::from(self.height) * 0.5) / f64::from(z_cam * tan_half);
        let px = f64::from(self.width) * 0.5 + f64::from(x_cam) * scale;
        let py = f64::from(self.height) * 0.5 - f64::from(y_cam) * scale;

        if px < 0.0
            || py < 0.0
            || px >= f64::from(self.width)
            || py >= f64::from(self.height)
        {
            return None;
        }

        return Some(PixelPoint::new(px, py));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
