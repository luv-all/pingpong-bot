//! ChArUco 등으로 측정한 카메라 번들.

use crate::camera;

/// ChArUco 등으로 측정한 카메라 번들. `cameras[i]` <-> `camera::Id(i)`.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Calibration {
    /// 등록된 카메라 목록
    pub cameras: Vec<camera::Params>,
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
    pub fn params(&self, camera_id: camera::Id) -> Option<&camera::Params> {
        return self.cameras.iter().find(|c| c.camera_id == camera_id);
    }

    /// sim 기본 배치로 N대 Calibration을 만든다.
    pub fn sim(camera_count: u8) -> Self {
        let n = camera_count.max(2);
        return Self {
            cameras: (0..n)
                .map(|i| camera::Params::sim_layout(camera::Id(i), n))
                .collect(),
        };
    }

    /// JSON 캘리브 파일을 읽는다.
    pub fn load_json(path: &std::path::Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("calibration 읽기: {}: {e}", path.display()))?;
        return serde_json::from_str(&text)
            .map_err(|e| format!("calibration JSON: {}: {e}", path.display()));
    }
}

impl Default for Calibration {
    fn default() -> Self {
        return Self::sim(3);
    }
}
