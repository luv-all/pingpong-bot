use crate::camera;

/// 스틸 한 장의 GT. `pixel`이 `None`이면 **무공**(검출되면 FP, 없으면 TN).
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StillItem {
    /// manifest 기준 상대 경로 (`fly_01_left_t048.png`).
    pub path: String,
    pub camera_id: camera::Id,
    /// 출처 클립 이름 (`fly_01`).
    pub clip: String,
    /// 출처 프레임 번호.
    pub frame: usize,
    /// 공 중심 `[u, v]`. 무공이면 `None`.
    pub pixel: Option<[f64; 2]>,
}

impl StillItem {
    pub fn has_ball(&self) -> bool {
        return self.pixel.is_some();
    }
}
