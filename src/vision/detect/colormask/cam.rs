use crate::camera;

use super::ColormaskParams;

/// tune-colormask 픽 샘플 — BGR 트리플. detector는 무시.
pub type ColormaskBgr = [u8; 3];

/// 한 카메라의 colormask 엔트리 (`camera_id` + flatten params + optional samples).
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ColormaskCam {
    pub camera_id: camera::Id,
    #[serde(flatten)]
    pub params: ColormaskParams,
    /// `[[B,G,R], …]` — 픽셀 좌표 없음 (공은 움직이므로 색만 SSOT).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub samples: Vec<ColormaskBgr>,
}
