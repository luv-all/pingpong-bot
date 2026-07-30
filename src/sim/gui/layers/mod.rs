//! 씬 레이어 핸들 — 개체별 원시 R/W (jog 커맨드 enum 없음).

mod builder;

use crate::sim::gui::ball;
use crate::sim::gui::robot;
use crate::sim::gui::shooter;

pub use builder::SceneLayersBuilder;

/// 호스트에 붙일 레이어 조합 (없는 것은 렌더·IO 모두 생략).
#[derive(Clone, Default)]
pub struct SceneLayers {
    pub ball: Option<ball::Handle>,
    pub robot: Option<robot::Handle>,
    pub shooter: Option<shooter::Handle>,
}

impl SceneLayers {
    pub fn builder() -> SceneLayersBuilder {
        return SceneLayersBuilder::default();
    }
}
