//! 씬 레이어 핸들 — 개체별 원시 R/W (jog 커맨드 enum 없음).
//!
//! | 레이어 | Read | Write |
//! |--------|------|-------|
//! | ball | 표시 위치 | `set_position` / hide |
//! | robot | pose · FK · busy | `set_pose` · `set_targets` · `play` · `cancel` |
//! | shooter | settings · position | settings · shoot/park |
//!
//! jog의 `ik`/`pose`/`swing` 등은 **툴**이 궤적·포즈로 만든 뒤 [`crate::robot::Handle`]에
//! write한다.

mod builder;

use crate::ball;
use crate::robot;
use crate::shooter;

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
