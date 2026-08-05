//! 씬 레이어 핸들 — 개체별 원시 R/W (jog 커맨드 enum 없음).

mod builder;

use crate::sim::gui::ball;
use crate::sim::gui::robot;
use crate::sim::gui::shooter;
use crate::sim::gui::trail;

pub use builder::SceneLayersBuilder;

/// 호스트에 붙일 레이어 조합 (없는 것은 렌더·IO 모두 생략).
#[derive(Clone, Default)]
pub struct SceneLayers {
    pub ball: Option<ball::Handle>,
    /// 비교용 반투명 공 — 같은 씬에 두 번째 위치를 겹쳐 볼 때 쓴다
    /// (예: verify-stereo의 생 삼각측량 vs EKF 출력).
    pub ghost: Option<ball::Handle>,
    pub robot: Option<robot::Handle>,
    pub shooter: Option<shooter::Handle>,
    /// 궤적 폴리라인 — 개수 제한 없음. 렌더 루프가 순서대로 그린다.
    pub trails: Vec<trail::Handle>,
}

impl SceneLayers {
    pub fn builder() -> SceneLayersBuilder {
        return SceneLayersBuilder::default();
    }
}
