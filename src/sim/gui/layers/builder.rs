//! [`SceneLayers`] 빌더.

use crate::sim::gui::ball;
use crate::sim::gui::shooter;

use super::SceneLayers;
use crate::sim::gui::robot;

#[derive(Default)]
pub struct SceneLayersBuilder {
    ball: Option<ball::Handle>,
    ghost: Option<ball::Handle>,
    robot: Option<robot::Handle>,
    shooter: Option<shooter::Handle>,
}

impl SceneLayersBuilder {
    pub fn ball(mut self, handle: ball::Handle) -> Self {
        self.ball = Some(handle);
        return self;
    }

    /// 비교용 반투명 공.
    pub fn ghost(mut self, handle: ball::Handle) -> Self {
        self.ghost = Some(handle);
        return self;
    }

    pub fn robot(mut self, handle: robot::Handle) -> Self {
        self.robot = Some(handle);
        return self;
    }

    pub fn shooter(mut self, handle: shooter::Handle) -> Self {
        self.shooter = Some(handle);
        return self;
    }

    pub fn build(self) -> SceneLayers {
        return SceneLayers {
            ball: self.ball,
            ghost: self.ghost,
            robot: self.robot,
            shooter: self.shooter,
        };
    }
}
