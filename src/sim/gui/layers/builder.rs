//! [`SceneLayers`] 빌더.

use crate::ball;
use crate::robot;
use crate::shooter;

use super::SceneLayers;

#[derive(Default)]
pub struct SceneLayersBuilder {
    ball: Option<ball::Handle>,
    robot: Option<robot::Handle>,
    shooter: Option<shooter::Handle>,
}

impl SceneLayersBuilder {
    pub fn ball(mut self, handle: ball::Handle) -> Self {
        self.ball = Some(handle);
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
            robot: self.robot,
            shooter: self.shooter,
        };
    }
}
