//! 씬 레이어 핸들 — 개체별 원시 R/W (jog 커맨드 enum 없음).
//!
//! | 레이어 | Read | Write |
//! |--------|------|-------|
//! | [`ball`] | 표시 위치 | `set_position` / hide |
//! | [`robot`] | pose · FK · busy | `set_pose` · `set_targets` · `play` · `cancel` |
//! | [`shooter`] | settings · position | settings · shoot/park |
//!
//! jog의 `ik`/`pose`/`swing` 등은 **툴**이 궤적·포즈로 만든 뒤 [`RobotHandle`]에
//! write한다. 커맨드가 바뀌어도 이 모듈은 그대로다.

mod ball;
mod robot;
mod shooter;

pub use ball::BallHandle;
pub use robot::RobotHandle;
pub use shooter::ShooterHandle;

/// 호스트에 붙일 레이어 조합 (없는 것은 렌더·IO 모두 생략).
#[derive(Clone, Default)]
pub struct SceneLayers {
    pub ball: Option<BallHandle>,
    pub robot: Option<RobotHandle>,
    pub shooter: Option<ShooterHandle>,
}

impl SceneLayers {
    pub fn builder() -> SceneLayersBuilder {
        return SceneLayersBuilder::default();
    }
}

#[derive(Default)]
pub struct SceneLayersBuilder {
    ball: Option<BallHandle>,
    robot: Option<RobotHandle>,
    shooter: Option<ShooterHandle>,
}

impl SceneLayersBuilder {
    pub fn ball(mut self, handle: BallHandle) -> Self {
        self.ball = Some(handle);
        return self;
    }

    pub fn robot(mut self, handle: RobotHandle) -> Self {
        self.robot = Some(handle);
        return self;
    }

    pub fn shooter(mut self, handle: ShooterHandle) -> Self {
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
