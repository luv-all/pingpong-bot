use std::time::Instant;

use crate::camera;

/// sim·구 경로: 이미 아는 픽셀 힌트 (검출기 우회).
pub trait HintSource: Send {
    fn next_hint(&mut self) -> Option<(camera::Id, Option<camera::Pixel>, Instant)>;
}
