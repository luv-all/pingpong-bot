//! 공 표시 위치 R/W.

use std::sync::{Arc, Mutex};

use crate::Point3;
use crate::sim::physics::world::SimWorld;

/// 공 표시 위치 R/W.
///
/// - **External**: `set_position`으로 직접 지정 (verify-stereo).
/// - **World**: external이 `None`이면 `SimWorld` 공 위치를 읽는다 (메인 sim).
#[derive(Clone)]
pub struct BallHandle {
    external: Arc<Mutex<Option<Point3>>>,
    external_velocity: Arc<Mutex<Option<[f64; 3]>>>,
    world: Option<Arc<Mutex<SimWorld>>>,
}

impl BallHandle {
    /// 외부 write 전용 (물리 월드 없음). 초기 위치 `None` = 숨김.
    pub fn new() -> Self {
        return Self {
            external: Arc::new(Mutex::new(None)),
            external_velocity: Arc::new(Mutex::new(None)),
            world: None,
        };
    }

    /// 월드 공 위치를 기본으로 읽고, `set_position(Some)`이면 덮어쓴다.
    pub fn from_world(world: Arc<Mutex<SimWorld>>) -> Self {
        return Self {
            external: Arc::new(Mutex::new(None)),
            external_velocity: Arc::new(Mutex::new(None)),
            world: Some(world),
        };
    }

    /// 이미 공유 슬롯이 있을 때 (하위 호환 `BallOnlyViewerOptions`).
    pub fn from_shared(external: Arc<Mutex<Option<Point3>>>) -> Self {
        return Self {
            external,
            external_velocity: Arc::new(Mutex::new(None)),
            world: None,
        };
    }

    /// 현재 표시에 쓸 위치. external 우선, 없으면 월드.
    pub fn position(&self) -> Option<Point3> {
        if let Ok(guard) = self.external.lock() {
            if let Some(p) = *guard {
                return Some(p);
            }
        }
        let Some(world) = &self.world else {
            return None;
        };
        let Ok(world) = world.lock() else {
            return None;
        };
        let v = world.ball_position();
        return Some(Point3::new(v.x as f64, v.y as f64, v.z as f64));
    }

    /// 표시 위치 설정. `None`이면 숨김(월드는 건드리지 않음).
    pub fn set_position(&self, position: Option<Point3>) {
        if let Ok(mut guard) = self.external.lock() {
            *guard = position;
        }
    }

    /// 표시용 속도 벡터 설정. `None`이면 벡터 숨김.
    pub fn set_velocity(&self, velocity: Option<[f64; 3]>) {
        if let Ok(mut guard) = self.external_velocity.lock() {
            *guard = velocity;
        }
    }

    /// 현재 표시에 쓸 속도 벡터.
    pub fn velocity(&self) -> Option<[f64; 3]> {
        let Ok(guard) = self.external_velocity.lock() else {
            return None;
        };
        return *guard;
    }

    /// 공유 슬롯 (호스트·레거시 옵션용).
    pub fn shared_slot(&self) -> Arc<Mutex<Option<Point3>>> {
        return Arc::clone(&self.external);
    }
}

impl Default for BallHandle {
    fn default() -> Self {
        return Self::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ball_handle_external_roundtrip() {
        let ball = BallHandle::new();
        assert!(ball.position().is_none());
        let p = Point3::new(0.5, 0.7, 0.8);
        ball.set_position(Some(p));
        assert_eq!(ball.position(), Some(p));
        ball.set_position(None);
        assert!(ball.position().is_none());
    }
}
