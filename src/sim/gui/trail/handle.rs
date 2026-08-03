//! 궤적 폴리라인 R/W.

use std::sync::{Arc, Mutex};

use kiss3d::prelude::Color;

use crate::Point3;

/// 씬에 그릴 궤적 하나.
///
/// 공([`crate::sim::gui::ball::Handle`])과 같은 방식이다 — 외부가 점을 써 넣고 렌더 루프가
/// 매 프레임 읽는다. 선은 노드가 아니라 [`kiss3d::window::Window::draw_line`]로 그리므로
/// 프레임마다 다시 그려야 하고, 그래서 점 개수가 바뀌어도 씬을 재조립할 필요가 없다.
#[derive(Clone)]
pub struct Handle {
    points: Arc<Mutex<Vec<Point3>>>,
    color: Color,
    width: f32,
    /// 범례에 쓸 이름. 없으면 범례에 안 나온다.
    label: Option<&'static str>,
}

impl Handle {
    /// `rgba`는 각 성분 0..1. kiss3d 타입을 받지 않는 건 툴이 kiss3d에 직접 의존하지
    /// 않게 하려는 것이다 — 씬 조립은 `sim::gui` 안에서만 kiss3d를 안다.
    pub fn new(rgba: [f32; 4], width: f32) -> Self {
        return Self {
            points: Arc::new(Mutex::new(Vec::new())),
            color: Color::new(rgba[0], rgba[1], rgba[2], rgba[3]),
            width,
            label: None,
        };
    }

    /// 범례에 이름을 단다.
    ///
    /// 색을 궤적 자신이 들고 있으므로 범례가 실제 선과 어긋날 수가 없다 — 툴마다 색
    /// 목록을 따로 적으면 한쪽만 고치는 날이 온다.
    pub fn labelled(mut self, label: &'static str) -> Self {
        self.label = Some(label);
        return self;
    }

    pub fn label(&self) -> Option<&'static str> {
        return self.label;
    }

    /// 궤적을 통째로 갈아 끼운다. 빈 벡터면 아무것도 안 그린다.
    pub fn set_points(&self, points: Vec<Point3>) {
        if let Ok(mut guard) = self.points.lock() {
            *guard = points;
        }
    }

    pub fn clear(&self) {
        self.set_points(Vec::new());
    }

    /// 렌더 루프가 읽는다. 락을 잡은 채로 그리지 않도록 복사해서 넘긴다 —
    /// 그리는 동안 stdin 스레드가 다음 프레임을 써 넣을 수 있다.
    pub fn points(&self) -> Vec<Point3> {
        let Ok(guard) = self.points.lock() else {
            return Vec::new();
        };
        return guard.clone();
    }

    pub fn color(&self) -> Color {
        return self.color;
    }

    pub fn width(&self) -> f32 {
        return self.width;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 범례는 궤적이 들고 있는 색을 그대로 쓴다 — 따로 적어 둔 색과 어긋날 자리가 없다.
    #[test]
    fn a_labelled_trail_carries_its_own_colour() {
        let trail = Handle::new([1.0, 0.0, 1.0, 1.0], 3.0).labelled("predicted");
        assert_eq!(trail.label(), Some("predicted"));
        assert!((trail.color().r - 1.0).abs() < 1e-6);
        assert_eq!(Handle::new([0.0; 4], 1.0).label(), None);
    }

    #[test]
    fn trail_handle_roundtrip() {
        let trail = Handle::new([1.0, 1.0, 1.0, 1.0], 2.0);
        assert!(trail.points().is_empty());
        trail.set_points(vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
        assert_eq!(trail.points().len(), 2);
        trail.clear();
        assert!(trail.points().is_empty());
    }
}
