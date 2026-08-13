//! Overshoot 목표 위치·방향 계산.

use nalgebra::Vector3;
use pingpong_bot::Point3;
use pingpong_bot::constants::table::{OPPONENT_HALF_CENTER_Y, WIDTH_X};

/// 목표 접촉점에서 상대 탁구대 중앙을 향하는 수평 단위벡터(`push_direction`)와,
/// 그 방향으로 `overshoot_m`만큼 더 나아간 지점(`overshoot_position`)을 계산한다.
/// `physics.rs`의 `ball_alignment_geometry`와 같은 방향 공식이지만, 공/라켓
/// 두께 오프셋 없이 목표점을 그대로 라켓 중심으로 쓴다(이 도구는 모션의
/// 모양을 보는 것이 목적이라 접촉 기하를 간략화했다).
pub fn overshoot_target(target: Point3, overshoot_m: f64) -> (Point3, Vector3<f64>) {
    let toward_opponent_center =
        Vector3::new(WIDTH_X * 0.5 - target.x, OPPONENT_HALF_CENTER_Y - target.y, 0.0);
    let push_direction = if toward_opponent_center.norm_squared() > 1e-12 {
        toward_opponent_center.normalize()
    } else {
        Vector3::y()
    };
    let overshoot_position = Point3::from(target.coords + push_direction * overshoot_m);
    return (overshoot_position, push_direction);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overshoot_position_moves_along_push_direction_by_the_requested_distance() {
        let target = Point3::new(WIDTH_X * 0.5, 1.0, 0.9);
        let (overshoot_position, push_direction) = overshoot_target(target, 0.05);
        assert!((push_direction.norm() - 1.0).abs() < 1e-9);
        let moved = overshoot_position - target;
        assert!((moved.norm() - 0.05).abs() < 1e-6);
        assert!(
            (moved.normalize() - push_direction).norm() < 1e-6,
            "overshoot should move exactly along push_direction"
        );
    }

    #[test]
    fn push_direction_points_toward_the_opponent_half() {
        // 목표점이 상대 탁구대 중앙(y가 더 큰 쪽)보다 가까운 쪽에 있다고 가정하면,
        // push_direction의 y 성분은 양수여야 한다(상대편 쪽으로 민다).
        let target = Point3::new(WIDTH_X * 0.5, 0.2, 0.9);
        let (_, push_direction) = overshoot_target(target, 0.05);
        assert!(
            push_direction.y > 0.0,
            "push_direction should point toward the opponent half: {push_direction:?}"
        );
    }

    #[test]
    fn zero_overshoot_leaves_the_position_unchanged() {
        let target = Point3::new(WIDTH_X * 0.3, 0.8, 0.95);
        let (overshoot_position, _) = overshoot_target(target, 0.0);
        assert!((overshoot_position - target).norm() < 1e-9);
    }
}
