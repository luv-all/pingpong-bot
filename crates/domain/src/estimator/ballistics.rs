//! 공 탄도 적분 — hit-plane 교차 예측 (plan §6.3).
//!
//! EKF 짧은 전파와 hit-plane 예측 모두 **반암시적(세미-임플리싯) 오일러**.

use nalgebra::Vector3;

use crate::constants::{ball, estimator as est, table};
use crate::physics_config::PhysicsParams;
use crate::planner::physics::accel;
use crate::types::{HitPlane, Point3, Prediction, World};

/// 위치·속도에서 접수 평면 교차를 반암시적 오일러(+바운스)로 예측한다.
///
/// 테이블 위 구름·너무 낮은 궤적은 `None` (제어 스팸·도달 불가 IK 방지).
pub fn predict_hit_plane(
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    plane: HitPlane,
    drag_coefficient: f64,
) -> Option<Prediction> {
    return predict_hit_plane_with(
        position,
        velocity,
        plane,
        &PhysicsParams {
            drag: drag_coefficient,
            ..PhysicsParams::default()
        },
    );
}

/// [`PhysicsParams`]로 항력·바운스를 지정한 hit-plane 예측.
pub fn predict_hit_plane_with(
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    plane: HitPlane,
    physics: &PhysicsParams,
) -> Option<Prediction> {
    let vy = velocity.y;
    if vy > -est::MIN_APPROACH_SPEED_Y {
        return None;
    }
    if is_table_rolling(position, velocity) {
        return None;
    }

    if position.y <= plane.y + 1e-3 {
        return None;
    }

    let floor_z = table::SURFACE_Z + ball::RADIUS;
    let mut pos = position;
    let mut vel = velocity;
    let mut t = 0.0;

    while t < est::MAX_LEAD {
        let prev_y = pos.y;
        let (next_pos, next_vel) =
            semi_implicit_euler_with(pos, vel, est::INTEGRATE_DT, physics);
        pos = next_pos;
        vel = next_vel;
        t += est::INTEGRATE_DT;

        if prev_y > plane.y && pos.y <= plane.y {
            let denom = pos.y - prev_y;
            let frac = if denom.abs() < 1e-12 {
                0.0
            } else {
                (plane.y - prev_y) / denom
            };
            let t_cross = t - est::INTEGRATE_DT + est::INTEGRATE_DT * frac;
            if t_cross <= est::MIN_LEAD || t_cross > est::MAX_LEAD {
                return None;
            }
            let mut impact = pos;
            impact.y = plane.y;
            if impact.z < floor_z {
                impact.z = floor_z;
            }
            if impact.z > table::SURFACE_Z + 1.2 {
                return None;
            }
            return Some(Prediction {
                time_to_impact_secs: t_cross,
                impact_position: Point3::<World>::from_vector(impact),
                incoming_velocity: vel,
            });
        }
    }

    return None;
}

fn rest_height() -> f64 {
    return table::SURFACE_Z + ball::RADIUS;
}

/// 테이블에 붙어 느리게 구르는 상태 (비행·바운스 중이면 false).
fn is_table_rolling(position: Vector3<f64>, velocity: Vector3<f64>) -> bool {
    let on_table = position.z <= rest_height() + est::MIN_STRIKE_CLEARANCE;
    let flat = velocity.z.abs() < 0.5;
    return on_table && flat;
}

/// 반암시적 오일러: `v += a dt`, 그다음 `p += v_new dt` (+ 테이블 바운스).
///
/// 바운스 \(e,\mu\)는 [`PhysicsParams::default`]. 항력만 `drag_coefficient`.
pub fn semi_implicit_euler(
    pos: Vector3<f64>,
    vel: Vector3<f64>,
    dt: f64,
    drag_coefficient: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    return semi_implicit_euler_with(
        pos,
        vel,
        dt,
        &PhysicsParams {
            drag: drag_coefficient,
            ..PhysicsParams::default()
        },
    );
}

/// [`PhysicsParams`]로 항력·바운스를 지정한 적분 스텝.
pub fn semi_implicit_euler_with(
    pos: Vector3<f64>,
    vel: Vector3<f64>,
    dt: f64,
    physics: &PhysicsParams,
) -> (Vector3<f64>, Vector3<f64>) {
    let a = accel(vel, physics.drag);
    let next_vel = vel + a * dt;
    let next_pos = pos + next_vel * dt;
    let floor_z = table::SURFACE_Z + ball::RADIUS;
    if next_pos.z <= floor_z && next_vel.z < 0.0 {
        let mu = physics.friction.clamp(0.0, 1.0);
        let tang_scale = 1.0 - mu;
        return (
            Vector3::new(next_pos.x, next_pos.y, floor_z),
            Vector3::new(
                next_vel.x * tang_scale,
                next_vel.y * tang_scale,
                -next_vel.z * physics.restitution,
            ),
        );
    }
    return (next_pos, next_vel);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{estimator::MIN_LEAD, table};

    #[test]
    fn default_shot_like_impact_at_hit_plane() {
        let position = Vector3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y - 0.15,
            table::SURFACE_Z + 0.15,
        );
        let velocity = Vector3::new(0.0, -7.5, 0.5);
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let pred = predict_hit_plane(position, velocity, plane, 0.0).expect("예측");
        assert!((pred.impact_position.v.y - plane.y).abs() < 1e-5);
        assert!(pred.time_to_impact_secs > MIN_LEAD);
        assert!(pred.incoming_velocity.y < 0.0);
    }

    #[test]
    fn rolling_on_table_is_ignored() {
        let position = Vector3::new(
            table::WIDTH_X * 0.5,
            1.0,
            table::SURFACE_Z + ball::RADIUS + 0.01,
        );
        let velocity = Vector3::new(0.2, -1.5, 0.05);
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        assert!(predict_hit_plane(position, velocity, plane, 0.0).is_none());
    }
}
