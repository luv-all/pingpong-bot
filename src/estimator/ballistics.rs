//! 공 탄도 적분. hit-plane 교차 예측.
//!
//! EKF 짧은 전파랑 hit-plane 예측 둘 다 반암시적(세미-임플리싯) 오일러.
//! Model C: `a = g - k|v|v + k_m(ω × v)` (비행 중 ω는 상수).

use nalgebra::Vector3;

use crate::constants::{ball, table};
use crate::defaults;
use crate::defaults::PhysicsParams;
use crate::planner::physics::accel;
use crate::{HitPlane, Point3, Prediction};

/// 현재 탄도가 네트 클리어 높이를 통과하는지 (슬랙 포함).
///
/// `predict_hit_plane`과 동일 적분·게이트. 네트 전에 지면/타임아웃이면 `true`
/// (네트와 무관한 실패는 게이트 실패로 보지 않음).
pub fn clears_net_gate(
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    omega: Vector3<f64>,
    physics: &PhysicsParams,
) -> bool {
    let est = defaults::estimator();
    let net_y = table::LENGTH_Y * 0.5;
    let net_clear_z = table::SURFACE_Z + table::NET_HEIGHT + ball::RADIUS;
    const NET_GATE_SLACK_M: f64 = 0.012;

    if position.y <= net_y {
        return true;
    }

    let mut pos = position;
    let mut vel = velocity;
    let mut t = 0.0;
    while t < est.max_lead {
        let prev_y = pos.y;
        let prev_z = pos.z;
        let (next_pos, next_vel) = semi_implicit_euler(pos, vel, omega, est.integrate_dt, physics);
        pos = next_pos;
        vel = next_vel;
        t += est.integrate_dt;
        if prev_y > net_y && pos.y <= net_y {
            let denom = pos.y - prev_y;
            let frac = if denom.abs() < 1e-12 {
                0.0
            } else {
                (net_y - prev_y) / denom
            };
            let z_at_net = prev_z + (pos.z - prev_z) * frac;
            return z_at_net + NET_GATE_SLACK_M >= net_clear_z;
        }
    }
    return true;
}

/// 위치/속도/각속도에서 접수 평면 교차를 반암시적 오일러(+바운스)로 예측한다.
///
/// 테이블 위 구름/너무 낮은 궤적은 `None`.
pub fn predict_hit_plane(
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    omega: Vector3<f64>,
    plane: HitPlane,
    physics: &PhysicsParams,
) -> Option<Prediction> {
    let est = defaults::estimator();
    let vy = velocity.y;
    if vy > -est.min_approach_speed_y {
        return None;
    }
    if is_table_rolling(position, velocity) {
        return None;
    }

    if position.y <= plane.y + 1e-3 {
        return None;
    }

    let floor_z = table::SURFACE_Z + ball::RADIUS;
    let net_y = table::LENGTH_Y * 0.5;
    let net_clear_z = table::SURFACE_Z + table::NET_HEIGHT + ball::RADIUS;
    let mut pos = position;
    let mut vel = velocity;
    let mut t = 0.0;

    while t < est.max_lead {
        let prev_y = pos.y;
        let prev_z = pos.z;
        let (next_pos, next_vel) = semi_implicit_euler(pos, vel, omega, est.integrate_dt, physics);
        pos = next_pos;
        vel = next_vel;
        t += est.integrate_dt;

        // 네트 라인 교차: 높이 미달이면 이 탄도는 접수 불가로 본다.
        // 기하 클리어는 SURFACE+NET+R. ballistics 바운스가 Rapier보다 ~1cm
        // 낮게 나와 기본 샷이 오탐지되므로 슬랙을 둔다.
        if prev_y > net_y && pos.y <= net_y {
            let denom = pos.y - prev_y;
            let frac = if denom.abs() < 1e-12 {
                0.0
            } else {
                (net_y - prev_y) / denom
            };
            let z_at_net = prev_z + (pos.z - prev_z) * frac;
            const NET_GATE_SLACK_M: f64 = 0.012;
            if z_at_net + NET_GATE_SLACK_M < net_clear_z {
                return None;
            }
        }

        if prev_y > plane.y && pos.y <= plane.y {
            let denom = pos.y - prev_y;
            let frac = if denom.abs() < 1e-12 {
                0.0
            } else {
                (plane.y - prev_y) / denom
            };
            let t_cross = t - est.integrate_dt + est.integrate_dt * frac;
            if t_cross <= est.min_lead || t_cross > est.max_lead {
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
                impact_position: Point3::from(impact),
                incoming_velocity: vel,
            });
        }
    }

    return None;
}

fn rest_height() -> f64 {
    return table::SURFACE_Z + ball::RADIUS;
}

/// 테이블에 붙어 느리게 구르는 상태 (비행/바운스 중이면 false).
fn is_table_rolling(position: Vector3<f64>, velocity: Vector3<f64>) -> bool {
    let on_table = position.z <= rest_height() + defaults::estimator().min_strike_clearance;
    let flat = velocity.z.abs() < 0.5;
    return on_table && flat;
}

/// 반암시적 오일러: `v += a dt`, 그다음 `p += v_new dt` (+ 테이블 바운스).
pub fn semi_implicit_euler(
    pos: Vector3<f64>,
    vel: Vector3<f64>,
    omega: Vector3<f64>,
    dt: f64,
    physics: &PhysicsParams,
) -> (Vector3<f64>, Vector3<f64>) {
    let a = accel(vel, omega, physics.drag, physics.magnus);
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
    use crate::constants::table;
    use crate::defaults;

    #[test]
    fn default_shot_like_impact_at_hit_plane() {
        // 네트 위로 넘어가는 합성 탄도 (게이트 통과).
        let position = Vector3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y - 0.15,
            table::SURFACE_Z + 0.25,
        );
        let velocity = Vector3::new(0.0, -7.5, 1.0);
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let physics = defaults::physics();
        let pred =
            predict_hit_plane(position, velocity, Vector3::zeros(), plane, &physics).expect("예측");
        assert!((pred.impact_position.coords.y - plane.y).abs() < 1e-5);
        assert!(pred.time_to_impact_secs > defaults::estimator().min_lead);
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
        assert!(
            predict_hit_plane(
                position,
                velocity,
                Vector3::zeros(),
                plane,
                &defaults::physics()
            )
            .is_none()
        );
    }

    #[test]
    fn net_clipping_trajectory_is_rejected() {
        // 네트 높이 아래로 지나가는 탄도 → 예측 None.
        let position = Vector3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y - 0.1,
            table::SURFACE_Z + table::NET_HEIGHT * 0.4,
        );
        let velocity = Vector3::new(0.0, -6.0, 0.0);
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let physics = defaults::physics();
        assert!(
            predict_hit_plane(position, velocity, Vector3::zeros(), plane, &physics).is_none(),
            "네트 미달 탄도는 None"
        );
        assert!(
            !clears_net_gate(position, velocity, Vector3::zeros(), &physics),
            "네트 미달이면 clears_net_gate=false"
        );
    }

    #[test]
    fn topspin_drops_relative_to_backspin() {
        let physics = defaults::physics();
        let position = Vector3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y - 0.2,
            table::SURFACE_Z + 0.30,
        );
        let velocity = Vector3::new(0.0, -6.0, 0.2);
        // 슈터 쪽에서 로봇(-y)으로 날아갈 때 +x ω = topspin → Magnus 하향.
        let topspin = Vector3::new(40.0, 0.0, 0.0);
        let backspin = Vector3::new(-40.0, 0.0, 0.0);
        let dt = 0.05;
        let (_, v_top) = semi_implicit_euler(position, velocity, topspin, dt, &physics);
        let (_, v_back) = semi_implicit_euler(position, velocity, backspin, dt, &physics);
        assert!(
            v_top.z < v_back.z,
            "topspin vz={} should be below backspin vz={}",
            v_top.z,
            v_back.z
        );
    }
}
