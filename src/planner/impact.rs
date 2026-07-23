//! 라켓-공 임팩트 역산.

use nalgebra::Vector3;

use crate::Point3;
use crate::constants::physics::G_Z;
use crate::constants::table;
use crate::error::SwingPlanError;
use crate::defaults;

/// 네트를 넘고 상대 코트 중앙에 바운드하는 출사 속도.
///
/// 목표 바운드는 `(WIDTH/2, LENGTH*3/4, SURFACE+BALL_RADIUS)`이며,
/// 무저항 중력 탄도의 경계값 문제를 풀어 `v_out`을 구한다.
pub fn rally_return_velocity(impact: Point3, _v_in: Vector3<f64>) -> Vector3<f64> {
    let impact_cfg = defaults::impact();
    let target = Vector3::new(
        table::WIDTH_X * 0.5,
        table::LENGTH_Y * 0.75,
        table::SURFACE_Z + crate::constants::BALL_RADIUS,
    );
    let t = impact_cfg.rally_time_to_bounce;
    let gravity_displacement = Vector3::new(0.0, 0.0, 0.5 * G_Z * t * t);
    let mut v_out = (target - impact.coords - gravity_displacement) / t;

    let speed = v_out.norm();
    if speed > impact_cfg.max_return_speed && speed > f64::EPSILON {
        v_out *= impact_cfg.max_return_speed / speed;
    }
    return v_out;
}

/// 면 법선 normal 기준으로 원하는 출사 속도를 만드는 라켓 속도를 역산한다.
///
/// 법선: v_out.n = (1+e)*(v_r.n) - e*(v_in.n)
/// 접선(스핀 무시): 라켓 접선 속도 ~= 출사 접선 속도.
pub fn required_racket_velocity(
    v_in: Vector3<f64>,
    v_out: Vector3<f64>,
    normal: Vector3<f64>,
    restitution: f64,
) -> Result<Vector3<f64>, SwingPlanError> {
    let n = normal.normalize();
    if n.norm() < f64::EPSILON {
        return Err(SwingPlanError::ReturnVelocityUnreachable {
            incoming_velocity: vector3_to_array(v_in),
            outgoing_velocity: vector3_to_array(v_out),
        });
    }

    let v_in_n = v_in.dot(&n);
    let v_out_n = v_out.dot(&n);
    let v_r_n = (v_out_n + restitution * v_in_n) / (1.0 + restitution);

    if !v_r_n.is_finite() {
        return Err(SwingPlanError::ReturnVelocityUnreachable {
            incoming_velocity: vector3_to_array(v_in),
            outgoing_velocity: vector3_to_array(v_out),
        });
    }

    let v_out_t = v_out - n * v_out_n;
    return Ok(n * v_r_n + v_out_t);
}

/// v_in, v_out, normal, e 가 임팩트 모델과 맞는지 본다.
pub fn verify_impact_model(
    v_in: Vector3<f64>,
    v_out: Vector3<f64>,
    v_r: Vector3<f64>,
    normal: Vector3<f64>,
    restitution: f64,
) -> bool {
    let n = normal.normalize();
    let lhs = (v_out - v_r).dot(&n);
    let rhs = -restitution * (v_in - v_r).dot(&n);
    return (lhs - rhs).abs() < 1e-4;
}

/// 무저항 탄도로 네트 통과 높이를 검사한다.
pub fn clears_net_ballistic(impact: Point3, v_out: Vector3<f64>) -> bool {
    let y_net = table::LENGTH_Y * 0.5;
    let z_min =
        table::SURFACE_Z + table::NET_HEIGHT + defaults::impact().net_clearance * 0.5;
    if v_out.y <= 1e-6 {
        return false;
    }
    let t = (y_net - impact.coords.y) / v_out.y;
    if t <= 0.0 || t > 2.0 {
        return false;
    }
    let z = impact.coords.z + v_out.z * t + 0.5 * G_Z * t * t;
    return z >= z_min;
}

fn vector3_to_array(v: Vector3<f64>) -> [f64; 3] {
    return [v.x, v.y, v.z];
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rally_return_clears_net_toward_far_half() {
        let impact = Point3::new(
            table::WIDTH_X * 0.5,
            table::DEFAULT_HIT_PLANE_Y,
            table::SURFACE_Z + 0.12,
        );
        let v_in = Vector3::new(0.0, -4.0, -1.0);
        let v_out = rally_return_velocity(impact, v_in);
        assert!(v_out.y > 0.0);
        assert!(clears_net_ballistic(impact, v_out));
    }

    #[test]
    fn required_racket_matches_impact_model() {
        let impact = Point3::new(0.42, table::DEFAULT_HIT_PLANE_Y, table::SURFACE_Z + 0.08);
        let v_out = rally_return_velocity(impact, Vector3::new(0.2, -5.0, -0.7));
        let bounce_z = table::SURFACE_Z + crate::constants::BALL_RADIUS;
        assert!(v_out.y > 0.0);
        let t = defaults::impact().rally_time_to_bounce;
        let z_at_bounce = impact.coords.z + v_out.z * t + 0.5 * G_Z * t * t;
        assert!((z_at_bounce - bounce_z).abs() < 1e-6);
    }

    #[test]
    fn verify_roundtrip() {
        let impact = Point3::new(0.5, 0.3, 0.9);
        let v_in = Vector3::new(0.1, -5.0, -0.5);
        let v_out = rally_return_velocity(impact, v_in);
        let normal = (v_out - v_in).normalize();
        let e = defaults::impact().racket_effective_restitution;
        let v_r = required_racket_velocity(v_in, v_out, normal, e).expect("v_r");
        assert!(verify_impact_model(v_in, v_out, v_r, normal, e));
    }
}
