//! 라켓-공 임팩트 역산 (plan §7.1).

use nalgebra::Vector3;

use crate::constants::impact::{
    COOPERATIVE_RETURN_SCALE, LOFT_TIME_TO_NET, MAX_RETURN_SPEED, NET_CLEARANCE,
};
use crate::constants::physics::G_Z;
use crate::constants::table;
use crate::error::SwingPlanError;
use crate::types::{Point3, World};

/// 임팩트에 필요한 라켓 속도·면 법선.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RacketImpactTarget {
    /// 라켓 중심 선속도 [m/s]
    pub velocity: Vector3<f64>,
    /// 라켓 면 법선 (단위 벡터)
    pub normal: Vector3<f64>,
}

/// 네트 중앙을 여유 높이로 넘는 출사 속도 (아래에서 위로 로프트).
///
/// 임팩트점 → `y = LENGTH_Y/2` 에서
/// `z ≥ SURFACE_Z + NET_HEIGHT + NET_CLEARANCE` 가 되도록
/// 비행시간 `LOFT_TIME_TO_NET` 탄도로 \(v_{out}\) 을 잡는다.
pub fn loft_return_velocity(impact: Point3<World>, _v_in: Vector3<f64>) -> Vector3<f64> {
    let y_net = table::LENGTH_Y * 0.5;
    let z_net = table::SURFACE_Z + table::NET_HEIGHT + NET_CLEARANCE;
    let x_aim = table::WIDTH_X * 0.5;

    let t = LOFT_TIME_TO_NET;
    let dy = (y_net - impact.v.y).max(0.25);
    let dx = x_aim - impact.v.x;
    let dz = z_net - impact.v.z;

    // z(t) = z0 + vz t + ½ G.z t²  →  vz = (z_net - z0)/t - ½ G.z t
    let mut v_out = Vector3::new(dx / (t * 2.0), dy / t, dz / t - 0.5 * G_Z * t);

    let speed = v_out.norm();
    if speed > MAX_RETURN_SPEED && speed > f64::EPSILON {
        v_out *= MAX_RETURN_SPEED / speed;
    }
    if v_out.y < 1.0 {
        v_out.y = 1.0;
    }
    if v_out.z < 0.5 {
        v_out.z = 0.5;
    }
    return v_out;
}

/// 레거시 협력 랠리용 부드러운 리턴 (스케일만). 본선 타격은 [`loft_return_velocity`].
pub fn cooperative_return_velocity(v_in: Vector3<f64>) -> Vector3<f64> {
    let mut v_out = -COOPERATIVE_RETURN_SCALE * v_in;
    if v_in.y < 0.0 && v_out.y < 1.5 {
        v_out.y = 1.5;
    }
    let speed = v_out.norm();
    if speed > MAX_RETURN_SPEED && speed > f64::EPSILON {
        v_out *= MAX_RETURN_SPEED / speed;
    }
    return v_out;
}

/// 면 법선 `normal` 기준으로 원하는 출사 속도를 만드는 라켓 속도를 역산한다.
///
/// 법선 방향: `v_out·n = (1+e)·(v_r·n) − e·(v_in·n)`
/// 접선 방향(스핀 무시): 라켓 접선 속도 ≈ 출사 접선 속도.
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

/// `v_in`·`v_out`·`normal`·`e`가 임팩트 모델과 일치하는지 검증한다.
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

/// 무저항 탄도로 네트 통과 높이를 검증한다.
pub fn clears_net_ballistic(impact: Point3<World>, v_out: Vector3<f64>) -> bool {
    let y_net = table::LENGTH_Y * 0.5;
    let z_min = table::SURFACE_Z + table::NET_HEIGHT + NET_CLEARANCE * 0.5;
    if v_out.y <= 1e-6 {
        return false;
    }
    let t = (y_net - impact.v.y) / v_out.y;
    if t <= 0.0 || t > 2.0 {
        return false;
    }
    let z = impact.v.z + v_out.z * t + 0.5 * G_Z * t * t;
    return z >= z_min;
}

fn vector3_to_array(v: Vector3<f64>) -> [f64; 3] {
    return [v.x, v.y, v.z];
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::DEFAULT_RESTITUTION;

    #[test]
    fn loft_return_has_upward_and_forward() {
        let impact = Point3::new(
            table::WIDTH_X * 0.5,
            table::DEFAULT_HIT_PLANE_Y,
            table::SURFACE_Z + 0.05,
        );
        let v_in = Vector3::new(0.0, -7.5, -1.0);
        let v_out = loft_return_velocity(impact, v_in);
        assert!(v_out.y > 1.0, "앞으로: {v_out:?}");
        assert!(v_out.z > 0.5, "위로: {v_out:?}");
        assert!(
            clears_net_ballistic(impact, v_out),
            "네트 통과 실패 v_out={v_out:?}"
        );
    }

    #[test]
    fn loft_required_racket_satisfies_impact_model() {
        let impact = Point3::new(0.76, 0.30, 0.80);
        let v_in = Vector3::new(0.1, -5.0, -0.8);
        let v_out = loft_return_velocity(impact, v_in);
        let normal = Vector3::new(0.0, 0.85, 0.53).normalize();
        let v_r = required_racket_velocity(v_in, v_out, normal, DEFAULT_RESTITUTION).expect("v_r");
        assert!(v_r.z > 0.0, "라켓도 위로: {v_r:?}");
        assert!(verify_impact_model(
            v_in,
            v_out,
            v_r,
            normal,
            DEFAULT_RESTITUTION
        ));
    }

    #[test]
    fn cooperative_return_slows_incoming_ball() {
        let v_in = Vector3::new(0.0, -5.0, 0.0);
        let v_out = cooperative_return_velocity(v_in);
        assert!(v_out.y > 0.0);
        assert!(v_out.norm() < v_in.norm());
    }
}
