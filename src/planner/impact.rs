//! 라켓-공 임팩트 역산.

use nalgebra::Vector3;

use crate::Point3;
use crate::constants::physics::G_Z;
use crate::constants::table;
use crate::defaults;
use crate::error::SwingPlanError;

/// 네트를 넘고 상대 코트 중앙에 바운드하는 출사 속도.
///
/// 목표 바운드는 `(WIDTH/2, LENGTH*3/4, SURFACE+BALL_RADIUS)`이며,
/// 무저항 중력 탄도의 경계값 문제를 풀어 `v_out`을 구한다.
pub fn rally_return_velocity(impact: Point3, _v_in: Vector3<f64>) -> Vector3<f64> {
    let impact_cfg = defaults::ImpactParams::default();
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
/// 법선: \(v_{\mathrm{out}}\cdot n = (1+e)\,(v_r\cdot n) - e\,(v_{\mathrm{in}}\cdot n)\)
///
/// 접선은 **월드 +Z 리프트만** 싣는다. 예전 점착 가정
/// \(v_r = n\,v_{r,n} + v_{\mathrm{out},t}\) 는 출사 접선 전체(특히 횡방향)를
/// 라켓이 통째로 나르라고 해서, 관절 예산의 ~90%를 효과 없는 축에 쓰고
/// `fit_end_velocity` 균일 스케일로 법선까지 같이 깎였다. 횡방향 조준은
/// 라켓 면 방향(IK)에 맡기고, 스윙 속도는 법선 + 네트 클리어용 올려치기만
/// 책임진다.
/// 필요한 라켓 속도를 **필수 법선 성분**과 **선택 리프트 성분**으로 나눠 준다.
///
/// 임팩트 모델은 접선을 건드리지 않으므로 \(v_{\mathrm{out}} - v_{\mathrm{in}}\)은
/// 항상 \(n\)에 평행하다. 즉 **`v_out`을 실제로 결정하는 건 법선 성분뿐이고**,
/// 리프트는 [`verify_impact_model`]이 검사하는 식에 아예 들어가지 않는다
/// (Rapier 접선 마찰로만 2차 효과가 남는다).
///
/// 그런데 최소노름 속도 IK는 제곱노름을 최소화하므로, 두 성분을 합쳐 통째로
/// 넘기면 관절 예산이 **크기 비율의 제곱으로** 배분된다. 실측(2026-07-27,
/// `tests/diag_weak_return.rs`): 법선 1.070 / 리프트 1.157 → 예산의 54%가
/// 기여 0인 축으로 갔고, 부풀려진 피크 관절속도 때문에 균일 스케일이 걸려
/// **정작 유효한 법선 성분까지 0.178 m/s로 뭉개졌다**(필요치의 1/6).
///
/// 그래서 둘을 분리해 돌려준다. 호출부는 법선을 온전히 확보한 뒤 관절 예산이
/// 남는 만큼만 리프트를 실으면 된다 — 속도 IK가 `v_r`에 대해 선형이라
/// \(\dot q(a + \alpha b) = \dot q(a) + \alpha\,\dot q(b)\)가 정확히 성립한다.
pub fn required_racket_velocity_parts(
    v_in: Vector3<f64>,
    v_out: Vector3<f64>,
    normal: Vector3<f64>,
    restitution: f64,
) -> Result<(Vector3<f64>, Vector3<f64>), SwingPlanError> {
    let unreachable = || SwingPlanError::ReturnVelocityUnreachable {
        incoming_velocity: vector3_to_array(v_in),
        outgoing_velocity: vector3_to_array(v_out),
    };

    let n = normal.normalize();
    if n.norm() < f64::EPSILON {
        return Err(unreachable());
    }

    let v_in_n = v_in.dot(&n);
    let v_out_n = v_out.dot(&n);
    let v_r_n = (v_out_n + restitution * v_in_n) / (1.0 + restitution);

    if !v_r_n.is_finite() {
        return Err(unreachable());
    }

    let v_out_t = v_out - n * v_out_n;
    // 월드 수직 성분만 접선 요구에 남긴다 (올려치기). 수평 접선은 버린다.
    let lift_t = Vector3::new(0.0, 0.0, v_out_t.z);
    let lift_t = lift_t - n * lift_t.dot(&n);
    return Ok((n * v_r_n, lift_t));
}

/// [`required_racket_velocity_parts`]의 두 성분 합 — 예산을 나눠 쓸 필요가
/// 없는 호출부(검증·진단·테스트)용.
pub fn required_racket_velocity(
    v_in: Vector3<f64>,
    v_out: Vector3<f64>,
    normal: Vector3<f64>,
    restitution: f64,
) -> Result<Vector3<f64>, SwingPlanError> {
    let (normal_part, lift_part) =
        required_racket_velocity_parts(v_in, v_out, normal, restitution)?;
    return Ok(normal_part + lift_part);
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
    let z_min = table::SURFACE_Z
        + table::NET_HEIGHT
        + defaults::ImpactParams::default().net_clearance * 0.5;
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
        let t = defaults::ImpactParams::default().rally_time_to_bounce;
        let z_at_bounce = impact.coords.z + v_out.z * t + 0.5 * G_Z * t * t;
        assert!((z_at_bounce - bounce_z).abs() < 1e-6);
    }

    #[test]
    fn verify_roundtrip() {
        let impact = Point3::new(0.5, 0.3, 0.9);
        let v_in = Vector3::new(0.1, -5.0, -0.5);
        let v_out = rally_return_velocity(impact, v_in);
        let normal = (v_out - v_in).normalize();
        let e = defaults::ImpactParams::default().racket_effective_restitution;
        let v_r = required_racket_velocity(v_in, v_out, normal, e).expect("v_r");
        assert!(verify_impact_model(v_in, v_out, v_r, normal, e));
    }

    /// 관절 속도 예산은 유한하다. 횡방향 접선까지 실어 나르면
    /// `fit_end_velocity` 균일 스케일이 법선·리프트까지 같이 죽인다.
    #[test]
    fn required_racket_velocity_drops_lateral_tangent() {
        let impact = Point3::new(
            table::WIDTH_X * 0.35,
            table::DEFAULT_HIT_PLANE_Y,
            table::SURFACE_Z + 0.24,
        );
        let v_in = Vector3::new(0.8, -5.5, 0.5);
        let v_out = rally_return_velocity(impact, v_in);
        let normal = (v_out - v_in).normalize();
        let e = defaults::ImpactParams::default().racket_effective_restitution;
        let v_r = required_racket_velocity(v_in, v_out, normal, e).expect("v_r");
        let v_r_n = v_r.dot(&normal);
        let v_r_t = v_r - normal * v_r_n;
        let sticky_t = v_out - normal * v_out.dot(&normal);
        let sticky = normal * v_r_n + sticky_t;
        let sticky_horiz = (sticky_t.x * sticky_t.x + sticky_t.y * sticky_t.y).sqrt();
        let vr_horiz = (v_r_t.x * v_r_t.x + v_r_t.y * v_r_t.y).sqrt();
        assert!(
            vr_horiz < sticky_horiz * 0.5,
            "수평 접선 요구가 줄어야 함: vr_horiz={vr_horiz:.2} sticky_horiz={sticky_horiz:.2}"
        );
        assert!(
            v_r_t.z > 0.05,
            "네트 클리어용 올려치기(+Z)는 남겨야 함: v_r_t.z={}",
            v_r_t.z
        );
        assert!(
            v_r.norm() < sticky.norm(),
            "전체 점착 가정보다 요구 |v_r|가 작아야 함: {:.2} vs {:.2}",
            v_r.norm(),
            sticky.norm()
        );
    }
}
