//! 라켓-공 임팩트 역산.

use nalgebra::Vector3;

use crate::Point3;
use crate::constants::physics::G_Z;
use crate::constants::table;
use crate::defaults;
use crate::error::SwingPlanError;

/// 임팩트 역산의 공개 진입점.
pub struct Impact;

impl Impact {
    pub fn rally_return(impact: Point3, incoming_velocity: Vector3<f64>) -> Vector3<f64> {
        return rally_return_velocity(impact, incoming_velocity);
    }

    /// [`Self::rally_return`]의 WP3 이전 구현(고정 깊이 목표) — A/B 비교·
    /// 방어적 폴백 전용. 프로덕션 경로는 항상 [`Self::rally_return`]을 쓴다.
    pub fn rally_return_fixed_point(impact: Point3, cfg: &defaults::ImpactParams) -> Vector3<f64> {
        return rally_return_velocity_fixed_point(impact, cfg);
    }

    pub fn required_racket_velocity(
        incoming_velocity: Vector3<f64>,
        outgoing_velocity: Vector3<f64>,
        normal: Vector3<f64>,
        restitution: f64,
    ) -> Result<Vector3<f64>, SwingPlanError> {
        return compute_required_racket_velocity(
            incoming_velocity,
            outgoing_velocity,
            normal,
            restitution,
        );
    }

    pub fn verify(
        incoming_velocity: Vector3<f64>,
        outgoing_velocity: Vector3<f64>,
        racket_velocity: Vector3<f64>,
        normal: Vector3<f64>,
        restitution: f64,
    ) -> bool {
        return verify_impact_model(
            incoming_velocity,
            outgoing_velocity,
            racket_velocity,
            normal,
            restitution,
        );
    }

    pub fn clears_net(impact: Point3, outgoing_velocity: Vector3<f64>) -> bool {
        return clears_net_ballistic(impact, outgoing_velocity);
    }
}

/// 네트를 넘고 상대 코트에 바운드하는 출사 속도.
///
/// **WP3(2026-07-30) — 최소속도 네트클리어 공식을 실측 후 기각, 고정목표
/// 유지.** "깊은 지점을 정확히 맞힐 필요 없이 네트만 넘기면 된다"는
/// 지적(사용자, 2026-07-30) 자체는 타당해 `rally_return_velocity_min_effort`로
/// 구현했지만, **A/B 실측 결과 오히려 `peak_joint_speed_ratio`(r)가 나빠짐을
/// 확인했다**(`diag_wp3_target_distance_sweep`: r_mean 2.076→2.721,
/// r_max 3.555→4.822, 60개 샘플 전 지점).
///
/// 원인은 반직관적이다 — `|v_r|`(요구 라켓속도 **크기**)는 두 공식이
/// 거의 같다(1.785 vs 1.846, 3% 차이). `r`을 실제로 좌우하는 건 크기가
/// 아니라 **방향**이 그 임팩트 포즈의 자코비안 조건수와 얼마나 맞는가다.
/// 대표 사례 실측(`v_in=(0,-5,0.7)`, `impact=(WIDTH/2,0.08,SURFACE+0.18)`):
/// 두 공식의 `v_r`이 `y`성분 **부호가 반대**로 나온다(fixed:+0.555,
/// min_effort:−0.126) — 크기는 비슷해도 이 특정 자세에서 그 방향으로
/// 가는 게 base yaw(q0) 관절엔 훨씬 무리라(−6.95→−10.53 rad/s) r이
/// 되레 커진다. 즉 "필요 출사속도를 줄인다"는 목표는 `r`(실제 병목)의
/// 대리지표로 부적절했다 — `r`은 v_out formula가 아니라 IK가 고르는
/// 자세의 자코비안에 지배된다. 상세: `docs/wp3-rally-target-distance.md`.
pub fn rally_return_velocity(impact: Point3, _v_in: Vector3<f64>) -> Vector3<f64> {
    return rally_return_velocity_fixed_point(impact, &defaults::ImpactParams::default());
}

/// 네트 y위치에서 클리어런스 높이(`clears_net_ballistic`과 같은 `z_min`)에
/// **정확히 닿는 최소속도** 탄도. 고전 탄도학의 "주어진 지점을 최소속도로
/// 통과하는 발사각" 문제 — y-z 평면에서 임팩트 위치 대비 목표까지
/// `(Δy,Δz)`일 때 `v_min = √(g(Δz+R))`, `R=√(Δy²+Δz²)`, 최적 발사각은
/// `tanθ=(Δz+R)/Δy`(유도: `docs/wp3-rally-target-distance.md` §2). x속도는
/// 그 통과 시각(`t_net`)에 x가 코트 중앙에 오도록 역산한다 — 좌우
/// 조준 자체는 바꾸지 않는다(원래 계획의 WP3, "좌우 중앙 타겟팅 필요성"은
/// 별개 질문으로 남는다).
#[cfg(test)]
pub fn rally_return_velocity_min_effort(impact: Point3, _v_in: Vector3<f64>) -> Vector3<f64> {
    let impact_cfg = defaults::ImpactParams::default();
    let y_net = table::LENGTH_Y * 0.5;
    // `clears_net_ballistic`가 쓰는 판정 기준 z_min 자체를 목표로 삼으면
    // 부동소수점 반올림에 따라 미세하게 미달할 수 있다(접선이라 여유가
    // 0이라서 어느 쪽으로도 튈 수 있음) — 그 판정 함수와 같은 산식을 다시
    // 밟는 이상 완전히 결정적으로 맞아떨어진다는 보장이 없다. 판정 기준
    // 자체(`net_clearance`)는 그대로 두고, 이 최소화 목표에만 작은 수치
    // 안전여유를 더한다(실제 마진은 그대로 유지, 접선 계산만 강건하게).
    const TANGENCY_SAFETY_MARGIN_M: f64 = 0.003;
    let z_min = table::SURFACE_Z
        + table::NET_HEIGHT
        + impact_cfg.net_clearance * 0.5
        + TANGENCY_SAFETY_MARGIN_M;
    let g = -G_Z;

    let dy = y_net - impact.coords.y;
    let dz = z_min - impact.coords.z;
    if dy <= f64::EPSILON {
        // InterceptWindow는 항상 로봇 쪽(네트 이전) y만 주므로 실제로는
        // 도달하지 않는 방어적 폴백 — 옛 고정목표 공식으로.
        return rally_return_velocity_fixed_point(impact, &impact_cfg);
    }
    let r = (dy * dy + dz * dz).sqrt();
    let v_min_sq = g * (dz + r);
    if !v_min_sq.is_finite() || v_min_sq <= 0.0 {
        return rally_return_velocity_fixed_point(impact, &impact_cfg);
    }
    let v_min = v_min_sq.sqrt();
    let d = (dy * dy + (dz + r) * (dz + r)).sqrt();
    let v_y = v_min * dy / d;
    let v_z = v_min * (dz + r) / d;

    let t_net = dy / v_y;
    let v_x = (table::WIDTH_X * 0.5 - impact.coords.x) / t_net;

    let mut v_out = Vector3::new(v_x, v_y, v_z);
    let speed = v_out.norm();
    if speed > impact_cfg.max_return_speed && speed > f64::EPSILON {
        v_out *= impact_cfg.max_return_speed / speed;
    }
    return v_out;
}

/// WP3 이전의 고정목표 2점 경계값 공식 — `rally_return_velocity_min_effort`가
/// 물리적으로 이상한 입력(네트 너머 임팩트 등)을 만나면 쓰는 방어적
/// 폴백이자, WP3 진단이 새 공식과 A/B 비교할 때 쓰는 기준선.
pub fn rally_return_velocity_fixed_point(
    impact: Point3,
    impact_cfg: &defaults::ImpactParams,
) -> Vector3<f64> {
    let target = Vector3::new(
        table::WIDTH_X * 0.5,
        table::LENGTH_Y * impact_cfg.rally_target_y_frac,
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
fn compute_required_racket_velocity(
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

    /// WP3 이전 고정목표 공식 — `rally_return_velocity_fixed_point`로
    /// 이름이 바뀐 뒤에도 원래 성질(고정 시간에 정확히 바운드 지점 도달)은
    /// 그대로 유지되는지 확인한다. 프로덕션 기본값은 더 이상 이 함수가
    /// 아니다(아래 `rally_return_min_effort_grazes_net_exactly` 참고).
    #[test]
    fn fixed_point_formula_matches_impact_model() {
        let impact = Point3::new(0.42, table::DEFAULT_HIT_PLANE_Y, table::SURFACE_Z + 0.08);
        let cfg = defaults::ImpactParams::default();
        let v_out = rally_return_velocity_fixed_point(impact, &cfg);
        let bounce_z = table::SURFACE_Z + crate::constants::BALL_RADIUS;
        assert!(v_out.y > 0.0);
        let t = cfg.rally_time_to_bounce;
        let z_at_bounce = impact.coords.z + v_out.z * t + 0.5 * G_Z * t * t;
        assert!((z_at_bounce - bounce_z).abs() < 1e-6);
    }

    /// WP3 — `rally_return_velocity_min_effort` 자체가 네트를
    /// **최소한으로만**(안전여유 `TANGENCY_SAFETY_MARGIN_M` 정도만)
    /// 넘기는지 확인한다. **주의: 이 함수는 더 이상 프로덕션 기본값이
    /// 아니다** — A/B 실측 결과 `r`이 오히려 나빠져 기각됐다(위
    /// `rally_return_velocity` doc comment 참고). 이 테스트는 함수 자체의
    /// 기하학적 정확성(공식이 의도대로 접선을 찾는지)만 검증하는 회귀
    /// 가드로 남긴다.
    #[test]
    fn rally_return_min_effort_grazes_net_exactly() {
        for (x, y, z) in [
            (
                table::WIDTH_X * 0.5,
                table::DEFAULT_HIT_PLANE_Y,
                table::SURFACE_Z + 0.08,
            ),
            (table::WIDTH_X * 0.2, 0.10, table::SURFACE_Z + 0.15),
            (table::WIDTH_X * 0.8, 0.30, table::SURFACE_Z + 0.30),
        ] {
            let impact = Point3::new(x, y, z);
            let v_out = rally_return_velocity_min_effort(impact, Vector3::new(0.0, -5.0, -0.7));
            assert!(v_out.y > 0.0, "네트 쪽으로 날아가야 함");
            assert!(clears_net_ballistic(impact, v_out));

            let y_net = table::LENGTH_Y * 0.5;
            let z_min = table::SURFACE_Z
                + table::NET_HEIGHT
                + defaults::ImpactParams::default().net_clearance * 0.5;
            let t_net = (y_net - impact.coords.y) / v_out.y;
            let z_at_net = impact.coords.z + v_out.z * t_net + 0.5 * G_Z * t_net * t_net;
            // 판정 기준보다는 높되, 옛 공식(코트 3/4 깊이를 맞히려고 네트를
            // 몇십 cm씩 여유있게 넘기던 것)보다는 훨씬 작은 여유여야 한다.
            assert!(
                z_at_net >= z_min && z_at_net < z_min + 0.02,
                "네트 통과 높이가 z_min 바로 위(안전여유만)여야 함: \
                 z_at_net={z_at_net:.6} z_min={z_min:.6}"
            );

            // x가 네트 통과 시각에 정확히 코트 중앙이어야 한다(조준 불변).
            let x_at_net = impact.coords.x + v_out.x * t_net;
            assert!(
                (x_at_net - table::WIDTH_X * 0.5).abs() < 1e-6,
                "네트 통과 시각 x가 코트 중앙이어야 함: {x_at_net:.6}"
            );
        }
    }

    #[test]
    fn verify_roundtrip() {
        let impact = Point3::new(0.5, 0.3, 0.9);
        let v_in = Vector3::new(0.1, -5.0, -0.5);
        let v_out = rally_return_velocity(impact, v_in);
        let normal = (v_out - v_in).normalize();
        let e = defaults::ImpactParams::default().racket_effective_restitution;
        let v_r = compute_required_racket_velocity(v_in, v_out, normal, e).expect("v_r");
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
        let v_r = compute_required_racket_velocity(v_in, v_out, normal, e).expect("v_r");
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
