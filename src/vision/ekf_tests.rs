use super::*;

/// `Kinematics::step` 을 속도로 중심차분한 야코비안. 적분기가 진실이고 [`jacobian`]은
/// 그걸 손으로 미분한 것이라, 둘이 갈라지면 손 쪽이 틀렸다.
fn numeric(
    v: Vector3,
    position: Vector3,
    dt: f64,
    physics: &PhysicsParams,
) -> (Matrix3<f64>, Matrix3<f64>) {
    const H: f64 = 1e-6;
    let step = |v: Vector3| {
        let (p, v, _) = Kinematics::step(position, v, Vector3::zeros(), dt, physics);
        return (p, v);
    };
    let (mut dp, mut dv) = (Matrix3::zeros(), Matrix3::zeros());
    for axis in 0..3 {
        let (mut plus, mut minus) = (v, v);
        plus[axis] += H;
        minus[axis] -= H;
        let (p_up, v_up) = step(plus);
        let (p_down, v_down) = step(minus);
        dp.set_column(axis, &((p_up - p_down) / (2.0 * H)));
        dv.set_column(axis, &((v_up - v_down) / (2.0 * H)));
    }
    return (dp, dv);
}

fn check(position: Vector3, velocity: Vector3, dt: f64, drag: f64) {
    let physics = PhysicsParams {
        drag,
        ..PhysicsParams::default()
    };
    let (_, after, _) = Kinematics::step(position, velocity, Vector3::zeros(), dt, &physics);
    let (dp, dv) = jacobian(velocity, after, dt, &physics);
    let (dp_numeric, dv_numeric) = numeric(velocity, position, dt, &physics);
    assert!(
        (dp - dp_numeric).norm() < 1e-6,
        "dp/dv\n해석:{dp}수치:{dp_numeric}"
    );
    assert!(
        (dv - dv_numeric).norm() < 1e-6,
        "dv/dv\n해석:{dv}수치:{dv_numeric}"
    );
}

/// 지금 `PhysicsParams::default().drag` 는 0 이라 항력 항이 사라진다. 그래도 맞아야 한다.
#[test]
fn the_flight_jacobian_matches_the_integrator_without_drag() {
    check(
        Vector3::new(0.7, 2.0, table::SURFACE_Z + 0.5),
        Vector3::new(0.5, -6.0, 1.5),
        0.013,
        0.0,
    );
}

/// 항력이 실리면 등속 근사와 갈라진다. 아직 실측 전이라 기본값이 0 이지만
/// (`PhysicsIdentify` 가 채울 자리), 채워지는 날 필터가 조용히 틀리면 안 된다.
#[test]
fn the_flight_jacobian_matches_the_integrator_with_drag() {
    let (position, velocity, dt, drag) = (
        Vector3::new(0.7, 2.0, table::SURFACE_Z + 0.5),
        Vector3::new(0.5, -6.0, 1.5),
        0.013,
        0.01,
    );
    check(position, velocity, dt, drag);
    // 항력은 속도의 2차라 빠를수록 야코비안이 커진다 — 느린 공에서만 맞으면 의미가 없다.
    check(position, Vector3::new(-1.0, -12.0, 3.0), dt, drag);

    // 그리고 그 값이 실제로 단위행렬과 다른지 — 0 을 맞히는 테스트는 테스트가 아니다.
    let physics = PhysicsParams {
        drag,
        ..PhysicsParams::default()
    };
    let (_, after, _) = Kinematics::step(position, velocity, Vector3::zeros(), dt, &physics);
    let (_, dv) = jacobian(velocity, after, dt, &physics);
    assert!(
        (dv - Matrix3::identity()).norm() > 1e-3,
        "항력 항이 안 실렸다: {dv}"
    );
}

/// 바운스는 속도의 불연속 사상이라 비행 야코비안과 완전히 다른 것이 나와야 한다.
#[test]
fn the_bounce_jacobian_matches_the_integrator() {
    let physics = PhysicsParams::default();
    let dt = 0.013;
    // 이 한 걸음 안에서 테이블을 친다.
    let position = Vector3::new(
        0.7,
        2.0,
        table::SURFACE_Z + crate::constants::ball::RADIUS + 0.02,
    );
    let velocity = Vector3::new(0.3, -5.0, -3.0);
    let (_, after, _) = Kinematics::step(position, velocity, Vector3::zeros(), dt, &physics);
    assert!(after.z > 0.0, "이 케이스가 실제로 튀어야 한다");

    let (_, dv) = jacobian(velocity, after, dt, &physics);
    let (_, dv_numeric) = numeric(velocity, position, dt, &physics);
    // 바운스 야코비안은 걸음 **시작** 속도에서 잰다 (본문 주석 참고). 중력이 그 한 걸음
    // 동안 v_z 를 바꾼 만큼 어긋나므로 비행보다 느슨하게 본다. dt 에 대해 1차다.
    assert!(
        (dv - dv_numeric).norm() < 0.05,
        "바운스 dv/dv\n해석:{dv}수치:{dv_numeric}"
    );
    // 반발계수가 z 를 뒤집는다 — 등속 근사로는 절대 안 나오는 부호다.
    assert!(dv[(2, 2)] < 0.0, "dvz/dvz={} 여야 음수", dv[(2, 2)]);
}
