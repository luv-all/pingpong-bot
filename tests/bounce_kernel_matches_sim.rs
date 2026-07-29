//! 테이블 바운스 SSOT(`estimator::bounce`)가 Rapier 실제 접촉과 맞는지.
//!
//! 커널이 어긋나면 `predict_hit_plane`이 임팩트 위치를 크게 빗맞히고,
//! 플래너가 라켓을 엉뚱한 높이에 갖다 놓는다 (2026-07-27 실측: 105 mm).

use nalgebra::Vector3;

use pingpong_bot::HitPlane;
use pingpong_bot::constants::{ball, table};
use pingpong_bot::defaults;
use pingpong_bot::estimator::BallKinematics;
use pingpong_bot::sim::SimWorld;
use pingpong_bot::sim::eval_protocol::{EvalMode, EvalProtocol};

fn v3(v: rapier3d::prelude::Vector) -> Vector3<f64> {
    return Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z));
}

fn ball_touches_racket(world: &SimWorld) -> bool {
    let Some(ball) = world
        .collider_set
        .iter()
        .find_map(|(h, c)| (c.parent() == Some(world.ball_handle)).then_some(h))
    else {
        return false;
    };
    let Some(racket) = world
        .collider_set
        .iter()
        .find_map(|(h, c)| (c.parent() == Some(world.racket_handle)).then_some(h))
    else {
        return false;
    };
    return world
        .narrow_phase
        .contact_pair(ball, racket)
        .is_some_and(|p| p.has_any_active_contact());
}

struct BounceSample {
    v_before: Vector3<f64>,
    omega_before: Vector3<f64>,
    v_after: Vector3<f64>,
    omega_after: Vector3<f64>,
}

/// 테이블 위로 공을 던져 첫 바운스 전후 상태를 Rapier에서 관측한다.
fn observe_bounce(velocity: Vector3<f64>, omega: Vector3<f64>) -> BounceSample {
    const DT: f64 = 1.0 / 1000.0;
    let robot = defaults::robot().expect("robot");
    let mut world = SimWorld::with_physics(robot, defaults::PhysicsParams::default());
    // 자동 스윙 없이 순수 탄도만 본다.
    world.set_use_ground_truth(false);
    // 라켓·네트에서 먼 미드코트 상공에서 시작.
    let start = [
        (table::WIDTH_X * 0.5) as f32,
        1.05_f32,
        (table::SURFACE_Z + 0.25) as f32,
    ];
    world.launch_ball_at(
        start,
        [velocity.x as f32, velocity.y as f32, velocity.z as f32],
        [omega.x as f32, omega.y as f32, omega.z as f32],
    );

    let mut prev_v = velocity;
    let mut prev_w = omega;
    let mut before: Option<(Vector3<f64>, Vector3<f64>)> = None;
    for _ in 0..2_000 {
        world.step(DT, None);
        let v = v3(world.ball_velocity());
        let w = v3(world.ball_angular_velocity());
        // vz 부호 반전 = 바운스 발생.
        if before.is_none() && prev_v.z < 0.0 && v.z > 0.0 {
            before = Some((prev_v, prev_w));
        } else if let Some((v_before, omega_before)) = before {
            // 접촉이 끝나고 안정된 다음 스텝의 값을 쓴다.
            return BounceSample {
                v_before,
                omega_before,
                v_after: v,
                omega_after: w,
            };
        }
        prev_v = v;
        prev_w = w;
    }
    panic!("바운스를 관측하지 못함");
}

#[test]
fn table_bounce_kernel_matches_rapier_contact() {
    let physics = defaults::PhysicsParams::default();
    // (입사속도, 입사 스핀) — eval 슈터가 실제로 만드는 범위.
    let cases = [
        (Vector3::new(0.0, -6.5, -2.5), Vector3::zeros()),
        (Vector3::new(0.0, -5.0, -3.0), Vector3::zeros()),
        (Vector3::new(0.3, -7.0, -2.0), Vector3::new(40.0, 0.0, 0.0)),
        (Vector3::new(0.0, -6.0, -2.5), Vector3::new(-40.0, 0.0, 0.0)),
    ];

    let mut failures = Vec::new();
    for (velocity, omega) in cases {
        let sample = observe_bounce(velocity, omega);
        let (kernel_v, kernel_w) =
            BallKinematics::bounce_on_table(sample.v_before, sample.omega_before, &physics);
        let error = (kernel_v - sample.v_after).norm();
        println!(
            "v_in=[{:.2} {:.2} {:.2}] w_in=[{:.1} {:.1} {:.1}]\n  \
             rapier v_out=[{:.3} {:.3} {:.3}] w_out=[{:.1} {:.1} {:.1}]\n  \
             kernel v_out=[{:.3} {:.3} {:.3}] w_out=[{:.1} {:.1} {:.1}]  err={:.3} m/s",
            sample.v_before.x,
            sample.v_before.y,
            sample.v_before.z,
            sample.omega_before.x,
            sample.omega_before.y,
            sample.omega_before.z,
            sample.v_after.x,
            sample.v_after.y,
            sample.v_after.z,
            sample.omega_after.x,
            sample.omega_after.y,
            sample.omega_after.z,
            kernel_v.x,
            kernel_v.y,
            kernel_v.z,
            kernel_w.x,
            kernel_w.y,
            kernel_w.z,
            error,
        );
        // 접선과 법선을 나눠 본다. 접선은 커널 형태(Coulomb)가 지배하고,
        // 법선은 Rapier 솔버가 e=0.88을 완전히 전달하지 못하는 잔차가 있다
        // (실측 실효 e 0.79~0.87). 접선이 어긋나면 임팩트 **시각·y**가,
        // 법선이 어긋나면 임팩트 **높이**가 밀린다.
        let tangential_error = ((kernel_v.x - sample.v_after.x).powi(2)
            + (kernel_v.y - sample.v_after.y).powi(2))
        .sqrt();
        let normal_error = (kernel_v.z - sample.v_after.z).abs();
        if tangential_error > 0.15 {
            failures.push(format!(
                "접선 err={tangential_error:.3} m/s (kernel vy={:.2} vs rapier vy={:.2})",
                kernel_v.y, sample.v_after.y
            ));
        }
        if normal_error > 0.30 {
            failures.push(format!(
                "법선 err={normal_error:.3} m/s (kernel vz={:.2} vs rapier vz={:.2})",
                kernel_v.z, sample.v_after.z
            ));
        }
    }
    assert!(
        failures.is_empty(),
        "바운스 커널이 Rapier와 어긋남:\n{}",
        failures.join("\n")
    );
}

/// 바운스를 사이에 낀 hit-plane 예측이 실제 공 위치와 맞는지.
///
/// 이게 최종 목적이다 — 어긋나면 플래너가 라켓을 엉뚱한 높이에 갖다 놓아
/// 공이 라켓 아래로 지나가거나 모서리에 스친다.
#[test]
fn hit_plane_prediction_matches_simulated_ball() {
    const DT: f64 = 1.0 / 1000.0;
    let physics = defaults::PhysicsParams::default();
    let plane = HitPlane {
        y: table::DEFAULT_HIT_PLANE_Y,
    };

    let mut worst = 0.0_f64;
    let mut net_clipped = 0;
    let mut report = Vec::new();
    for (index, (zone, index_in_zone)) in EvalProtocol::shot_schedule(EvalMode::Block)
        .into_iter()
        .enumerate()
        .filter(|(i, _)| i % 4 == 0)
    {
        let settings = EvalProtocol::settings_for_zone_shot(
            &pingpong_bot::sim::EvalLaunchParams::default(),
            zone,
            index_in_zone,
        );
        let robot = defaults::robot().expect("robot");
        let mut world = SimWorld::with_physics(robot, physics);
        // 팔은 움직이지 않게 — 순수 탄도 예측만 본다.
        world.set_use_ground_truth(false);
        world.shoot_ball(&settings);

        // 로봇 쪽 코트로 넘어온 직후(= 플래너가 commit하는 시점) 예측한다.
        let mut prediction = None;
        let mut steps_at_prediction = 0;
        let mut prediction_start = (Vector3::zeros(), Vector3::zeros(), Vector3::zeros());
        for step in 0..4_000 {
            world.step(DT, None);
            if prediction.is_some() {
                continue;
            }
            let position = v3(world.ball_position());
            if position.y > table::LENGTH_Y * 0.55 {
                continue;
            }
            let velocity = v3(world.ball_velocity());
            let omega = v3(world.ball_angular_velocity());
            prediction = BallKinematics::predict_to(position, velocity, omega, plane, &physics);
            if prediction.is_some() {
                prediction_start = (position, velocity, omega);
            }
            steps_at_prediction = step;
        }
        let Some(prediction) = prediction else {
            continue;
        };

        // 예측 시점부터 다시 굴려 실제로 평면을 지날 때의 높이를 잰다.
        let robot = defaults::robot().expect("robot");
        let mut world = SimWorld::with_physics(robot, physics);
        world.set_use_ground_truth(false);
        world.shoot_ball(&settings);
        let mut actual_z = None;
        let mut previous = v3(world.ball_position());
        let mut previous_vz = v3(world.ball_velocity()).z;
        let mut bounces = 0;
        let mut touched_racket = false;
        let mut actual_bounce_y = f64::NAN;
        let mut hit_net = false;
        for step in 0..4_000 {
            world.step(DT, None);
            let position = v3(world.ball_position());
            let vz = v3(world.ball_velocity()).z;
            if previous_vz < 0.0 && vz > 0.0 {
                bounces += 1;
                if bounces == 1 {
                    actual_bounce_y = position.y;
                }
            }
            if ball_touches_racket(&world) {
                touched_racket = true;
            }
            if world.ball_intersects_net() {
                hit_net = true;
            }
            if step > steps_at_prediction && previous.y > plane.y && position.y <= plane.y {
                let frac = (plane.y - previous.y) / (position.y - previous.y);
                actual_z = Some(previous.z + (position.z - previous.z) * frac);
                break;
            }
            previous = position;
            previous_vz = vz;
        }
        // 같은 시작 상태로 커널 적분만 돌려 1차 바운스 y를 예측한다.
        let mut kernel_pos = prediction_start.0;
        let mut kernel_vel = prediction_start.1;
        let mut kernel_omega = prediction_start.2;
        let mut predicted_bounce_y = f64::NAN;
        for _ in 0..2_000 {
            let previous_vz = kernel_vel.z;
            let (p, v, w) =
                BallKinematics::step(kernel_pos, kernel_vel, kernel_omega, 1.0 / 1000.0, &physics);
            kernel_pos = p;
            kernel_vel = v;
            kernel_omega = w;
            if previous_vz < 0.0 && kernel_vel.z > 0.0 {
                predicted_bounce_y = kernel_pos.y;
                break;
            }
        }
        let context = format!(
            "bounces={bounces} racket={touched_racket} bounce_y pred={predicted_bounce_y:.3} act={actual_bounce_y:.3}"
        );
        let Some(actual_z) = actual_z else {
            continue;
        };
        if hit_net {
            // 입사 공이 네트에 맞은 샷 — 예측 정확도와 무관한 슈터 문제.
            net_clipped += 1;
            continue;
        }
        let error = (prediction.impact_position.coords.z - actual_z).abs();
        worst = worst.max(error);
        report.push(format!(
            "  shot {:>2} {:>6}: predicted z={:.4} actual z={:.4} err={:.1} mm  {}",
            index + 1,
            zone.label(),
            prediction.impact_position.coords.z,
            actual_z,
            error * 1000.0,
            context
        ));
    }
    println!("{}", report.join("\n"));
    println!(
        "worst hit-plane z error = {:.1} mm  (네트에 걸린 입사 샷 {net_clipped}개 제외)",
        worst * 1000.0
    );
    assert!(
        !report.is_empty(),
        "예측을 하나도 못 얻음 — 테스트 설정 문제"
    );
    // 라켓 반높이가 80 mm. 남은 오차는 Rapier가 e=0.88을 다 전달하지
    // 못하는 법선 잔차(실효 0.79~0.87)에서 온다 — 접선/바운스 지점 자체는
    // 1 mm 이내로 맞는다. 이 상한을 넘으면 라켓 모서리 타격이 시작된다.
    assert!(
        worst < 0.050,
        "hit-plane 높이 예측 오차 {:.1} mm — 라켓이 공을 빗맞힌다",
        worst * 1000.0
    );
}

/// 커널이 스핀 변화를 전혀 모델링하지 않는 것이 실제와 맞는지.
#[test]
fn table_bounce_kernel_models_spin_change() {
    let sample = observe_bounce(Vector3::new(0.0, -6.5, -2.5), Vector3::zeros());
    let spin_change = (sample.omega_after - sample.omega_before).norm();
    let (_, kernel_w) = BallKinematics::bounce_on_table(
        sample.v_before,
        sample.omega_before,
        &defaults::PhysicsParams::default(),
    );
    let kernel_change = (kernel_w - sample.omega_before).norm();
    println!(
        "rapier |Δω|={spin_change:.1} rad/s, kernel |Δω|={kernel_change:.1} rad/s, R={:.3}",
        ball::RADIUS
    );
    assert!(
        (kernel_change - spin_change).abs() < 0.25 * spin_change.max(1.0),
        "커널 Δω={kernel_change:.1}가 Rapier Δω={spin_change:.1}와 어긋남"
    );
}
