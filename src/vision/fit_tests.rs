use super::*;
use crate::camera::Params;

fn rig() -> Calibration {
    return Calibration::sim(3);
}

/// 진짜 탄도에서 두 카메라 픽셀을 만들어 낸다 — 잡음 없음.
fn sightings(
    calibration: &Calibration,
    truth: &Ballistic,
    frames: usize,
    fps: f64,
) -> Vec<(camera::Id, camera::Pixel, Duration)> {
    let physics = PhysicsParams::default();
    let step = INTEGRATE_DT.as_secs_f64();
    let (mut position, mut velocity) = (truth.position.coords, truth.velocity);
    let mut elapsed = 0.0;
    let mut out = Vec::new();
    for frame in 0..frames {
        let target = frame as f64 / fps;
        while elapsed < target {
            let (p, v, _) = Kinematics::step(position, velocity, Vector3::zeros(), step, &physics);
            position = p;
            velocity = v;
            elapsed += step;
        }
        for params in &calibration.cameras[..2] {
            if let Some(pixel) = params.project_world_unclipped(Point3::from(position)) {
                out.push((params.camera_id, pixel, Duration::from_secs_f64(target)));
            }
        }
    }
    return out;
}

fn feed(fit: &mut Fit, samples: &[(camera::Id, camera::Pixel, Duration)]) {
    for (id, pixel, t) in samples {
        fit.observe(
            *id,
            Some(Candidate {
                pixel: *pixel,
                radius_px: 5.0,
                circularity: 0.9,
            }),
            *t,
        );
    }
}

fn truth() -> Ballistic {
    return Ballistic {
        t0: Duration::ZERO,
        position: Point3::new(0.8, 2.4, table::SURFACE_Z + 0.35),
        velocity: Vector3::new(-0.3, -5.5, 1.2),
    };
}

/// 잡음이 없으면 초기 조건을 정확히 되찾아야 한다. 못 찾으면 적합이 틀린 것이다.
#[test]
fn a_noise_free_flight_recovers_its_initial_conditions() {
    let calibration = rig();
    let truth = truth();
    let mut fit = Fit::new(
        &calibration,
        Box::new(crate::vision::triggers::PlaneCrossing {
            y: table::LENGTH_Y * 0.5,
        }),
    );
    feed(&mut fit, &sightings(&calibration, &truth, 10, 120.0));

    let solution = fit.solution.expect("궤적이 서야 한다");
    assert!(
        (solution.position - truth.position).norm() < 0.01,
        "p0 {:?} != {:?}",
        solution.position,
        truth.position
    );
    assert!(
        (solution.velocity - truth.velocity).norm() < 0.05,
        "v0 {:?} != {:?}",
        solution.velocity,
        truth.velocity
    );
}

/// 한 프레임이 크게 튀어도 나머지가 잡아야 한다 — 재귀 필터가 못 하던 것이다.
#[test]
fn one_bad_sighting_does_not_move_the_fit() {
    let calibration = rig();
    let truth = truth();
    let clean = sightings(&calibration, &truth, 12, 120.0);

    let mut good = Fit::new(&calibration, Box::new(never()));
    feed(&mut good, &clean);
    let before = good.solution.expect("궤적");

    let mut poisoned = Fit::new(&calibration, Box::new(never()));
    let mut spiked = clean.clone();
    // 가운데 한 관측을 200 px 옆으로 옮긴다 — 완전히 다른 것을 잡은 셈.
    let middle = spiked.len() / 2;
    spiked[middle].1.x += 200.0;
    feed(&mut poisoned, &spiked);
    let after = poisoned.solution.expect("궤적");

    assert!(
        (after.velocity - before.velocity).norm() < 0.1,
        "튄 관측 하나에 v0 가 {:.2} m/s 움직였다",
        (after.velocity - before.velocity).norm()
    );
}

/// 한 대만 보이면 깊이가 안 잡힌다 — 그때는 궤적을 세우지 않는다.
#[test]
fn a_single_camera_never_starts_a_track() {
    let calibration = rig();
    let truth = truth();
    let mut fit = Fit::new(&calibration, Box::new(never()));
    let only_first: Vec<_> = sightings(&calibration, &truth, 12, 120.0)
        .into_iter()
        .filter(|(id, _, _)| *id == camera::Id(0))
        .collect();
    feed(&mut fit, &only_first);
    assert!(!fit.has_track());
}

/// 관측이 끊기면 다음 공이다.
#[test]
fn a_long_gap_starts_a_new_track() {
    let calibration = rig();
    let truth = truth();
    let mut fit = Fit::new(&calibration, Box::new(never()));
    feed(&mut fit, &sightings(&calibration, &truth, 10, 120.0));
    assert!(fit.has_track());
    let seq = fit.seq();

    let late: Vec<_> = sightings(&calibration, &truth, 10, 120.0)
        .into_iter()
        .map(|(id, pixel, t)| (id, pixel, t + STALE_GAP + Duration::from_millis(10)))
        .collect();
    feed(&mut fit, &late);
    assert!(fit.seq() > seq, "트랙이 갈렸어야 한다");
}

/// 아무 때도 안 걸리는 트리거 — 예측 적분을 빼고 적합만 보고 싶을 때.
fn never() -> impl Trigger {
    struct Never;
    impl Trigger for Never {
        fn name(&self) -> &'static str {
            return "never";
        }
        fn ready(&self, _measured: &[State]) -> bool {
            return false;
        }
    }
    return Never;
}

/// 캘리브 rmse 가 나쁜 카메라는 가중치가 낮아야 한다.
#[test]
fn a_badly_calibrated_camera_counts_for_less() {
    let mut params: Params = Calibration::sim(2).cameras[0].clone();
    params.reprojection_rmse_px = None;
    let clean = sigma_px(&params);
    params.reprojection_rmse_px = Some(4.15);
    let dirty = sigma_px(&params);
    assert!(dirty > clean * 2.0, "{dirty} vs {clean}");
}
