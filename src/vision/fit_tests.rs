use super::*;
use crate::camera::Params;
use crate::constants::ball;

fn rig() -> Calibration {
    return Calibration::sim(3);
}

/// 진짜 탄도에서 두 카메라 픽셀을 만들어 낸다 — 잡음 없음. 입사 스핀은 `ASSUMED_SPIN`
/// (파일 관례 — 적합 내부가 그걸 사전값으로 쓰므로 맞춰 둔다). 다른 스핀으로 합성하려면
/// [`sightings_with_spin`].
fn sightings(
    calibration: &Calibration,
    truth: &Ballistic,
    frames: usize,
    fps: f64,
) -> Vec<(camera::Id, camera::Pixel, Duration)> {
    return sightings_with_spin(calibration, truth, ASSUMED_SPIN, frames, fps);
}

/// [`sightings`]와 같지만 입사 스핀을 밖에서 받는다 — `refine_spin`처럼 스핀 자체를
/// 검증할 땐 사전값(0)이 아닌 진짜 스핀으로 합성해야 회복이 뭘 확인하는지가 뜻이 있다.
fn sightings_with_spin(
    calibration: &Calibration,
    truth: &Ballistic,
    spin0: Vector3,
    frames: usize,
    fps: f64,
) -> Vec<(camera::Id, camera::Pixel, Duration)> {
    // 적합과 **같은 물리**로 합성해야 한다. 다른 모델로 만든 데이터를 못 맞힌다고
    // 적합을 탓하면 안 된다.
    let physics = PhysicsParams {
        restitution: RESTITUTION,
        friction: FRICTION,
        drag: DRAG,
        ..PhysicsParams::default()
    };
    let step = INTEGRATE_DT.as_secs_f64();
    let (mut position, mut velocity, mut spin) = (truth.position.coords, truth.velocity, spin0);
    let mut elapsed = 0.0;
    let mut out = Vec::new();
    for frame in 0..frames {
        let target = frame as f64 / fps;
        while elapsed < target {
            let (p, v, w) = Kinematics::step(position, velocity, spin, step, &physics);
            position = p;
            velocity = v;
            spin = w;
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

/// 공이 화면을 떠나면 트랙도 죽어야 한다. 안 죽으면 제어가 지나간 공의 예측을 계속 받는다.
#[test]
fn silence_after_the_ball_leaves_kills_the_track() {
    let calibration = rig();
    let truth = truth();
    let mut fit = Fit::new(&calibration, Box::new(never()));
    let samples = sightings(&calibration, &truth, 10, 120.0);
    feed(&mut fit, &samples);
    assert!(fit.has_track());

    // 검출 없는 프레임만 계속 들어온다 — 공이 나갔다.
    let last = samples.last().expect("표본").2;
    let mut t = last;
    for _ in 0..80 {
        t += Duration::from_millis(13);
        fit.observe(camera::Id(0), None, t);
    }
    assert!(!fit.has_track(), "관측이 끊기면 트랙을 버려야 한다");
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

/// 가파른 입사(`bounce.rs::steep_impact_reaches_rolling_and_caps_tangential_loss`와 같은
/// 케이스)로 구름 전이가 일어나는 바운스를 겪게 하면, 그 뒤로는 `solved_spin`이 채워져야
/// 한다. `sightings()`가 `ASSUMED_SPIN`으로 합성하므로(파일 관례) 참값도 그 스핀으로
/// 바운스한 결과와 같아야 정확히 맞는다 — `table_bounce(v_in, ASSUMED_SPIN, ..)`를 직접
/// 굴려 기대값을 얻는다.
///
/// `Fit::observe`(즉 `feed`)를 안 태운다 — 바운스 뒤에도 앞단 6-미지수 적합이
/// `self.assumed_spin()`(사전값 0)으로 계속 재투영하니, 실제 스핀 60대가 실린 합성
/// 궤적과는 바운스 직후일수록 더 어긋난다. 그 어긋난 관측이 하나씩 아웃라이어로 빠지면
/// `solve_spin_from_bounce`가 보는 창이 바운스에서 먼 표본들로 채워져(raw_len 60개 중
/// 14개만 살아남는 걸 실측) v_out 창 회귀가 바운스 직후가 아니라 Magnus로 이미 휜 한참
/// 뒤 구간을 재 버린다 — `a_sliding_bounce_falls_back_to_the_pixel_refit`과 같은 이유로
/// 같은 우회를 쓴다.
#[test]
fn a_rolling_bounce_gets_a_solved_spin_that_matches_truth() {
    let calibration = rig();
    let bouncing = Ballistic {
        t0: Duration::ZERO,
        position: Point3::new(0.8, 2.0, table::SURFACE_Z + ball::RADIUS + 0.6),
        velocity: Vector3::new(0.0, -2.0, -6.0),
    };
    let mut fit = Fit::new(&calibration, Box::new(never()));
    fit.solution = Some(bouncing);
    fit.sightings = sightings(&calibration, &bouncing, 60, 120.0)
        .iter()
        .map(|(id, pixel, t)| Sighting {
            camera: fit
                .cameras
                .iter()
                .position(|p| p.camera_id == *id)
                .expect("합성 픽셀의 캠이 rig에 있어야 한다"),
            pixel: *pixel,
            t: *t,
        })
        .collect();

    let solved = fit
        .solve_spin_from_bounce()
        .expect("가파른 바운스는 구름으로 잡혀야 한다");
    // 바운스 직전 v_in을 다시 굴려 참 ω_out을 얻는다 — 물리 커널 SSOT 재사용.
    let physics = PhysicsParams {
        restitution: RESTITUTION,
        friction: FRICTION,
        drag: DRAG,
        ..PhysicsParams::default()
    };
    let mut position = bouncing.position.coords;
    let mut velocity = bouncing.velocity;
    let mut spin = ASSUMED_SPIN;
    let mut prev_vz = velocity.z;
    let mut truth_spin = None;
    let step = INTEGRATE_DT.as_secs_f64();
    for _ in 0..2000 {
        let (p, v, w) = Kinematics::step(position, velocity, spin, step, &physics);
        if prev_vz < 0.0 && v.z >= 0.0 {
            truth_spin = Some(w);
            break;
        }
        prev_vz = v.z;
        position = p;
        velocity = v;
        spin = w;
    }
    let truth_spin = truth_spin.expect("합성 낙하는 실제로 바운스해야 한다");
    // 정확히 0은 아니다 — 창 회귀·가우스-뉴턴 적합 둘 다 근사라 몇 % 오차가 낀다.
    // 노이즈 없는 합성 데이터에서도 15%(절대 10 rad/s 아래로는 안 깎는다 — truth가
    // ASSUMED_SPIN 값에 좌우돼 작을 때 상대 문턱만 쓰면 같은 절대오차에도 잘못 실패한다)
    // 면 넉넉하고, 그보다 벌어지면 회귀다.
    let tolerance = (0.15 * truth_spin.norm()).max(10.0);
    assert!(
        (solved - truth_spin).norm() < tolerance,
        "solved={solved:?} truth={truth_spin:?} tolerance={tolerance:.1}"
    );
}

/// 얕은 입사(`bounce.rs::shallow_impact_slips_and_keeps_most_tangential_speed`와 같은
/// 케이스)는 슬립으로 끝나 닫힌식이 못 푼다 — 바운스 뒤 비행의 Magnus로 `refine_spin`이
/// 대신 잡아야 한다. `sightings_with_spin`으로 스핀을 실어야 뜻이 있다 — 사전값(0)으로
/// 합성하면 회복이 그냥 사전값을 되돌려주는 것과 구분이 안 된다.
///
/// `Fit::observe`(즉 `feed`)를 안 태운다 — 진짜 스핀을 실은 합성 데이터를 그 경로로
/// 흘리면, `p0,v0`를 찾는 6-미지수 적합 **자체**가 스핀을 모른 채(사전값 0) 재투영해서
/// 바운스 프레임 앞뒤로 잔차가 튀고, 그게 3연속이면 아웃라이어로 트랙째 버려진다 — 이건
/// `refine_spin`이 아니라 앞단 적합의 별개 한계고, 여기서 검증하려는 것도 아니다. 그래서
/// `p0,v0`는 참값을 바로 주고 `solve_spin_from_bounce`만 단독으로 부른다.
#[test]
fn a_sliding_bounce_falls_back_to_the_pixel_refit() {
    let calibration = rig();
    let truth_spin = Vector3::new(35.0, -12.0, 0.0);
    let sliding = Ballistic {
        t0: Duration::ZERO,
        position: Point3::new(0.8, 2.0, table::SURFACE_Z + ball::RADIUS + 0.6),
        velocity: Vector3::new(0.0, -8.0, -2.0),
    };
    let mut fit = Fit::new(&calibration, Box::new(never()));
    fit.solution = Some(sliding);
    fit.sightings = sightings_with_spin(&calibration, &sliding, truth_spin, 80, 120.0)
        .iter()
        .map(|(id, pixel, t)| Sighting {
            camera: fit
                .cameras
                .iter()
                .position(|p| p.camera_id == *id)
                .expect("합성 픽셀의 캠이 rig에 있어야 한다"),
            pixel: *pixel,
            t: *t,
        })
        .collect();

    let solved = fit
        .solve_spin_from_bounce()
        .expect("슬립 바운스도 픽셀 재적합으로 풀려야 한다");
    // 얕은 입사가 정말 슬립인지 확인 — 이 테스트가 재적합 경로를 태우는 게 맞는지 보장.
    let physics = PhysicsParams {
        restitution: RESTITUTION,
        friction: FRICTION,
        drag: DRAG,
        ..PhysicsParams::default()
    };
    assert!(
        crate::physics::PhysicsIdentify::spin_after_bounce_if_rolling(
            sliding.velocity,
            Kinematics::bounce_on_table(sliding.velocity, truth_spin, &physics).0,
            &physics,
        )
        .is_none(),
        "이 테스트 케이스는 슬립이어야 한다 — 아니면 닫힌식 경로를 태운 것이라 뜻이 없다"
    );
    let tolerance = (0.15 * truth_spin.norm()).max(15.0);
    assert!(
        (solved - truth_spin).norm() < tolerance,
        "solved={solved:?} truth={truth_spin:?} tolerance={tolerance:.1}"
    );
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
