//! 카메라 배치를 실제로 옮기기 전에 합성 데이터로 먼저 검증한다.
//!
//! 이 세션에서 확인한 것: 지금 리그(두 캠 다 테이블 +X 옆면에서 비스듬히 봄)는 X축
//! 관측 민감도가 낮아서(광축과 가까운 방향) v0.x 추정이 유독 흔들린다(fly_48).
//! "한 캠은 로봇 뒤에서 -y→+y로, 한 캠은 옆에서 +x→-x로" 직교 배치를 제안받았는데,
//! 실제로 카메라를 옮기려면 캘리브까지 다시 해야 하는 물리적 작업이다 — 그 전에
//! 시뮬레이션으로 "정말 나아지나, X만 좋아지고 Y·Z가 나빠지진 않나"를 먼저 본다.
//!
//! ```bash
//! cargo test --release --test diag_camera_geometry -- --ignored --nocapture
//! ```

use std::time::Duration;

use nalgebra::Vector3;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use pingpong_bot::camera::{self, Calibration, Params};
use pingpong_bot::constants::{ball, table};
use pingpong_bot::defaults::PhysicsParams;
use pingpong_bot::defaults::vision::fit::{DRAG, FRICTION, RESTITUTION};
use pingpong_bot::physics::Kinematics;
use pingpong_bot::vision::{Candidate, Fit, State, Trigger};
use pingpong_bot::Point3;

const FPS: f64 = 120.0;
const FRAMES: usize = 20;
const PIXEL_SIGMA: f64 = 1.5;
const WIDTH: u32 = 1280;
const HEIGHT: u32 = 800;
const FOV_Y_DEG: f64 = 47.3;

fn table_center() -> Vector3<f64> {
    return Vector3::new(table::WIDTH_X * 0.5, table::LENGTH_Y * 0.5, table::SURFACE_Z);
}

fn cam(id: u8, eye: Vector3<f64>) -> Params {
    return Params::look_at(
        camera::Id(id),
        None,
        eye,
        table_center(),
        Vector3::new(0.0, 0.0, 1.0),
        WIDTH,
        HEIGHT,
        FOV_Y_DEG.to_radians(),
    );
}

/// 지금 리그와 같은 배치 — 두 캠 다 +X 옆면에서 비스듬히 내려본다
/// (`data/calibration.json` 실측 위치 근사, `floor_edge.rs` 테스트의 `near_end_cam`·
/// `far_end_cam`과 같은 자리).
fn current_like_rig() -> Calibration {
    return Calibration {
        cameras: vec![
            cam(0, Vector3::new(2.87, 0.10, 2.08)),
            cam(1, Vector3::new(2.94, 2.97, 2.07)),
        ],
    };
}

/// 제안받은 배치 — 한 캠은 로봇 뒤에서 -y→+y, 한 캠은 옆에서 +x→-x.
fn orthogonal_rig() -> Calibration {
    return Calibration {
        cameras: vec![
            cam(0, Vector3::new(table::WIDTH_X * 0.5, -1.0, 1.2)),
            cam(1, Vector3::new(table::WIDTH_X + 2.2, table::LENGTH_Y * 0.5, 1.5)),
        ],
    };
}

/// 다음 후보 — Y 베이스라인(지금 리그의 숨은 강점, 근단·원단으로 갈라놓은 것)은
/// 그대로 넓게 두고, X만 넓힌다. 지금은 두 캠이 +X 옆면 같은 쪽에 있어서 X
/// 베이스라인이 얕다 — 한 캠을 반대쪽(-X)으로 보내 X도 Y만큼 넓혀 본다.
fn wide_xy_rig() -> Calibration {
    return Calibration {
        cameras: vec![
            cam(0, Vector3::new(2.87, 0.10, 2.08)),
            cam(1, Vector3::new(-1.4, table::LENGTH_Y - 0.10, 2.07)),
        ],
    };
}

fn never() -> Box<dyn Trigger> {
    struct Never;
    impl Trigger for Never {
        fn name(&self) -> &'static str {
            return "never";
        }
        fn ready(&self, _measured: &[State]) -> bool {
            return false;
        }
    }
    return Box::new(Never);
}

/// Box-Muller — 새 의존성 없이 정규분포 잡음.
fn gaussian(rng: &mut StdRng, sigma: f64) -> f64 {
    let u1: f64 = rng.gen_range(1e-9..1.0);
    let u2: f64 = rng.gen_range(0.0..1.0);
    return sigma * (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos();
}

/// `Fit::new`가 내부적으로 쓰는 것과 **정확히 같은** 물리로 굴린다 — 다르면 잡음이
/// 0이어도 모델 불일치 자체가 편향을 만들어서(실측: 이걸 빠뜨렸다가 drag=0.151을
/// 안 넣었더니 잡음 0에서도 v0.y가 6% 어긋났다) 카메라 기하 효과를 가린다. 바운스는
/// 프레임 수·속도를 짧게 잡아 창 안에서 안 일어나게 한다(회전 없음, e·μ는 안 씀).
fn fly(p0: Point3, v0: Vector3<f64>, t: f64) -> Point3 {
    let physics = PhysicsParams {
        restitution: RESTITUTION,
        friction: FRICTION,
        drag: DRAG,
        ..PhysicsParams::default()
    };
    let (mut pos, mut vel, mut omega) = (p0.coords, v0, Vector3::zeros());
    let step: f64 = 1.0 / 1000.0;
    let mut elapsed = 0.0;
    while elapsed < t {
        let dt = step.min(t - elapsed);
        let (np, nv, nw) = Kinematics::step(pos, vel, omega, dt, &physics);
        pos = np;
        vel = nv;
        omega = nw;
        elapsed += dt;
    }
    return Point3::from(pos);
}

/// 잡음 낀 픽셀을 먹여서 초기 속도를 추정한다. `Fit::measured()`의 첫 표본 속도가
/// 곧 풀린 v0다(적분 0초 지점).
fn solve_v0(
    calibration: &Calibration,
    p0: Point3,
    v0: Vector3<f64>,
    rng: &mut StdRng,
) -> Option<Vector3<f64>> {
    let mut fit = Fit::new(calibration, never());
    for frame in 0..FRAMES {
        let t = frame as f64 / FPS;
        let pos = fly(p0, v0, t);
        for params in &calibration.cameras {
            let Some(mut pixel) = params.project_world_unclipped(pos) else {
                continue;
            };
            pixel.x += gaussian(rng, PIXEL_SIGMA);
            pixel.y += gaussian(rng, PIXEL_SIGMA);
            fit.observe(
                params.camera_id,
                Some(Candidate {
                    pixel,
                    radius_px: ball::RADIUS * 4000.0,
                    circularity: 0.9,
                }),
                Duration::from_secs_f64(t),
            );
        }
    }
    return fit.measured().0.first().map(|s| s.velocity);
}

#[test]
#[ignore = "합성 시뮬레이션: cargo test --release --test diag_camera_geometry -- --ignored --nocapture"]
fn orthogonal_cameras_estimate_v0_better_than_same_side() {
    // 사람 서브 몇 가지를 흉내(속도·시작 x 다양하게) — 한 궤적에만 우연히 좋은 게
    // 아님을 보려고 여러 개를 쓴다. 시작 x를 `SHOOTER_X`(0.820, 소프트 사전값) 근처가
    // 아니라 일부러 멀리 잡는다 — 가까우면 사전값이 카메라 기하와 무관하게 p0.x를
    // 대신 잡아 줘서, 정작 보려는 "기하만으로 얼마나 잘 보이나"가 사전값에 가려진다.
    let shots: Vec<(Point3, Vector3<f64>)> = vec![
        (
            Point3::new(0.25, 2.3, table::SURFACE_Z + 0.4),
            Vector3::new(0.1, -5.0, 1.5),
        ),
        (
            Point3::new(1.35, 2.3, table::SURFACE_Z + 0.4),
            Vector3::new(0.6, -6.0, 2.0),
        ),
        (
            Point3::new(0.3, 2.3, table::SURFACE_Z + 0.4),
            Vector3::new(-0.4, -4.0, 1.0),
        ),
        (
            Point3::new(1.4, 2.3, table::SURFACE_Z + 0.4),
            Vector3::new(0.0, -7.0, 2.5),
        ),
    ];
    const TRIALS: u64 = 300;

    for (name, calibration) in [
        ("현재(같은 쪽)", current_like_rig()),
        ("직교(뒤+옆)", orthogonal_rig()),
        ("Y넓게+X도넓게", wide_xy_rig()),
    ] {
        let mut err: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        let mut failed = 0u32;
        for (shot_index, &(p0, v0)) in shots.iter().enumerate() {
            for trial in 0..TRIALS {
                let mut rng = StdRng::seed_from_u64(shot_index as u64 * 1000 + trial);
                match solve_v0(&calibration, p0, v0, &mut rng) {
                    Some(solved) => {
                        for axis in 0..3 {
                            err[axis].push((solved[axis] - v0[axis]).abs());
                        }
                    }
                    None => failed += 1,
                }
            }
        }
        let rmse = |values: &[f64]| -> f64 {
            return (values.iter().map(|v| v * v).sum::<f64>() / values.len().max(1) as f64)
                .sqrt();
        };
        println!(
            "{name:<12} v0 RMSE  x={:.3}  y={:.3}  z={:.3} m/s   (실패 {failed}/{})",
            rmse(&err[0]),
            rmse(&err[1]),
            rmse(&err[2]),
            shots.len() as u64 * TRIALS,
        );
    }
}

#[test]
#[ignore = "합성 시뮬레이션 자체 점검: 잡음 0일 때 v0가 정확히 복원되는지"]
fn zero_noise_recovers_v0_exactly() {
    let p0 = Point3::new(0.25, 2.3, table::SURFACE_Z + 0.4);
    let v0 = Vector3::new(0.1, -5.0, 1.5);
    let mut rng = StdRng::seed_from_u64(0);
    for (name, calibration) in [
        ("현재(같은 쪽)", current_like_rig()),
        ("직교(뒤+옆)", orthogonal_rig()),
    ] {
        // PIXEL_SIGMA를 0으로 흉내내려고 gaussian 호출을 건너뛰는 대신, solve_v0를
        // 그대로 쓰되 시드 고정 상태에서 결과만 본다(완전 무잡음 버전은 아래서 직접).
        let mut fit = Fit::new(&calibration, never());
        for frame in 0..FRAMES {
            let t = frame as f64 / FPS;
            let pos = fly(p0, v0, t);
            for params in &calibration.cameras {
                let Some(pixel) = params.project_world_unclipped(pos) else {
                    continue;
                };
                fit.observe(
                    params.camera_id,
                    Some(Candidate {
                        pixel,
                        radius_px: ball::RADIUS * 4000.0,
                        circularity: 0.9,
                    }),
                    Duration::from_secs_f64(t),
                );
            }
        }
        let solved = fit.measured().0.first().map(|s| s.velocity);
        println!("{name}: 무잡음 v0={solved:?} truth={v0:?}");
        let _ = &mut rng; // 시드는 여기선 안 쓴다(무잡음이라 결정적).
    }
}
