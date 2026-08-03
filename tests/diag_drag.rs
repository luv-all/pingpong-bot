//! 항력계수 `drag` 를 클립에서 실측한다.
//!
//! 새 장비도 새 촬영도 필요 없다. 이미 찍어 둔 클립의 생 삼각측량 궤적에 `drag` 를
//! **7번째 미지수**로 넣고 초기 조건과 같이 푼다. 클립마다 나온 값이 서로 맞으면 그게
//! 측정이고, 안 맞으면 모델이 뭔가를 빠뜨린 것이다 (스핀이 유력하다).
//!
//! 이론값과 견주는 게 정직성 검사다:
//!
//! ```text
//! k = ½ ρ C_d A / m = 0.5 × 1.2 × C_d × π(0.02)² / 0.0027
//! ```
//!
//! 구의 `C_d` 는 이 레이놀즈수대에서 0.4~0.5 라 `k ≈ 0.11~0.14 m⁻¹` 다. 5 m/s 면
//! 감속이 `k·v² ≈ 3 m/s²` — **중력과 같은 자릿수**다. 지금 기본값 0 은 그걸 통째로 뺀 것이다.
//!
//! ```bash
//! cargo test --release --test diag_drag -- --ignored --nocapture
//! ```

use std::path::Path;
use std::time::{Duration, Instant};

use pingpong_bot::camera::{self, Calibration, Frame, FrameSource, OpenCvCapture, Triangulate};
use pingpong_bot::constants::ball;
use pingpong_bot::constants::table;
use pingpong_bot::defaults::{self, PhysicsParams};
use pingpong_bot::estimator::Kinematics;
use pingpong_bot::vision::triggers::PlaneCrossing;
use pingpong_bot::vision::Vision;
use pingpong_bot::{Point3, Vector3};

const CLIPS: [&str; 9] = [
    "fly_01", "fly_02", "fly_03", "fly_04", "fly_05", "fly_06", "fly_07", "fly_08", "fly_09",
];
const DT: f64 = 0.001;
/// 공기 밀도 [kg/m³].
const AIR_DENSITY: f64 = 1.2;

/// `k = ½ ρ C_d A / m`.
fn theoretical(drag_coefficient: f64) -> f64 {
    let area = std::f64::consts::PI * ball::RADIUS * ball::RADIUS;
    return 0.5 * AIR_DENSITY * drag_coefficient * area / ball::MASS;
}

/// 미지수 7개: `p0`(3), `v0`(3), `drag`(1).
#[derive(Debug, Clone, Copy)]
struct Guess {
    position: Point3,
    velocity: Vector3,
    drag: f64,
}

impl Guess {
    fn nudged(mut self, axis: usize, step: f64) -> Self {
        match axis {
            0..=2 => self.position[axis] += step,
            3..=5 => self.velocity[axis - 3] += step,
            _ => self.drag += step,
        }
        return self;
    }

    fn apply(&mut self, axis: usize, delta: f64) {
        match axis {
            0..=2 => self.position[axis] -= delta,
            3..=5 => self.velocity[axis - 3] -= delta,
            // 항력은 음수가 될 수 없다. 물리적으로 없는 값을 해로 받으면 안 된다.
            _ => self.drag = (self.drag - delta).max(0.0),
        }
    }
}

fn walk(guess: &Guess, times: &[f64]) -> Vec<Point3> {
    let physics = PhysicsParams {
        drag: guess.drag,
        ..PhysicsParams::default()
    };
    let (mut p, mut v) = (guess.position.coords, guess.velocity);
    let (mut t, mut next, mut out) = (0.0_f64, 0usize, Vec::with_capacity(times.len()));
    while next < times.len() {
        while next < times.len() && times[next] <= t {
            out.push(Point3::from(p));
            next += 1;
        }
        let (np, nv, _) = Kinematics::step(p, v, Vector3::zeros(), DT, &physics);
        p = np;
        v = nv;
        t += DT;
    }
    return out;
}

fn residuals(guess: &Guess, times: &[f64], measured: &[Point3]) -> Vec<f64> {
    return walk(guess, times)
        .iter()
        .zip(measured)
        .flat_map(|(g, m)| [g.x - m.x, g.y - m.y, g.z - m.z])
        .collect();
}

fn rmse(residual: &[f64]) -> f64 {
    if residual.is_empty() {
        return f64::INFINITY;
    }
    return (residual.iter().map(|r| r * r).sum::<f64>() / (residual.len() / 3) as f64).sqrt();
}

/// 미지수 `n` 개를 가우스-뉴턴으로 푼다. `n = 6` 이면 항력을 고정한 셈이다.
fn solve(samples: &[(f64, Point3)], unknowns: usize, drag: f64) -> Option<(Guess, f64)> {
    if samples.len() < 6 {
        return None;
    }
    let t0 = samples[0].0;
    let times: Vec<f64> = samples.iter().map(|(t, _)| t - t0).collect();
    let measured: Vec<Point3> = samples.iter().map(|(_, p)| *p).collect();
    let span = times[times.len() - 1];
    let mut guess = Guess {
        position: measured[0],
        velocity: (measured[measured.len() - 1] - measured[0]) / span.max(1e-6),
        drag,
    };

    const STEP: f64 = 1e-5;
    for _ in 0..40 {
        let base = residuals(&guess, &times, &measured);
        let mut columns: Vec<Vec<f64>> = Vec::with_capacity(unknowns);
        for axis in 0..unknowns {
            let moved = residuals(&guess.nudged(axis, STEP), &times, &measured);
            if moved.len() != base.len() {
                return None;
            }
            columns.push(
                moved
                    .iter()
                    .zip(&base)
                    .map(|(b, a)| (b - a) / STEP)
                    .collect(),
            );
        }
        let mut normal = nalgebra::DMatrix::<f64>::zeros(unknowns, unknowns);
        let mut gradient = nalgebra::DVector::<f64>::zeros(unknowns);
        for a in 0..unknowns {
            for b in 0..unknowns {
                normal[(a, b)] = (0..base.len()).map(|i| columns[a][i] * columns[b][i]).sum();
            }
            normal[(a, a)] += 1e-9;
            gradient[a] = (0..base.len()).map(|i| columns[a][i] * base[i]).sum();
        }
        let delta = normal.try_inverse()? * gradient;
        for axis in 0..unknowns {
            guess.apply(axis, delta[axis]);
        }
        if delta.norm() < 1e-12 {
            break;
        }
    }
    let final_rmse = rmse(&residuals(&guess, &times, &measured));
    return Some((guess, final_rmse));
}

/// 클립의 생 삼각측량 궤적. `stop_at_bounce` 면 첫 바운스에서 끊는다 —
/// 항력을 잴 때는 반발계수 오차가 섞이면 안 된다.
fn flight(clip: &str, stop_at_bounce: bool) -> Option<Vec<(f64, Point3)>> {
    let dir = Path::new("data/clips").join(clip);
    let text = std::fs::read_to_string(dir.join("meta.json")).ok()?;
    let meta: serde_json::Value = serde_json::from_str(&text).ok()?;
    let fps = meta["meas_fps"].as_f64().unwrap_or(30.0);
    let calibration = Calibration::load_json(&defaults::calibration_path()).ok()?;
    let mut vision = Vision::load(
        &calibration,
        Box::new(PlaneCrossing {
            y: table::LENGTH_Y * 0.5,
        }),
    )
    .ok()?;

    let mut left = OpenCvCapture::from_path(camera::Id(0), &dir.join("left.avi")).ok()?;
    let mut right = OpenCvCapture::from_path(camera::Id(1), &dir.join("right.avi")).ok()?;
    let epoch = Instant::now();
    let (mut index, mut raw) = (0usize, Vec::new());
    loop {
        let (Some(l), Some(r)) = (left.next_frame(), right.next_frame()) else {
            break;
        };
        let at = epoch + Duration::from_secs_f64(index as f64 / fps);
        let mut pixels = [None, None];
        for (slot, frame) in [
            (0usize, Frame::new(camera::Id(0), l.image, at)),
            (1usize, Frame::new(camera::Id(1), r.image, at)),
        ] {
            vision.feed(&frame).ok()?;
            pixels[slot] = vision.last_found().map(|c| c.pixel);
        }
        if let (Some(a), Some(b)) = (pixels[0], pixels[1])
            && let Some(point) =
                Triangulate::pixels(&[(camera::Id(0), a), (camera::Id(1), b)], &calibration)
        {
            raw.push((index as f64 / fps, point));
        }
        index += 1;
    }

    // 비행 구간만 — 구간 밖 오검출이 앞뒤에 섞여 있다. 프레임이 이어지는 가장 긴 토막을 쓴다.
    let gap = 4.0 / fps;
    let mut runs: Vec<Vec<(f64, Point3)>> = Vec::new();
    for sample in raw {
        match runs.last_mut() {
            Some(run) if sample.0 - run[run.len() - 1].0 <= gap => run.push(sample),
            _ => runs.push(vec![sample]),
        }
    }
    let mut flight = runs.into_iter().max_by_key(Vec::len)?;

    // 첫 바운스까지 — 반발계수 오차가 항력으로 새면 안 된다. 테이블 **가까이서** z 가
    // 다시 오르는 지점만 바운스로 본다. 잡음이 만드는 높은 곳의 극소점은 바운스가 아니다.
    let near_table = table::SURFACE_Z + 0.15;
    if stop_at_bounce
        && let Some(bounce) = (1..flight.len().saturating_sub(1)).find(|&i| {
            flight[i].1.z < near_table
                && flight[i].1.z <= flight[i - 1].1.z
                && flight[i + 1].1.z > flight[i].1.z
        })
    {
        flight.truncate(bounce + 1);
    }
    return (flight.len() >= 8).then_some(flight);
}

#[test]
#[ignore = "클립 필요: cargo test --release --test diag_drag -- --ignored --nocapture"]
fn fit_drag_from_the_clips() {
    println!(
        "이론값 k = ½ρC_dA/m  →  C_d 0.40 이면 {:.3},  0.50 이면 {:.3} m⁻¹",
        theoretical(0.40),
        theoretical(0.50)
    );
    println!(
        "{:<8} {:>5} {:>8} {:>10} {:>10} {:>8} {:>12}",
        "clip", "점", "|v0|", "drag=0 RMSE", "적합 RMSE", "drag", "바운스 포함"
    );

    let mut fitted: Vec<f64> = Vec::new();
    for clip in CLIPS {
        let Some(samples) = flight(clip, true) else {
            println!("{clip:<8} {:>5}", "부족");
            continue;
        };
        // 바운스까지 포함해 같은 모델로 풀어 본다. 여기서 RMSE 가 튀면 항력이 아니라
        // 반발 모델이 문제다.
        let with_bounce = flight(clip, false)
            .and_then(|all| solve(&all, 7, 0.12))
            .map(|(_, rmse)| rmse);
        // 항력을 0 으로 고정한 적합 — 지금 코드가 하는 일이다.
        let fixed = solve(&samples, 6, 0.0);
        // 항력까지 푼 적합.
        let free = solve(&samples, 7, 0.12);
        match (fixed, free) {
            (Some((_, fixed_rmse)), Some((guess, free_rmse))) => {
                fitted.push(guess.drag);
                println!(
                    "{clip:<8} {:>5} {:>8.2} {:>9.1}cm {:>9.1}cm {:>8.3} {:>11}",
                    samples.len(),
                    guess.velocity.norm(),
                    fixed_rmse * 100.0,
                    free_rmse * 100.0,
                    guess.drag,
                    with_bounce.map_or("--".to_owned(), |r| format!("{:.1}cm", r * 100.0))
                );
            }
            _ => println!("{clip:<8} {:>5}", "실패"),
        }
    }

    if fitted.is_empty() {
        println!("적합된 클립이 없다");
        return;
    }
    fitted.sort_by(|a, b| a.partial_cmp(b).expect("유한"));
    let median = fitted[fitted.len() / 2];
    let mean = fitted.iter().sum::<f64>() / fitted.len() as f64;
    let spread = fitted[fitted.len() - 1] - fitted[0];
    println!(
        "\ndrag  중앙값 {median:.3}  평균 {mean:.3}  범위 {:.3}~{:.3} (폭 {spread:.3})",
        fitted[0],
        fitted[fitted.len() - 1]
    );
    println!(
        "이론값 대비 C_d = {:.2}  (구의 이 속도대 기대값 0.4~0.5)",
        median / theoretical(1.0)
    );
    println!(
        "클립마다 값이 흩어지면 모델이 뭔가를 빠뜨린 것이다 — 스핀이 유력하다.\n\
         Magnus 는 속도에 수직이라 항력으로 새면 클립마다 부호와 크기가 달라진다."
    );
}
