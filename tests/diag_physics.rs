//! 물리 상수를 클립에서 실측한다 — 항력 `drag` 와 스핀 `ω`.
//!
//! 새 장비도 새 촬영도 필요 없다. 이미 찍어 둔 클립의 생 삼각측량 궤적에 미지수로 넣고
//! 초기 조건과 같이 푼다. 클립마다 나온 값이 서로 맞으면 그게 측정이고, 안 맞으면 모델이
//! 아직 뭔가를 빠뜨린 것이다.
//!
//! 미지수를 늘려 가며 RMSE 가 얼마나 주는지 본다:
//!
//! | 미지수 | 무엇 |
//! |---|---|
//! | 6 | `p0`, `v0` — 지금 코드가 푸는 것 |
//! | 7 | `+ drag` |
//! | 10 | `+ ω` — Magnus 와 바운스 접선 임펄스 |
//!
//! `ω_z`(수직축 스핀)는 바운스에서 안 보인다. 접촉점 속도가
//! `v + ω × (0,0,-R) = (v_x - Rω_y, v_y + Rω_x, 0)` 이라 `ω_z` 가 아예 안 들어간다.
//! Magnus 로만 보이므로 다른 둘보다 훨씬 약하게 잡힌다 — 흩어지면 그게 이유다.
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
use pingpong_bot::physics::Kinematics;
use pingpong_bot::vision::Vision;
use pingpong_bot::vision::triggers::PlaneCrossing;
use pingpong_bot::{Point3, Vector3};

/// 지금 리그로 찍은 클립만 쓴다 — 카메라를 옮겼으므로 옛 클립은 캘리브와 안 맞는다.
use pingpong_bot::defaults::CURRENT_RIG_CLIPS as CLIPS;
const DT: f64 = 0.001;
/// 공기 밀도 [kg/m³].
const AIR_DENSITY: f64 = 1.2;
/// 물리적으로 가능한 스핀 상한 [rad/s]. 세계 정상급이 150 rps ≈ 940 rad/s 다.
///
/// 없으면 최적화가 잔차를 못 줄이는 클립에서 ω 를 무한대로 밀어 버린다 (실측 787847).
const OMEGA_MAX: f64 = 1000.0;

/// `k = ½ ρ C_d A / m`.
fn theoretical(drag_coefficient: f64) -> f64 {
    let area = std::f64::consts::PI * ball::RADIUS * ball::RADIUS;
    return 0.5 * AIR_DENSITY * drag_coefficient * area / ball::MASS;
}

/// 미지수 10개: `p0`(3), `v0`(3), `drag`(1), `ω`(3).
#[derive(Debug, Clone, Copy)]
struct Guess {
    position: Point3,
    velocity: Vector3,
    drag: f64,
    omega: Vector3,
}

/// 축별 수치 미분 폭. 단위가 달라 하나로 못 쓴다 — ω 는 rad/s 라 훨씬 크다.
fn step_of(axis: usize) -> f64 {
    return match axis {
        0..=5 => 1e-5,
        6 => 1e-4,
        _ => 1e-2,
    };
}

impl Guess {
    fn nudged(mut self, axis: usize, step: f64) -> Self {
        match axis {
            0..=2 => self.position[axis] += step,
            3..=5 => self.velocity[axis - 3] += step,
            6 => self.drag += step,
            _ => self.omega[axis - 7] += step,
        }
        return self;
    }

    fn apply(&mut self, axis: usize, delta: f64) {
        match axis {
            0..=2 => self.position[axis] -= delta,
            3..=5 => self.velocity[axis - 3] -= delta,
            // 항력은 음수가 될 수 없다. 물리적으로 없는 값을 해로 받으면 안 된다.
            6 => self.drag = (self.drag - delta).clamp(0.0, 1.0),
            _ => {
                self.omega[axis - 7] = (self.omega[axis - 7] - delta).clamp(-OMEGA_MAX, OMEGA_MAX);
            }
        }
    }
}

/// ω 는 바운스에서 바뀌므로 들고 다녀야 한다 — 버리면 반발 뒤가 통째로 틀린다.
fn walk(guess: &Guess, times: &[f64]) -> Vec<Point3> {
    let physics = PhysicsParams {
        drag: guess.drag,
        ..PhysicsParams::default()
    };
    let (mut p, mut v, mut w) = (guess.position.coords, guess.velocity, guess.omega);
    let (mut t, mut next, mut out) = (0.0_f64, 0usize, Vec::with_capacity(times.len()));
    while next < times.len() {
        while next < times.len() && times[next] <= t {
            out.push(Point3::from(p));
            next += 1;
        }
        let (np, nv, nw) = Kinematics::step(p, v, w, DT, &physics);
        p = np;
        v = nv;
        w = nw;
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

/// 미지수 `unknowns` 개를 레벤버그-마쿼트로 푼다.
///
/// 순수 가우스-뉴턴을 안 쓰는 이유는 바운스 때문이다. 반발 시각이 미지수에 따라 움직여서
/// 잔차가 그 지점에서 매끄럽지 않고, 감쇠 없이는 한 걸음에 뛰어넘어 발산한다.
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
        omega: Vector3::zeros(),
    };
    let mut best = rmse(&residuals(&guess, &times, &measured));
    let mut lambda = 1e-3;

    for _ in 0..120 {
        let base = residuals(&guess, &times, &measured);
        let mut columns: Vec<Vec<f64>> = Vec::with_capacity(unknowns);
        for axis in 0..unknowns {
            let step = step_of(axis);
            let moved = residuals(&guess.nudged(axis, step), &times, &measured);
            if moved.len() != base.len() {
                return None;
            }
            columns.push(
                moved
                    .iter()
                    .zip(&base)
                    .map(|(b, a)| (b - a) / step)
                    .collect(),
            );
        }
        let mut normal = nalgebra::DMatrix::<f64>::zeros(unknowns, unknowns);
        let mut gradient = nalgebra::DVector::<f64>::zeros(unknowns);
        for a in 0..unknowns {
            for b in 0..unknowns {
                normal[(a, b)] = (0..base.len()).map(|i| columns[a][i] * columns[b][i]).sum();
            }
            gradient[a] = (0..base.len()).map(|i| columns[a][i] * base[i]).sum();
        }

        // 감쇠를 키워 가며 실제로 줄어드는 걸음을 찾는다.
        let mut improved = false;
        for _ in 0..8 {
            let mut damped = normal.clone();
            for a in 0..unknowns {
                damped[(a, a)] += lambda * (normal[(a, a)].abs() + 1e-12);
            }
            let Some(delta) = damped.try_inverse().map(|inverse| &inverse * &gradient) else {
                lambda *= 10.0;
                continue;
            };
            let mut candidate = guess;
            for axis in 0..unknowns {
                candidate.apply(axis, delta[axis]);
            }
            let score = rmse(&residuals(&candidate, &times, &measured));
            if score < best {
                guess = candidate;
                best = score;
                lambda = (lambda * 0.3).max(1e-9);
                improved = true;
                break;
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
    return Some((guess, best));
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
#[ignore = "클립 필요: cargo test --release --test diag_physics -- --ignored --nocapture"]
fn fit_physics_from_the_clips() {
    println!(
        "이론 항력 k = ½ρC_dA/m  →  C_d 0.40 이면 {:.3},  0.50 이면 {:.3} m⁻¹",
        theoretical(0.40),
        theoretical(0.50)
    );
    println!("바운스를 **포함한** 구간으로 푼다 — 스핀은 거기서 속도로 바뀌며 보인다.\n");
    println!(
        "{:<8} {:>4} {:>9} {:>9} {:>9} {:>7} {:>22}",
        "clip", "점", "p,v", "+drag", "+omega", "drag", "omega [rad/s]"
    );

    let mut drags: Vec<f64> = Vec::new();
    let mut spins: Vec<f64> = Vec::new();
    let mut gain: Vec<f64> = Vec::new();
    for clip in CLIPS {
        let Some(samples) = flight(clip, false) else {
            println!("{clip:<8} {:>4}", "부족");
            continue;
        };
        let plain = solve(&samples, 6, 0.0).map(|(_, rmse)| rmse);
        let with_drag = solve(&samples, 7, 0.12).map(|(_, rmse)| rmse);
        let full = solve(&samples, 10, 0.12);
        let (Some(plain), Some(with_drag), Some((guess, best))) = (plain, with_drag, full) else {
            println!("{clip:<8} {:>4}", "실패");
            continue;
        };
        drags.push(guess.drag);
        spins.push(guess.omega.norm());
        gain.push(plain / best.max(1e-9));
        println!(
            "{clip:<8} {:>4} {:>7.1}cm {:>7.1}cm {:>7.1}cm {:>7.3} {:>7.0}{:>7.0}{:>7.0}",
            samples.len(),
            plain * 100.0,
            with_drag * 100.0,
            best * 100.0,
            guess.drag,
            guess.omega.x,
            guess.omega.y,
            guess.omega.z
        );
    }

    if drags.is_empty() {
        println!("적합된 클립이 없다");
        return;
    }
    let median = |values: &mut Vec<f64>| -> f64 {
        values.sort_by(|a, b| a.partial_cmp(b).expect("유한"));
        return values[values.len() / 2];
    };
    let spread = |values: &[f64]| -> (f64, f64) {
        return (
            values.iter().copied().fold(f64::INFINITY, f64::min),
            values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        );
    };
    let (drag_low, drag_high) = spread(&drags);
    let (spin_low, spin_high) = spread(&spins);
    println!(
        "\ndrag   중앙값 {:.3}  범위 {drag_low:.3}~{drag_high:.3}   (이론 대비 C_d {:.2})",
        median(&mut drags),
        median(&mut drags.clone()) / theoretical(1.0)
    );
    println!(
        "|omega| 중앙값 {:.0} rad/s  범위 {spin_low:.0}~{spin_high:.0}  ({:.0} rps)",
        median(&mut spins),
        median(&mut spins.clone()) / (2.0 * std::f64::consts::PI)
    );
    println!(
        "RMSE 개선 배수  중앙값 {:.1}배  (p,v 만 풀었을 때 대비)",
        median(&mut gain)
    );
    println!(
        "\n주의: omega_z 는 바운스에서 안 보인다 (접촉점 속도에 안 들어간다). Magnus 로만\n\
         잡히므로 다른 둘보다 훨씬 약하다 — 흩어지면 그게 이유다."
    );
}

/// 공이 실제로 테이블 중앙에서 출발하나, 그리고 옆으로 얼마나 흐르나.
///
/// 슈터가 `x = W/2` 에 있고 똑바로 쏜다면 궤적의 x 가 거의 안 변해야 한다. 사실이면 그건
/// 적합에 넣을 수 있는 **공짜 정보**다 — x 는 두 카메라가 다 +X 옆면이라 가장 안 보이는
/// 축이라서, 아는 것을 넣어 주는 값이 크다.
#[test]
#[ignore = "클립 필요"]
fn does_the_ball_start_from_the_middle() {
    let middle = table::WIDTH_X * 0.5;
    println!("테이블 중앙 x = {middle:.3} m");
    println!(
        "{:<8} {:>8} {:>8} {:>9} {:>9}",
        "clip", "첫 x", "끝 x", "중앙 대비", "v_x"
    );
    let mut starts = Vec::new();
    let mut lateral = Vec::new();
    for clip in CLIPS {
        let Some(flight) = flight(clip, true) else {
            continue;
        };
        let (first, last) = (flight[0], flight[flight.len() - 1]);
        let dt = last.0 - first.0;
        let vx = if dt > 1e-6 {
            (last.1.x - first.1.x) / dt
        } else {
            0.0
        };
        starts.push(first.1.x - middle);
        lateral.push(vx);
        println!(
            "{clip:<8} {:>8.3} {:>8.3} {:>+9.3} {:>+9.2}",
            first.1.x,
            last.1.x,
            first.1.x - middle,
            vx
        );
    }
    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len().max(1) as f64;
    let sd = |v: &[f64]| {
        let m = mean(v);
        return (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len().max(1) as f64).sqrt();
    };
    println!(
        "\n첫 x - 중앙  평균 {:+.3} m  표준편차 {:.3} m",
        mean(&starts),
        sd(&starts)
    );
    println!(
        "v_x          평균 {:+.2} m/s  표준편차 {:.2} m/s",
        mean(&lateral),
        sd(&lateral)
    );
}
