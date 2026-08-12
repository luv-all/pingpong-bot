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
//! | 9 | `+ restitution, friction` — 바운스 법선·접선 계수 |
//! | 12 | `+ ω` — Magnus 와 바운스 접선 임펄스 |
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

/// 미지수 12개: `p0`(3), `v0`(3), `drag`(1), `restitution`(1), `friction`(1), `ω`(3).
#[derive(Debug, Clone, Copy)]
struct Guess {
    position: Point3,
    velocity: Vector3,
    drag: f64,
    restitution: f64,
    friction: f64,
    omega: Vector3,
}

/// 축별 수치 미분 폭. 단위가 달라 하나로 못 쓴다 — ω 는 rad/s 라 훨씬 크다.
fn step_of(axis: usize) -> f64 {
    return match axis {
        0..=5 => 1e-5,
        6..=8 => 1e-4,
        _ => 1e-2,
    };
}

impl Guess {
    fn nudged(mut self, axis: usize, step: f64) -> Self {
        match axis {
            0..=2 => self.position[axis] += step,
            3..=5 => self.velocity[axis - 3] += step,
            6 => self.drag += step,
            7 => self.restitution += step,
            8 => self.friction += step,
            _ => self.omega[axis - 9] += step,
        }
        return self;
    }

    fn apply(&mut self, axis: usize, delta: f64) {
        match axis {
            0..=2 => self.position[axis] -= delta,
            3..=5 => self.velocity[axis - 3] -= delta,
            // 항력·반발계수·마찰계수는 음수가 될 수 없다. 반발·마찰은 1을 넘지도 않는다.
            6 => self.drag = (self.drag - delta).clamp(0.0, 1.0),
            7 => self.restitution = (self.restitution - delta).clamp(0.0, 1.0),
            8 => self.friction = (self.friction - delta).clamp(0.0, 1.0),
            _ => {
                self.omega[axis - 9] = (self.omega[axis - 9] - delta).clamp(-OMEGA_MAX, OMEGA_MAX);
            }
        }
    }
}

/// ω 는 바운스에서 바뀌므로 들고 다녀야 한다 — 버리면 반발 뒤가 통째로 틀린다.
fn walk(guess: &Guess, times: &[f64]) -> Vec<Point3> {
    let physics = PhysicsParams {
        drag: guess.drag,
        restitution: guess.restitution,
        friction: guess.friction,
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

/// `free` 에 있는 축만 레벤버그-마쿼트로 푼다. 나머지는 `seed` 값에 고정된다.
///
/// 순수 가우스-뉴턴을 안 쓰는 이유는 바운스 때문이다. 반발 시각이 미지수에 따라 움직여서
/// 잔차가 그 지점에서 매끄럽지 않고, 감쇠 없이는 한 걸음에 뛰어넘어 발산한다.
///
/// 예전엔 `unknowns: usize`(0..unknowns 연속 구간)였다 — `calibrate_shared_physics`가
/// drag·e·mu는 고정하고 p0,v0만(ω 빼고) 풀어야 해서 임의의 부분집합을 받게 넓혔다. 축
/// 번호 자체(`step_of`·`nudged`·`apply`)는 그대로라 기존 호출부는 `0..unknowns`를
/// 그대로 넘기면 이전과 동일하게 동작한다.
fn solve(samples: &[(f64, Point3)], free: &[usize], seed: PhysicsParams) -> Option<(Guess, f64)> {
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
        drag: seed.drag,
        restitution: seed.restitution,
        friction: seed.friction,
        omega: Vector3::zeros(),
    };
    let mut best = rmse(&residuals(&guess, &times, &measured));
    let mut lambda = 1e-3;
    let n = free.len();

    for _ in 0..120 {
        let base = residuals(&guess, &times, &measured);
        let mut columns: Vec<Vec<f64>> = Vec::with_capacity(n);
        for &axis in free {
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
        let mut normal = nalgebra::DMatrix::<f64>::zeros(n, n);
        let mut gradient = nalgebra::DVector::<f64>::zeros(n);
        for a in 0..n {
            for b in 0..n {
                normal[(a, b)] = (0..base.len()).map(|i| columns[a][i] * columns[b][i]).sum();
            }
            gradient[a] = (0..base.len()).map(|i| columns[a][i] * base[i]).sum();
        }

        // 감쇠를 키워 가며 실제로 줄어드는 걸음을 찾는다.
        let mut improved = false;
        for _ in 0..8 {
            let mut damped = normal.clone();
            for a in 0..n {
                damped[(a, a)] += lambda * (normal[(a, a)].abs() + 1e-12);
            }
            let Some(delta) = damped.try_inverse().map(|inverse| &inverse * &gradient) else {
                lambda *= 10.0;
                continue;
            };
            let mut candidate = guess;
            for (i, &axis) in free.iter().enumerate() {
                candidate.apply(axis, delta[i]);
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

const P_V: [usize; 6] = [0, 1, 2, 3, 4, 5];
const P_V_DRAG: [usize; 7] = [0, 1, 2, 3, 4, 5, 6];
const P_V_BOUNCE: [usize; 9] = [0, 1, 2, 3, 4, 5, 6, 7, 8];
const P_V_ALL: [usize; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];

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

    // 첫 바운스를 찾는다. 테이블 **가까이서** z 가 다시 오르는 지점만 바운스로 본다 —
    // 잡음이 만드는 높은 곳의 극소점은 바운스가 아니다.
    let near_table = table::SURFACE_Z + 0.15;
    let bounce = (1..flight.len().saturating_sub(1)).find(|&i| {
        flight[i].1.z < near_table
            && flight[i].1.z <= flight[i - 1].1.z
            && flight[i + 1].1.z > flight[i].1.z
    });
    if let Some(bounce) = bounce {
        if stop_at_bounce {
            // 항력만 잴 때는 반발계수 오차가 안 섞이게 바운스 자체를 뺀다.
            flight.truncate(bounce + 1);
        } else {
            // 바운스는 포함하되 그 뒤로 딱 한 토막만 남긴다 — 서브 촬영엔 던지기·되받아치기
            // 등 두 번째 바운스·방향 반전이 같은 "가장 긴 토막" 안에 이어붙어 있을 수 있다.
            // 테이블 바운스 모델 하나로 못 맞는 국면까지 들어오면 반발·마찰 적합이 그걸
            // 억지로 설명하려고 극단값(e=1, ω=OMEGA_MAX)으로 도망간다 (실측 fly_45~54).
            const POST_BOUNCE_TAIL: usize = 20;
            flight.truncate((bounce + 1 + POST_BOUNCE_TAIL).min(flight.len()));
        }
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
    println!("바운스를 **포함한** 구간으로 푼다 — 반발·마찰·스핀 다 거기서 속도로 바뀌며 보인다.\n");
    println!(
        "{:<8} {:>4} {:>9} {:>9} {:>9} {:>9} {:>7} {:>7} {:>7} {:>22}",
        "clip", "점", "p,v", "+drag", "+bounce", "+omega", "drag", "e", "mu", "omega [rad/s]"
    );

    let mut drags: Vec<f64> = Vec::new();
    let mut restitutions: Vec<f64> = Vec::new();
    let mut frictions: Vec<f64> = Vec::new();
    let mut spins: Vec<f64> = Vec::new();
    let mut gain: Vec<f64> = Vec::new();
    for clip in CLIPS {
        let Some(samples) = flight(clip, false) else {
            println!("{clip:<8} {:>4}", "부족");
            continue;
        };
        let no_drag = PhysicsParams {
            drag: 0.0,
            ..PhysicsParams::default()
        };
        let with_drag_seed = PhysicsParams {
            drag: 0.12,
            ..PhysicsParams::default()
        };
        let plain = solve(&samples, &P_V, no_drag).map(|(_, rmse)| rmse);
        let with_drag = solve(&samples, &P_V_DRAG, with_drag_seed).map(|(_, rmse)| rmse);
        let with_bounce = solve(&samples, &P_V_BOUNCE, with_drag_seed).map(|(_, rmse)| rmse);
        let full = solve(&samples, &P_V_ALL, with_drag_seed);
        let (Some(plain), Some(with_drag), Some(with_bounce), Some((guess, best))) =
            (plain, with_drag, with_bounce, full)
        else {
            println!("{clip:<8} {:>4}", "실패");
            continue;
        };
        drags.push(guess.drag);
        restitutions.push(guess.restitution);
        frictions.push(guess.friction);
        spins.push(guess.omega.norm());
        gain.push(plain / best.max(1e-9));
        println!(
            "{clip:<8} {:>4} {:>7.1}cm {:>7.1}cm {:>7.1}cm {:>7.1}cm {:>7.3} {:>7.3} {:>7.3} {:>7.0}{:>7.0}{:>7.0}",
            samples.len(),
            plain * 100.0,
            with_drag * 100.0,
            with_bounce * 100.0,
            best * 100.0,
            guess.drag,
            guess.restitution,
            guess.friction,
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
    let (rest_low, rest_high) = spread(&restitutions);
    let (fric_low, fric_high) = spread(&frictions);
    let (spin_low, spin_high) = spread(&spins);
    println!(
        "\ndrag   중앙값 {:.3}  범위 {drag_low:.3}~{drag_high:.3}   (이론 대비 C_d {:.2})",
        median(&mut drags),
        median(&mut drags.clone()) / theoretical(1.0)
    );
    println!(
        "restitution 중앙값 {:.3}  범위 {rest_low:.3}~{rest_high:.3}",
        median(&mut restitutions.clone())
    );
    println!(
        "friction    중앙값 {:.3}  범위 {fric_low:.3}~{fric_high:.3}",
        median(&mut frictions.clone())
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
    println!(
        "\n{}",
        pingpong_bot::physics::PhysicsIdentify::format_physics_for_defaults(
            Some(median(&mut restitutions)),
            Some(median(&mut frictions)),
            Some(median(&mut drags)),
        )
    );
}

/// 중심값 근처를 촘촘히 훑어 `try_value`가 가장 작아지는 자리로 옮긴다. 실패(`None`)한
/// 후보는 무시 — 셋 다 유효 범위 안이라 보통 안 일어나지만, 경계 근처에서 LM이 못
/// 풀 때를 대비한다.
fn sweep(
    center: f64,
    radius: f64,
    steps: usize,
    (lo, hi): (f64, f64),
    mut try_value: impl FnMut(f64) -> Option<f64>,
) -> f64 {
    let mut best = (center, try_value(center).unwrap_or(f64::INFINITY));
    for i in 0..steps {
        let frac = i as f64 / (steps - 1) as f64 * 2.0 - 1.0; // -1..1
        let candidate = (center + frac * radius).clamp(lo, hi);
        if let Some(score) = try_value(candidate)
            && score < best.1
        {
            best = (candidate, score);
        }
    }
    return best.0;
}

/// e·mu·drag를 **클립 하나가 아니라 9개 전부에 공유**로 걸어 좌표하강으로 찾는다.
///
/// `fit_physics_from_the_clips`의 클립별 12-미지수 적합은 점이 20~40개뿐이라 못 믿는다 —
/// restitution이 경계 1.0으로 튀는 클립이 태반이었다(실측 2026-08-12). 여기서는 e·mu·drag를
/// **9클립 공유**로 묶고 클립마다는 p0,v0만 푼다(ω=0 고정, 축 6개뿐) — 실질 미지수가
/// 27(9클립×3)이 아니라 3개짜리 문제가 되어 훨씬 잘 걸린다.
///
/// ω를 0으로 고정하는 이유: 라이브 파이프라인도 바운스에서 스핀이 안 풀리면 0으로
/// 접는다(`ASSUMED_SPIN`, 9클립 중 6개가 지금 이 경로) — 그 조건에서 e·mu·drag가
/// 최선이어야 실전과 맞는다. ω도 같이 풀게 두면 ω가 e·mu 오차를 조용히 흡수해서 셋을
/// 못 가른다 — 이게 클립별 12-미지수 적합이 못 미더웠던 바로 그 원인이다.
///
/// 좌표하강(e→mu→drag 순, 반경을 라운드마다 줄임)을 쓴다 — 3차원 완전 그리드서치보다
/// 훨씬 적은 평가로 같은 자리에 수렴하고, 목적함수가 부드럽지 않을 수 있어서(바운스
/// 시각이 미지수에 따라 움직인다, `solve`의 감쇠 설명과 같은 이유) 경사법보다 안전하다.
///
/// **점수는 `solve`가 본 바로 그 구간의 잔차가 아니다.** 처음 그렇게 짰다가 drag가
/// 0으로 수렴했다 — 원인을 보니, 그 구간을 그대로 다시 맞히는 거라 v0를 물리에 맞게
/// 조정해 버리면 drag=0이든 0.126이든 **본 구간 안에서는** 둘 다 똑같이 잘 맞더라(v0가
/// 자유 미지수라 그 구간 안에서 서로 보상함). 그런데 그건 로봇이 실제로 겪는 문제가
/// 아니다 — 로봇은 트리거 시점까지 본 것으로 **아직 못 본 나머지**를 맞혀야 한다
/// (`clip-review`의 리드타임 오차와 정확히 같은 질문). 그래서 각 클립을 앞 60%(적합용)
/// 뒤 40%(채점용)로 쪼갠다 — 앞부분만으로 p0,v0를 풀고, 그 결과를 뒷부분 시각까지
/// **굴려서** 뒷부분과 비교한다. 이게 실측 drag=0.126이 `clip-review`에서 크게 이겼던
/// 것과 이 도구가 같은 답을 내게 만드는 핵심 수정이다.
fn split_extrapolation_rmse(samples: &[(f64, Point3)], physics: PhysicsParams) -> Option<f64> {
    // 6(최소 관측)+3(채점용 최소) 은 너무 빠듯하다 — 양쪽 다 넉넉히 두려고 12를 요구한다.
    if samples.len() < 12 {
        return None;
    }
    let split = (samples.len() * 3 / 5).max(6);
    let (train, test) = samples.split_at(split);
    if test.len() < 3 {
        return None;
    }
    let (guess, _) = solve(train, &P_V, physics)?;
    let t0 = train[0].0;
    let times: Vec<f64> = test.iter().map(|(t, _)| t - t0).collect();
    let predicted = walk(&guess, &times);
    if predicted.len() != test.len() {
        return None;
    }
    let sum_sq: f64 = predicted
        .iter()
        .zip(test)
        .map(|(p, (_, m))| (p.coords - m.coords).norm_squared())
        .sum();
    return Some((sum_sq / test.len() as f64).sqrt());
}

#[test]
#[ignore = "클립 필요: cargo test --release --test diag_physics calibrate_shared_physics -- --ignored --nocapture"]
fn calibrate_shared_physics_across_clips() {
    let clips: Vec<Vec<(f64, Point3)>> = CLIPS
        .iter()
        .filter_map(|clip| flight(clip, false))
        .collect();
    println!("{}개 클립 로드 (전 {}개 중)", clips.len(), CLIPS.len());

    let score = |e: f64, mu: f64, drag: f64| -> Option<f64> {
        let physics = PhysicsParams {
            restitution: e,
            friction: mu,
            drag,
            ..PhysicsParams::default()
        };
        let mut total = 0.0;
        let mut n = 0usize;
        for samples in &clips {
            let Some(rmse) = split_extrapolation_rmse(samples, physics) else {
                continue;
            };
            total += rmse;
            n += 1;
        }
        return (n > 0).then_some(total / n as f64);
    };

    let default = PhysicsParams::default();
    let (mut e, mut mu, mut drag) = (default.restitution, default.friction, default.drag);
    println!(
        "시작(지금 기본값): e={e:.3} mu={mu:.3} drag={drag:.3}  평균 외삽오차={:.2}cm",
        score(e, mu, drag).unwrap_or(f64::INFINITY) * 100.0
    );

    const ROUNDS: [f64; 4] = [0.20, 0.08, 0.03, 0.01];
    for (round, &radius) in ROUNDS.iter().enumerate() {
        e = sweep(e, radius, 9, (0.3, 0.98), |cand| score(cand, mu, drag));
        mu = sweep(mu, radius, 9, (0.0, 0.9), |cand| score(e, cand, drag));
        drag = sweep(drag, radius.min(0.15), 9, (0.0, 0.35), |cand| {
            score(e, mu, cand)
        });
        println!(
            "라운드 {round} (반경 {radius:.2}): e={e:.4} mu={mu:.4} drag={drag:.4}  평균 외삽오차={:.3}cm",
            score(e, mu, drag).unwrap_or(f64::INFINITY) * 100.0
        );
    }

    println!(
        "\n주의: ω=0 고정(스핀은 여전히 라이브 `solve_spin_from_bounce`가 담당) — 이 e·mu·drag는\n\
         '스핀 못 풀렸을 때 최선'을 찾은 것이지 스핀까지 아는 참값이 아니다."
    );
    println!(
        "\n{}",
        pingpong_bot::physics::PhysicsIdentify::format_physics_for_defaults(
            Some(e),
            Some(mu),
            Some(drag),
        )
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
