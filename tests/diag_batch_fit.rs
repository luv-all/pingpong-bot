//! 재귀 필터(EKF) 대 일괄 적합(batch fit) — 트리거까지의 관측 **전부**로 탄도를 맞추면
//! 예측이 얼마나 안정되나.
//!
//! EKF 는 매 순간의 상태 하나를 들고 앞으로 굴린다. 그 순간이 나쁘면 예측이 통째로
//! 나쁘고, 트랙을 버렸다 다시 세우면 이력이 사라진다. 일괄 적합은 그 구간의 점을 전부
//! 써서 물리 모델의 초기 조건 6개(p0, v0)를 푼다 — 한 프레임이 튀어도 나머지가 잡는다.
//!
//! ```bash
//! cargo test --release --test diag_batch_fit -- --ignored --nocapture
//! ```

use std::path::Path;
use std::time::{Duration, Instant};

use pingpong_bot::camera::{self, Calibration, Frame, FrameSource, OpenCvCapture, Triangulate};
use pingpong_bot::constants::table;
use pingpong_bot::defaults::{self, PhysicsParams};
use pingpong_bot::physics::Kinematics;
use pingpong_bot::vision::triggers::{Any, PlaneCrossing, SigmaThreshold};
use pingpong_bot::vision::{State, Track, Trigger, Vision};
use pingpong_bot::{Point3, Vector3};

/// 지금 리그로 찍은 클립만 쓴다 — 카메라를 옮겼으므로 옛 클립은 캘리브와 안 맞는다.
use pingpong_bot::defaults::CURRENT_RIG_CLIPS as CLIPS;
/// 적합·예측 적분 스텝 [s].
const DT: f64 = 0.001;

fn trigger() -> Box<dyn Trigger> {
    return Box::new(Any(vec![
        Box::new(SigmaThreshold {
            position: Vector3::repeat(0.05),
            velocity: Vector3::repeat(0.5),
        }),
        Box::new(PlaneCrossing {
            y: table::LENGTH_Y * 0.5,
        }),
    ]));
}

/// `(p0, v0)`에서 `until`까지 굴리며 각 표본 시각의 위치를 뽑는다.
fn simulate(p0: Point3, v0: Vector3, times: &[f64], physics: &PhysicsParams) -> Vec<Point3> {
    let (mut p, mut v) = (p0.coords, v0);
    let (mut t, mut out, mut next) = (0.0_f64, Vec::with_capacity(times.len()), 0usize);
    while next < times.len() {
        while next < times.len() && times[next] <= t {
            out.push(Point3::from(p));
            next += 1;
        }
        let (np, nv, _) = Kinematics::step(p, v, Vector3::zeros(), DT, physics);
        p = np;
        v = nv;
        t += DT;
    }
    return out;
}

/// 표본 전부에 탄도를 맞춘다 — 미지수는 초기 위치·속도 6개.
///
/// 가우스-뉴턴에 수치 야코비안. 미지수가 6개뿐이라 해석 미분을 유도할 이유가 없고,
/// 적분기를 그대로 쓰므로 예측과 같은 물리를 푼다.
fn fit(samples: &[(f64, Point3)], physics: &PhysicsParams) -> Option<(Point3, Vector3)> {
    if samples.len() < 4 {
        return None;
    }
    let t0 = samples[0].0;
    let times: Vec<f64> = samples.iter().map(|(t, _)| t - t0).collect();
    let measured: Vec<Point3> = samples.iter().map(|(_, p)| *p).collect();

    let span = times[times.len() - 1];
    let mut p0 = measured[0];
    let mut v0 = if span > 1e-6 {
        (measured[measured.len() - 1] - measured[0]) / span
    } else {
        Vector3::zeros()
    };

    let residual = |p0: Point3, v0: Vector3| -> Vec<f64> {
        let guess = simulate(p0, v0, &times, physics);
        return guess
            .iter()
            .zip(&measured)
            .flat_map(|(g, m)| [g.x - m.x, g.y - m.y, g.z - m.z])
            .collect();
    };

    for _ in 0..12 {
        let base = residual(p0, v0);
        let rows = base.len();
        let mut jt_j = nalgebra::Matrix6::<f64>::zeros();
        let mut jt_r = nalgebra::Vector6::<f64>::zeros();
        let step = 1e-4;
        let mut columns: Vec<Vec<f64>> = Vec::with_capacity(6);
        for k in 0..6 {
            let (mut pp, mut vv) = (p0, v0);
            if k < 3 {
                pp[k] += step;
            } else {
                vv[k - 3] += step;
            }
            let bumped = residual(pp, vv);
            if bumped.len() != rows {
                return None;
            }
            columns.push(
                bumped
                    .iter()
                    .zip(&base)
                    .map(|(b, a)| (b - a) / step)
                    .collect(),
            );
        }
        for a in 0..6 {
            for b in 0..6 {
                jt_j[(a, b)] = (0..rows).map(|i| columns[a][i] * columns[b][i]).sum();
            }
            jt_r[a] = (0..rows).map(|i| columns[a][i] * base[i]).sum();
        }
        // 리지 항으로 특이해를 막는다.
        for a in 0..6 {
            jt_j[(a, a)] += 1e-9;
        }
        let delta = jt_j.try_inverse()? * jt_r;
        for k in 0..3 {
            p0[k] -= delta[k];
            v0[k] -= delta[k + 3];
        }
        if delta.norm() < 1e-9 {
            break;
        }
    }
    return Some((p0, v0));
}

/// `(p0, v0)`에서 굴린 궤적을 [`Track`]으로 — 평면 통과는 계약이 푼다.
fn roll(p0: Point3, v0: Vector3, physics: &PhysicsParams) -> Track {
    let (mut p, mut v, mut t) = (p0.coords, v0, 0.0_f64);
    let mut out = Vec::new();
    while t < 2.0 && p.y > -0.2 {
        out.push(State {
            t: Duration::from_secs_f64(t),
            position: Point3::from(p),
            velocity: v,
            ..State::default()
        });
        let (np, nv, _) = Kinematics::step(p, v, Vector3::zeros(), DT, physics);
        p = np;
        v = nv;
        t += DT;
    }
    return Track(out);
}

fn as_track(samples: &[(f64, Point3)]) -> Track {
    return Track(
        samples
            .iter()
            .map(|(t, position)| State {
                t: Duration::from_secs_f64(*t),
                position: *position,
                ..State::default()
            })
            .collect(),
    );
}

/// 표본을 몇 개까지 쓸지 — 트리거를 늦추면 이만큼 모인다.
const BUDGETS: [usize; 5] = [5, 8, 12, 16, usize::MAX];

/// (EKF 오차, 표본 수별 일괄 적합 오차, 트리거까지 쓴 표본 수) [m]
fn run(clip: &str) -> (Option<f64>, Vec<Option<f64>>, usize) {
    let dir = Path::new("data/clips").join(clip);
    let text = std::fs::read_to_string(dir.join("meta.json")).expect("meta");
    let meta: serde_json::Value = serde_json::from_str(&text).expect("meta");
    let fps = meta["meas_fps"].as_f64().unwrap_or(30.0);
    let calibration = Calibration::load_json(&defaults::calibration_path()).expect("calibration");
    let physics = PhysicsParams::default();
    let mut vision = Vision::load(&calibration, trigger()).expect("vision");

    let mut left = OpenCvCapture::from_path(camera::Id(0), &dir.join("left.avi")).expect("left");
    let mut right = OpenCvCapture::from_path(camera::Id(1), &dir.join("right.avi")).expect("right");
    let epoch = Instant::now();
    let mut index = 0usize;
    let mut raw: Vec<(f64, Point3)> = Vec::new();
    let mut ekf_guess: Option<Point3> = None;
    let mut at_trigger = 0usize;
    let plane = table::DEFAULT_HIT_PLANE_Y;

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
            vision.feed(&frame).expect("feed");
            pixels[slot] = vision.last_found().map(|c| c.pixel);
        }
        if let (Some(a), Some(b)) = (pixels[0], pixels[1])
            && let Some(point) =
                Triangulate::pixels(&[(camera::Id(0), a), (camera::Id(1), b)], &calibration)
        {
            raw.push((index as f64 / fps, point));
        }
        if ekf_guess.is_none()
            && let Some(trajectory) = vision.trajectory()
        {
            at_trigger = raw.len();
            ekf_guess = trajectory.predicted.at_plane(plane).map(|s| s.position);
            if ekf_guess.is_none() {
                // 예측이 평면을 안 지나도 그 시점은 기록해 둔다.
                ekf_guess = Some(Point3::new(f64::NAN, f64::NAN, f64::NAN));
            }
        }
        index += 1;
    }

    let Some(truth) = as_track(&raw).at_plane(plane) else {
        return (None, vec![None; BUDGETS.len()], at_trigger);
    };
    let ekf = ekf_guess
        .filter(|p| p.coords.iter().all(|c| c.is_finite()))
        .map(|p| (p - truth.position).norm());
    // 접수 평면을 지난 뒤의 점을 쓰면 컨닝이다 — 타점 전 관측만 적합에 넣는다.
    let usable = raw
        .iter()
        .position(|(_, p)| p.y < plane)
        .unwrap_or(raw.len());
    let batch = BUDGETS
        .iter()
        .map(|budget| {
            let take = (*budget).min(usable);
            return fit(&raw[..take], &physics)
                .map(|(p0, v0)| roll(p0, v0, &physics))
                .and_then(|track| track.at_plane(plane))
                .map(|s| (s.position - truth.position).norm());
        })
        .collect();
    return (ekf, batch, at_trigger);
}

#[test]
#[ignore = "클립 필요: cargo test --release --test diag_batch_fit -- --ignored --nocapture"]
fn batch_fit_against_the_recursive_filter() {
    print!("{:<8} {:>6} {:>9}", "clip", "트리거", "EKF");
    for budget in BUDGETS {
        print!(
            " {:>7}",
            if budget == usize::MAX {
                "전부".to_owned()
            } else {
                format!("{budget}개")
            }
        );
    }
    println!();

    let (mut ekf_sum, mut ekf_n) = (0.0, 0usize);
    let mut totals = vec![(0usize, 0.0_f64); BUDGETS.len()];
    for clip in CLIPS {
        let (ekf, batch, samples) = run(clip);
        if let Some(e) = ekf {
            ekf_sum += e;
            ekf_n += 1;
        }
        print!(
            "{clip:<8} {samples:>6} {:>9}",
            ekf.map_or("-".to_owned(), |e| format!("{:.1}", e * 100.0))
        );
        for (i, error) in batch.iter().enumerate() {
            match error {
                Some(e) => {
                    totals[i].0 += 1;
                    totals[i].1 += e;
                    print!(" {:>7.1}", e * 100.0);
                }
                None => print!(" {:>7}", "-"),
            }
        }
        println!();
    }
    print!(
        "{:<8} {:>6} {:>9}",
        "평균",
        "",
        format!("{:.1}", ekf_sum / ekf_n.max(1) as f64 * 100.0)
    );
    for (count, sum) in &totals {
        print!(" {:>7.1}", sum / (*count).max(1) as f64 * 100.0);
    }
    println!();
    print!("{:<8} {:>6} {:>9}", "표본수", "", format!("{ekf_n}/9"));
    for (count, _) in &totals {
        print!(" {:>7}", format!("{count}/9"));
    }
    println!("\n(접수 평면 타점 오차 cm — 생 삼각측량 기준. 열 = 적합에 쓴 관측 수)");
}
