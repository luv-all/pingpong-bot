//! 예측을 **언제** 얼릴지에 따라 타점 오차가 어떻게 달라지나.
//!
//! 정답은 필터 밖에서 온다 — 생 삼각측량이 접수 평면을 지난 지점이다. 예측을 필터 자신의
//! measured 와 견주면 필터가 통째로 밀려도 안 보인다.
//!
//! ```bash
//! cargo test --release --test diag_trigger -- --ignored --nocapture
//! ```

use std::path::Path;
use std::time::{Duration, Instant};

use pingpong_bot::camera::{self, Calibration, Frame, FrameSource, OpenCvCapture, Triangulate};
use pingpong_bot::constants::table;
use pingpong_bot::vision::triggers::{Any, PlaneCrossing, SigmaThreshold};
use pingpong_bot::vision::{State, Track, Trigger, Vision};
use pingpong_bot::{Point3, Vector3, defaults};

const CLIPS: [&str; 9] = [
    "fly_01", "fly_02", "fly_03", "fly_04", "fly_05", "fly_06", "fly_07", "fly_08", "fly_09",
];

fn net() -> f64 {
    return table::LENGTH_Y * 0.5;
}

/// 축별 σ 임계. 위치는 넉넉히, 속도는 리드 0.3 s 에 그 오차가 실린다고 보고 잡는다.
fn sigma(position: f64, velocity: f64) -> Box<dyn Trigger> {
    return Box::new(SigmaThreshold {
        position: Vector3::repeat(position),
        velocity: Vector3::repeat(velocity),
    });
}

/// 실기 트리거 — 필터가 좁혀졌거나, 늦어도 네트를 넘으면.
fn default_trigger() -> Box<dyn Trigger> {
    return Box::new(Any(vec![
        sigma(0.05, 0.5),
        Box::new(PlaneCrossing { y: net() }),
    ]));
}

/// 재볼 리드타임 [s] — 실제 접수까지 남은 시간.
const LEADS: [f64; 5] = [0.5, 0.4, 0.3, 0.2, 0.1];

/// 생 삼각측량 점들을 [`Track`]으로 감싼다. 평면 통과는 계약이 이미 푸는 문제다.
fn as_track(raw: &[(Duration, Point3)]) -> Track {
    return Track(
        raw.iter()
            .map(|(t, position)| State {
                t: *t,
                position: *position,
                ..State::default()
            })
            .collect(),
    );
}

/// 클립 하나 — 리드타임별 타점 오차 [m].
///
/// 예측은 트리거 뒤 매 프레임 다시 적분되므로, 프레임마다 그때의 예측이 접수 평면에서
/// 어디를 찍는지 재고 실제 도달까지 남은 시간으로 줄 세운다. 제어가 "지금 커밋하면
/// 얼마나 틀리나"를 묻는 것과 같은 질문이다.
fn run(clip: &str) -> (Option<usize>, Vec<Option<f64>>) {
    let dir = Path::new("data/clips").join(clip);
    let text = std::fs::read_to_string(dir.join("meta.json")).expect("meta.json");
    let meta: serde_json::Value = serde_json::from_str(&text).expect("meta");
    let fps = meta["meas_fps"].as_f64().unwrap_or(30.0);
    let calibration = Calibration::load_json(&defaults::calibration_path()).expect("calibration");
    let mut vision = Vision::load(&calibration, default_trigger()).expect("vision");

    let mut left = OpenCvCapture::from_path(camera::Id(0), &dir.join("left.avi")).expect("left");
    let mut right = OpenCvCapture::from_path(camera::Id(1), &dir.join("right.avi")).expect("right");
    let epoch = Instant::now();
    let (mut index, mut raw): (usize, Vec<(Duration, Point3)>) = (0, Vec::new());
    let mut started: Option<usize> = None;
    // (그 프레임의 시각, 그때 예측이 접수 평면에서 찍은 점)
    let mut guesses: Vec<(f64, Point3)> = Vec::new();
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
        let now = index as f64 / fps;
        if let (Some(a), Some(b)) = (pixels[0], pixels[1])
            && let Some(point) =
                Triangulate::pixels(&[(camera::Id(0), a), (camera::Id(1), b)], &calibration)
        {
            raw.push((Duration::from_secs_f64(now), point));
        }
        if let Some(trajectory) = vision.trajectory() {
            started.get_or_insert(index);
            if let Some(hit) = trajectory.predicted.at_plane(plane) {
                guesses.push((now, hit.position));
            }
        }
        index += 1;
    }

    // 실제로 접수 평면을 지난 시각과 지점 — 필터 밖의 정답.
    let truth = as_track(&raw).at_plane(plane);
    let Some(truth) = truth else {
        return (started, vec![None; LEADS.len()]);
    };
    let impact_t = truth.t.as_secs_f64();
    let errors = LEADS
        .iter()
        .map(|lead| {
            let target = impact_t - lead;
            // 프레임 반 칸 안쪽에서 가장 가까운 예측만 짝으로 인정한다.
            let tolerance = 0.5 / fps;
            return guesses
                .iter()
                .filter(|(t, _)| (t - target).abs() <= tolerance)
                .min_by(|a, b| {
                    (a.0 - target)
                        .abs()
                        .partial_cmp(&(b.0 - target).abs())
                        .expect("유한")
                })
                .map(|(_, guess)| (guess - truth.position).norm());
        })
        .collect();
    return (started, errors);
}

#[test]
#[ignore = "클립 필요: cargo test --release --test diag_trigger -- --ignored --nocapture"]
fn prediction_error_by_lead_time() {
    print!("{:<8} {:>7}", "clip", "시작");
    for lead in LEADS {
        print!(" {:>7}", format!("{lead:.1}s"));
    }
    println!();

    let mut totals = vec![(0usize, 0.0_f64); LEADS.len()];
    for clip in CLIPS {
        let (started, errors) = run(clip);
        print!(
            "{clip:<8} {:>7}",
            started.map_or("없음".to_owned(), |f| format!("f{f}"))
        );
        for (i, error) in errors.iter().enumerate() {
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
    print!("{:<8} {:>7}", "평균", "");
    for (count, sum) in &totals {
        if *count == 0 {
            print!(" {:>7}", "-");
        } else {
            print!(" {:>7.1}", sum / *count as f64 * 100.0);
        }
    }
    println!();
    print!("{:<8} {:>7}", "표본", "");
    for (count, _) in &totals {
        print!(" {:>7}", format!("{count}/9"));
    }
    println!("\n(접수 평면 타점 오차 cm — 생 삼각측량 기준. 열 = 실제 도달까지 남은 시간)");
}
