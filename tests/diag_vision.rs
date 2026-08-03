//! 새 `vision` 파이프라인을 클립으로 돌려 검출률과 예측 오차를 낸다.
//!
//! ```bash
//! cargo test --release --test diag_vision -- --ignored --nocapture
//! CLIP=fly_02 cargo test --release --test diag_vision -- --ignored --nocapture
//! ```

use std::path::Path;

use std::time::{Duration, Instant};

use pingpong_bot::camera::{self, Calibration, Frame, FrameSource, OpenCvCapture};
use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::vision::triggers::PlaneCrossing;
use pingpong_bot::vision::{Trajectory, Vision};

fn clip() -> String {
    return std::env::var("CLIP").unwrap_or_else(|_| "fly_04".to_owned());
}

fn meas_fps(dir: &Path) -> f64 {
    let text = std::fs::read_to_string(dir.join("meta.json")).expect("meta.json");
    let json: serde_json::Value = serde_json::from_str(&text).expect("meta 파싱");
    return json["meas_fps"].as_f64().unwrap_or(30.0);
}

#[test]
#[ignore = "클립 필요: cargo test --release --test diag_vision -- --ignored --nocapture"]
fn vision_pipeline_on_a_clip() {
    let name = clip();
    let dir = Path::new("data/clips").join(&name);
    let fps = meas_fps(&dir);
    let calibration = Calibration::load_json(&defaults::calibration_path()).expect("calibration");

    let mut vision = Vision::load(
        &calibration,
        Box::new(PlaneCrossing {
            y: table::LENGTH_Y * 0.5,
        }),
    )
    .expect("vision");

    let mut left = OpenCvCapture::from_path(camera::Id(0), &dir.join("left.avi")).expect("left");
    let mut right = OpenCvCapture::from_path(camera::Id(1), &dir.join("right.avi")).expect("right");

    let mut frames = 0usize;
    let mut first_track: Option<usize> = None;
    let mut committed: Option<(usize, Trajectory)> = None;
    let mut last: Option<Trajectory> = None;
    let mut detected = [0usize; 2];
    // clip-review 기준 fly_04 비행 구간. 이 밖의 검출은 오검출이다.
    let flight = 380..=460;
    let mut in_flight = [0usize; 2];
    let (mut accepted, mut rejected, mut idle, mut seeded) = (0, 0, 0, 0);

    // 두 캡처가 각자 epoch 를 잡으면 타임스탬프가 어긋난다. 클립은 동시 녹화라
    // 같은 인덱스가 같은 시각이므로 공유 epoch 로 다시 찍는다.
    let epoch = Instant::now();

    loop {
        let (Some(l), Some(r)) = (left.next_frame(), right.next_frame()) else {
            break;
        };
        let at = epoch + Duration::from_secs_f64(frames as f64 / fps);
        let l = Frame::new(camera::Id(0), l.image, at);
        let r = Frame::new(camera::Id(1), r.image, at);
        for (slot, frame) in [(0usize, &l), (1usize, &r)] {
            let before = vision.ekf().has_track();
            let got = vision.feed(frame).expect("feed");
            let _ = (slot, before);
            if !before && vision.ekf().has_track() {
                seeded += 1;
                if first_track.is_none() {
                    first_track = Some(frames);
                }
            }
            match vision.last_outcome() {
                Some(pingpong_bot::vision::Outcome::Accepted) => accepted += 1,
                Some(pingpong_bot::vision::Outcome::Rejected { .. }) => rejected += 1,
                Some(pingpong_bot::vision::Outcome::Idle) => idle += 1,
                _ => {}
            }
            if vision.last_found().is_some() {
                detected[slot] += 1;
                if flight.contains(&frames) {
                    in_flight[slot] += 1;
                }
            }
            if let Some(trajectory) = got {
                if committed.is_none() {
                    committed = Some((frames, trajectory.clone()));
                }
                last = Some(trajectory);
            }
        }
        frames += 1;
    }

    for params in &calibration.cameras {
        let volume = pingpong_bot::vision::detect::Volume::from_calib(params).expect("volume");
        println!(
            "cam{} 부피 keep {:.0}%",
            params.camera_id.0,
            volume.keep_ratio().expect("ratio")
        );
    }
    println!("clip={name} frames={frames} fps={fps:.1}");
    println!(
        "검출 cam0={} cam1={}  ({:.0}% / {:.0}%)",
        detected[0],
        detected[1],
        100.0 * detected[0] as f64 / frames as f64,
        100.0 * detected[1] as f64 / frames as f64
    );
    let span = flight.end() - flight.start() + 1;
    println!(
        "  비행구간({}~{}) cam0={}/{span} cam1={}/{span}   구간밖(오검출) cam0={} cam1={}",
        flight.start(),
        flight.end(),
        in_flight[0],
        in_flight[1],
        detected[0] - in_flight[0],
        detected[1] - in_flight[1]
    );
    println!("시드 {seeded}회  accepted {accepted}  rejected {rejected}  idle {idle}");
    println!(
        "첫 트랙 frame={}  트리거 frame={}",
        first_track.map_or("없음".to_owned(), |f| f.to_string()),
        committed
            .as_ref()
            .map_or("없음".to_owned(), |(f, _)| f.to_string())
    );

    let Some((_, trajectory)) = committed else {
        println!(
            "트리거가 안 걸렸다 — 관측 {}개",
            vision.ekf().measured().len()
        );
        return;
    };
    let full = last.expect("트리거가 걸렸으면 마지막도 있다");
    println!(
        "measured {}개  predicted {}개",
        full.measured.len(),
        trajectory.predicted.len()
    );

    let plane = table::DEFAULT_HIT_PLANE_Y;
    match (
        trajectory.predicted.at_plane(plane),
        full.measured.at_plane(plane),
    ) {
        (Some(pred), Some(real)) => {
            let miss = (pred.position - real.position).norm();
            println!(
                "at y={plane:.2}  pred x{:+.2} z{:+.2}  real x{:+.2} z{:+.2}  MISS {:.1}cm",
                pred.position.x,
                pred.position.z,
                real.position.x,
                real.position.z,
                miss * 100.0
            );
            println!(
                "  축별 |dx|{:.1} |dy|{:.1} |dz|{:.1} cm",
                (pred.position.x - real.position.x).abs() * 100.0,
                (pred.position.y - real.position.y).abs() * 100.0,
                (pred.position.z - real.position.z).abs() * 100.0,
            );
        }
        (pred, real) => println!(
            "평면 통과 없음 — pred={} real={}",
            pred.is_some(),
            real.is_some()
        ),
    }
}

/// 필터 상태가 생 삼각측량에서 얼마나 떨어져 있나 — 프레임마다.
///
/// [`vision_pipeline_on_a_clip`]의 MISS 는 예측을 **필터 자신의** measured 와 견주므로
/// 필터가 통째로 밀려 있으면 그걸 못 본다. 여기서는 필터 밖의 값과 견준다.
#[test]
#[ignore = "클립 필요: cargo test --release --test diag_vision -- --ignored --nocapture"]
fn filtered_state_against_raw_triangulation() {
    use pingpong_bot::camera::Triangulate;

    let name = clip();
    let dir = Path::new("data/clips").join(&name);
    let fps = meas_fps(&dir);
    let calibration = Calibration::load_json(&defaults::calibration_path()).expect("calibration");
    let mut vision = Vision::load(
        &calibration,
        Box::new(PlaneCrossing {
            y: table::LENGTH_Y * 0.5,
        }),
    )
    .expect("vision");

    let mut left = OpenCvCapture::from_path(camera::Id(0), &dir.join("left.avi")).expect("left");
    let mut right = OpenCvCapture::from_path(camera::Id(1), &dir.join("right.avi")).expect("right");
    let epoch = Instant::now();
    let mut index = 0usize;
    let mut rows: Vec<Row> = Vec::new();

    loop {
        let (Some(l), Some(r)) = (left.next_frame(), right.next_frame()) else {
            break;
        };
        let at = epoch + Duration::from_secs_f64(index as f64 / fps);
        let l = Frame::new(camera::Id(0), l.image, at);
        let r = Frame::new(camera::Id(1), r.image, at);

        let mut pixels = [None, None];
        let mut marks = [' ', ' '];
        for (slot, frame) in [(0usize, &l), (1usize, &r)] {
            vision.feed(frame).expect("feed");
            pixels[slot] = vision.last_found().map(|c| c.pixel);
            marks[slot] = match vision.last_outcome() {
                Some(pingpong_bot::vision::Outcome::Accepted) => 'o',
                Some(pingpong_bot::vision::Outcome::Rejected { .. }) => 'x',
                Some(pingpong_bot::vision::Outcome::Seeded) => 'S',
                _ => '-',
            };
        }

        if let (Some(a), Some(b)) = (pixels[0], pixels[1])
            && let Some(raw) =
                Triangulate::pixels(&[(camera::Id(0), a), (camera::Id(1), b)], &calibration)
            && let Some(state) = vision.ekf().measured().last()
        {
            let d = state.position - raw;
            rows.push(Row {
                frame: index,
                seq: vision.ekf().seq(),
                marks,
                raw,
                gap: d,
                sigma_position: state.sigma_position,
                sigma_velocity: state.sigma_velocity,
                speed: state.velocity.norm(),
            });
        }
        index += 1;
    }

    if rows.is_empty() {
        println!("두 캠이 같이 잡은 프레임이 없다");
        return;
    }
    println!("clip={name}  두 캠 동시 검출 {}프레임", rows.len());
    println!("o=보정 x=거부 S=시드 -=없음");
    println!(
        "frame seq c0c1 | raw x    y    z  | 필터-raw dx dy dz [cm] | sigma_p [cm] | sigma_v |v|"
    );
    for r in &rows {
        println!(
            "{:<5} {:<3} {}{}   | {:+5.2} {:+5.2} {:+5.2} | {:+6.1} {:+6.1} {:+6.1} = {:5.1} | \
             {:4.0}{:4.0}{:4.0} | {:4.1}{:4.1}{:4.1} {:4.1}",
            r.frame,
            r.seq,
            r.marks[0],
            r.marks[1],
            r.raw.x,
            r.raw.y,
            r.raw.z,
            r.gap.x * 100.0,
            r.gap.y * 100.0,
            r.gap.z * 100.0,
            r.gap.norm() * 100.0,
            r.sigma_position.x * 100.0,
            r.sigma_position.y * 100.0,
            r.sigma_position.z * 100.0,
            r.sigma_velocity.x,
            r.sigma_velocity.y,
            r.sigma_velocity.z,
            r.speed,
        );
    }

    // raw 자체가 얼마나 튀는지 — 필터를 탓하기 전에 입력의 잡음을 먼저 안다.
    // 2계 차분은 매끄러운 성분(중력·항력)을 지우고 프레임 단위 흔들림만 남긴다.
    let jitter: Vec<f64> = rows
        .windows(3)
        .filter(|w| w[2].frame - w[0].frame == 2)
        .map(|w| (w[0].raw.coords - 2.0 * w[1].raw.coords + w[2].raw.coords).norm() / 4.0)
        .collect();
    if !jitter.is_empty() {
        let mut sorted = jitter.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).expect("유한"));
        println!(
            "raw 잡음 (2계 차분/4) p50 {:.1} cm  p95 {:.1} cm",
            sorted[sorted.len() / 2] * 100.0,
            sorted[sorted.len() * 95 / 100] * 100.0
        );
    }
    let mean = |f: fn(&Row) -> f64| rows.iter().map(f).sum::<f64>() / rows.len() as f64;
    println!(
        "평균 편차 dx{:+.1} dy{:+.1} dz{:+.1} cm — 부호가 한쪽으로 쏠리면 계통 오차다",
        mean(|r| r.gap.x) * 100.0,
        mean(|r| r.gap.y) * 100.0,
        mean(|r| r.gap.z) * 100.0
    );
    println!(
        "트랙 교체 {}회 (seq 증가)",
        rows.last().expect("비지 않음").seq
    );
}

struct Row {
    frame: usize,
    seq: u64,
    marks: [char; 2],
    raw: pingpong_bot::Point3,
    gap: pingpong_bot::Vector3,
    sigma_position: pingpong_bot::Vector3,
    sigma_velocity: pingpong_bot::Vector3,
    speed: f64,
}
