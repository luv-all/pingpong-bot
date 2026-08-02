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
            if vision.last_detected() {
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
