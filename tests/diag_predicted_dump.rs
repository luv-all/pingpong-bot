//! 트리거 순간의 예측 궤적을 그대로 찍는다 — 그림 대신 숫자로 본다.
//!
//! ```bash
//! CLIP=fly_04 cargo test --release --test diag_predicted_dump -- --ignored --nocapture
//! ```

use std::path::Path;
use std::time::{Duration, Instant};

use pingpong_bot::camera::{self, Calibration, Frame, FrameSource, OpenCvCapture, Triangulate};
use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::vision::triggers::PlaneCrossing;
use pingpong_bot::vision::{Trajectory, Vision};

#[test]
#[ignore = "클립 필요"]
fn dump_the_predicted_track() {
    let name = std::env::var("CLIP").unwrap_or_else(|_| "fly_04".to_owned());
    let dir = Path::new("data/clips").join(&name);
    let text = std::fs::read_to_string(dir.join("meta.json")).expect("meta");
    let meta: serde_json::Value = serde_json::from_str(&text).expect("meta");
    let fps = meta["meas_fps"].as_f64().unwrap_or(30.0);
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
    let mut first: Option<(usize, Trajectory)> = None;
    let mut raw: Vec<(usize, pingpong_bot::Point3)> = Vec::new();

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
            raw.push((index, point));
        }
        if first.is_none()
            && let Some(trajectory) = vision.trajectory()
        {
            first = Some((index, trajectory));
        }
        index += 1;
    }

    let Some((frame, trajectory)) = first else {
        println!("트리거 안 걸림");
        return;
    };
    println!("clip={name} fps={fps:.1}  트리거 frame={frame}");
    println!(
        "테이블: WIDTH_X={:.2} LENGTH_Y={:.2} SURFACE_Z={:.2}",
        table::WIDTH_X,
        table::LENGTH_Y,
        table::SURFACE_Z
    );

    let start = trajectory.predicted.first().expect("예측 시작");
    println!(
        "\n예측 시작   p({:+.3},{:+.3},{:+.3})  v({:+.2},{:+.2},{:+.2})  |v|={:.2}",
        start.position.x,
        start.position.y,
        start.position.z,
        start.velocity.x,
        start.velocity.y,
        start.velocity.z,
        start.velocity.norm()
    );
    println!(
        "  sigma_p ({:.3},{:.3},{:.3})  sigma_v ({:.2},{:.2},{:.2})",
        start.sigma_position.x,
        start.sigma_position.y,
        start.sigma_position.z,
        start.sigma_velocity.x,
        start.sigma_velocity.y,
        start.sigma_velocity.z
    );

    println!("\n생 삼각측량, 트리거 앞뒤 6프레임:");
    for (i, p) in raw.iter().filter(|(i, _)| i.abs_diff(frame) <= 6) {
        println!("  f{i:<4} ({:+.3},{:+.3},{:+.3})", p.x, p.y, p.z);
    }
    // 프레임 간 차분으로 실제 속도를 대충 낸다 — 필터와 견줄 기준.
    let near: Vec<_> = raw.iter().filter(|(i, _)| i.abs_diff(frame) <= 4).collect();
    if near.len() >= 2 {
        let (i0, p0) = near[0];
        let (i1, p1) = near[near.len() - 1];
        let dt = (i1 - i0) as f64 / fps;
        let v = (p1 - p0) / dt;
        println!(
            "생 차분 속도 v({:+.2},{:+.2},{:+.2})  |v|={:.2}  (f{i0}~f{i1})",
            v.x,
            v.y,
            v.z,
            v.norm()
        );
    }

    println!("\n예측 궤적 (25 ms 마다):");
    for state in trajectory.predicted.iter().step_by(5) {
        println!(
            "  t={:>6.3}  p({:+.3},{:+.3},{:+.3})  v({:+.2},{:+.2},{:+.2})",
            state.t.as_secs_f64(),
            state.position.x,
            state.position.y,
            state.position.z,
            state.velocity.x,
            state.velocity.y,
            state.velocity.z
        );
    }
    let last = trajectory.predicted.last().expect("끝");
    println!(
        "  끝 t={:.3}  p({:+.3},{:+.3},{:+.3})  ({}개)",
        last.t.as_secs_f64(),
        last.position.x,
        last.position.y,
        last.position.z,
        trajectory.predicted.len()
    );
}
