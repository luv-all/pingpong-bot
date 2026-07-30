//! 클립에서 **카메라별 검출이 어느 프레임에 성공했는지** 재는 진단.
//!
//! 스테레오 삼각측량은 두 캠이 **같은 시각**에 잡아야 성립한다. 한쪽 검출률이 낮으면
//! 3D 점이 그만큼 안 나오고, EKF에 들어가는 측정 수가 무너져 속도 추정이 깨진다
//! (fly_02 실측: cam0 49회 / cam1 21회 → triangulated 14, accepted 4).
//!
//! 이 진단은 "왜 안 잡히나"를 좁힌다:
//! - 검출 구간이 **연속인데 한쪽만 짧다** → 시야·기하 문제 (공이 화각을 벗어남)
//! - 구간은 같은데 **드문드문 빠진다** → 임계값·노이즈 문제 (colormask·scorer)
//!
//! ```bash
//! cargo test --release --test diag_clip_detection -- --ignored --nocapture
//! CLIP=fly_01 cargo test --release --test diag_clip_detection -- --ignored --nocapture
//! ```

use std::path::PathBuf;

use pingpong_bot::camera::{self, FrameSource, OpenCvCapture};
use pingpong_bot::defaults;
use pingpong_bot::detector::Detector;

/// 카메라 한 대의 프레임별 검출 결과.
struct Detected {
    camera_id: camera::Id,
    /// 검출에 성공한 프레임 인덱스.
    frames: Vec<usize>,
    total_frames: usize,
}

impl Detected {
    fn span(&self) -> Option<(usize, usize)> {
        return Some((*self.frames.first()?, *self.frames.last()?));
    }

    /// 검출 구간 안에서의 성공률 — 전체 프레임 대비가 아니라 **공이 있는 동안**의 비율.
    ///
    /// 클립은 preroll·postroll이 길어서 전체 대비 비율은 항상 낮게 나온다.
    fn rate_within_span(&self) -> f64 {
        let Some((first, last)) = self.span() else {
            return 0.0;
        };
        let window = last - first + 1;
        return self.frames.len() as f64 / window as f64;
    }
}

fn detect_side(path: &PathBuf, camera_id: camera::Id) -> Detected {
    let mut source = OpenCvCapture::from_path(camera_id, path).expect("클립 열기");
    let params = defaults::camera_params_for(camera_id).expect("camera_params_for");
    let mut detector = defaults::detector_for(camera_id).expect("detector_for");
    let needs_undistort = !params.dist.is_empty();

    let mut frames = Vec::new();
    let mut index = 0_usize;
    while let Some(frame) = source.next_frame() {
        let frame = if needs_undistort {
            Detector::undistort(&frame, &params).expect("undistort")
        } else {
            frame
        };
        if detector.detect(&frame).is_some() {
            frames.push(index);
        }
        index += 1;
    }
    return Detected {
        camera_id,
        frames,
        total_frames: index,
    };
}

#[test]
#[ignore = "순수 진단(클립 필요). 실행: cargo test --release --test diag_clip_detection -- --ignored --nocapture"]
fn diag_clip_detection_per_camera() {
    let name = std::env::var("CLIP").unwrap_or_else(|_| "fly_02".to_owned());
    let dir = PathBuf::from(defaults::DEFAULT_CLIPS_DIR).join(&name);
    assert!(dir.is_dir(), "클립 없음: {}", dir.display());

    let left = detect_side(&dir.join("left.avi"), camera::Id(0));
    let right = detect_side(&dir.join("right.avi"), camera::Id(1));

    for side in [&left, &right] {
        let span = side
            .span()
            .map(|(a, b)| format!("{a}~{b} ({}프레임)", b - a + 1))
            .unwrap_or_else(|| "없음".to_owned());
        println!(
            "cam{}  검출 {:>4}/{:<4}  구간 {span}  구간내 성공률 {:.0}%",
            side.camera_id.0,
            side.frames.len(),
            side.total_frames,
            side.rate_within_span() * 100.0
        );
    }

    // 스테레오가 실제로 성립하는 프레임 — 이게 EKF가 받을 수 있는 측정의 상한이다.
    let both: Vec<usize> = left
        .frames
        .iter()
        .filter(|index| right.frames.contains(index))
        .copied()
        .collect();
    println!(
        "동시 검출 {}프레임 — 삼각측량 상한 (cam0 {} · cam1 {})",
        both.len(),
        left.frames.len(),
        right.frames.len()
    );

    // 한쪽만 잡은 프레임이 구간 안에 흩어져 있으면 임계값 문제, 한쪽 끝에 몰려 있으면 시야 문제.
    let only_left: Vec<usize> = left
        .frames
        .iter()
        .filter(|index| !right.frames.contains(index))
        .copied()
        .collect();
    let only_right: Vec<usize> = right
        .frames
        .iter()
        .filter(|index| !left.frames.contains(index))
        .copied()
        .collect();
    println!("cam0만: {only_left:?}");
    println!("cam1만: {only_right:?}");
    println!("cam0 전체: {:?}", left.frames);
    println!("cam1 전체: {:?}", right.frames);

    // 실제 비행 구간 = 두 캠 검출 구간의 **교집합**. 총 검출 수는 착지 후 정지한 공이나
    // 오검출까지 세므로, 비행 중 성능은 이 창 안에서만 봐야 한다.
    if let (Some((l0, l1)), Some((r0, r1))) = (left.span(), right.span()) {
        let (start, end) = (l0.max(r0), l1.min(r1));
        if start <= end {
            let window = end - start + 1;
            let in_window =
                |frames: &[usize]| frames.iter().filter(|i| (start..=end).contains(i)).count();
            let (c0, c1) = (in_window(&left.frames), in_window(&right.frames));
            let both_in = both.iter().filter(|i| (start..=end).contains(i)).count();
            println!(
                "겹치는 구간 {start}~{end} ({window}프레임): cam0 {c0} ({:.0}%) · cam1 {c1} ({:.0}%) · 동시 {both_in} ({:.0}%)",
                c0 as f64 / window as f64 * 100.0,
                c1 as f64 / window as f64 * 100.0,
                both_in as f64 / window as f64 * 100.0,
            );
        }
    }
}
