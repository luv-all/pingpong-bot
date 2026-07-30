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
//! CLIP=fly_02 cargo test --release --test diag_clip_detection -- --ignored --nocapture
//! ```

use std::path::PathBuf;

use pingpong_bot::camera::{self, FrameSource, OpenCvCapture};
use pingpong_bot::defaults;
use pingpong_bot::detector::{
    ColormaskDetector, ColormaskParams, ContourDetector, Detector, FloorEdgeMask, RoiParams,
    Scorer, ScorerParams,
};

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

/// 검출 단계별 비용 [ms/frame] — ROI 켬/끔.
///
/// 클립 fps(34~41)는 `record_stereo`(검출 없음)의 캡처+인코딩 한계다. 라이브는 인코딩이
/// 없으니 검출이 얼마나 비싼지가 실제 처리율을 정한다.
fn time_detection(path: &PathBuf, camera_id: camera::Id) {
    let params = defaults::camera_params_for(camera_id).expect("camera_params_for");
    let needs_undistort = !params.dist.is_empty();

    for (label, roi) in [("ROI 켬", true), ("ROI 끔", false)] {
        let mut source = OpenCvCapture::from_path(camera_id, path).expect("클립 열기");
        let mut detector = defaults::detector_for(camera_id).expect("detector_for");
        detector.set_roi_enabled(roi);

        let (mut detect_ns, mut undistort_ns, mut frames) = (0_u128, 0_u128, 0_u32);
        while let Some(frame) = source.next_frame() {
            let frame = if needs_undistort {
                let start = std::time::Instant::now();
                let out = Detector::undistort(&frame, &params).expect("undistort");
                undistort_ns += start.elapsed().as_nanos();
                out
            } else {
                frame
            };
            let start = std::time::Instant::now();
            let _ = detector.detect(&frame);
            detect_ns += start.elapsed().as_nanos();
            frames += 1;
        }
        let per = |ns: u128| ns as f64 / frames as f64 / 1e6;
        println!(
            "  cam{} {label}  검출 {:.2} ms/f  (undistort {:.2})  → 캠당 최대 {:.0} fps",
            camera_id.0,
            per(detect_ns),
            per(undistort_ns),
            1000.0 / per(detect_ns).max(1e-6)
        );
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
    let name = std::env::var("CLIP").unwrap_or_else(|_| "fly_01".to_owned());
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

    println!("검출 비용:");
    time_detection(&dir.join("left.avi"), camera::Id(0));
    time_detection(&dir.join("right.avi"), camera::Id(1));

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

/// `defaults::detector_for`와 같은 조립이되 colormask·ROI를 바꿔 끼울 수 있게 한 것.
///
/// `defaults`의 조립 함수는 비공개라 여기서 같은 순서로 다시 쌓는다 —
/// mask → colormask → contour → scorer + ROI.
fn detector_with(camera_id: camera::Id, color: &ColormaskParams, roi: bool) -> Detector {
    return detector_tuned(
        camera_id,
        color,
        roi,
        ScorerParams::default().min_circularity,
    );
}

/// [`detector_with`] + 원형도 하한도 바꾼다.
///
/// 모션 블러가 걸린 공은 길게 번져 원형도가 떨어진다 — colormask와 **별개 게이트**라
/// 따로 재야 어느 쪽이 막고 있는지 안다.
fn detector_tuned(
    camera_id: camera::Id,
    color: &ColormaskParams,
    roi: bool,
    min_circularity: f64,
) -> Detector {
    let cam = defaults::camera_params_for(camera_id).expect("camera_params_for");
    let scorer = ScorerParams::from_calib(&cam, min_circularity).expect("scorer from calib");
    let mut detector = Detector::builder()
        .mask(FloorEdgeMask::from_params(&cam).expect("floor mask"))
        .then(ColormaskDetector::new(color.clone()))
        .then(ContourDetector::from(&scorer))
        .scorer(Scorer::from(&scorer).with_motion_weight(defaults::MOTION_WEIGHT))
        .roi(RoiParams::default())
        .build()
        .expect("detector");
    detector.set_roi_enabled(roi);
    return detector;
}

/// `(구간 안 검출 수, 구간 **밖** 검출 수)`.
///
/// 구간 밖은 공이 아직 없거나(preroll) 이미 착지한 뒤라, 거기서 잡히는 건 대부분
/// 오검출이다 — 임계를 풀 때 같이 봐야 할 대가.
fn count_in_window(
    path: &PathBuf,
    camera_id: camera::Id,
    mut detector: Detector,
    window: (usize, usize),
) -> (usize, usize) {
    let mut source = OpenCvCapture::from_path(camera_id, path).expect("클립 열기");
    let params = defaults::camera_params_for(camera_id).expect("camera_params_for");
    let needs_undistort = !params.dist.is_empty();
    let (mut inside, mut outside) = (0_usize, 0_usize);
    let mut index = 0_usize;
    while let Some(frame) = source.next_frame() {
        let frame = if needs_undistort {
            Detector::undistort(&frame, &params).expect("undistort")
        } else {
            frame
        };
        if detector.detect(&frame).is_some() {
            if (window.0..=window.1).contains(&index) {
                inside += 1;
            } else {
                outside += 1;
            }
        }
        index += 1;
    }
    return (inside, outside);
}

/// cam0이 비행 중 61%를 놓치는 원인을 좁힌다 — ROI 유실인가, colormask 임계인가.
///
/// ROI를 끄면 크게 오르면 추적 유실(한 번 놓치면 좁아진 ROI 밖으로 공이 나가 복구가 안 됨),
/// 임계를 풀 때만 오르면 colormask 문제다.
/// 두 캠 검출 구간의 교집합 = 공이 실제로 날아간 구간.
///
/// 총 검출 수는 착지해 정지한 공이나 오검출까지 세므로 그것만 보면 병목을 잘못 짚는다
/// (fly_02: cam0 총 49회 중 40회가 착지 후였다).
fn flight_window(dir: &std::path::Path) -> Option<(usize, usize)> {
    let left = detect_side(&dir.join("left.avi"), camera::Id(0));
    let right = detect_side(&dir.join("right.avi"), camera::Id(1));
    let ((l0, l1), (r0, r1)) = (left.span()?, right.span()?);
    let (start, end) = (l0.max(r0), l1.min(r1));
    return (start <= end).then_some((start, end));
}

#[test]
#[ignore = "순수 진단(클립 필요). 실행: cargo test --release --test diag_clip_detection -- --ignored --nocapture"]
fn diag_clip_detection_sweep() {
    let name = std::env::var("CLIP").unwrap_or_else(|_| "fly_01".to_owned());
    let dir = PathBuf::from(defaults::DEFAULT_CLIPS_DIR).join(&name);
    assert!(dir.is_dir(), "클립 없음: {}", dir.display());

    // 비행 구간은 두 캠 검출 구간의 교집합으로 **클립마다 자동으로** 잡는다.
    // 하드코딩하면 클립이 바뀔 때 조용히 엉뚱한 창을 재게 된다.
    let window = flight_window(&dir).expect("두 캠 모두 검출된 구간이 없다");
    let span = window.1 - window.0 + 1;
    println!(
        "클립 {name} — 비행 구간 {}~{} ({span}프레임)",
        window.0, window.1
    );

    for (camera_id, file) in [(camera::Id(0), "left.avi"), (camera::Id(1), "right.avi")] {
        let path = dir.join(file);
        let base = defaults::colormask_for(camera_id).expect("colormask");
        println!(
            "\ncam{}  기준 colormask c0 {}~{} c1 {}~{} c2 {}~{}",
            camera_id.0,
            base.c0_min,
            base.c0_max,
            base.c1_min,
            base.c1_max,
            base.c2_min,
            base.c2_max
        );

        for (label, roi) in [("ROI 켬", true), ("ROI 끔", false)] {
            let (hits, outside) = count_in_window(
                &path,
                camera_id,
                detector_with(camera_id, &base, roi),
                window,
            );
            println!(
                "  {label:<8} {hits:>3}/{span} ({:>3.0}%)   구간밖 {outside}",
                hits as f64 / span as f64 * 100.0
            );
        }

        // 원형도만 풀어본다 — 색은 그대로. 블러로 늘어난 공이 여기서 막히는지.
        for circularity in [0.45_f64, 0.35, 0.25] {
            let (hits, outside) = count_in_window(
                &path,
                camera_id,
                detector_tuned(camera_id, &base, false, circularity),
                window,
            );
            println!(
                "  원형도 ≥{circularity:.2}          {hits:>3}/{span} ({:>3.0}%)   구간밖 {outside}",
                hits as f64 / span as f64 * 100.0
            );
        }

        // 채도·명도 하한을 풀어본다 (공은 밝고 채도 높은 주황이라 하한이 주 게이트).
        for relax in [10_u8, 25, 40] {
            let loosened = ColormaskParams {
                c1_min: base.c1_min.saturating_sub(relax),
                c2_min: base.c2_min.saturating_sub(relax),
                ..base.clone()
            };
            let (hits, outside) = count_in_window(
                &path,
                camera_id,
                detector_with(camera_id, &loosened, false),
                window,
            );
            println!(
                "  하한 -{relax:<3} (c1≥{:>3} c2≥{:>3})  {hits:>3}/{span} ({:>3.0}%)   구간밖 {outside}",
                loosened.c1_min,
                loosened.c2_min,
                hits as f64 / span as f64 * 100.0
            );
        }
    }
}
