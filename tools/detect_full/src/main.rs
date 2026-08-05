//! `vision` 검출 캐스케이드 단계 뷰어.
//!
//! 패널은 `raw` + 레이어마다 하나 + 최종 후보다. **레이어 구성을 하드코딩하지 않으므로**
//! 새 레이어를 꽂아도 이 툴은 안 고친다 — [`Detector::trace`]가 이름과 마스크를 준다.
//!
//! 키: `Space` 일시정지 · `q`/ESC 종료
//!
//! 패널 라벨은 **ASCII 만** 쓴다. OpenCV Hershey 폰트가 유니코드를 못 그려서 한글을 넣으면
//! `??????` 로 나온다.

mod cli;

use anyhow::{Context, Result, bail};
use clap::Parser;
use opencv::core::{Mat, Scalar};
use opencv::prelude::*;
use pingpong_bot::camera::{self, Calibration, Preview, PreviewAction};
use pingpong_bot::defaults;
use pingpong_bot::vision::detect::{self, Detector, Layer};

use cli::Args;

/// 그리드 최대 열 수. 이보다 넓게 늘어놓으면 패널이 작아 못 본다.
const MAX_COLS: usize = 3;

const WHITE: Scalar = Scalar::new(255.0, 255.0, 255.0, 0.0);
const GREEN: Scalar = Scalar::new(0.0, 255.0, 0.0, 0.0);
const YELLOW: Scalar = Scalar::new(0.0, 255.0, 255.0, 0.0);
/// 마스크 패널 라벨. 원본이 깔려 있어 흰 글씨가 밝은 곳에서 묻힌다.
const LABEL_ON_MASK: Scalar = Scalar::new(255.0, 0.0, 255.0, 0.0);
/// 마스크가 지운 곳의 밝기. 0 이면 경계만 보이고 장면이 안 보인다.
const DIMMED: f64 = 0.22;

/// 본선과 같은 조립. 여기가 유일한 SSOT 라 툴과 런타임이 갈릴 수 없다.
fn detector_for(params: &camera::Params) -> Result<Detector> {
    let layers: Vec<Box<dyn Layer>> = vec![
        Box::new(detect::Background::new(
            detect::background::HISTORY,
            detect::background::VAR_THRESHOLD,
            detect::background::SCALE,
            detect::background::LEARNING_RATE,
        )?),
        Box::new(detect::ColorBox::load(params.camera_id)?),
    ];
    return Ok(Detector::new(
        layers,
        detect::Picker::from_calib(params, detect::MIN_CIRCULARITY)?,
    ));
}

/// 패널 개수에 맞는 열 수 — 최대한 정사각에 가깝게, [`MAX_COLS`] 이하로.
///
/// 가로로만 붙이면 패널마다 폭이 1/N 이라 못 본다. 그렇다고 항상 최대 열을 쓰면 4장일 때
/// 3+1 로 어긋난다. `ceil(sqrt(n))` 이면 4장은 2x2, 5~6장은 3x2, 9장은 3x3 이 된다.
fn columns_for(count: usize) -> usize {
    let square = (count as f64).sqrt().ceil() as usize;
    return square.clamp(1, MAX_COLS).min(count.max(1));
}

/// 패널을 그리드로 쌓는다.
///
/// 레이어 개수는 조립에 따라 변하므로 열 수를 세어서 정하고 줄을 늘린다. 마지막 줄이
/// 모자라면 검은 패널로 채운다 — 크기가 다르면 `vconcat` 이 거부한다.
fn grid_bgr(panels: &[Mat]) -> Result<Mat> {
    let cols = columns_for(panels.len());
    let first = panels.first().context("패널이 하나는 있어야 한다")?;
    let blank =
        Mat::new_rows_cols_with_default(first.rows(), first.cols(), first.typ(), Scalar::all(0.0))?;

    let mut rows = opencv::core::Vector::<Mat>::new();
    for chunk in panels.chunks(cols) {
        let mut line: Vec<Mat> = chunk.to_vec();
        while line.len() < cols {
            line.push(blank.try_clone()?);
        }
        rows.push(Preview::hstack_bgr(&line)?);
    }
    let mut out = Mat::default();
    opencv::core::vconcat(&rows, &mut out)?;
    return Ok(out);
}

/// 마스크가 살린 곳은 원본 그대로, 죽인 곳은 어둡게.
///
/// 흑백 마스크만 보면 모양은 보여도 **거기에 뭐가 있는지**는 안 보인다. 부피 레이어가
/// 정말 탁구대 위만 남겼는지 같은 건 원본을 깔아야 판정된다. 지운 쪽을 0 이 아니라
/// [`DIMMED`] 로 두는 것도 같은 이유다 — 경계를 장면 안에서 봐야 한다.
fn as_panel(frame: &Mat, mask: &Mat) -> Result<Mat> {
    let mut dim = Mat::default();
    frame.convert_to(&mut dim, -1, DIMMED, 0.0)?;
    // 마스크가 켜진 화소만 원본으로 덮는다.
    frame.copy_to_masked(&mut dim, mask)?;
    return Ok(dim);
}

fn main() -> Result<()> {
    let args = Args::parse();
    if let Some(dir) = &args.output {
        std::fs::create_dir_all(dir).ok();
    }

    let cam_id = args.cam.camera_id().map_err(anyhow::Error::msg)?;
    let mut source = args
        .cam
        .open_mono_input(&args.offline)
        .map_err(anyhow::Error::msg)?;

    let calibration =
        Calibration::load_json(&defaults::calibration_path()).map_err(anyhow::Error::msg)?;
    let Some(params) = calibration.params(cam_id).cloned() else {
        bail!("calibration 에 cam{} 없음", cam_id.0);
    };
    let mut detector = detector_for(&params)?;

    let window = "detect:full";
    let wait_ms = args
        .wait_ms
        .unwrap_or(if args.offline.has_offline() { 33 } else { 1 });
    let mut paused = false;
    let mut frames = 0usize;
    let mut hits = 0usize;

    while let Some(frame) = source.next_frame() {
        let (stages, candidates) = detector.trace(&frame, None)?;
        let chosen = candidates
            .iter()
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        if chosen.is_some() {
            hits += 1;
        }

        let mut panels = vec![frame.image.try_clone()?];
        let mut labels = vec!["raw".to_owned()];
        for (name, mask) in &stages {
            let on = opencv::core::count_non_zero(mask)?;
            panels.push(as_panel(&frame.image, mask)?);
            labels.push(format!("{name} on={on}"));
        }

        // 마지막 패널에 후보를 얹는다 — 무엇을 골랐고 무엇을 버렸는지.
        let mut picked = panels.last().expect("최소 raw").try_clone()?;
        for (candidate, deviation) in &candidates {
            let color = if Some(candidate) == chosen.map(|(c, _)| c) {
                GREEN
            } else {
                YELLOW
            };
            Preview::draw_circle_px(&mut picked, candidate.pixel, 12, color, 2)?;
            let _ = deviation;
        }
        panels.push(picked);
        labels.push(format!("picked cand={}", candidates.len()));

        // 0 번은 손대지 않은 원본이라 흰 글씨로 구분한다.
        for (index, (panel, label)) in panels.iter_mut().zip(&labels).enumerate() {
            let color = if index == 0 { WHITE } else { LABEL_ON_MASK };
            Preview::draw_cam_label(panel, label, color)?;
        }
        let mosaic = grid_bgr(&panels)?;
        let shown = Preview::fit_bgr_downscale(
            &mosaic,
            (f64::from(mosaic.cols()) * args.scale).round() as i32,
            (f64::from(mosaic.rows()) * args.scale).round() as i32,
        )?;

        if let Some(dir) = &args.output {
            let out = dir.join(format!("full_{frames:04}.png"));
            opencv::imgcodecs::imwrite(
                out.to_str().context("out path")?,
                &shown.image,
                &opencv::core::Vector::new(),
            )?;
        }

        if !args.no_preview {
            let wait = if paused { 0 } else { wait_ms };
            match Preview::show_bgr(window, &shown.image, wait)?.action {
                PreviewAction::Quit => break,
                PreviewAction::Key(key) if key == i32::from(b' ') => paused = !paused,
                _ => {}
            }
        }

        frames += 1;
        if frames >= args.max_frames {
            break;
        }
    }

    if !args.no_preview {
        Preview::destroy_window(window);
    }
    println!(
        "frames={frames} hits={hits} ({:.0}%)",
        100.0 * hits as f64 / frames.max(1) as f64
    );
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn panel(w: i32, h: i32) -> Mat {
        return Mat::new_rows_cols_with_default(h, w, opencv::core::CV_8UC3, Scalar::all(7.0))
            .expect("panel");
    }

    /// 레이어가 몇 개든 열은 상한을 안 넘고 최대한 정사각이다.
    #[test]
    fn the_grid_stays_square_under_the_column_cap() {
        // (패널 수, 기대 열 수) — 4장이 2x2 가 되는 게 이 규칙의 요점이다.
        for (count, want) in [
            (1, 1),
            (2, 2),
            (3, 2),
            (4, 2),
            (5, 3),
            (6, 3),
            (9, 3),
            (12, 3),
        ] {
            assert_eq!(columns_for(count), want, "패널 {count}장");
        }

        let (w, h) = (40, 30);
        for count in 1..=12usize {
            let panels: Vec<Mat> = (0..count).map(|_| panel(w, h)).collect();
            let grid = grid_bgr(&panels).expect("grid");
            let cols = columns_for(count);
            assert!(cols <= MAX_COLS, "열이 상한을 넘었다");
            assert_eq!(grid.cols(), w * cols as i32, "count={count} 가로");
            assert_eq!(
                grid.rows(),
                h * count.div_ceil(cols) as i32,
                "count={count} 세로"
            );
        }
    }

    /// 살린 곳은 원본 그대로, 죽인 곳은 어둡게. 둘 다 0 이 아니어야 경계가 장면 안에서 읽힌다.
    #[test]
    fn a_panel_keeps_the_frame_under_the_mask() {
        let frame = panel(4, 1);
        let mut mask =
            Mat::new_rows_cols_with_default(1, 4, opencv::core::CV_8UC1, Scalar::all(0.0))
                .expect("mask");
        *mask.at_2d_mut::<u8>(0, 1).expect("at") = 255;

        let out = as_panel(&frame, &mask).expect("panel");
        let at = |x: i32| out.at_2d::<opencv::core::Vec3b>(0, x).expect("at")[0];
        assert_eq!(at(1), 7, "마스크가 켜진 곳은 원본 그대로");
        assert_eq!(at(0), (7.0 * DIMMED).round() as u8, "꺼진 곳은 어둡게");
        assert!(at(0) < at(1), "경계가 보여야 한다");
    }

    /// 빈 칸은 검게 채운다 — 크기가 어긋나면 `vconcat` 이 거부한다.
    #[test]
    fn short_last_row_is_padded() {
        // 5장이면 3x2 라 마지막 칸 하나가 빈다.
        let panels: Vec<Mat> = (0..5).map(|_| panel(40, 30)).collect();
        let grid = grid_bgr(&panels).expect("grid");
        assert_eq!((grid.cols(), grid.rows()), (120, 60));

        // 마지막 줄 오른쪽 칸은 검다.
        let empty: opencv::core::Vec3b = *grid.at_2d(45, 100).expect("픽셀");
        assert_eq!(empty, opencv::core::Vec3b::from([0, 0, 0]));
        // 그 왼쪽 칸은 패널이 들어 있다 — 채운 게 아니라 남은 칸만 검어야 한다.
        let filled: opencv::core::Vec3b = *grid.at_2d(45, 20).expect("픽셀");
        assert_ne!(filled, opencv::core::Vec3b::from([0, 0, 0]));
    }

    #[test]
    fn a_single_panel_is_returned_as_one_row() {
        let grid = grid_bgr(&[panel(40, 30)]).expect("grid");
        assert_eq!((grid.cols(), grid.rows()), (40, 30));
    }
}
