//! OpenCV 다중 웹캠 프리뷰 — 장치 배열을 가로로 이어 붙인다.
//!
//! 장치 인덱스는 아래 `DEVICES`를 고친다. `q` / ESC 종료.

use anyhow::{Context, Result, bail};
use opencv::core::{Mat, Point, Scalar, Vector};
use opencv::highgui;
use opencv::imgproc;
use opencv::prelude::*;
use opencv::videoio::{self, VideoCapture, VideoCaptureTrait, VideoCaptureTraitConst};

const DEVICES: &[i32] = &[0, 1];

fn main() -> Result<()> {
    if DEVICES.is_empty() {
        bail!("DEVICES 가 비어 있음");
    }

    let mut caps = Vec::with_capacity(DEVICES.len());
    for &id in DEVICES {
        let cap = VideoCapture::new(id, videoio::CAP_ANY)
            .with_context(|| format!("VideoCapture open device {id}"))?;
        if !cap.is_opened()? {
            bail!("device {id} failed to open");
        }
        caps.push((id, cap));
    }

    let window = "cam_preview";
    highgui::named_window(window, highgui::WINDOW_NORMAL)?;
    println!("devices={DEVICES:?}  q/ESC quit");

    loop {
        let mut panels = Vec::with_capacity(caps.len());
        for (id, cap) in caps.iter_mut() {
            let mut frame = Mat::default();
            if !cap.read(&mut frame)? || frame.empty() {
                bail!("device {id}: 프레임 읽기 실패");
            }
            label_cam(&mut frame, *id)?;
            panels.push(frame);
        }

        let mosaic = hstack_bgr(&panels)?;
        highgui::imshow(window, &mosaic)?;
        let key = highgui::wait_key(1)? & 0xff;
        if key == 27 || key == i32::from(b'q') || key == i32::from(b'Q') {
            break;
        }
    }

    let _ = highgui::destroy_window(window);
    return Ok(());
}

fn label_cam(img: &mut Mat, id: i32) -> Result<()> {
    imgproc::put_text(
        img,
        &format!("cam{id}"),
        Point::new(8, 28),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.8,
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        2,
        imgproc::LINE_8,
        false,
    )?;
    return Ok(());
}

/// 높이를 최댓값에 맞추고(부족분은 검정 패딩) 가로 연결 — 리사이즈로 정보 잃지 않음.
fn hstack_bgr(panels: &[Mat]) -> Result<Mat> {
    if panels.len() == 1 {
        return Ok(panels[0].try_clone()?);
    }
    let max_h = panels.iter().map(|p| p.rows()).max().unwrap_or(1).max(1);
    let mut padded = Vec::with_capacity(panels.len());
    for p in panels {
        if p.rows() == max_h {
            padded.push(p.try_clone()?);
            continue;
        }
        let mut canvas = Mat::zeros(max_h, p.cols(), p.typ())?.to_mat()?;
        let roi = opencv::core::Rect::new(0, 0, p.cols(), p.rows());
        let mut dst = Mat::roi_mut(&mut canvas, roi)?;
        p.copy_to(&mut dst)?;
        padded.push(canvas);
    }
    let mut mosaic = Mat::default();
    opencv::core::hconcat(&Vector::<Mat>::from_iter(padded), &mut mosaic)?;
    return Ok(mosaic);
}
