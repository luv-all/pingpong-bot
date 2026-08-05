//! 배경 차분이 클립에서 얼마나 지우는지.
//!
//! 뒤 단계(색·윤곽)가 볼 픽셀이 몇 %나 남는지가 이 설계의 전제다 — 색이 1차 판별기가
//! 아니라 후보 생성기로 내려가려면 여기서 대부분 꺼져야 한다.
use opencv::core::{Mat, Scalar};
use opencv::prelude::*;
use pingpong_bot::camera::{self, FrameSource, OpenCvCapture};
use pingpong_bot::vision::detect::{Background, Layer};

#[test]
#[ignore = "클립 필요: cargo test --release --test diag_background -- --ignored --nocapture"]
fn mog2_shrinks_foreground_over_time() {
    let mut source = OpenCvCapture::from_path(
        camera::Id(0),
        std::path::Path::new("data/clips/fly_04/left.avi"),
    )
    .expect("클립 열기");
    let mut bg = Background::new(500, 16.0, 0.5, -1.0).expect("MOG2");
    let mut ratios = Vec::new();

    for i in 0..200 {
        let Some(frame) = source.next_frame() else {
            break;
        };
        let mut mask = Mat::new_rows_cols_with_default(
            frame.image.rows(),
            frame.image.cols(),
            opencv::core::CV_8UC1,
            Scalar::all(255.0),
        )
        .expect("mask");
        bg.narrow(&frame, &mut mask).expect("narrow");
        let on = opencv::core::count_non_zero(&mask).expect("count");
        let total = mask.rows() * mask.cols();
        let r = 100.0 * f64::from(on) / f64::from(total);
        if i % 20 == 0 {
            println!("frame {i:3}  전경 {r:6.2}%");
        }
        ratios.push(r);
    }

    let early: f64 = ratios[1..11].iter().sum::<f64>() / 10.0;
    let late: f64 = ratios[ratios.len() - 10..].iter().sum::<f64>() / 10.0;
    println!("초기 {early:.2}%  →  안정 {late:.2}%");
    assert!(late < early, "배경이 학습되면 전경 비율이 줄어야 한다");
    assert!(late < 5.0, "정지 장면에서 전경이 {late:.2}% 나 남는다");
}
