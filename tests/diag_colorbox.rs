use opencv::core::{Mat, Scalar};
use opencv::prelude::*;
use pingpong_bot::camera::{self, Frame, FrameSource, OpenCvCapture};
use pingpong_bot::vision::detect::{Background, ColorBox, Layer};

#[test]
#[ignore]
fn colorbox_actually_removes_pixels() {
    let mut src = OpenCvCapture::from_path(
        camera::Id(0),
        std::path::Path::new("data/clips/fly_04/left.avi"),
    )
    .expect("clip");
    let mut bg = Background::new(500, 16.0, 0.5, -1.0).expect("bg");
    let mut cb = ColorBox::load(camera::Id(0)).expect("colorbox");
    println!(
        "params = {:?}",
        pingpong_bot::defaults::colormask_for(camera::Id(0)).unwrap()
    );

    for i in 0..420 {
        let Some(f) = src.next_frame() else { break };
        let mut mask = Mat::new_rows_cols_with_default(
            f.image.rows(),
            f.image.cols(),
            opencv::core::CV_8UC1,
            Scalar::all(255.0),
        )
        .unwrap();
        bg.narrow(&f, &mut mask).unwrap();
        let before = opencv::core::count_non_zero(&mask).unwrap();
        cb.narrow(&f, &mut mask).unwrap();
        let after = opencv::core::count_non_zero(&mask).unwrap();
        if i >= 390 && i <= 400 {
            println!("frame {i}: 배경뒤 {before} → 색뒤 {after}");
        }
        let _ = Frame::new(camera::Id(0), Mat::default(), f.timestamp);
    }
}
