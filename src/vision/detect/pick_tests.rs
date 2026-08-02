//! [`super`] 단위 테스트.

use super::*;
use crate::camera::Pixel;

fn picker() -> Picker {
    return Picker {
        min_radius_px: 3.0,
        max_radius_px: 18.0,
        min_circularity: 0.35,
        contours: opencv::core::Vector::new(),
    };
}

fn candidate(radius_px: f64, circularity: f64) -> Candidate {
    return Candidate {
        pixel: Pixel::new(100.0, 100.0),
        radius_px,
        circularity,
    };
}

#[test]
fn deviation_is_zero_at_the_expected_radius() {
    assert!(picker().deviation(&candidate(8.0, 0.9), 8.0).abs() < 1e-12);
}

/// 지금 규칙("가장 큰 것")이면 팔이 이긴다. 편차 규칙이면 공이 이긴다.
#[test]
fn the_arm_loses_to_the_ball_under_deviation() {
    let p = picker();
    let ball = candidate(8.0, 0.6);
    let arm = candidate(17.0, 0.5);
    assert!(p.deviation(&ball, 8.0) < p.deviation(&arm, 8.0));
}

/// 원형도는 순위에 안 쓴다 — 블러로 찌그러진 빠른 공이 져선 안 된다.
#[test]
fn a_blurred_ball_still_wins_on_radius() {
    let p = picker();
    let blurred_ball = candidate(8.0, 0.36);
    let round_blob = candidate(15.0, 0.98);
    assert!(p.passes(&blurred_ball), "느슨한 하한은 통과해야 한다");
    assert!(p.deviation(&blurred_ball, 8.0) < p.deviation(&round_blob, 8.0));
}

#[test]
fn band_and_circularity_are_hard_cuts() {
    let p = picker();
    assert!(!p.passes(&candidate(2.0, 0.9)), "너무 작다");
    assert!(!p.passes(&candidate(30.0, 0.9)), "너무 크다");
    assert!(!p.passes(&candidate(8.0, 0.1)), "원형도 하한");
}

/// 그린 원에서 중심·반지름·원형도가 나오는가.
#[test]
fn picks_a_drawn_circle() {
    use opencv::core::{Scalar, Size};
    use opencv::prelude::*;

    let mut mask =
        Mat::new_size_with_default(Size::new(200, 200), opencv::core::CV_8UC1, Scalar::all(0.0))
            .expect("mask");
    opencv::imgproc::circle(
        &mut mask,
        opencv::core::Point::new(120, 80),
        8,
        Scalar::all(255.0),
        -1,
        opencv::imgproc::LINE_8,
        0,
    )
    .expect("draw");

    let found = picker()
        .pick(&mask, Some(8.0))
        .expect("pick")
        .expect("찾음");
    assert!((found.pixel.x - 120.0).abs() < 1.0, "x={}", found.pixel.x);
    assert!((found.pixel.y - 80.0).abs() < 1.0, "y={}", found.pixel.y);
    assert!((found.radius_px - 8.0).abs() < 1.0, "r={}", found.radius_px);
    // 완벽한 원이어도 8 px 반지름이면 래스터화 계단 때문에 0.8 언저리다.
    assert!(found.circularity > 0.7, "circ={}", found.circularity);
}

/// 두 원 중 기대 반지름에 가까운 쪽을 고른다. 큰 쪽이 아니다.
#[test]
fn prefers_the_expected_radius_over_the_bigger_blob() {
    use opencv::core::{Scalar, Size};
    use opencv::prelude::*;

    let mut mask =
        Mat::new_size_with_default(Size::new(300, 200), opencv::core::CV_8UC1, Scalar::all(0.0))
            .expect("mask");
    let draw = |m: &mut Mat, x: i32, r: i32| {
        opencv::imgproc::circle(
            m,
            opencv::core::Point::new(x, 100),
            r,
            Scalar::all(255.0),
            -1,
            opencv::imgproc::LINE_8,
            0,
        )
        .expect("draw");
    };
    draw(&mut mask, 80, 6); // 공
    draw(&mut mask, 220, 16); // 팔

    let found = picker()
        .pick(&mask, Some(6.0))
        .expect("pick")
        .expect("찾음");
    assert!(
        (found.pixel.x - 80.0).abs() < 2.0,
        "큰 쪽을 골랐다: x={}",
        found.pixel.x
    );
}

/// 반지름이 작을수록 래스터화만으로 원형도가 떨어진다. 하한을 느슨하게 잡는 근거.
#[test]
fn small_circles_lose_circularity_to_rasterisation() {
    use opencv::core::{Scalar, Size};
    use opencv::prelude::*;

    let circ_of = |r: i32| -> f64 {
        let mut mask = Mat::new_size_with_default(
            Size::new(120, 120),
            opencv::core::CV_8UC1,
            Scalar::all(0.0),
        )
        .expect("mask");
        opencv::imgproc::circle(
            &mut mask,
            opencv::core::Point::new(60, 60),
            r,
            Scalar::all(255.0),
            -1,
            opencv::imgproc::LINE_8,
            0,
        )
        .expect("draw");
        let mut p = Picker {
            min_radius_px: 1.0,
            max_radius_px: 100.0,
            min_circularity: 0.0,
            contours: opencv::core::Vector::new(),
        };
        return p
            .pick(&mask, Some(f64::from(r)))
            .expect("pick")
            .expect("찾음")
            .circularity;
    };

    let (small, large) = (circ_of(3), circ_of(20));
    println!("r=3 circ={small:.3}  r=20 circ={large:.3}");
    assert!(small < large, "작을수록 낮아야 한다: {small} vs {large}");
    assert!(
        small > MIN_CIRCULARITY,
        "하한 {MIN_CIRCULARITY} 이 r=3 을 죽이면 안 된다"
    );
}
