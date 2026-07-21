//! 움직임 prior — 독립 검출기가 아니라 fuse scorer용 마스크.

use opencv::core::{Point, Rect, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::candidate::Candidate;
use crate::camera::Frame;

const DEFAULT_DIFF_THRESH: f64 = 25.0;

/// 연속 프레임 absdiff → 이진 마스크. soft motion score에 쓴다.
pub struct MotionPrior {
    previous: Option<Mat>,
    diff_thresh: f64,
}

impl MotionPrior {
    pub fn new() -> Self {
        return Self {
            previous: None,
            diff_thresh: DEFAULT_DIFF_THRESH,
        };
    }

    pub fn with_diff_thresh(mut self, thresh: f64) -> Self {
        self.diff_thresh = thresh;
        return self;
    }

    /// 이번 프레임 motion 마스크(8UC1). 첫 프레임은 `None`.
    pub fn update(&mut self, frame: &Frame) -> Option<Mat> {
        let mut gray = Mat::default();
        if imgproc::cvt_color(
            &frame.image,
            &mut gray,
            imgproc::COLOR_BGR2GRAY,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .is_err()
        {
            return None;
        }
        let prev = self.previous.replace(gray.try_clone().ok()?)?;
        let mut diff = Mat::default();
        if opencv::core::absdiff(&prev, &gray, &mut diff).is_err() {
            return None;
        }
        let mut mask = Mat::default();
        if imgproc::threshold(
            &diff,
            &mut mask,
            self.diff_thresh,
            255.0,
            imgproc::THRESH_BINARY,
        )
        .is_err()
        {
            return None;
        }
        return Some(mask);
    }

    /// 후보 컨투어 bbox 안 motion 픽셀 비율 `[0,1]`.
    pub fn overlap(mask: &Mat, candidate: &Candidate) -> f64 {
        let Ok(rect) = imgproc::bounding_rect(&candidate.contour) else {
            return centroid_hit(mask, candidate.pixel.x, candidate.pixel.y);
        };
        return mean_in_rect(mask, rect);
    }
}

impl Default for MotionPrior {
    fn default() -> Self {
        return Self::new();
    }
}

fn centroid_hit(mask: &Mat, x: f64, y: f64) -> f64 {
    let ix = x.round() as i32;
    let iy = y.round() as i32;
    if ix < 0 || iy < 0 || ix >= mask.cols() || iy >= mask.rows() {
        return 0.0;
    }
    let Ok(v) = mask.at_2d::<u8>(iy, ix) else {
        return 0.0;
    };
    return if *v > 0 { 1.0 } else { 0.0 };
}

fn mean_in_rect(mask: &Mat, rect: Rect) -> f64 {
    if rect.width <= 0 || rect.height <= 0 {
        return 0.0;
    }
    let x1 = rect.x.clamp(0, mask.cols());
    let y1 = rect.y.clamp(0, mask.rows());
    let x2 = (rect.x + rect.width).clamp(0, mask.cols());
    let y2 = (rect.y + rect.height).clamp(0, mask.rows());
    let w = x2 - x1;
    let h = y2 - y1;
    if w <= 0 || h <= 0 {
        return 0.0;
    }
    let roi = Rect::new(x1, y1, w, h);
    let Ok(view) = Mat::roi(mask, roi) else {
        return 0.0;
    };
    let Ok(m) = opencv::core::mean(&view, &Mat::default()) else {
        return 0.0;
    };
    return (m[0] / 255.0).clamp(0.0, 1.0);
}

pub fn mask_to_bgr(mask: &Mat) -> Mat {
    let mut bgr = Mat::default();
    let _ = imgproc::cvt_color(
        mask,
        &mut bgr,
        imgproc::COLOR_GRAY2BGR,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    );
    return bgr;
}

pub fn draw_candidate_contour(img: &mut Mat, contour: &Vector<Point>) {
    let _ = imgproc::draw_contours(
        img,
        &Vector::<Vector<Point>>::from_iter([contour.clone()]),
        0,
        Scalar::new(0.0, 255.0, 0.0, 0.0),
        2,
        imgproc::LINE_8,
        &Mat::default(),
        i32::MAX,
        Point::new(0, 0),
    );
}
