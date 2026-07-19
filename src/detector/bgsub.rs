//! 배경 차분 검출 (연속 프레임 absdiff).

use opencv::core::{Point, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::BallDetector;
use crate::PixelPoint;
use crate::camera::Frame;

pub struct BgSubDetector {
    previous: Option<Mat>,
    min_area_px: f64,
    max_area_px: f64,
}

impl BgSubDetector {
    pub fn new() -> Self {
        return Self {
            previous: None,
            min_area_px: 20.0,
            max_area_px: 20_000.0,
        };
    }
}

impl Default for BgSubDetector {
    fn default() -> Self {
        return Self::new();
    }
}

impl BallDetector for BgSubDetector {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        let mut gray = Mat::default();
        imgproc::cvt_color(
            &frame.image,
            &mut gray,
            imgproc::COLOR_BGR2GRAY,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .ok()?;

        let prev = match self.previous.replace(gray.try_clone().ok()?) {
            Some(p) => p,
            None => return None,
        };

        let mut diff = Mat::default();
        opencv::core::absdiff(&prev, &gray, &mut diff).ok()?;
        let mut mask = Mat::default();
        imgproc::threshold(&diff, &mut mask, 25.0, 255.0, imgproc::THRESH_BINARY).ok()?;

        let mut contours = Vector::<Vector<Point>>::new();
        imgproc::find_contours(
            &mask,
            &mut contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_SIMPLE,
            Point::new(0, 0),
        )
        .ok()?;

        let mut best: Option<(f64, PixelPoint)> = None;
        for contour in contours.iter() {
            let area = imgproc::contour_area(&contour, false).ok()?;
            if area < self.min_area_px || area > self.max_area_px {
                continue;
            }
            let moments = imgproc::moments(&contour, false).ok()?;
            if moments.m00.abs() < f64::EPSILON {
                continue;
            }
            let pixel = PixelPoint::new(moments.m10 / moments.m00, moments.m01 / moments.m00);
            match best {
                Some((a, _)) if a >= area => {}
                _ => best = Some((area, pixel)),
            }
        }
        return best.map(|(_, p)| p);
    }
}
