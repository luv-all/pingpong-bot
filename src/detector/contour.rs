//! Contour + 원형도 게이팅 검출.

use opencv::core::{Point, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::BallDetector;
use crate::PixelPoint;
use crate::camera::Frame;

pub struct ContourDetector {
    min_area_px: f64,
    max_area_px: f64,
    min_circularity: f64,
}

impl ContourDetector {
    pub fn new() -> Self {
        return Self {
            min_area_px: 20.0,
            max_area_px: 20_000.0,
            min_circularity: 0.55,
        };
    }
}

impl Default for ContourDetector {
    fn default() -> Self {
        return Self::new();
    }
}

impl BallDetector for ContourDetector {
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
        let mut blur = Mat::default();
        imgproc::gaussian_blur(
            &gray,
            &mut blur,
            opencv::core::Size::new(5, 5),
            0.0,
            0.0,
            opencv::core::BORDER_DEFAULT,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .ok()?;
        let mut edges = Mat::default();
        imgproc::canny(&blur, &mut edges, 50.0, 150.0, 3, false).ok()?;

        let mut contours = Vector::<Vector<Point>>::new();
        imgproc::find_contours(
            &edges,
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
            let peri = imgproc::arc_length(&contour, true).ok()?;
            if peri < f64::EPSILON {
                continue;
            }
            let circularity = 4.0 * std::f64::consts::PI * area / (peri * peri);
            if circularity < self.min_circularity {
                continue;
            }
            let moments = imgproc::moments(&contour, false).ok()?;
            if moments.m00.abs() < f64::EPSILON {
                continue;
            }
            let pixel = PixelPoint::new(moments.m10 / moments.m00, moments.m01 / moments.m00);
            let score = area * circularity;
            match best {
                Some((s, _)) if s >= score => {}
                _ => best = Some((score, pixel)),
            }
        }
        return best.map(|(_, p)| p);
    }
}
