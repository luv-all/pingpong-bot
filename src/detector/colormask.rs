//! YCrCb / HSV 색 마스크로 공 검출.

use opencv::core::{Point, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::BallDetector;
use crate::PixelPoint;
use crate::camera::Frame;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorSpace {
    Ycrcb,
    Hsv,
}

impl ColorSpace {
    pub fn parse(s: &str) -> Option<Self> {
        return match s {
            "ycrcb" | "YCrCb" => Some(Self::Ycrcb),
            "hsv" | "HSV" => Some(Self::Hsv),
            _ => None,
        };
    }
}

#[derive(Debug, Clone)]
pub struct ColormaskConfig {
    pub space: ColorSpace,
    pub c0_min: u8,
    pub c0_max: u8,
    pub c1_min: u8,
    pub c1_max: u8,
    pub c2_min: u8,
    pub c2_max: u8,
    pub min_area_px: f64,
    pub max_area_px: f64,
}

impl Default for ColormaskConfig {
    fn default() -> Self {
        // YCrCb 주황공 출발점 (스펙)
        return Self {
            space: ColorSpace::Ycrcb,
            c0_min: 0,
            c0_max: 255,
            c1_min: 133,
            c1_max: 173,
            c2_min: 77,
            c2_max: 127,
            min_area_px: 20.0,
            max_area_px: 20_000.0,
        };
    }
}

pub struct ColormaskDetector {
    config: ColormaskConfig,
}

impl ColormaskDetector {
    pub fn new(config: ColormaskConfig) -> Self {
        return Self { config };
    }
}

impl BallDetector for ColormaskDetector {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        let mut converted = Mat::default();
        let code = match self.config.space {
            ColorSpace::Ycrcb => imgproc::COLOR_BGR2YCrCb,
            ColorSpace::Hsv => imgproc::COLOR_BGR2HSV,
        };
        imgproc::cvt_color(
            &frame.image,
            &mut converted,
            code,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .ok()?;

        let lo = opencv::core::Scalar::new(
            f64::from(self.config.c0_min),
            f64::from(self.config.c1_min),
            f64::from(self.config.c2_min),
            0.0,
        );
        let hi = opencv::core::Scalar::new(
            f64::from(self.config.c0_max),
            f64::from(self.config.c1_max),
            f64::from(self.config.c2_max),
            0.0,
        );
        let mut mask = Mat::default();
        opencv::core::in_range(&converted, &lo, &hi, &mut mask).ok()?;

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
            if area < self.config.min_area_px || area > self.config.max_area_px {
                continue;
            }
            let moments = imgproc::moments(&contour, false).ok()?;
            if moments.m00.abs() < f64::EPSILON {
                continue;
            }
            let cx = moments.m10 / moments.m00;
            let cy = moments.m01 / moments.m00;
            let pixel = PixelPoint::new(cx, cy);
            match best {
                Some((a, _)) if a >= area => {}
                _ => best = Some((area, pixel)),
            }
        }
        return best.map(|(_, p)| p);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CameraId;
    use opencv::core::{CV_8UC3, Scalar, Size};
    use std::time::Instant;

    #[test]
    fn colormask_finds_blob_center() {
        let mut img =
            Mat::new_size_with_default(Size::new(200, 200), CV_8UC3, Scalar::all(0.0)).unwrap();
        imgproc::circle(
            &mut img,
            Point::new(100, 80),
            15,
            Scalar::new(200.0, 200.0, 200.0, 0.0),
            -1,
            imgproc::LINE_8,
            0,
        )
        .unwrap();
        let frame = Frame::new(CameraId(0), img, Instant::now());
        let config = ColormaskConfig {
            space: ColorSpace::Ycrcb,
            c0_min: 50,
            c0_max: 255,
            c1_min: 0,
            c1_max: 255,
            c2_min: 0,
            c2_max: 255,
            min_area_px: 20.0,
            max_area_px: 20_000.0,
        };
        let mut det = ColormaskDetector::new(config);
        let pixel = det.detect(&frame).expect("should find blob");
        assert!((pixel.x - 100.0).abs() < 5.0, "x={}", pixel.x);
        assert!((pixel.y - 80.0).abs() < 5.0, "y={}", pixel.y);
    }
}
