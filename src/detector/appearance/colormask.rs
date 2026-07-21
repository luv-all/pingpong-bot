//! YCrCb / HSV 색 마스크로 공 검출.

use clap::ValueEnum;
use opencv::core::{Point, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::super::BallDetector;
use super::super::candidate::{Candidate, candidates_from_contours};
use super::super::fuse::CandidateGenerator;
use super::super::motion::draw_candidate_contour;
use super::super::scorer::Scorer;
use crate::PixelPoint;
use crate::camera::Frame;

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, ValueEnum, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "lowercase")]
#[value(rename_all = "lower")]
pub enum ColorSpace {
    #[default]
    Ycrcb,
    Hsv,
}

impl std::str::FromStr for ColorSpace {
    type Err = ParseColorSpaceError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        return match s {
            "ycrcb" | "YCrCb" => Ok(Self::Ycrcb),
            "hsv" | "HSV" => Ok(Self::Hsv),
            _ => Err(ParseColorSpaceError),
        };
    }
}

impl std::fmt::Display for ColorSpace {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(match self {
            Self::Ycrcb => "ycrcb",
            Self::Hsv => "hsv",
        });
    }
}

/// [`ColorSpace`] 파싱 실패.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParseColorSpaceError;

impl std::fmt::Display for ParseColorSpaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str("expected ycrcb|hsv");
    }
}

impl std::error::Error for ParseColorSpaceError {}

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

pub struct ColormaskDetector {
    config: ColormaskConfig,
}

impl ColormaskDetector {
    pub fn new(config: ColormaskConfig) -> Self {
        return Self { config };
    }

    pub fn space(&self) -> ColorSpace {
        return self.config.space;
    }

    fn color_mask(&self, frame: &Frame) -> Option<Mat> {
        let mut converted = Mat::default();
        let code = match self.config.space {
            ColorSpace::Ycrcb => imgproc::COLOR_BGR2YCrCb,
            ColorSpace::Hsv => imgproc::COLOR_BGR2HSV,
        };
        if imgproc::cvt_color(
            &frame.image,
            &mut converted,
            code,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .is_err()
        {
            return None;
        }

        let lo = Scalar::new(
            f64::from(self.config.c0_min),
            f64::from(self.config.c1_min),
            f64::from(self.config.c2_min),
            0.0,
        );
        let hi = Scalar::new(
            f64::from(self.config.c0_max),
            f64::from(self.config.c1_max),
            f64::from(self.config.c2_max),
            0.0,
        );
        let mut mask = Mat::default();
        if opencv::core::in_range(&converted, &lo, &hi, &mut mask).is_err() {
            return None;
        }
        return Some(mask);
    }

    /// 검출 + 색 마스크(BGR). 선택 컨투어는 초록.
    pub fn detect_debug(&mut self, frame: &Frame) -> (Option<PixelPoint>, Mat) {
        let empty = || empty_bgr(frame);
        let Some(mask) = self.color_mask(frame) else {
            return (None, empty());
        };

        let mut mask_bgr = Mat::default();
        if imgproc::cvt_color(
            &mask,
            &mut mask_bgr,
            imgproc::COLOR_GRAY2BGR,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .is_err()
        {
            return (None, empty());
        }

        let cands = self.candidates_from_mask(&mask);
        // 단독 사용: area만 hard cut, circularity 미적용 (기존 동작).
        let scorer = Scorer::new(
            self.config.min_area_px,
            self.config.max_area_px,
            0.0,
            0.0,
        );
        let best = scorer.pick_best(&cands, |_| 0.0);
        if let Some(c) = best {
            draw_candidate_contour(&mut mask_bgr, &c.contour);
            return (Some(c.pixel), mask_bgr);
        }
        return (None, mask_bgr);
    }

    fn candidates_from_mask(&self, mask: &Mat) -> Vec<Candidate> {
        let mut contours = Vector::<Vector<Point>>::new();
        if imgproc::find_contours(
            mask,
            &mut contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_SIMPLE,
            Point::new(0, 0),
        )
        .is_err()
        {
            return Vec::new();
        }
        return candidates_from_contours(&contours);
    }
}

impl CandidateGenerator for ColormaskDetector {
    fn generate(&mut self, frame: &Frame) -> Vec<Candidate> {
        let Some(mask) = self.color_mask(frame) else {
            return Vec::new();
        };
        return self.candidates_from_mask(&mask);
    }
}

impl BallDetector for ColormaskDetector {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        return self.detect_debug(frame).0;
    }
}

fn empty_bgr(frame: &Frame) -> Mat {
    return Mat::zeros(frame.image.rows(), frame.image.cols(), frame.image.typ())
        .ok()
        .and_then(|m| m.to_mat().ok())
        .unwrap_or_default();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CameraId;
    use opencv::core::{CV_8UC3, Size};
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
