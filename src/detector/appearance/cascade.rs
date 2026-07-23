//! colormask → contour 파이프라인 appearance.
//!
//! 1. 색 마스크로 후보 영역 축소  
//! 2. 그 영역에서만 Canny → `color ∩ dilate(edges)`  
//! 색이 비면 contour를 건너뛴다.

use opencv::core::{Point, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::super::BallDetector;
use super::super::candidate::{Candidate, candidates_from_contours};
use super::super::fuse::CandidateGenerator;
use super::super::motion::draw_candidate_contour;
use super::super::params::ScorerParams;
use super::super::scorer::Scorer;
use super::colormask::{ColormaskDetector, ColormaskParams};
use super::contour::ContourDetector;
use crate::PixelPoint;
use crate::camera::Frame;

/// defaults 본선: **colormask → contour** (싼 필터 먼저).
pub struct ColorContourCascade {
    color: ColormaskDetector,
    edges: ContourDetector,
    last_area: Option<f64>,
}

impl ColorContourCascade {
    pub fn new(color: ColormaskParams, scorer: &ScorerParams) -> Self {
        return Self {
            color: ColormaskDetector::new(color),
            edges: ContourDetector::from(scorer),
            last_area: None,
        };
    }

    /// 단계 마스크 `(color, cascade)`. cascade는 color 통과 뒤에만 계산.
    pub fn stage_masks(&self, frame: &Frame) -> Option<(Mat, Mat)> {
        let color = self.color.color_mask(frame)?;
        let nz = opencv::core::count_non_zero(&color).ok()?;
        if nz == 0 {
            return Some((color.clone(), color));
        }
        let edges = self.edges.edge_mask_gated(frame, &color)?;
        let kernel = imgproc::get_structuring_element(
            imgproc::MORPH_ELLIPSE,
            opencv::core::Size::new(5, 5),
            Point::new(-1, -1),
        )
        .ok()?;
        let mut thick = Mat::default();
        if imgproc::dilate(
            &edges,
            &mut thick,
            &kernel,
            Point::new(-1, -1),
            2,
            opencv::core::BORDER_CONSTANT,
            imgproc::morphology_default_border_value().ok()?,
        )
        .is_err()
        {
            return None;
        }
        let mut combined = Mat::default();
        if opencv::core::bitwise_and(&color, &thick, &mut combined, &Mat::default()).is_err() {
            return None;
        }
        return Some((color, combined));
    }

    /// 누적(최종) 마스크.
    pub fn combined_mask(&self, frame: &Frame) -> Option<Mat> {
        return self.stage_masks(frame).map(|(_, c)| c);
    }

    fn candidates_from_mask(mask: &Mat) -> Vec<Candidate> {
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

    /// `(pixel, color_bgr, cascade_bgr)` — 단계 패널용.
    pub fn detect_debug(
        &mut self,
        frame: &Frame,
        scorer: &Scorer,
    ) -> (Option<PixelPoint>, Mat, Mat) {
        self.last_area = None;
        let empty = || empty_bgr(frame);
        let Some((color, combined)) = self.stage_masks(frame) else {
            return (None, empty(), empty());
        };

        let color_bgr = mask_to_bgr(&color);
        let mut cascade_bgr = mask_to_bgr(&combined);
        let cands = Self::candidates_from_mask(&combined);
        if let Some(c) = scorer.pick_best(&cands, |_| 0.0) {
            self.last_area = Some(c.area);
            draw_candidate_contour(&mut cascade_bgr, &c.contour);
            return (Some(c.pixel), color_bgr, cascade_bgr);
        }
        return (None, color_bgr, cascade_bgr);
    }
}

impl CandidateGenerator for ColorContourCascade {
    fn generate(&mut self, frame: &Frame) -> Vec<Candidate> {
        let Some(mask) = self.combined_mask(frame) else {
            return Vec::new();
        };
        return Self::candidates_from_mask(&mask);
    }
}

impl BallDetector for ColorContourCascade {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        let scorer = Scorer::from(&ScorerParams::default());
        return self.detect_debug(frame, &scorer).0;
    }

    fn last_area(&self) -> Option<f64> {
        return self.last_area;
    }
}

fn mask_to_bgr(mask: &Mat) -> Mat {
    let mut bgr = Mat::default();
    if imgproc::cvt_color(
        mask,
        &mut bgr,
        imgproc::COLOR_GRAY2BGR,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )
    .is_err()
    {
        return Mat::default();
    }
    return bgr;
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
    use crate::detector::{ColorSpace, ColormaskParams};
    use opencv::core::{CV_8UC3, Scalar, Size};
    use std::time::Instant;

    #[test]
    fn cascade_finds_bright_blob() {
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
        let color = ColormaskParams {
            space: ColorSpace::Ycrcb,
            c0_min: 50,
            c0_max: 255,
            c1_min: 0,
            c1_max: 255,
            c2_min: 0,
            c2_max: 255,
        };
        let scorer = ScorerParams::default();
        let mut det = ColorContourCascade::new(color, &scorer);
        let pixel = det.detect(&frame).expect("cascade hit");
        assert!((pixel.x - 100.0).abs() < 8.0, "x={}", pixel.x);
        assert!((pixel.y - 80.0).abs() < 8.0, "y={}", pixel.y);
    }

    #[test]
    fn empty_color_skips_without_panic() {
        let img =
            Mat::new_size_with_default(Size::new(64, 64), CV_8UC3, Scalar::all(0.0)).unwrap();
        let frame = Frame::new(CameraId(0), img, Instant::now());
        let color = ColormaskParams {
            space: ColorSpace::Ycrcb,
            c0_min: 200,
            c0_max: 255,
            c1_min: 200,
            c1_max: 255,
            c2_min: 200,
            c2_max: 255,
        };
        let scorer = ScorerParams::default();
        let det = ColorContourCascade::new(color, &scorer);
        let (cm, cas) = det.stage_masks(&frame).expect("masks");
        assert_eq!(opencv::core::count_non_zero(&cm).unwrap(), 0);
        assert_eq!(opencv::core::count_non_zero(&cas).unwrap(), 0);
    }
}
