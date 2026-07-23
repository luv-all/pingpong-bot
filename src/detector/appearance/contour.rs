//! Canny contour appearance generator + 단독 검출.

use opencv::core::{Point, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::super::BallDetector;
use super::super::candidate::{Candidate, candidates_from_contours};
use super::super::fuse::CandidateGenerator;
use super::super::motion::draw_candidate_contour;
use super::super::params::ScorerParams;
use super::super::scorer::Scorer;
use crate::PixelPoint;
use crate::camera::Frame;

pub struct ContourDetector {
    min_area_px: f64,
    max_area_px: f64,
    min_circularity: f64,
    last_area: Option<f64>,
}

impl ContourDetector {
    pub fn new(params: ScorerParams) -> Self {
        return Self {
            min_area_px: params.min_area_px,
            max_area_px: params.max_area_px,
            min_circularity: params.min_circularity,
            last_area: None,
        };
    }

    /// Canny 엣지 마스크. cascade·디버그용.
    pub fn edge_mask(&self, frame: &Frame) -> Option<Mat> {
        return self.edge_mask_from_gray(&Self::gray(frame)?);
    }

    /// `gate`가 0인 픽셀은 무시하고 Canny — colormask 통과 영역만 contour.
    pub fn edge_mask_gated(&self, frame: &Frame, gate: &Mat) -> Option<Mat> {
        let gray = Self::gray(frame)?;
        let mut gated = Mat::zeros(gray.rows(), gray.cols(), gray.typ())
            .ok()?
            .to_mat()
            .ok()?;
        gray.copy_to_masked(&mut gated, gate).ok()?;
        return self.edge_mask_from_gray(&gated);
    }

    fn gray(frame: &Frame) -> Option<Mat> {
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
        return Some(gray);
    }

    fn edge_mask_from_gray(&self, gray: &Mat) -> Option<Mat> {
        let mut blur = Mat::default();
        if imgproc::gaussian_blur(
            gray,
            &mut blur,
            opencv::core::Size::new(5, 5),
            0.0,
            0.0,
            opencv::core::BORDER_DEFAULT,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .is_err()
        {
            return None;
        }
        let mut edges = Mat::default();
        if imgproc::canny(&blur, &mut edges, 50.0, 150.0, 3, false).is_err() {
            return None;
        }
        return Some(edges);
    }

    fn candidates_from_edges(&self, edges: &Mat) -> Vec<Candidate> {
        let mut contours = Vector::<Vector<Point>>::new();
        if imgproc::find_contours(
            edges,
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

    /// 검출 + Canny 엣지(BGR). 선택 컨투어는 초록.
    pub fn detect_debug(&mut self, frame: &Frame) -> (Option<PixelPoint>, Mat) {
        self.last_area = None;
        let empty = || {
            Mat::zeros(frame.image.rows(), frame.image.cols(), frame.image.typ())
                .ok()
                .and_then(|m| m.to_mat().ok())
                .unwrap_or_default()
        };

        let Some(edges) = self.edge_mask(frame) else {
            return (None, empty());
        };

        let mut edges_bgr = Mat::default();
        if imgproc::cvt_color(
            &edges,
            &mut edges_bgr,
            imgproc::COLOR_GRAY2BGR,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .is_err()
        {
            return (None, empty());
        }

        let cands = self.candidates_from_edges(&edges);
        let scorer = Scorer::new(
            self.min_area_px,
            self.max_area_px,
            self.min_circularity,
            0.0,
        );
        if let Some(c) = scorer.pick_best(&cands, |_| 0.0) {
            self.last_area = Some(c.area);
            draw_candidate_contour(&mut edges_bgr, &c.contour);
            return (Some(c.pixel), edges_bgr);
        }
        return (None, edges_bgr);
    }
}

impl CandidateGenerator for ContourDetector {
    fn generate(&mut self, frame: &Frame) -> Vec<Candidate> {
        let Some(edges) = self.edge_mask(frame) else {
            return Vec::new();
        };
        return self.candidates_from_edges(&edges);
    }
}

impl Default for ContourDetector {
    fn default() -> Self {
        return Self::new(crate::detector::ScorerParams {
            min_area_px: 20.0,
            max_area_px: 20_000.0,
            min_circularity: 0.55,
        });
    }
}

impl From<ScorerParams> for ContourDetector {
    fn from(params: ScorerParams) -> Self {
        return Self::new(params);
    }
}

impl From<&ScorerParams> for ContourDetector {
    fn from(params: &ScorerParams) -> Self {
        return Self::new(params.clone());
    }
}

impl BallDetector for ContourDetector {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        return self.detect_debug(frame).0;
    }

    fn last_area(&self) -> Option<f64> {
        return self.last_area;
    }
}
