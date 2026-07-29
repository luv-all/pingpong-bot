//! Appearance 레이어 체인 — `.then` 호출 순서 = 게이트 순서.

use opencv::core::Point;
use opencv::imgproc;
use opencv::prelude::*;

use super::super::motion::draw_candidate_contour;
use super::super::scoring::candidate::{Candidate, candidates_from_contours};
use super::super::scoring::scorer::Scorer;
use super::colormask::ColormaskDetector;
use super::contour::ContourDetector;
use super::generator::CandidateGenerator;
use crate::Pixel;
use crate::camera::Frame;

/// 이전 마스크를 게이트로 삼아 다음 이진 마스크를 만든다.
pub trait AppearanceLayer: Send {
    fn apply(&mut self, frame: &Frame, prior: Option<&Mat>) -> Option<Mat>;
}

impl AppearanceLayer for ColormaskDetector {
    fn apply(&mut self, frame: &Frame, prior: Option<&Mat>) -> Option<Mat> {
        let color = self.color_mask(frame)?;
        let Some(gate) = prior else {
            return Some(color);
        };
        let mut out = Mat::default();
        opencv::core::bitwise_and(&color, gate, &mut out, &Mat::default()).ok()?;
        return Some(out);
    }
}

impl AppearanceLayer for ContourDetector {
    fn apply(&mut self, frame: &Frame, prior: Option<&Mat>) -> Option<Mat> {
        let edges = match prior {
            Some(gate) => self.edge_mask_gated(frame, gate)?,
            None => self.edge_mask(frame)?,
        };
        let thick = dilate_edges(&edges)?;
        let Some(gate) = prior else {
            return Some(thick);
        };
        let mut out = Mat::default();
        opencv::core::bitwise_and(gate, &thick, &mut out, &Mat::default()).ok()?;
        return Some(out);
    }
}

fn dilate_edges(edges: &Mat) -> Option<Mat> {
    let kernel = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE,
        opencv::core::Size::new(5, 5),
        Point::new(-1, -1),
    )
    .ok()?;
    let mut thick = Mat::default();
    imgproc::dilate(
        edges,
        &mut thick,
        &kernel,
        Point::new(-1, -1),
        2,
        opencv::core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value().ok()?,
    )
    .ok()?;
    return Some(thick);
}

/// `.then`으로 쌓인 appearance 파이프라인.
pub struct AppearanceChain {
    stages: Vec<Box<dyn AppearanceLayer>>,
}

impl AppearanceChain {
    pub fn new() -> Self {
        return Self { stages: Vec::new() };
    }

    pub fn then(mut self, layer: impl AppearanceLayer + 'static) -> Self {
        self.stages.push(Box::new(layer));
        return self;
    }

    pub fn push(&mut self, layer: impl AppearanceLayer + 'static) {
        self.stages.push(Box::new(layer));
    }

    pub fn len(&self) -> usize {
        return self.stages.len();
    }

    pub fn is_empty(&self) -> bool {
        return self.stages.is_empty();
    }

    /// 각 단계 누적 마스크 (디버그·패널용).
    pub fn stage_masks(&mut self, frame: &Frame) -> Option<Vec<Mat>> {
        if self.stages.is_empty() {
            return None;
        }
        let mut out = Vec::with_capacity(self.stages.len());
        let mut prior: Option<Mat> = None;
        for stage in &mut self.stages {
            let mask = stage.apply(frame, prior.as_ref())?;
            prior = Some(mask.clone());
            out.push(mask);
        }
        return Some(out);
    }

    pub fn combined_mask(&mut self, frame: &Frame) -> Option<Mat> {
        return self.stage_masks(frame).and_then(|m| m.into_iter().last());
    }

    /// `(pixel, first_stage_bgr, last_stage_bgr)` — detect_full 패널용.
    pub fn detect_debug(&mut self, frame: &Frame, scorer: &Scorer) -> (Option<Pixel>, Mat, Mat) {
        let empty = || empty_bgr(frame);
        let Some(stages) = self.stage_masks(frame) else {
            return (None, empty(), empty());
        };
        let first_bgr = stages.first().map(mask_to_bgr).unwrap_or_else(empty);
        let Some(combined) = stages.last() else {
            return (None, first_bgr, empty());
        };
        let mut last_bgr = mask_to_bgr(combined);
        let cands = candidates_from_mask(combined);
        if let Some(c) = scorer.pick_best(&cands, |_| 0.0) {
            draw_candidate_contour(&mut last_bgr, &c.contour);
            return (Some(c.pixel), first_bgr, last_bgr);
        }
        return (None, first_bgr, last_bgr);
    }
}

impl Default for AppearanceChain {
    fn default() -> Self {
        return Self::new();
    }
}

impl CandidateGenerator for AppearanceChain {
    fn generate(&mut self, frame: &Frame) -> Vec<Candidate> {
        let Some(mask) = self.combined_mask(frame) else {
            return Vec::new();
        };
        return candidates_from_mask(&mask);
    }
}

fn candidates_from_mask(mask: &Mat) -> Vec<Candidate> {
    let mut contours = opencv::core::Vector::<opencv::core::Vector<Point>>::new();
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
    use crate::Id;
    use crate::detector::{ColorSpace, ColormaskParams, ScorerParams};
    use opencv::core::{CV_8UC3, Scalar, Size};
    use std::time::Instant;

    fn bright_blob_frame() -> Frame {
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
        return Frame::new(Id(0), img, Instant::now());
    }

    #[test]
    fn then_order_color_then_contour_finds_blob() {
        let color = ColormaskDetector::new(ColormaskParams {
            space: ColorSpace::Ycrcb,
            c0_min: 50,
            c0_max: 255,
            c1_min: 0,
            c1_max: 255,
            c2_min: 0,
            c2_max: 255,
        });
        let edges = ContourDetector::new(ScorerParams::default());
        let mut chain = AppearanceChain::new().then(color).then(edges);
        assert_eq!(chain.len(), 2);
        let frame = bright_blob_frame();
        let mask = chain.combined_mask(&frame).expect("mask");
        assert!(opencv::core::count_non_zero(&mask).unwrap() > 0);
        let (px, _, _) = chain.detect_debug(&frame, &Scorer::from(&ScorerParams::default()));
        let pixel = px.expect("debug hit");
        assert!((pixel.x - 100.0).abs() < 8.0);
        assert!((pixel.y - 80.0).abs() < 8.0);
    }
}
