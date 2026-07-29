//! Appearance 레이어 — 이전 마스크를 게이트로 삼아 다음 이진 마스크를 만든다.

use opencv::core::Point;
use opencv::imgproc;
use opencv::prelude::*;

use super::colormask::ColormaskDetector;
use super::contour::ContourDetector;
use crate::camera::Frame;

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
