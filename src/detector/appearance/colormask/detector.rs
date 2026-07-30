use opencv::core::{Point, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::super::super::motion::draw_candidate_contour;
use super::super::super::scoring::candidate::{Candidate, candidates_from_contours};
use super::super::super::scoring::scorer::Scorer;
use super::{ColorSpace, ColormaskParams};
use crate::camera;
use crate::camera::Frame;

pub struct ColormaskDetector {
    params: ColormaskParams,
    last_area: Option<f64>,
}

impl ColormaskDetector {
    pub fn new(params: ColormaskParams) -> Self {
        return Self {
            params,
            last_area: None,
        };
    }

    pub fn space(&self) -> ColorSpace {
        return self.params.space;
    }

    /// 색 마스크 (단일 채널). cascade·디버그용.
    pub fn color_mask(&self, frame: &Frame) -> Option<Mat> {
        let converted = self.params.space.convert(&frame.image).ok()?;

        let lo = Scalar::new(
            f64::from(self.params.c0_min),
            f64::from(self.params.c1_min),
            f64::from(self.params.c2_min),
            0.0,
        );
        let hi = Scalar::new(
            f64::from(self.params.c0_max),
            f64::from(self.params.c1_max),
            f64::from(self.params.c2_max),
            0.0,
        );
        let mut mask = Mat::default();
        if opencv::core::in_range(&converted, &lo, &hi, &mut mask).is_err() {
            return None;
        }
        return Some(mask);
    }

    /// 검출 + 색 마스크(BGR). 선택 컨투어는 초록.
    /// hard cut은 호출측 `Scorer`를 쓴다.
    pub fn detect_debug(&mut self, frame: &Frame, scorer: &Scorer) -> (Option<camera::Pixel>, Mat) {
        self.last_area = None;
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
        let best = scorer.pick_best(&cands, |_| 0.0);
        if let Some(c) = best {
            self.last_area = Some(c.area);
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

fn empty_bgr(frame: &Frame) -> Mat {
    return Mat::zeros(frame.image.rows(), frame.image.cols(), frame.image.typ())
        .ok()
        .and_then(|m| m.to_mat().ok())
        .unwrap_or_default();
}

#[cfg(test)]
mod tests {
    use super::*;
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
        let frame = Frame::new(camera::Id(0), img, Instant::now());
        let params = ColormaskParams {
            space: ColorSpace::Ycrcb,
            c0_min: 50,
            c0_max: 255,
            c1_min: 0,
            c1_max: 255,
            c2_min: 0,
            c2_max: 255,
        };
        let mut det = ColormaskDetector::new(params);
        let scorer = Scorer::shape(20.0, 20_000.0, 0.55);
        let pixel = det
            .detect_debug(&frame, &scorer)
            .0
            .expect("should find blob");
        assert!((pixel.x - 100.0).abs() < 5.0, "x={}", pixel.x);
        assert!((pixel.y - 80.0).abs() < 5.0, "y={}", pixel.y);
    }
}
