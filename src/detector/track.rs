//! ROI 추적 — appearance 점수 고르기 + acquire/track 탐색.
//!
//! [`Detector::builder`](crate::detector::Detector::builder)의 `.roi` / `.scorer`가 조립한다.

use opencv::core::Rect;
use opencv::prelude::*;

use super::appearance::{AppearanceChain, CandidateGenerator};
use super::motion::MotionPrior;
use super::roi_params::RoiParams;
use super::scoring::candidate::Candidate;
use super::scoring::scorer::Scorer;
use crate::Pixel;
use crate::camera::Frame;

/// appearance + scorer(+motion) + ROI 탐색 정책.
pub struct RoiTrack {
    appearance: AppearanceChain,
    scorer: Scorer,
    motion: Option<MotionPrior>,
    pub params: RoiParams,
    pub half_px: i32,
    pub roi_enabled: bool,
    last: Option<Pixel>,
    last_area: Option<f64>,
    last_delta_px: f64,
    pub last_roi: Option<Rect>,
    pub used_roi: bool,
}

/// 빌더 전용. `scorer.motion_weight > 0`이면 motion prior on.
pub(crate) fn track(
    appearance: AppearanceChain,
    scorer: Scorer,
    params: impl Into<RoiParams>,
) -> RoiTrack {
    let params = params.into();
    let half_px = params.half_min;
    let motion = if scorer.motion_weight > 0.0 {
        Some(MotionPrior::new())
    } else {
        None
    };
    return RoiTrack {
        appearance,
        scorer,
        motion,
        params,
        half_px,
        roi_enabled: true,
        last: None,
        last_area: None,
        last_delta_px: 0.0,
        last_roi: None,
        used_roi: false,
    };
}

impl std::fmt::Display for RoiTrack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return write!(
            f,
            "track(half={}, radius_scale={:.1}, motion_scale={:.1}, padding={}, roi={})",
            self.half_px,
            self.params.radius_scale,
            self.params.motion_scale,
            self.params.padding,
            if self.roi_enabled { "on" } else { "off" }
        );
    }
}

impl RoiTrack {
    pub fn set_roi_enabled(&mut self, enabled: bool) {
        self.roi_enabled = enabled;
        if !enabled {
            self.last = None;
            self.last_area = None;
            self.last_delta_px = 0.0;
            self.last_roi = None;
            self.used_roi = false;
            self.half_px = self.params.half_min;
        }
    }

    pub fn recompute_half(&mut self) {
        self.half_px = self.params.compute_half(self.last_area, self.last_delta_px);
    }

    pub fn last_area(&self) -> Option<f64> {
        return self.last_area;
    }

    pub fn detect(&mut self, frame: &Frame) -> Option<Pixel> {
        self.last_roi = None;
        self.used_roi = false;

        if !self.roi_enabled {
            if let Some(p) = self.detect_region(frame, None) {
                self.note_hit(p);
                return Some(p);
            }
            self.clear_track();
            return None;
        }

        if let Some(prev) = self.last {
            if let Some(r) = Self::roi_rect(prev, self.half_px, frame) {
                self.last_roi = Some(r);
                if let Some(p) = self.detect_region(frame, Some(r)) {
                    self.note_hit(p);
                    self.used_roi = true;
                    return Some(p);
                }
            }
        }

        self.last_roi = None;
        if let Some(p) = self.detect_region(frame, None) {
            self.note_hit(p);
            self.used_roi = false;
            return Some(p);
        }

        self.clear_track();
        return None;
    }

    fn roi_rect(prev: Pixel, half: i32, frame: &Frame) -> Option<Rect> {
        let size = frame.image.size().ok()?;
        let x0 = (prev.x as i32 - half).max(0);
        let y0 = (prev.y as i32 - half).max(0);
        let x1 = (prev.x as i32 + half).min(size.width);
        let y1 = (prev.y as i32 + half).min(size.height);
        let w = (x1 - x0).max(1);
        let h = (y1 - y0).max(1);
        return Some(Rect::new(x0, y0, w, h));
    }

    fn pick_in_frame(&mut self, frame: &Frame) -> Option<Pixel> {
        let motion_mask = self.motion.as_mut().and_then(|m| m.update(frame));
        let overlap = |c: &Candidate| match &motion_mask {
            Some(mask) => MotionPrior::overlap(mask, c),
            None => 0.0,
        };
        let cands = self.appearance.generate(frame);
        let best = self.scorer.pick_best(&cands, &overlap)?;
        self.last_area = Some(best.area);
        return Some(best.pixel);
    }

    fn detect_region(&mut self, frame: &Frame, roi: Option<Rect>) -> Option<Pixel> {
        let Some(r) = roi else {
            return self.pick_in_frame(frame);
        };
        let roi_mat = Mat::roi(&frame.image, r).ok()?;
        let owned = roi_mat.try_clone().ok()?;
        let local = Frame {
            camera_id: frame.camera_id,
            image: owned,
            timestamp: frame.timestamp,
        };
        return self
            .pick_in_frame(&local)
            .map(|p| Pixel::new(p.x + f64::from(r.x), p.y + f64::from(r.y)));
    }

    fn note_hit(&mut self, p: Pixel) {
        let delta = self
            .last
            .map(|prev| {
                let dx = p.x - prev.x;
                let dy = p.y - prev.y;
                (dx * dx + dy * dy).sqrt()
            })
            .unwrap_or(0.0);
        self.last_delta_px = delta;
        self.last = Some(p);
        self.half_px = self.params.compute_half(self.last_area, self.last_delta_px);
    }

    fn clear_track(&mut self) {
        self.last = None;
        self.last_area = None;
        self.last_delta_px = 0.0;
        self.half_px = self.params.half_min;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Id;
    use crate::detector::{
        AppearanceChain, ColorSpace, ColormaskDetector, ColormaskParams, ContourDetector,
        ScorerParams,
    };
    use opencv::core::{CV_8UC3, Point, Scalar, Size};
    use opencv::imgproc;
    use std::time::Instant;

    fn blob_frame() -> Frame {
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

    fn loose_color() -> ColormaskParams {
        return ColormaskParams {
            space: ColorSpace::Ycrcb,
            c0_min: 50,
            c0_max: 255,
            c1_min: 0,
            c1_max: 255,
            c2_min: 0,
            c2_max: 255,
        };
    }

    fn color_track(half: impl Into<RoiParams>) -> RoiTrack {
        let appearance = AppearanceChain::new().then(ColormaskDetector::new(loose_color()));
        return track(appearance, Scorer::shape(20.0, 20_000.0, 0.55), half);
    }

    #[test]
    fn track_acquires_then_uses_roi() {
        let frame = blob_frame();
        let mut d = color_track(40);
        let p0 = d.detect(&frame).expect("acquire");
        assert!(!d.used_roi);
        assert!((p0.x - 100.0).abs() < 5.0);

        let p1 = d.detect(&frame).expect("track");
        assert!(d.used_roi);
        assert!((p1.x - 100.0).abs() < 5.0);

        d.set_roi_enabled(false);
        let p2 = d.detect(&frame).expect("roi off");
        assert!(!d.used_roi);
        assert!((p2.x - 100.0).abs() < 5.0);
    }

    #[test]
    fn adaptive_half_uses_area() {
        let frame = blob_frame();
        let params = RoiParams {
            radius_scale: 2.0,
            padding: 10,
            motion_scale: 0.0,
            half_min: 20,
            half_max: 200,
        };
        let mut d = color_track(params.clone());
        d.detect(&frame).expect("acquire");
        let area = d.last_area().expect("area");
        assert_eq!(d.half_px, params.compute_half(Some(area), 0.0));
        assert!(d.half_px > 20);
    }

    #[test]
    fn appearance_chain_scores_blob() {
        let frame = blob_frame();
        let scorer_p = ScorerParams {
            min_area_px: 20.0,
            max_area_px: 20_000.0,
            min_circularity: 0.55,
        };
        let appearance = AppearanceChain::new()
            .then(ColormaskDetector::new(loose_color()))
            .then(ContourDetector::from(&scorer_p));
        let mut d = track(
            appearance,
            Scorer::from(&scorer_p).with_motion_weight(0.5),
            80,
        );
        let p = d.detect(&frame).expect("hit");
        assert!((p.x - 100.0).abs() < 8.0);
        assert!(d.last_area().unwrap() > 0.0);
    }
}
