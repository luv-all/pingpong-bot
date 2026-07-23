//! ROI 추적 래퍼 — 전체 탐색으로 잡은 뒤, 놓치기 전까지 ROI만 본다.
//!
//! ```ignore
//! let mut d = track(inner, 80);
//! d.set_roi_enabled(false); // 전체 프레임만 (detect-full `r` 토글)
//! ```
//!
//! 1. `last` 없음 → 전체 프레임에서 `inner` 검출 (acquire)
//! 2. `last` 있음 → 그 주변 ROI에서 `inner` (track)
//! 3. ROI miss → 전체에서 다시 (reacquire); 그것도 miss면 `last` 클리어

use opencv::core::Rect;
use opencv::prelude::*;

use super::BallDetector;
use crate::PixelPoint;
use crate::camera::Frame;

/// 탐색 영역 정책. 안쪽은 아무 [`BallDetector`].
pub struct RoiTrack {
    inner: Box<dyn BallDetector>,
    /// 직전 hit 기준 정사각 ROI 한 변의 절반 [px]. `80` → 최대 160×160.
    pub roi_half_px: i32,
    /// `false`면 매 프레임 전체 탐색만 ([`Self::set_roi_enabled`]).
    pub roi_enabled: bool,
    last: Option<PixelPoint>,
    /// 이번 프레임에 쓴 ROI (전체 탐색이면 `None`).
    pub last_roi: Option<Rect>,
    /// 이번 hit이 ROI track에서 나왔는지.
    pub used_roi: bool,
}

/// `inner` + [`roi_half_px`](RoiTrack::roi_half_px). ROI 기본 on.
pub fn track(inner: impl BallDetector + 'static, roi_half_px: i32) -> RoiTrack {
    return RoiTrack {
        inner: Box::new(inner),
        roi_half_px,
        roi_enabled: true,
        last: None,
        last_roi: None,
        used_roi: false,
    };
}

impl std::fmt::Display for RoiTrack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return write!(
            f,
            "track(roi_half={}, roi={})",
            self.roi_half_px,
            if self.roi_enabled { "on" } else { "off" }
        );
    }
}

impl RoiTrack {
    /// ROI 추적 on/off. off로 바꿀 때 track 상태를 비운다.
    pub fn set_roi_enabled(&mut self, enabled: bool) {
        self.roi_enabled = enabled;
        if !enabled {
            self.last = None;
            self.last_roi = None;
            self.used_roi = false;
        }
    }

    fn roi_rect(prev: PixelPoint, half: i32, frame: &Frame) -> Option<Rect> {
        let size = frame.image.size().ok()?;
        let x0 = (prev.x as i32 - half).max(0);
        let y0 = (prev.y as i32 - half).max(0);
        let x1 = (prev.x as i32 + half).min(size.width);
        let y1 = (prev.y as i32 + half).min(size.height);
        let w = (x1 - x0).max(1);
        let h = (y1 - y0).max(1);
        return Some(Rect::new(x0, y0, w, h));
    }

    fn detect_region(
        inner: &mut dyn BallDetector,
        frame: &Frame,
        roi: Option<Rect>,
    ) -> Option<PixelPoint> {
        let Some(r) = roi else {
            return inner.detect(frame);
        };
        let roi_mat = Mat::roi(&frame.image, r).ok()?;
        let owned = roi_mat.try_clone().ok()?;
        let local = Frame {
            camera_id: frame.camera_id,
            image: owned,
            timestamp: frame.timestamp,
        };
        return inner
            .detect(&local)
            .map(|p| PixelPoint::new(p.x + f64::from(r.x), p.y + f64::from(r.y)));
    }
}

impl BallDetector for RoiTrack {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        self.last_roi = None;
        self.used_roi = false;

        if !self.roi_enabled {
            if let Some(p) = Self::detect_region(self.inner.as_mut(), frame, None) {
                self.last = Some(p);
                return Some(p);
            }
            self.last = None;
            return None;
        }

        if let Some(prev) = self.last {
            if let Some(r) = Self::roi_rect(prev, self.roi_half_px, frame) {
                self.last_roi = Some(r);
                if let Some(p) = Self::detect_region(self.inner.as_mut(), frame, Some(r)) {
                    self.last = Some(p);
                    self.used_roi = true;
                    return Some(p);
                }
            }
            // ROI miss → 전체 재탐색
        }

        self.last_roi = None;
        if let Some(p) = Self::detect_region(self.inner.as_mut(), frame, None) {
            self.last = Some(p);
            self.used_roi = false;
            return Some(p);
        }

        self.last = None;
        return None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CameraId;
    use crate::detector::{ColorSpace, ColormaskParams, ColormaskDetector};
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
        return Frame::new(CameraId(0), img, Instant::now());
    }

    #[test]
    fn track_acquires_then_uses_roi() {
        let cfg = ColormaskParams {
            space: ColorSpace::Ycrcb,
            c0_min: 50,
            c0_max: 255,
            c1_min: 0,
            c1_max: 255,
            c2_min: 0,
            c2_max: 255,
        };
        let frame = blob_frame();
        let mut d = track(ColormaskDetector::new(cfg), 40);
        assert_eq!(d.to_string(), "track(roi_half=40, roi=on)");

        let p0 = d.detect(&frame).expect("acquire");
        assert!(!d.used_roi);
        assert!(d.last_roi.is_none());
        assert!((p0.x - 100.0).abs() < 5.0);

        let p1 = d.detect(&frame).expect("track");
        assert!(d.used_roi);
        assert!(d.last_roi.is_some());
        assert!((p1.x - 100.0).abs() < 5.0);

        d.set_roi_enabled(false);
        let p2 = d.detect(&frame).expect("roi off");
        assert!(!d.used_roi);
        assert!(d.last_roi.is_none());
        assert!((p2.x - 100.0).abs() < 5.0);
    }
}
