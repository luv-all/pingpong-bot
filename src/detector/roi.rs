//! ROI 창 추적 검출 (이전 검출 주변 탐색).

use opencv::core::Rect;
use opencv::prelude::*;

use super::{BallDetector, ColormaskConfig, ColormaskDetector};
use crate::PixelPoint;
use crate::camera::Frame;

pub struct RoiDetector {
    inner: ColormaskDetector,
    last: Option<PixelPoint>,
    half_window: i32,
}

impl RoiDetector {
    pub fn new() -> Self {
        return Self {
            inner: ColormaskDetector::new(ColormaskConfig::default()),
            last: None,
            half_window: 80,
        };
    }
}

impl Default for RoiDetector {
    fn default() -> Self {
        return Self::new();
    }
}

impl BallDetector for RoiDetector {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        let size = frame.image.size().ok()?;
        let pixel = if let Some(prev) = self.last {
            let x0 = (prev.x as i32 - self.half_window).max(0);
            let y0 = (prev.y as i32 - self.half_window).max(0);
            let x1 = (prev.x as i32 + self.half_window).min(size.width);
            let y1 = (prev.y as i32 + self.half_window).min(size.height);
            let w = (x1 - x0).max(1);
            let h = (y1 - y0).max(1);
            let roi = Rect::new(x0, y0, w, h);
            let cropped = Mat::roi(&frame.image, roi).ok()?;
            let local = Frame {
                camera_id: frame.camera_id,
                image: cropped.try_clone().ok()?,
                timestamp: frame.timestamp,
            };
            self.inner
                .detect(&local)
                .map(|p| PixelPoint::new(p.x + f64::from(x0), p.y + f64::from(y0)))
                .or_else(|| self.inner.detect(frame))
        } else {
            self.inner.detect(frame)
        }?;
        self.last = Some(pixel);
        return Some(pixel);
    }
}
