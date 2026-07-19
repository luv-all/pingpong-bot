//! 공 검출 — `BallDetector` 구현 + sim 패스스루.

mod bgsub;
mod colormask;
mod contour;
mod roi;
pub mod run_tool;
mod undistort;

use crate::PixelPoint;
use crate::camera::Frame;

pub use bgsub::BgSubDetector;
pub use colormask::{ColorSpace, ColormaskConfig, ColormaskDetector};
pub use contour::ContourDetector;
pub use roi::RoiDetector;
pub use run_tool::{DetectToolOptions, open_frame_source, run_detect_tool};
pub use undistort::undistort_frame;

/// 프레임에서 공 픽셀을 찾는다. `detect_*` 툴과 런타임이 공유한다.
pub trait BallDetector: Send {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint>;
}

/// sim: 카메라가 이미 넣은 힌트 픽셀을 그대로 쓴다.
pub fn passthrough_detect(hint: Option<PixelPoint>) -> Option<PixelPoint> {
    return hint;
}

/// hint 패스스루 검출기 (상태 없음).
#[derive(Debug, Default, Clone, Copy)]
pub struct PassthroughDetector;

impl PassthroughDetector {
    pub fn new() -> Self {
        return Self;
    }

    pub fn detect(&mut self, hint: Option<PixelPoint>) -> Option<PixelPoint> {
        return passthrough_detect(hint);
    }
}

/// TOML `vision.detector` 이름.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectorKind {
    Colormask,
    Bgsub,
    Contour,
    Roi,
}

impl DetectorKind {
    pub fn parse(s: &str) -> Option<Self> {
        return match s {
            "colormask" => Some(Self::Colormask),
            "bgsub" => Some(Self::Bgsub),
            "contour" => Some(Self::Contour),
            "roi" => Some(Self::Roi),
            _ => None,
        };
    }

    pub fn as_str(self) -> &'static str {
        return match self {
            Self::Colormask => "colormask",
            Self::Bgsub => "bgsub",
            Self::Contour => "contour",
            Self::Roi => "roi",
        };
    }
}

/// 설정에서 검출기 인스턴스를 만든다.
pub fn build_detector(kind: DetectorKind, colormask: ColormaskConfig) -> Box<dyn BallDetector> {
    return match kind {
        DetectorKind::Colormask => Box::new(ColormaskDetector::new(colormask)),
        DetectorKind::Bgsub => Box::new(BgSubDetector::new()),
        DetectorKind::Contour => Box::new(ContourDetector::new()),
        DetectorKind::Roi => Box::new(RoiDetector::new()),
    };
}
