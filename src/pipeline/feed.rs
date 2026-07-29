//! 카메라 입력 피드.

use crate::camera;
use crate::camera::{FrameSource, HintSource};
use crate::detector::Detector;

/// 카메라 입력: sim 힌트 또는 실캠 프레임+검출.
pub enum CameraFeed {
    /// sim — 투영 픽셀을 그대로 observation으로.
    Hint(Box<dyn HintSource>),
    /// 실물 — capture → undistort → detect.
    Detect {
        source: Box<dyn FrameSource>,
        detector: Box<Detector>,
        params: camera::Params,
    },
}
