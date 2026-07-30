use crate::camera;

use super::Frame;

/// 카메라/파일에서 BGR 프레임을 낸다.
pub trait FrameSource: Send {
    fn next_frame(&mut self) -> Option<Frame>;

    fn camera_id(&self) -> camera::Id;
}
