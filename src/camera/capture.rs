//! 프레임 소스 (sim / OpenCV VideoCapture).

use std::time::Instant;

use crate::{CameraId, PixelPoint};

/// 카메라에서 (id, 검출 힌트 픽셀, 시각)을 낸다.
///
/// domain `CameraSource` 포트 대신 infra에 둔다. OpenCV 캡처도 이걸 구현한다.
pub trait FrameSource: Send {
    /// 프레임이 없으면 `None`. 힌트 픽셀이 없으면 `Some((id, None, t))`.
    fn next(&mut self) -> Option<(CameraId, Option<PixelPoint>, Instant)>;
}
