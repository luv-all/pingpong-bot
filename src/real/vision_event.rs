//! 카메라 워커 → 추정 워커 메시지.

use pingpong_bot::camera;

/// 프레임 1장 + 그 프레임의 검출 결과.
///
/// `frame`은 `Mat`을 **소유**한 채 채널로 이동한다 (복사 없음). 추정 워커가 픽셀을 쓰고 나면
/// 프레임은 그대로 프리뷰로 넘어가거나, 프리뷰가 꺼져 있으면 거기서 drop된다.
pub struct VisionEvent {
    pub frame: camera::Frame,
    /// `None` = 이 프레임에서 공을 못 찾음.
    pub pixel: Option<camera::Pixel>,
}
