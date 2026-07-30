//! 추정 워커 → 메인 스레드 프리뷰 메시지.

use pingpong_bot::camera;

/// 프리뷰 창에 그릴 프레임 1장 + 오버레이.
///
/// 추정 워커가 `try_send`로 보내고 채널이 차면 **버린다** — 프리뷰가 핫패스를 막지 않는다.
pub struct PreviewEvent {
    pub frame: camera::Frame,
    pub pixel: Option<camera::Pixel>,
    /// 화면 좌상단 HUD 줄들 (추정 상태·게이트 단계).
    pub hud: Vec<String>,
}
