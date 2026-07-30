//! 추정 워커 → 메인 스레드 프리뷰 메시지.

use pingpong_bot::camera;

/// 프리뷰 창에 그릴 프레임 1장 + 오버레이.
///
/// 추정 워커가 `try_send`로 보내고 채널이 차면 **버린다** — 프리뷰가 핫패스를 막지 않는다.
pub struct PreviewEvent {
    pub frame: camera::Frame,
    pub pixel: Option<camera::Pixel>,
    /// 예측 도달 위치를 **이 카메라로 재투영**한 픽셀. 카메라 뒤면 `None`.
    ///
    /// 프레임 밖 좌표도 그대로 담는다 (`project_world_unclipped`) — 잘라서 `None`으로
    /// 만들면 "예측이 없는 것"과 "화면 밖인 것"이 구분되지 않는다.
    pub impact_pixel: Option<camera::Pixel>,
    /// [`Self::impact_pixel`]이 이 프레임 밖인가.
    pub impact_offscreen: bool,
    /// **필터를 거치지 않은** 생 삼각측량 점을 이 카메라로 되쏜 픽셀.
    ///
    /// 초록(검출)과 벌어진 거리가 곧 재투영 오차다 — 2D는 멀쩡한데 3D가 나쁜 경우를
    /// 눈으로 잡는다.
    pub raw_pixel: Option<camera::Pixel>,
    /// 화면 좌상단 HUD 줄들 (추정 상태·게이트 단계).
    pub hud: Vec<String>,
}
