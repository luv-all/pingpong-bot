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
    pub target_pixel: Option<camera::Pixel>,
    /// [`Self::target_pixel`]이 이 프레임 밖인가.
    pub target_offscreen: bool,
    /// 이 예측이 속한 트랙 번호. 예측 전이면 `None`.
    ///
    /// 프리뷰가 마커를 다음 커밋까지 붙들고 있으므로, 공이 바뀐 걸 알 방법이 필요하다 —
    /// 이 값이 바뀌면 지난 공의 마커를 버린다. 없으면 새 공에 옛 조준점이 겹쳐 보인다.
    pub track_seq: Option<u64>,
    /// 새 `vision::Fit`의 현재 3D 상태를 이 카메라로 되쏜 픽셀.
    pub fitted_pixel: Option<camera::Pixel>,
    /// 화면 좌상단 HUD 줄들 (추정 상태·게이트 단계).
    pub hud: Vec<String>,
}
