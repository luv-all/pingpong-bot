//! 메인 스레드 검출 프리뷰 (highgui는 macOS에서 메인 스레드 전용).
//!
//! 추정 워커가 drop-on-full로 보내주므로 여기가 느려도 핫패스는 막히지 않는다.
//! 오버레이는 전부 `camera::Preview` 파사드를 쓴다.

use std::collections::BTreeMap;

use opencv::core::{Mat, Scalar};
use opencv::prelude::*;
use pingpong_bot::camera;
use pingpong_bot::camera::PreviewAction;
use tracing::warn;

use super::PreviewEvent;

/// 검출 마커 (BGR).
const DETECTION_COLOR: Scalar = Scalar::new(64.0, 220.0, 64.0, 0.0);
const HUD_COLOR: Scalar = Scalar::new(0.0, 255.0, 255.0, 0.0);
const MARKER_RADIUS_PX: i32 = 12;
const MARKER_THICKNESS_PX: i32 = 2;

/// 캠별 최신 프레임을 모아 한 창에 가로로 붙여 띄운다.
pub struct PreviewWindow {
    window: String,
    /// `camera::Id.0` → 마커까지 그려 넣은 프레임. 좌/우 순서 유지를 위해 BTreeMap.
    panels: BTreeMap<u8, Mat>,
    hud: Vec<String>,
}

impl PreviewWindow {
    pub fn new(window: impl Into<String>) -> Self {
        return Self {
            window: window.into(),
            panels: BTreeMap::new(),
            hud: Vec::new(),
        };
    }

    /// 프레임 1장을 받아 마커를 그려 보관한다. 프레임은 여기서 소비된다.
    pub fn push(&mut self, event: PreviewEvent) {
        let camera_id = event.frame.camera_id;
        let mut image = event.frame.image;
        if let Some(pixel) = event.pixel
            && let Err(error) = camera::Preview::draw_circle_px(
                &mut image,
                pixel,
                MARKER_RADIUS_PX,
                DETECTION_COLOR,
                MARKER_THICKNESS_PX,
            )
        {
            warn!(%error, "검출 마커 그리기 실패");
        }
        self.hud = event.hud;
        self.panels.insert(camera_id.0, image);
    }

    /// 창을 갱신한다. 반환 `true` = 사용자가 종료(ESC/`q`)를 눌렀다.
    pub fn show(&mut self) -> bool {
        if self.panels.is_empty() {
            return false;
        }
        return match self.render() {
            Ok(action) => action == PreviewAction::Quit,
            Err(error) => {
                warn!(%error, "프리뷰 렌더 실패");
                false
            }
        };
    }

    fn render(&self) -> opencv::Result<PreviewAction> {
        let mut panels = Vec::with_capacity(self.panels.len());
        for image in self.panels.values() {
            panels.push(image.try_clone()?);
        }
        let mut mosaic = camera::Preview::hstack_bgr(&panels)?;
        if !self.hud.is_empty() {
            camera::Preview::draw_debug_lines(&mut mosaic, &self.hud, HUD_COLOR)?;
        }
        return Ok(camera::Preview::show_bgr(&self.window, &mosaic, 1)?.action);
    }

    pub fn close(&self) {
        camera::Preview::destroy_window(&self.window);
    }
}
