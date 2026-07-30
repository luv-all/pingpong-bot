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
/// 예측 도달 위치 재투영 마커 — 검출과 헷갈리지 않게 다른 색·다른 크기.
const IMPACT_COLOR: Scalar = Scalar::new(80.0, 80.0, 255.0, 0.0);
const HUD_COLOR: Scalar = Scalar::new(0.0, 255.0, 255.0, 0.0);
/// 샷이 끝난 뒤 고정으로 남기는 결과 줄 (커밋 요약·포기 사유).
const STICKY_COLOR: Scalar = Scalar::new(255.0, 200.0, 120.0, 0.0);
const MARKER_RADIUS_PX: i32 = 12;
const IMPACT_RADIUS_PX: i32 = 18;
const MARKER_THICKNESS_PX: i32 = 2;

/// 캠별 최신 프레임을 모아 한 창에 가로로 붙여 띄운다.
pub struct PreviewWindow {
    window: String,
    /// `camera::Id.0` → 마커까지 그려 넣은 프레임. 좌/우 순서 유지를 위해 BTreeMap.
    panels: BTreeMap<u8, Mat>,
    hud: Vec<String>,
    /// 샷 결과 — 한 번 정해지면 창을 닫을 때까지 남는다.
    sticky: Vec<String>,
}

impl PreviewWindow {
    pub fn new(window: impl Into<String>) -> Self {
        return Self {
            window: window.into(),
            panels: BTreeMap::new(),
            hud: Vec::new(),
            sticky: Vec::new(),
        };
    }

    /// 샷 결과를 화면에 고정한다 (창을 닫을 때까지 남는다).
    pub fn set_result(&mut self, lines: Vec<String>) {
        self.sticky = lines;
    }

    /// 프레임 1장을 받아 마커를 그려 보관한다. 프레임은 여기서 소비된다.
    pub fn push(&mut self, event: PreviewEvent) {
        let camera_id = event.frame.camera_id;
        let mut image = event.frame.image;
        // 초록 = 검출한 공, 빨강 = 예측 도달 위치를 이 카메라로 재투영한 자리.
        draw_marker(&mut image, event.pixel, MARKER_RADIUS_PX, DETECTION_COLOR);
        draw_marker(
            &mut image,
            event.impact_pixel,
            IMPACT_RADIUS_PX,
            IMPACT_COLOR,
        );
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
        // 결과는 우하단 도움말 자리에 — 좌상단 실시간 HUD와 겹치지 않는다.
        if !self.sticky.is_empty() {
            camera::Preview::draw_help_lines(&mut mosaic, &self.sticky, STICKY_COLOR)?;
        }
        return Ok(camera::Preview::show_bgr(&self.window, &mosaic, 1)?.action);
    }

    pub fn close(&self) {
        camera::Preview::destroy_window(&self.window);
    }
}

fn draw_marker(image: &mut Mat, pixel: Option<camera::Pixel>, radius: i32, color: Scalar) {
    let Some(pixel) = pixel else {
        return;
    };
    if let Err(error) =
        camera::Preview::draw_circle_px(image, pixel, radius, color, MARKER_THICKNESS_PX)
    {
        warn!(%error, "마커 그리기 실패");
    }
}
