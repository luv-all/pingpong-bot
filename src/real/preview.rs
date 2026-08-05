//! 메인 스레드 검출 프리뷰 (highgui는 macOS에서 메인 스레드 전용).
//!
//! 추정 워커가 drop-on-full로 보내주므로 여기가 느려도 핫패스는 막히지 않는다.
//! 오버레이는 전부 `camera::Preview` 파사드를 쓴다.

use std::collections::BTreeMap;
use std::time::Instant;

use opencv::core::{Mat, Scalar};
use opencv::prelude::*;
use pingpong_bot::camera;
use pingpong_bot::camera::PreviewAction;
use tracing::warn;

use super::ControlStateSnapshot;
use super::PreviewEvent;
use super::TestZone;

/// 검출 마커 (BGR).
const DETECTION_COLOR: Scalar = Scalar::new(64.0, 220.0, 64.0, 0.0);
/// 제어 목표 위치 재투영 마커 — 검출과 헷갈리지 않게 다른 색·다른 크기.
const TARGET_COLOR: Scalar = Scalar::new(80.0, 80.0, 255.0, 0.0);
/// 새 일괄 적합 상태 재투영.
const FITTED_COLOR: Scalar = Scalar::new(255.0, 255.0, 255.0, 0.0);
const HUD_COLOR: Scalar = Scalar::new(0.0, 255.0, 255.0, 0.0);
/// 최근 제어 명령이나 오류를 고정으로 남기는 결과 줄.
const STICKY_COLOR: Scalar = Scalar::new(255.0, 200.0, 120.0, 0.0);
const MARKER_RADIUS_PX: i32 = 12;
const TARGET_RADIUS_PX: i32 = 18;
const MARKER_THICKNESS_PX: i32 = 2;

/// 상태 패널 — 고정 크기, `--preview` 모자이크 우상단.
const STATE_PANEL_W: i32 = 250;
const STATE_PANEL_H: i32 = 150;
const STATE_PANEL_MARGIN_PX: i32 = 14;
const STATE_NODE_W: i32 = 80;
const STATE_NODE_H: i32 = 32;
const STATE_NODE_Y_OFFSET: i32 = 34;
const STATE_NODE_GAP: i32 = 20;
const STATE_BG_COLOR: Scalar = Scalar::new(18.0, 14.0, 10.0, 0.0);
const STATE_IDLE_COLOR: Scalar = Scalar::new(120.0, 110.0, 100.0, 0.0);
const STATE_ACTIVE_COLOR: Scalar = Scalar::new(20.0, 150.0, 235.0, 0.0);

/// 캠별 최신 프레임을 모아 한 창에 가로로 붙여 띄운다.
pub struct PreviewWindow {
    window: String,
    /// `camera::Id.0` → 마커까지 그려 넣은 프레임. 좌/우 순서 유지를 위해 BTreeMap.
    panels: BTreeMap<u8, Mat>,
    hud: Vec<String>,
    /// 최근 제어 결과 — 다음 결과가 올 때까지 남는다.
    sticky: Vec<String>,
    /// `camera::Id.0` → **마지막으로 본** 예측 도달점 픽셀.
    ///
    /// 프레임마다 새로 그리므로 예측이 몇 프레임만 잡히면 마커가 번쩍하고 사라진다
    /// (30~120 fps에서는 눈에 안 띈다). 마지막 값을 붙들어 계속 그려서 "어디를 치려
    /// 했는지"가 화면에 남게 한다.
    last_target: BTreeMap<u8, camera::Pixel>,
    /// 최근 제어 상태 — 다음 상태가 올 때까지 남는다.
    control_state: Option<ControlStateSnapshot>,
    /// 현재 테스트 존과 그 준비 레일 x — 상태 패널이 소비한다.
    current_zone: Option<(TestZone, f64)>,
}

impl PreviewWindow {
    pub fn new(window: impl Into<String>) -> Self {
        return Self {
            window: window.into(),
            panels: BTreeMap::new(),
            hud: Vec::new(),
            sticky: Vec::new(),
            last_target: BTreeMap::new(),
            control_state: None,
            current_zone: None,
        };
    }

    /// 최근 제어 결과를 화면에 고정한다.
    pub fn set_result(&mut self, lines: Vec<String>) {
        self.sticky = lines;
    }

    /// 최근 제어 상태를 화면에 반영한다.
    pub fn set_control_state(&mut self, state: ControlStateSnapshot) {
        self.control_state = Some(state);
    }

    /// 현재 테스트 존과 그 준비 레일 x를 화면에 반영한다.
    pub fn set_zone(&mut self, zone: TestZone, home_rail_x: f64) {
        self.current_zone = Some((zone, home_rail_x));
    }

    /// 프레임 1장을 받아 마커를 그려 보관한다. 프레임은 여기서 소비된다.
    pub fn push(&mut self, event: PreviewEvent) {
        let camera_id = event.frame.camera_id;
        let mut image = event.frame.image;

        // 초록 = 이 프레임의 검출 후보, 흰색 = 새 vision::Fit 상태 재투영.
        draw_marker(&mut image, event.pixel, MARKER_RADIUS_PX, DETECTION_COLOR);
        draw_marker(
            &mut image,
            event.fitted_pixel,
            MARKER_RADIUS_PX / 2,
            FITTED_COLOR,
        );

        // 빨강 = 예측 도달 위치. 화면 밖이면 가장자리로 끌어와 방향만 보여준다 —
        // 안 그리면 "예측이 없다"와 구분이 안 된다.
        if let Some(pixel) = event.target_pixel {
            let pixel = if event.target_offscreen {
                clamp_to_frame(pixel, &image)
            } else {
                pixel
            };
            self.last_target.insert(camera_id.0, pixel);
        }
        if let Some(pixel) = self.last_target.get(&camera_id.0).copied() {
            draw_marker(&mut image, Some(pixel), TARGET_RADIUS_PX, TARGET_COLOR);
        }

        self.hud = event.hud;
        self.panels.insert(camera_id.0, image);
    }

    /// 창을 갱신한다. 반환값은 highgui 키 입력 그대로 — 호출측이 Quit/Key를 처리한다.
    pub fn show(&mut self) -> PreviewAction {
        if self.panels.is_empty() {
            return PreviewAction::Continue;
        }
        return match self.render() {
            Ok(action) => action,
            Err(error) => {
                warn!(%error, "프리뷰 렌더 실패");
                PreviewAction::Continue
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
        if let Some(state) = &self.control_state {
            draw_control_state_panel(&mut mosaic, state, self.current_zone)?;
        }
        return Ok(camera::Preview::show_bgr(&self.window, &mosaic, 1)?.action);
    }

    pub fn close(&self) {
        camera::Preview::destroy_window(&self.window);
    }
}

/// 프레임 밖 좌표를 테두리 안쪽으로 끌어온다 (방향만 남긴다).
fn clamp_to_frame(pixel: camera::Pixel, image: &Mat) -> camera::Pixel {
    let margin = f64::from(TARGET_RADIUS_PX);
    let max_x = (f64::from(image.cols()) - 1.0 - margin).max(margin);
    let max_y = (f64::from(image.rows()) - 1.0 - margin).max(margin);
    return camera::Pixel::new(pixel.x.clamp(margin, max_x), pixel.y.clamp(margin, max_y));
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

/// `IDLE`/`ALIGN` 두 노드 다이어그램을 모자이크 우상단에 그린다.
fn draw_control_state_panel(
    image: &mut Mat,
    state: &ControlStateSnapshot,
    zone: Option<(TestZone, f64)>,
) -> opencv::Result<()> {
    let panel_x = image.cols() - STATE_PANEL_W - STATE_PANEL_MARGIN_PX;
    let panel_y = STATE_PANEL_MARGIN_PX;
    camera::Preview::draw_rect_px(
        image,
        camera::Pixel::new(f64::from(panel_x), f64::from(panel_y)),
        STATE_PANEL_W,
        STATE_PANEL_H,
        STATE_BG_COLOR,
        -1,
    )?;

    let align_active = !matches!(state, ControlStateSnapshot::Idle);
    let idle_color = if align_active {
        STATE_IDLE_COLOR
    } else {
        STATE_ACTIVE_COLOR
    };
    let align_color = if align_active {
        STATE_ACTIVE_COLOR
    } else {
        STATE_IDLE_COLOR
    };

    let idle_x = panel_x + 14;
    let node_y = panel_y + STATE_NODE_Y_OFFSET;
    let struck_x = idle_x + STATE_NODE_W + STATE_NODE_GAP;

    camera::Preview::draw_rect_px(
        image,
        camera::Pixel::new(f64::from(idle_x), f64::from(node_y)),
        STATE_NODE_W,
        STATE_NODE_H,
        idle_color,
        -1,
    )?;
    camera::Preview::draw_text_at_px(
        image,
        camera::Pixel::new(f64::from(idle_x + 10), f64::from(node_y + 21)),
        "IDLE",
        0.5,
        Scalar::new(0.0, 0.0, 0.0, 0.0),
        1,
    )?;

    camera::Preview::draw_rect_px(
        image,
        camera::Pixel::new(f64::from(struck_x), f64::from(node_y)),
        STATE_NODE_W,
        STATE_NODE_H,
        align_color,
        -1,
    )?;
    camera::Preview::draw_text_at_px(
        image,
        camera::Pixel::new(f64::from(struck_x + 4), f64::from(node_y + 21)),
        "ALIGN",
        0.42,
        Scalar::new(0.0, 0.0, 0.0, 0.0),
        1,
    )?;

    if let ControlStateSnapshot::Aligning {
        track_seq,
        return_due_at,
        rail_commanded_m,
        aim_commanded_rad,
    } = state
    {
        let remaining = return_due_at
            .saturating_duration_since(Instant::now())
            .as_secs_f64();
        let line1 = format!("track {track_seq}  returns {remaining:.2}s");
        let line2 = format!(
            "rail {:.3}m  aim {:.1}deg",
            rail_commanded_m,
            aim_commanded_rad.to_degrees()
        );
        camera::Preview::draw_text_at_px(
            image,
            camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 80)),
            &line1,
            0.4,
            STATE_ACTIVE_COLOR,
            1,
        )?;
        camera::Preview::draw_text_at_px(
            image,
            camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 98)),
            &line2,
            0.4,
            STATE_ACTIVE_COLOR,
            1,
        )?;
    }

    if let Some((zone, home_rail_x)) = zone {
        let zone_line = format!("ZONE {}  x={home_rail_x:.3}", zone.label());
        camera::Preview::draw_text_at_px(
            image,
            camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 116)),
            &zone_line,
            0.42,
            STATE_ACTIVE_COLOR,
            1,
        )?;
    }
    camera::Preview::draw_text_at_px(
        image,
        camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 134)),
        "1/2/3 zone  w wait  r reset",
        0.35,
        STATE_IDLE_COLOR,
        1,
    )?;
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;
    use opencv::core::Vec3b;
    use std::time::{Duration, Instant};

    #[test]
    fn set_zone_stores_the_current_zone_and_home_x() {
        let mut window = PreviewWindow::new("test");
        assert!(window.current_zone.is_none());
        window.set_zone(TestZone::Right, 1.34);
        assert_eq!(window.current_zone, Some((TestZone::Right, 1.34)));
    }

    fn idle_pixel(img: &Mat) -> Vec3b {
        return *img.at_2d::<Vec3b>(64, 290).unwrap();
    }

    fn struck_pixel(img: &Mat) -> Vec3b {
        return *img.at_2d::<Vec3b>(64, 390).unwrap();
    }

    #[test]
    fn idle_state_highlights_the_idle_node() {
        let mut img = Mat::zeros(200, 500, opencv::core::CV_8UC3)
            .unwrap()
            .to_mat()
            .unwrap();
        draw_control_state_panel(&mut img, &ControlStateSnapshot::Idle, None).unwrap();
        assert_eq!(idle_pixel(&img), Vec3b::from([20, 150, 235]));
        assert_eq!(struck_pixel(&img), Vec3b::from([120, 110, 100]));
    }

    #[test]
    fn aligning_state_highlights_the_align_node() {
        let mut img = Mat::zeros(200, 500, opencv::core::CV_8UC3)
            .unwrap()
            .to_mat()
            .unwrap();
        let state = ControlStateSnapshot::Aligning {
            track_seq: 7,
            return_due_at: Instant::now() + Duration::from_millis(300),
            rail_commanded_m: 0.30,
            aim_commanded_rad: -0.40,
        };
        draw_control_state_panel(&mut img, &state, None).unwrap();
        assert_eq!(idle_pixel(&img), Vec3b::from([120, 110, 100]));
        assert_eq!(struck_pixel(&img), Vec3b::from([20, 150, 235]));
    }
}
