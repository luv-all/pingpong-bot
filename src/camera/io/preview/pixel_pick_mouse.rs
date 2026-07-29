use opencv::core::{Mat, Point, Scalar};
use opencv::imgproc;
use opencv::prelude::*;
use opencv::{Result as CvResult, highgui};

use super::ops::unscale_xy;

/// 픽셀 정밀 찍기용 loupe — [`crate::defaults::vision`].
pub use crate::defaults::vision::{PIXEL_LOUPE_SRC_HALF, PIXEL_LOUPE_ZOOM};

/// highgui 마우스: LMB/Enter 픽 큐 + Shift/nudge loupe + 방향키·hjkl 1px.
///
/// 좌표 규약 (툴은 매 프레임 [`Self::sync`] 후 읽기):
/// - [`Self::drain_clicks`] / [`Self::hover`] / aim → **원본 이미지** 픽셀
/// - 마우스가 움직이면 aim을 마우스에 즉시 재동기화
/// - 마우스 정지 중 방향키/`hjkl`은 aim만 ±1px (원본 기준)
///
/// loupe는 Shift **또는** 키보드 nudge 중에 표시.
#[derive(Debug, Default, Clone)]
pub struct PixelPickMouse {
    clicks: Vec<(i32, i32)>,
    /// loupe 중심 (이미지 좌표). [`Self::sync`]·[`Self::nudge`]가 갱신.
    pub hover: Option<(i32, i32)>,
    pub shift: bool,
    mouse_win: Option<(i32, i32)>,
    /// 마우스 좌표가 바뀌면 true → 다음 sync에서 aim = mouse.
    mouse_moved: bool,
    pending_lmb: bool,
    aim_img: Option<(i32, i32)>,
    /// 키보드로 aim을 옮긴 뒤. 마우스 이동 시 해제.
    nudged: bool,
}

impl PixelPickMouse {
    /// `set_mouse_callback`에서 호출. Shift는 `EVENT_FLAG_SHIFTKEY`(크로스플랫폼).
    pub fn on_event(&mut self, event: i32, x: i32, y: i32, flags: i32) {
        self.shift = (flags & highgui::EVENT_FLAG_SHIFTKEY) != 0;
        let moved = self
            .mouse_win
            .map(|(mx, my)| mx != x || my != y)
            .unwrap_or(true);
        self.mouse_win = Some((x, y));
        if moved {
            self.mouse_moved = true;
        }
        if event == highgui::EVENT_LBUTTONDOWN {
            self.pending_lmb = true;
        }
    }

    /// 창→이미지 동기화. 매 프레임 `drain`/`hover` 읽기 **전에** 호출.
    pub fn sync(&mut self, scale: f64, img_w: i32, img_h: i32) {
        if img_w <= 0 || img_h <= 0 {
            return;
        }
        if self.mouse_moved {
            if let Some((wx, wy)) = self.mouse_win {
                let (ix, iy) = unscale_xy(wx, wy, scale);
                self.aim_img = Some((ix.clamp(0, img_w - 1), iy.clamp(0, img_h - 1)));
            }
            self.mouse_moved = false;
            self.nudged = false;
        } else if self.aim_img.is_none() {
            if let Some((wx, wy)) = self.mouse_win {
                let (ix, iy) = unscale_xy(wx, wy, scale);
                self.aim_img = Some((ix.clamp(0, img_w - 1), iy.clamp(0, img_h - 1)));
            }
        }
        if self.pending_lmb {
            if let Some(a) = self.aim_img {
                self.clicks.push(a);
            }
            self.pending_lmb = false;
        }
        self.hover = if self.shift || self.nudged {
            self.aim_img
        } else {
            None
        };
    }

    /// 원본 이미지 기준 1px 단위 nudge. aim이 아직 없으면 no-op.
    pub fn nudge(&mut self, dx: i32, dy: i32, img_w: i32, img_h: i32) {
        if img_w <= 0 || img_h <= 0 {
            return;
        }
        let Some((x, y)) = self.aim_img else {
            return;
        };
        self.aim_img = Some(((x + dx).clamp(0, img_w - 1), (y + dy).clamp(0, img_h - 1)));
        self.nudged = true;
        self.hover = self.aim_img;
    }

    /// Enter 등: 현재 aim을 클릭 큐에 넣는다.
    pub fn confirm(&mut self) {
        if let Some(a) = self.aim_img {
            self.clicks.push(a);
        }
    }

    /// 이미지 좌표 클릭 큐.
    pub fn drain_clicks(&mut self) -> Vec<(i32, i32)> {
        return std::mem::take(&mut self.clicks);
    }

    pub fn clear_clicks(&mut self) {
        self.clicks.clear();
        self.pending_lmb = false;
    }
}

/// [`super::PreviewAction::Key`] (waitKeyEx) → 이미지 (dx, dy).
///
/// 백엔드마다 코드가 다르다:
/// - **Win32**: VK가 상위 16비트 (`0x25xxxx` …). `(key >> 16) & 0xff`로 매칭
/// - **Cocoa**: `0xF700`–`0xF703` (하위 16비트)
/// - **X11/GTK**: `0xFF51`–`0xFF54`
/// - `hjkl`: 어느 백엔드에서든 동작하는 폴백 (`s`는 툴 단축키라 WASD 미사용)
pub fn arrow_delta(key: i32) -> Option<(i32, i32)> {
    // Win32 VK_* (waitKeyEx: virtual-key << 16). Shift 등 수정자 비트는 무시.
    const VK_LEFT: i32 = 0x25;
    const VK_UP: i32 = 0x26;
    const VK_RIGHT: i32 = 0x27;
    const VK_DOWN: i32 = 0x28;
    let win_vk = (key >> 16) & 0xff;
    if let Some(d) = match win_vk {
        VK_LEFT => Some((-1, 0)),
        VK_RIGHT => Some((1, 0)),
        VK_UP => Some((0, -1)),
        VK_DOWN => Some((0, 1)),
        _ => None,
    } {
        return Some(d);
    }

    // macOS Cocoa / X11 — 하위 16비트 (상위 수정자 무시)
    const MAC_UP: i32 = 0xF700;
    const MAC_DOWN: i32 = 0xF701;
    const MAC_LEFT: i32 = 0xF702;
    const MAC_RIGHT: i32 = 0xF703;
    const XK_LEFT: i32 = 0xFF51;
    const XK_UP: i32 = 0xFF52;
    const XK_RIGHT: i32 = 0xFF53;
    const XK_DOWN: i32 = 0xFF54;
    let code = key & 0xffff;
    if let Some(d) = match code {
        MAC_LEFT | XK_LEFT => Some((-1, 0)),
        MAC_RIGHT | XK_RIGHT => Some((1, 0)),
        MAC_UP | XK_UP => Some((0, -1)),
        MAC_DOWN | XK_DOWN => Some((0, 1)),
        _ => None,
    } {
        return Some(d);
    }

    match key & 0xff {
        k if k == i32::from(b'h') || k == i32::from(b'H') => Some((-1, 0)),
        k if k == i32::from(b'l') || k == i32::from(b'L') => Some((1, 0)),
        k if k == i32::from(b'k') || k == i32::from(b'K') => Some((0, -1)),
        k if k == i32::from(b'j') || k == i32::from(b'J') => Some((0, 1)),
        _ => None,
    }
}

/// `src`의 `(cx,cy)` 주변을 8× nearest로 확대해 `dst` 커서 위에 원형 loupe를 그린다.
///
/// `src`·`dst` 크기가 달라도 됨(모자이크 왼쪽 패널 등). 좌표는 둘 다 같은 원본 픽셀 기준.
/// 가장자리는 clamp 샘플. 중심 십자로 1px 정렬을 보이게 한다.
pub fn draw_pixel_loupe(dst: &mut Mat, src: &Mat, cx: i32, cy: i32) -> CvResult<()> {
    if src.empty() || dst.empty() || src.channels() != 3 || dst.channels() != 3 {
        return Ok(());
    }
    let sw = src.cols();
    let sh = src.rows();
    if sw <= 0 || sh <= 0 || cx < 0 || cy < 0 || cx >= sw || cy >= sh {
        return Ok(());
    }

    let half = PIXEL_LOUPE_SRC_HALF;
    let side = 2 * half + 1;
    let zoom = PIXEL_LOUPE_ZOOM;
    let out_side = side * zoom;
    let loupe_r = out_side / 2;

    let mut crop = Mat::zeros(side, side, src.typ())?.to_mat()?;
    for dy in -half..=half {
        for dx in -half..=half {
            let sx = (cx + dx).clamp(0, sw - 1);
            let sy = (cy + dy).clamp(0, sh - 1);
            let pix = *src.at_2d::<opencv::core::Vec3b>(sy, sx)?;
            *crop.at_2d_mut::<opencv::core::Vec3b>(dy + half, dx + half)? = pix;
        }
    }

    let mut zoomed = Mat::default();
    imgproc::resize(
        &crop,
        &mut zoomed,
        opencv::core::Size::new(out_side, out_side),
        0.0,
        0.0,
        imgproc::INTER_NEAREST,
    )?;

    let mut mask = Mat::zeros(out_side, out_side, opencv::core::CV_8UC1)?.to_mat()?;
    imgproc::circle(
        &mut mask,
        Point::new(loupe_r, loupe_r),
        loupe_r - 1,
        Scalar::all(255.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;

    let dw = dst.cols();
    let dh = dst.rows();
    let x0 = cx - loupe_r;
    let y0 = cy - loupe_r;
    for y in 0..out_side {
        let dy = y0 + y;
        if dy < 0 || dy >= dh {
            continue;
        }
        for x in 0..out_side {
            let dx = x0 + x;
            if dx < 0 || dx >= dw {
                continue;
            }
            if *mask.at_2d::<u8>(y, x)? == 0 {
                continue;
            }
            let pix = *zoomed.at_2d::<opencv::core::Vec3b>(y, x)?;
            *dst.at_2d_mut::<opencv::core::Vec3b>(dy, dx)? = pix;
        }
    }

    let center = Point::new(cx, cy);
    imgproc::circle(
        dst,
        center,
        loupe_r,
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        2,
        imgproc::LINE_AA,
        0,
    )?;
    // 중심 픽셀(확대 블록) 테두리
    let block = zoom / 2;
    imgproc::rectangle(
        dst,
        opencv::core::Rect::new(cx - block, cy - block, zoom, zoom),
        Scalar::new(0.0, 0.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;
    // 십자
    imgproc::line(
        dst,
        Point::new(cx - loupe_r + 4, cy),
        Point::new(cx - block - 2, cy),
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::line(
        dst,
        Point::new(cx + block + 2, cy),
        Point::new(cx + loupe_r - 4, cy),
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::line(
        dst,
        Point::new(cx, cy - loupe_r + 4),
        Point::new(cx, cy - block - 2),
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::line(
        dst,
        Point::new(cx, cy + block + 2),
        Point::new(cx, cy + loupe_r - 4),
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;

    let label = format!("{cx},{cy}");
    let tx = (cx - loupe_r).clamp(2, (dw - 80).max(2));
    let ty = (cy - loupe_r - 6).clamp(14, (dh - 2).max(14));
    imgproc::put_text(
        dst,
        &label,
        Point::new(tx, ty),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.45,
        Scalar::new(0.0, 0.0, 0.0, 0.0),
        2,
        imgproc::LINE_AA,
        false,
    )?;
    imgproc::put_text(
        dst,
        &label,
        Point::new(tx, ty),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.45,
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_AA,
        false,
    )?;
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pixel_pick_mouse_nudge_then_mouse_resync() {
        let mut m = PixelPickMouse::default();
        m.on_event(highgui::EVENT_MOUSEMOVE, 10, 20, 0);
        m.sync(1.0, 100, 100);
        m.nudge(1, -1, 100, 100);

        m.on_event(highgui::EVENT_LBUTTONDOWN, 10, 20, 0);
        m.sync(1.0, 100, 100);
        assert_eq!(m.drain_clicks(), vec![(11, 19)]);

        m.on_event(highgui::EVENT_MOUSEMOVE, 50, 60, 0);
        m.sync(1.0, 100, 100);
        m.confirm();
        assert_eq!(m.drain_clicks(), vec![(50, 60)]);
    }

    #[test]
    fn pixel_pick_mouse_shift_hover_is_aim_image_coords() {
        let mut m = PixelPickMouse::default();
        m.on_event(
            highgui::EVENT_MOUSEMOVE,
            5,
            10,
            highgui::EVENT_FLAG_SHIFTKEY,
        );
        m.sync(0.5, 200, 200);
        assert_eq!(m.hover, Some((10, 20)));
        m.nudge(1, 0, 200, 200);
        assert_eq!(m.hover, Some((11, 20)));
    }

    #[test]
    fn pixel_pick_mouse_nudge_keeps_loupe_without_shift() {
        let mut m = PixelPickMouse::default();
        m.on_event(highgui::EVENT_MOUSEMOVE, 10, 20, 0);
        m.sync(1.0, 100, 100);
        assert_eq!(m.hover, None);
        m.nudge(1, 0, 100, 100);
        assert_eq!(m.hover, Some((11, 20)));
    }

    #[test]
    fn arrow_delta_win32_mac_x11_and_hjkl() {
        // Win32 waitKeyEx: VK << 16 (Shift 눌러도 동일 — 수정자 OR 없음)
        assert_eq!(arrow_delta(0x25 << 16), Some((-1, 0))); // Left 2424832
        assert_eq!(arrow_delta(0x26 << 16), Some((0, -1))); // Up
        assert_eq!(arrow_delta(0x27 << 16), Some((1, 0))); // Right
        assert_eq!(arrow_delta(0x28 << 16), Some((0, 1))); // Down
        assert_eq!(arrow_delta(0xF702), Some((-1, 0)));
        assert_eq!(arrow_delta(0xFF53), Some((1, 0)));
        assert_eq!(arrow_delta(0xF702 | 0x10000), Some((-1, 0)));
        assert_eq!(arrow_delta(i32::from(b'h')), Some((-1, 0)));
        assert_eq!(arrow_delta(i32::from(b'J')), Some((0, 1)));
        assert_eq!(arrow_delta(i32::from(b'q')), None);
    }
}
