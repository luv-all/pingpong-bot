use opencv::core::Mat;
use opencv::prelude::*;
use opencv::{Result as CvResult, highgui};

use super::{FittedBgr, PreviewAction, fit_bgr_downscale};

/// [`show_bgr`] 결과. `scale`은 디스플레이/원본 (항상 ≤ 1).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShowBgrResult {
    pub action: PreviewAction,
    pub scale: f64,
}

/// 타이틀바·독 여유 (px).
const DISPLAY_FIT_MARGIN_PX: i32 = 96;

/// 주 디스플레이 작업 영역(여유 마진 제외). 실패 시 None → fit 생략.
pub fn display_fit_bounds() -> Option<(i32, i32)> {
    let (w, h) = primary_display_px()?;
    let max_w = (w - DISPLAY_FIT_MARGIN_PX).max(320);
    let max_h = (h - DISPLAY_FIT_MARGIN_PX).max(240);
    return Some((max_w, max_h));
}

fn primary_display_px() -> Option<(i32, i32)> {
    #[cfg(target_os = "macos")]
    {
        #[link(name = "CoreGraphics", kind = "framework")]
        unsafe extern "C" {
            fn CGMainDisplayID() -> u32;
            fn CGDisplayPixelsWide(display: u32) -> usize;
            fn CGDisplayPixelsHigh(display: u32) -> usize;
        }
        // SAFETY: CoreGraphics display query; no owned resources.
        unsafe {
            let id = CGMainDisplayID();
            let w = CGDisplayPixelsWide(id) as i32;
            let h = CGDisplayPixelsHigh(id) as i32;
            if w > 0 && h > 0 {
                return Some((w, h));
            }
        }
        return None;
    }
    #[cfg(target_os = "windows")]
    {
        #[link(name = "user32")]
        unsafe extern "system" {
            fn GetSystemMetrics(index: i32) -> i32;
        }
        // SAFETY: Win32 metrics; no owned resources.
        unsafe {
            let w = GetSystemMetrics(0); // SM_CXSCREEN
            let h = GetSystemMetrics(1); // SM_CYSCREEN
            if w > 0 && h > 0 {
                return Some((w, h));
            }
        }
        return None;
    }
    #[cfg(not(any(target_os = "macos", target_os = "windows")))]
    {
        return None;
    }
}

/// BGR 이미지를 창에 띄운다. 모니터보다 크면 downscale만 한다. `q` / ESC → Quit.
pub fn show_bgr(window: &str, image: &Mat, wait_ms: i32) -> CvResult<ShowBgrResult> {
    let fitted = match display_fit_bounds() {
        Some((max_w, max_h)) => fit_bgr_downscale(image, max_w, max_h)?,
        None => FittedBgr {
            image: image.try_clone()?,
            scale: 1.0,
        },
    };
    highgui::imshow(window, &fitted.image)?;
    let action = poll_key(wait_ms)?;
    return Ok(ShowBgrResult {
        action,
        scale: fitted.scale,
    });
}

/// 새 프레임 없이 키 입력만 뽑는다 — 이미 떠 있는 창을 다시 그리지 않는다.
///
/// 화면에 변화가 없을 때(같은 프레임 반복 표시)도 highgui 이벤트 루프는 계속
/// 돌아야 창이 "응답 없음"으로 안 보인다. `imshow`(픽셀 재업로드)는 건너뛰고
/// 이 폴링만으로 그 역할을 한다.
pub fn poll_key(wait_ms: i32) -> CvResult<PreviewAction> {
    // waitKeyEx: 화살표가 macOS/X11에서 풀 키코드로 온다 (waitKey+&0xff는 Left≡'Q' 충돌).
    let key = highgui::wait_key_ex(wait_ms.max(1))?;
    return Ok(if key < 0 {
        PreviewAction::Continue
    } else if key == 27 || key == i32::from(b'q') || key == i32::from(b'Q') {
        PreviewAction::Quit
    } else {
        PreviewAction::Key(key)
    });
}

/// 창을 닫는다 (프로세스 종료 전 호출 권장).
pub fn destroy_window(window: &str) {
    let _ = highgui::destroy_window(window);
}
