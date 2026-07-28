//! Arducam B0332 (OV9281 USB2.0 UVC) — datasheet 고정 스펙.
//!
//! Datasheet: [OV9281 USB2.0 Low Distortion Camera Module B0332](https://cdn.robotshop.com/media/A/Adu/RB-Adu-256/pdf/arducam-1mp-ov9281-usb-camera-120fps-global-shutter-uvc-low-distortion-m12-lens-datasheet.pdf)
//! Manual summary: [Core Electronics B0332 Manual](https://core-electronics.com.au/attachments/localcontent/B0332_Manual_50837c28540.pdf)
//!
//! | 항목 | 스펙 |
//! |------|------|
//! | Sensor | Monochrome global shutter OV9281, 1/4″ |
//! | Resolution | 1MP **1280×800** |
//! | Format | **MJPG** / YUY2 |
//! | Frame rate | MJPG **120fps**@1280×800; YUY2 **10fps**만 |
//! | Lens | M12, EFL 2.8mm, distortion &lt;1%, **HFOV 70°** |
//! | UVC name | `Arducam OV9281 USB Camera` |
//!
//! YUY2로 열리면 ~10fps가 정상이므로 고FPS는 반드시 MJPG를 요청한다.
//! CLI 스트림 기본은 [`crate::defaults::calib`]가 이 값을 참조한다.

/// 네이티브 가로 [px].
pub const WIDTH: i32 = 1280;
/// 네이티브 세로 [px].
pub const HEIGHT: i32 = 800;
/// 대역 중간 프리셋 (듀얼 meas 올리기용).
pub const WIDTH_MID: i32 = 960;
pub const HEIGHT_MID: i32 = 600;
/// 대역 낮음 프리셋 (hinguri 스테레오급).
pub const WIDTH_LOW: i32 = 640;
pub const HEIGHT_LOW: i32 = 400;
/// MJPG 최대 FPS (datasheet: 120fps@1280×800).
pub const FPS_MJPG: f64 = 120.0;
/// YUY2 최대 FPS (고FPS 불가 — 진단용).
pub const FPS_YUY2: f64 = 10.0;
/// 고FPS용 FOURCC.
pub const FOURCC_MJPG: &str = "MJPG";
/// 저FPS 비압축 FOURCC.
pub const FOURCC_YUY2: &str = "YUY2";

/// 렌즈 수평 FOV [deg] (datasheet: 70°(H)).
pub const HFOV_DEG: f64 = 70.0;
/// 렌즈 EFL [mm].
pub const EFL_MM: f64 = 2.8;

/// `HFOV` + 네이티브 종횡비로 환산한 수직 FOV [deg].
///
/// `V = 2 atan( tan(H/2) * height/width )` → ≈ 47.3°.
/// table-PnP `intrins_from_fov`는 **수직** FOV를 받으므로 이걸 쓴다.
pub const VFOV_DEG: f64 = 47.3;

/// OpenCV `CAP_PROP_BUFFERSIZE` — UVC 지연 최소화 (이 캠 프로필).
pub const BUFFER_SIZE: i32 = 1;
