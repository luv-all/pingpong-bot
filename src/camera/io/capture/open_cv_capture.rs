use std::path::Path;
use std::time::{Duration, Instant};

use opencv::core::Mat;
use opencv::prelude::*;
use opencv::videoio::{self, VideoCapture, VideoCaptureTrait, VideoCaptureTraitConst};

use crate::camera;

use super::{CaptureBackend, ExposureReadout, Frame, FrameSource};

/// MSMF 등이 돌려주는 `????` / 비그래픽 FOURCC는 비교 불가.
pub fn fourcc_report_readable(got: &str) -> bool {
    return !got.is_empty() && got.bytes().all(|b| b.is_ascii_alphanumeric());
}

/// [`OpenCvCapture::warn_stream_mismatch`] 본문 (테스트 가능).
pub fn format_stream_mismatch(
    want_w: i32,
    want_h: i32,
    want_fps: f64,
    want_fourcc: &str,
    got_fourcc: Option<&str>,
    got_size: Option<(i32, i32)>,
    got_fps: Option<f64>,
) -> Option<String> {
    let mut parts = Vec::new();
    if let Some(got) = got_fourcc {
        if fourcc_report_readable(got) && !got.eq_ignore_ascii_case(want_fourcc) {
            parts.push(format!("fourcc got={got} want={want_fourcc}"));
        }
    }
    if let Some((w, h)) = got_size {
        if w != want_w || h != want_h {
            parts.push(format!("size got={w}x{h} want={want_w}x{want_h}"));
        }
    }
    // CAP_PROP_FPS는 허위 보고가 많아 경고에 넣지 않는다 — meas FPS를 보라.
    let _ = got_fps;
    let _ = want_fps;
    if parts.is_empty() {
        return None;
    }
    return Some(format!(
        "stream mismatch ({}) — trust meas FPS; Win dual: --backend msmf; YUY2≈{}fps(B0332)",
        parts.join(", "),
        crate::camera::arducam_b0332::FPS_YUY2
    ));
}

/// OpenCV `VideoCapture` (장치 인덱스 또는 경로).
pub struct OpenCvCapture {
    camera_id: camera::Id,
    cap: VideoCapture,
    frame_index: u64,
    /// `Some((epoch, fps))` 이면 `epoch + n/fps` 타임스탬프 (파일 재생).
    /// `None` 이면 `Instant::now()` (라이브).
    timeline: Option<(Instant, f64)>,
}

impl OpenCvCapture {
    /// [`CaptureBackend::recommended`]로 연다 (Windows → MSMF).
    pub fn from_device(camera_id: camera::Id, device: i32) -> Result<Self, String> {
        return Self::from_device_with_backend(camera_id, device, CaptureBackend::recommended());
    }

    pub fn from_device_with_backend(
        camera_id: camera::Id,
        device: i32,
        backend: CaptureBackend,
    ) -> Result<Self, String> {
        let cap = VideoCapture::new(device, backend.api_pref()).map_err(|e| {
            format!(
                "VideoCapture open device {device} backend={}: {e}",
                backend.as_str()
            )
        })?;
        if !cap
            .is_opened()
            .map_err(|e| format!("VideoCapture is_opened: {e}"))?
        {
            return Err(format!(
                "VideoCapture device {device} backend={} failed to open",
                backend.as_str()
            ));
        }
        let mut out = Self {
            camera_id,
            cap,
            frame_index: 0,
            timeline: None,
        };
        out.apply_buffer_size_one();
        return Ok(out);
    }

    pub fn from_path(camera_id: camera::Id, path: &Path) -> Result<Self, String> {
        let path_str = path
            .to_str()
            .ok_or_else(|| format!("non-utf8 path: {}", path.display()))?;
        let cap = VideoCapture::from_file(path_str, videoio::CAP_ANY)
            .map_err(|e| format!("VideoCapture open {path_str}: {e}"))?;
        if !cap
            .is_opened()
            .map_err(|e| format!("VideoCapture is_opened: {e}"))?
        {
            return Err(format!("VideoCapture path {path_str} failed to open"));
        }
        let fps = cap
            .get(videoio::CAP_PROP_FPS)
            .ok()
            .filter(|f| f.is_finite() && *f > 1.0)
            .unwrap_or(30.0);
        return Ok(Self {
            camera_id,
            cap,
            frame_index: 0,
            timeline: Some((Instant::now(), fps)),
        });
    }

    pub fn camera_id(&self) -> camera::Id {
        return self.camera_id;
    }

    fn apply_buffer_size_one(&mut self) {
        let _ = self.cap.set(
            videoio::CAP_PROP_BUFFERSIZE,
            f64::from(crate::camera::arducam_b0332::BUFFER_SIZE),
        );
    }

    /// 파일 타임라인 FPS를 덮어쓴다 (속도 추정용).
    pub fn set_timeline_fps(&mut self, fps: f64) {
        if fps > 1e-3 {
            let epoch = self.timeline.map(|(e, _)| e).unwrap_or_else(Instant::now);
            self.timeline = Some((epoch, fps));
        }
    }

    pub fn timeline_fps(&self) -> Option<f64> {
        return self.timeline.map(|(_, f)| f);
    }

    /// 드라이버가 보고하는 프레임 크기. 미지원이면 `None`.
    pub fn reported_size(&self) -> Option<(i32, i32)> {
        let w = self.cap.get(videoio::CAP_PROP_FRAME_WIDTH).ok()?;
        let h = self.cap.get(videoio::CAP_PROP_FRAME_HEIGHT).ok()?;
        if w > 0.0 && h > 0.0 {
            return Some((w.round() as i32, h.round() as i32));
        }
        return None;
    }

    /// 드라이버 `CAP_PROP_FPS`. 라이브 웹캠은 0/엉터리인 경우가 많다.
    pub fn reported_fps(&self) -> Option<f64> {
        let fps = self.cap.get(videoio::CAP_PROP_FPS).ok()?;
        if fps.is_finite() && fps > 1.0 {
            return Some(fps);
        }
        return None;
    }

    /// 현재 FOURCC 네 글자 (`MJPG`, `YUY2` 등). 미지원이면 `None`.
    pub fn reported_fourcc(&self) -> Option<String> {
        let code = self.cap.get(videoio::CAP_PROP_FOURCC).ok()? as i32;
        if code == 0 {
            return None;
        }
        let bytes = [
            (code & 0xff) as u8,
            ((code >> 8) & 0xff) as u8,
            ((code >> 16) & 0xff) as u8,
            ((code >> 24) & 0xff) as u8,
        ];
        let s: String = bytes
            .iter()
            .map(|&b| if b.is_ascii_graphic() { b as char } else { '?' })
            .collect();
        return Some(s);
    }

    /// UVC 스트림 모드 요청. **Arducam B0332**는 MJPG@1280×800@120이 아니면
    /// YUY2≈10fps로 떨어진다 — [`crate::camera::arducam_b0332`].
    ///
    /// 순서: BUFFERSIZE → FOURCC → size → fps. 드라이버가 무시할 수 있으니
    /// [`stream_summary`] / [`warn_stream_mismatch`]로 확인한다.
    pub fn request_stream(
        &mut self,
        width: i32,
        height: i32,
        fps: f64,
        fourcc: &[u8; 4],
    ) -> Result<(), String> {
        self.apply_buffer_size_one();
        let code = videoio::VideoWriter::fourcc(
            fourcc[0] as char,
            fourcc[1] as char,
            fourcc[2] as char,
            fourcc[3] as char,
        )
        .map_err(|e| format!("FOURCC: {e}"))?;
        let _ = self.cap.set(videoio::CAP_PROP_FOURCC, f64::from(code));
        let _ = self
            .cap
            .set(videoio::CAP_PROP_FRAME_WIDTH, f64::from(width));
        let _ = self
            .cap
            .set(videoio::CAP_PROP_FRAME_HEIGHT, f64::from(height));
        let _ = self.cap.set(videoio::CAP_PROP_FPS, fps);
        // 일부 백엔드는 재설정 후 버퍼가 풀리므로 한 번 더.
        self.apply_buffer_size_one();
        return Ok(());
    }

    /// 현재 스트림 한 줄 요약 (`backend=… fourcc=… fps=… size=…`).
    pub fn stream_summary(&self) -> String {
        let backend = self
            .cap
            .get_backend_name()
            .ok()
            .unwrap_or_else(|| "?".into());
        let fourcc = self.reported_fourcc().unwrap_or_else(|| "?".into());
        let fps = self
            .reported_fps()
            .map(|f| format!("{f:.0}"))
            .unwrap_or_else(|| "?".into());
        let size = self
            .reported_size()
            .map(|(w, h)| format!("{w}x{h}"))
            .unwrap_or_else(|| "?".into());
        return format!("backend={backend} fourcc={fourcc} fps={fps} size={size}");
    }

    /// 요청과 보고값이 다르면 경고 문자열. 일치하면 `None`.
    ///
    /// MSMF 등은 FOURCC를 `????`로 돌려주는 경우가 많아, **읽을 수 없는 FOURCC는
    /// mismatch로 치지 않는다** (meas FPS를 믿을 것).
    pub fn warn_stream_mismatch(
        &self,
        width: i32,
        height: i32,
        fps: f64,
        fourcc: &[u8; 4],
    ) -> Option<String> {
        let want_fcc: String = fourcc.iter().map(|&b| b as char).collect();
        let got_fcc = self.reported_fourcc();
        let got_size = self.reported_size();
        let got_fps = self.reported_fps();
        return format_stream_mismatch(
            width,
            height,
            fps,
            &want_fcc,
            got_fcc.as_deref(),
            got_size,
            got_fps,
        );
    }

    /// 노출 관련 드라이버 값 스냅샷 (macOS AVFoundation이면 대개 0 / 무시).
    pub fn exposure_readout(&self) -> ExposureReadout {
        return ExposureReadout {
            auto: self.cap.get(videoio::CAP_PROP_AUTO_EXPOSURE).ok(),
            exposure: self.cap.get(videoio::CAP_PROP_EXPOSURE).ok(),
            gain: self.cap.get(videoio::CAP_PROP_GAIN).ok(),
            backend: self
                .cap
                .get_backend_name()
                .ok()
                .unwrap_or_else(|| "?".into()),
        };
    }

    /// 자동노출 off + 짧은 노출 시도. `set`이 하나라도 true면 `Ok(true)`.
    /// macOS AVFoundation은 보통 전부 실패한다.
    pub fn request_short_exposure(&mut self) -> bool {
        // V4L2: 0.25=manual, 1=manual(일부). DirectShow도 유사.
        let mut any = false;
        for auto in [0.25, 1.0, 0.75] {
            if self
                .cap
                .set(videoio::CAP_PROP_AUTO_EXPOSURE, auto)
                .unwrap_or(false)
            {
                any = true;
                break;
            }
        }
        // 드라이버마다 스케일이 다름 — 짧은 쪽 후보를 여러 개 시도.
        for exp in [-13.0, -11.0, -8.0, -6.0, 1.0, 5.0, 10.0] {
            if self
                .cap
                .set(videoio::CAP_PROP_EXPOSURE, exp)
                .unwrap_or(false)
            {
                any = true;
                break;
            }
        }
        return any;
    }

    /// 자동노출 복구 시도.
    pub fn request_auto_exposure(&mut self) -> bool {
        let mut any = false;
        for auto in [3.0, 0.75, 1.0] {
            if self
                .cap
                .set(videoio::CAP_PROP_AUTO_EXPOSURE, auto)
                .unwrap_or(false)
            {
                any = true;
                break;
            }
        }
        return any;
    }
}

impl FrameSource for OpenCvCapture {
    fn next_frame(&mut self) -> Option<Frame> {
        let mut image = Mat::default();
        let ok = self.cap.read(&mut image).ok()?;
        if !ok || image.empty() {
            return None;
        }
        let timestamp = if let Some((epoch, fps)) = self.timeline {
            epoch + Duration::from_secs_f64(self.frame_index as f64 / fps)
        } else {
            Instant::now()
        };
        self.frame_index += 1;
        return Some(Frame::new(self.camera_id, image, timestamp));
    }

    fn camera_id(&self) -> camera::Id {
        return self.camera_id;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fourcc_readable_rejects_msmf_junk() {
        assert!(fourcc_report_readable("MJPG"));
        assert!(fourcc_report_readable("YUY2"));
        assert!(!fourcc_report_readable("????"));
        assert!(!fourcc_report_readable("MJ?G"));
        assert!(!fourcc_report_readable(""));
    }

    #[test]
    fn mismatch_skips_unreadable_fourcc() {
        assert!(
            format_stream_mismatch(
                1280,
                800,
                120.0,
                "MJPG",
                Some("????"),
                Some((1280, 800)),
                Some(120.0)
            )
            .is_none()
        );
    }

    #[test]
    fn mismatch_reports_yuy2_vs_mjpg() {
        let msg = format_stream_mismatch(
            1280,
            800,
            120.0,
            "MJPG",
            Some("YUY2"),
            Some((1280, 800)),
            Some(120.0),
        )
        .expect("warn");
        assert!(msg.contains("fourcc got=YUY2"));
        assert!(msg.contains("msmf"));
        assert!(!msg.contains("dshow"));
    }

    #[test]
    fn mismatch_ignores_cap_fps_lie() {
        // size/fourcc OK, only fps prop wrong → no warn (meas is SSOT)
        assert!(
            format_stream_mismatch(
                1280,
                800,
                120.0,
                "MJPG",
                Some("MJPG"),
                Some((1280, 800)),
                Some(30.0)
            )
            .is_none()
        );
    }
}
