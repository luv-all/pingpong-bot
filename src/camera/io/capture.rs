//! 프레임 소스 (sim 힌트 / OpenCV VideoCapture / 파일).

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use opencv::core::Mat;
use opencv::prelude::*;
use opencv::videoio::{self, VideoCapture, VideoCaptureTrait, VideoCaptureTraitConst};

use crate::{CameraId, PixelPoint};

/// BGR 이미지 한 장 + 메타.
pub struct Frame {
    pub camera_id: CameraId,
    pub image: Mat,
    pub timestamp: Instant,
}

impl Frame {
    pub fn new(camera_id: CameraId, image: Mat, timestamp: Instant) -> Self {
        return Self {
            camera_id,
            image,
            timestamp,
        };
    }
}

/// 카메라/파일에서 BGR 프레임을 낸다.
pub trait FrameSource: Send {
    fn next_frame(&mut self) -> Option<Frame>;
}

/// sim·구 경로: 이미 아는 픽셀 힌트 (검출기 우회).
pub trait HintSource: Send {
    fn next_hint(&mut self) -> Option<(CameraId, Option<PixelPoint>, Instant)>;
}

/// OpenCV `VideoCapture` API 백엔드.
///
/// Windows에서 `Any`→MSMF가 잡히면 MJPG/고FPS가 무시되는 경우가 많아
/// [`CaptureBackend::recommended`]는 Windows에서 DirectShow를 고른다.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CaptureBackend {
    /// `CAP_ANY` — OS 기본 선택.
    #[default]
    Any,
    /// DirectShow (Windows UVC 고FPS에 유리).
    DShow,
    /// Media Foundation (Windows).
    Msmf,
    /// V4L2 (Linux).
    V4l2,
    /// AVFoundation (macOS).
    AvFoundation,
}

impl CaptureBackend {
    /// 호스트에서 고FPS UVC에 안전한 기본값.
    pub fn recommended() -> Self {
        if cfg!(target_os = "windows") {
            return Self::DShow;
        }
        return Self::Any;
    }

    pub fn parse(s: &str) -> Result<Self, String> {
        return match s.trim().to_ascii_lowercase().as_str() {
            "any" | "default" | "" => Ok(Self::Any),
            "dshow" | "directshow" => Ok(Self::DShow),
            "msmf" | "mediafoundation" => Ok(Self::Msmf),
            "v4l" | "v4l2" => Ok(Self::V4l2),
            "avfoundation" | "avf" => Ok(Self::AvFoundation),
            "recommended" | "auto" => Ok(Self::recommended()),
            other => Err(format!(
                "unknown capture backend '{other}' (any|dshow|msmf|v4l2|avfoundation|recommended)"
            )),
        };
    }

    pub fn as_str(self) -> &'static str {
        return match self {
            Self::Any => "any",
            Self::DShow => "dshow",
            Self::Msmf => "msmf",
            Self::V4l2 => "v4l2",
            Self::AvFoundation => "avfoundation",
        };
    }

    pub fn api_pref(self) -> i32 {
        return match self {
            Self::Any => videoio::CAP_ANY,
            Self::DShow => videoio::CAP_DSHOW,
            Self::Msmf => videoio::CAP_MSMF,
            Self::V4l2 => videoio::CAP_V4L2,
            Self::AvFoundation => videoio::CAP_AVFOUNDATION,
        };
    }
}

/// OpenCV `VideoCapture` (장치 인덱스 또는 경로).
pub struct OpenCvCapture {
    camera_id: CameraId,
    cap: VideoCapture,
    frame_index: u64,
    /// `Some((epoch, fps))` 이면 `epoch + n/fps` 타임스탬프 (파일 재생).
    /// `None` 이면 `Instant::now()` (라이브).
    timeline: Option<(Instant, f64)>,
}

impl OpenCvCapture {
    /// [`CaptureBackend::recommended`]로 연다 (Windows → DirectShow).
    pub fn from_device(camera_id: CameraId, device: i32) -> Result<Self, String> {
        return Self::from_device_with_backend(camera_id, device, CaptureBackend::recommended());
    }

    pub fn from_device_with_backend(
        camera_id: CameraId,
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

    pub fn from_path(camera_id: CameraId, path: &Path) -> Result<Self, String> {
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

    pub fn camera_id(&self) -> CameraId {
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
    pub fn warn_stream_mismatch(
        &self,
        width: i32,
        height: i32,
        fps: f64,
        fourcc: &[u8; 4],
    ) -> Option<String> {
        let want_fcc: String = fourcc.iter().map(|&b| b as char).collect();
        let mut parts = Vec::new();
        if let Some(got) = self.reported_fourcc() {
            if !got.eq_ignore_ascii_case(&want_fcc) {
                parts.push(format!("fourcc got={got} want={want_fcc}"));
            }
        }
        if let Some((w, h)) = self.reported_size() {
            if w != width || h != height {
                parts.push(format!("size got={w}x{h} want={width}x{height}"));
            }
        }
        if let Some(got) = self.reported_fps() {
            if (got - fps).abs() > 5.0 {
                parts.push(format!("fps got={got:.0} want={fps:.0}"));
            }
        }
        if parts.is_empty() {
            return None;
        }
        return Some(format!(
            "stream mismatch ({}) — YUY2≈{}fps(B0332); Win은 --backend dshow, FOURCC=MJPG 확인",
            parts.join(", "),
            crate::camera::arducam_b0332::FPS_YUY2
        ));
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

/// [`OpenCvCapture::exposure_readout`] 결과.
#[derive(Debug, Clone)]
pub struct ExposureReadout {
    pub auto: Option<f64>,
    pub exposure: Option<f64>,
    pub gain: Option<f64>,
    pub backend: String,
}

impl ExposureReadout {
    pub fn summary_line(&self) -> String {
        let ae = self
            .auto
            .map(|v| format!("{v:.2}"))
            .unwrap_or_else(|| "-".into());
        let exp = self
            .exposure
            .map(|v| format!("{v:.2}"))
            .unwrap_or_else(|| "-".into());
        let gain = self
            .gain
            .map(|v| format!("{v:.1}"))
            .unwrap_or_else(|| "-".into());
        return format!("ae {ae} exp {exp} gain {gain}");
    }

    /// OpenCV macOS 백엔드는 width/height/fps 외 UVC 컨트롤을 거의 무시한다.
    pub fn likely_unsupported(&self) -> bool {
        let b = self.backend.to_ascii_lowercase();
        return b.contains("avfoundation") || b.contains("avf");
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
}

/// 디렉터리의 이미지를 정렬된 순서로 한 장씩 낸다 (`detect_*` 실험용).
pub struct ImageDirSource {
    camera_id: CameraId,
    paths: Vec<PathBuf>,
    index: usize,
    epoch: Instant,
    /// 이미지 시퀀스용 가상 FPS
    fps: f64,
}

impl ImageDirSource {
    pub fn open(camera_id: CameraId, dir: &Path) -> Result<Self, String> {
        let mut paths: Vec<_> = std::fs::read_dir(dir)
            .map_err(|e| format!("read_dir: {e}"))?
            .filter_map(|e| e.ok())
            .map(|e| e.path())
            .filter(|p| {
                matches!(
                    p.extension().and_then(|e| e.to_str()),
                    Some("png" | "jpg" | "jpeg" | "bmp")
                )
            })
            .collect();
        paths.sort();
        if paths.is_empty() {
            return Err(format!("이미지 없음: {}", dir.display()));
        }
        return Ok(Self {
            camera_id,
            paths,
            index: 0,
            epoch: Instant::now(),
            fps: 30.0,
        });
    }
}

impl FrameSource for ImageDirSource {
    fn next_frame(&mut self) -> Option<Frame> {
        let path = self.paths.get(self.index)?;
        let idx = self.index;
        self.index += 1;
        let path_str = path.to_str()?;
        let image = opencv::imgcodecs::imread(path_str, opencv::imgcodecs::IMREAD_COLOR).ok()?;
        if image.empty() {
            return self.next_frame();
        }
        let timestamp = self.epoch + Duration::from_secs_f64(idx as f64 / self.fps);
        return Some(Frame::new(self.camera_id, image, timestamp));
    }
}
