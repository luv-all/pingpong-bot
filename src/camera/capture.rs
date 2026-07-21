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
    pub fn from_device(camera_id: CameraId, device: i32) -> Result<Self, String> {
        let cap = VideoCapture::new(device, videoio::CAP_ANY)
            .map_err(|e| format!("VideoCapture open device {device}: {e}"))?;
        if !cap
            .is_opened()
            .map_err(|e| format!("VideoCapture is_opened: {e}"))?
        {
            return Err(format!("VideoCapture device {device} failed to open"));
        }
        return Ok(Self {
            camera_id,
            cap,
            frame_index: 0,
            timeline: None,
        });
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

    /// UVC 스트림 모드 요청 (Arducam B0332 등은 **MJPG**여야 고FPS).
    ///
    /// 예: `1280×800` + `120` + `MJPG`. 드라이버가 무시할 수 있으니
    /// [`reported_fourcc`] / [`reported_fps`]로 확인한다.
    pub fn request_stream(
        &mut self,
        width: i32,
        height: i32,
        fps: f64,
        fourcc: &[u8; 4],
    ) -> Result<(), String> {
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
        return Ok(());
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
