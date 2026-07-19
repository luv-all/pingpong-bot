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
            let epoch = self
                .timeline
                .map(|(e, _)| e)
                .unwrap_or_else(Instant::now);
            self.timeline = Some((epoch, fps));
        }
    }

    pub fn timeline_fps(&self) -> Option<f64> {
        return self.timeline.map(|(_, f)| f);
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
