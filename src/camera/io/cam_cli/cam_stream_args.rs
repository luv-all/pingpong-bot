//! 공통 스트림 요청.

use clap::Parser;

use super::parse_fourcc;
use super::stream_preset::StreamPreset;
use crate::camera::io::capture::{CaptureBackend, OpenCvCapture};
use crate::defaults::calib::{
    DEFAULT_STREAM_BACKEND, DEFAULT_STREAM_FOURCC, DEFAULT_STREAM_FPS, DEFAULT_STREAM_HEIGHT,
    DEFAULT_STREAM_THREADED, DEFAULT_STREAM_WIDTH,
};

/// 공통 스트림 요청 (`--backend --width --height --fps --fourcc [--threaded] [--preset]`).
#[derive(Parser, Debug, Clone)]
pub struct CamStreamArgs {
    /// OpenCV 백엔드: any|dshow|msmf|v4l2|avfoundation|recommended
    #[arg(long, default_value = DEFAULT_STREAM_BACKEND)]
    pub backend: String,

    #[arg(long, default_value_t = DEFAULT_STREAM_WIDTH)]
    pub width: i32,

    #[arg(long, default_value_t = DEFAULT_STREAM_HEIGHT)]
    pub height: i32,

    #[arg(long, default_value_t = DEFAULT_STREAM_FPS)]
    pub fps: f64,

    #[arg(long, default_value = DEFAULT_STREAM_FOURCC)]
    pub fourcc: String,

    /// 백그라운드 grab 스레드. 끄려면 `--threaded=false`
    #[arg(long, default_value_t = DEFAULT_STREAM_THREADED, action = clap::ArgAction::Set)]
    pub threaded: bool,

    /// 해상도 프리셋 (`full`|`mid`|`low`). 주면 `--width/--height`보다 우선.
    #[arg(long, value_enum)]
    pub preset: Option<StreamPreset>,
}

impl CamStreamArgs {
    pub fn backend(&self) -> Result<CaptureBackend, String> {
        return CaptureBackend::parse(&self.backend);
    }

    pub fn fourcc_bytes(&self) -> Result<[u8; 4], String> {
        return parse_fourcc(&self.fourcc);
    }

    /// `--preset`이 있으면 그 크기, 없으면 `--width/--height`.
    pub fn resolved_size(&self) -> (i32, i32) {
        if let Some(p) = self.preset {
            return p.size();
        }
        return (self.width, self.height);
    }

    pub fn apply(&self, cap: &mut OpenCvCapture) -> Result<(), String> {
        let fourcc = self.fourcc_bytes()?;
        let (width, height) = self.resolved_size();
        cap.request_stream(width, height, self.fps, &fourcc)?;
        let preset_tag = self
            .preset
            .map(|p| format!(" preset={p:?}"))
            .unwrap_or_default();
        println!(
            "cam {}: requested={}x{}@{:.0} {}{preset_tag} | {}",
            cap.camera_id().0,
            width,
            height,
            self.fps,
            self.fourcc,
            cap.stream_summary()
        );
        if let Some(warn) = cap.warn_stream_mismatch(width, height, self.fps, &fourcc) {
            println!("cam {}: WARN {warn}", cap.camera_id().0);
        }
        return Ok(());
    }
}
