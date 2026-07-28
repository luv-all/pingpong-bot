//! 카메라 CLI SSOT — 모든 툴이 같은 `--cam` / 스트림 기본값을 쓴다.
//!
//! - 역할: [`CameraRole`] (`--cam left` / `--cam left,right`)
//! - device 번호: [`CamRigConfig`] 내부 (CLI 비노출)
//! - 스트림·렌즈: [`crate::camera::arducam_b0332`]

use clap::Parser;

use super::capture::{CaptureBackend, OpenCvCapture};
use super::rig::{CamRigConfig, CameraRole};
use super::threaded::ThreadedCapture;
use super::FrameSource;
use crate::camera::arducam_b0332;
use crate::CameraId;

/// [`arducam_b0332::WIDTH`]
pub const DEFAULT_STREAM_WIDTH: i32 = arducam_b0332::WIDTH;
/// [`arducam_b0332::HEIGHT`]
pub const DEFAULT_STREAM_HEIGHT: i32 = arducam_b0332::HEIGHT;
/// [`arducam_b0332::FPS_MJPG`]
pub const DEFAULT_STREAM_FPS: f64 = arducam_b0332::FPS_MJPG;
/// [`arducam_b0332::FOURCC_MJPG`]
pub const DEFAULT_STREAM_FOURCC: &str = arducam_b0332::FOURCC_MJPG;
/// table-PnP 등 K 근사 기본 — [`arducam_b0332::VFOV_DEG`]
pub const DEFAULT_FOV_Y_DEG: f64 = arducam_b0332::VFOV_DEG;

/// 공통 스트림 요청 (`--backend --width --height --fps --fourcc [--threaded]`).
#[derive(Parser, Debug, Clone)]
pub struct CamStreamArgs {
    /// OpenCV 백엔드: any|dshow|msmf|v4l2|avfoundation|recommended
    #[arg(long, default_value = "recommended")]
    pub backend: String,

    #[arg(long, default_value_t = DEFAULT_STREAM_WIDTH)]
    pub width: i32,

    #[arg(long, default_value_t = DEFAULT_STREAM_HEIGHT)]
    pub height: i32,

    #[arg(long, default_value_t = DEFAULT_STREAM_FPS)]
    pub fps: f64,

    #[arg(long, default_value = DEFAULT_STREAM_FOURCC)]
    pub fourcc: String,

    /// 백그라운드 grab 스레드 (UI와 캡처 분리)
    #[arg(long, default_value_t = false)]
    pub threaded: bool,
}

impl Default for CamStreamArgs {
    fn default() -> Self {
        return Self {
            backend: "recommended".into(),
            width: DEFAULT_STREAM_WIDTH,
            height: DEFAULT_STREAM_HEIGHT,
            fps: DEFAULT_STREAM_FPS,
            fourcc: DEFAULT_STREAM_FOURCC.into(),
            threaded: false,
        };
    }
}

impl CamStreamArgs {
    pub fn backend(&self) -> Result<CaptureBackend, String> {
        return CaptureBackend::parse(&self.backend);
    }

    pub fn fourcc_bytes(&self) -> Result<[u8; 4], String> {
        return parse_fourcc(&self.fourcc);
    }

    pub fn apply(&self, cap: &mut OpenCvCapture) -> Result<(), String> {
        let fourcc = self.fourcc_bytes()?;
        cap.request_stream(self.width, self.height, self.fps, &fourcc)?;
        println!(
            "cam {}: requested={}x{}@{:.0} {} | {}",
            cap.camera_id().0,
            self.width,
            self.height,
            self.fps,
            self.fourcc,
            cap.stream_summary()
        );
        if let Some(warn) = cap.warn_stream_mismatch(self.width, self.height, self.fps, &fourcc) {
            println!("cam {}: WARN {warn}", cap.camera_id().0);
        }
        return Ok(());
    }
}

/// 단일 캠 툴용 (`--cam left` 기본). device는 [`CamRigConfig`]가 부여.
#[derive(Parser, Debug, Clone)]
pub struct CamCliArgs {
    /// 로봇 기준 역할. 예: `--cam left`
    #[arg(
        long = "cam",
        value_enum,
        value_delimiter = ',',
        default_values_t = [CameraRole::Left]
    )]
    pub cam: Vec<CameraRole>,

    #[command(flatten)]
    pub stream: CamStreamArgs,
}

impl Default for CamCliArgs {
    fn default() -> Self {
        return Self {
            cam: vec![CameraRole::Left],
            stream: CamStreamArgs::default(),
        };
    }
}

/// 스테레오/멀티 기본 (`left,right`). `cam_preview` · `measure_*` 용.
#[derive(Parser, Debug, Clone)]
pub struct StereoCamCliArgs {
    /// 로봇 기준 역할. 예: `--cam left,right`
    #[arg(
        long = "cam",
        value_enum,
        value_delimiter = ',',
        default_values_t = [CameraRole::Left, CameraRole::Right]
    )]
    pub cam: Vec<CameraRole>,

    #[command(flatten)]
    pub stream: CamStreamArgs,
}

impl StereoCamCliArgs {
    pub fn as_cam_cli(&self) -> CamCliArgs {
        return CamCliArgs {
            cam: self.cam.clone(),
            stream: self.stream.clone(),
        };
    }
}

/// resolve된 한 대 (device는 rig에서만).
#[derive(Debug, Clone, Copy)]
pub struct ResolvedCam {
    pub role: CameraRole,
    pub device: i32,
    pub camera_id: CameraId,
}

impl CamCliArgs {
    pub fn resolve(&self) -> Result<Vec<ResolvedCam>, String> {
        return resolve_cams(&self.cam);
    }

    pub fn resolve_one(&self) -> Result<ResolvedCam, String> {
        let all = self.resolve()?;
        if all.len() != 1 {
            return Err(format!(
                "--cam 은 이 툴에서 정확히 1개여야 함 (got {})",
                all.len()
            ));
        }
        return Ok(all[0]);
    }

    /// 논리 id만 (파일 입력 등). 첫 `--cam` 역할 기준.
    pub fn camera_id(&self) -> Result<CameraId, String> {
        return Ok(self.resolve_one()?.camera_id);
    }

    /// 라이브 캡처 열고 스트림 요청. `threaded`면 [`ThreadedCapture`]로 감싼다.
    pub fn open_sources(&self) -> Result<Vec<(ResolvedCam, Box<dyn FrameSource>)>, String> {
        let backend = self.stream.backend()?;
        let resolved = self.resolve()?;
        let mut out = Vec::with_capacity(resolved.len());
        for r in resolved {
            let mut cap =
                OpenCvCapture::from_device_with_backend(r.camera_id, r.device, backend)?;
            self.stream.apply(&mut cap)?;
            let src: Box<dyn FrameSource> = if self.stream.threaded {
                Box::new(ThreadedCapture::spawn(cap))
            } else {
                Box::new(cap)
            };
            out.push((r, src));
        }
        return Ok(out);
    }

    pub fn open_one(&self) -> Result<(ResolvedCam, Box<dyn FrameSource>), String> {
        let mut all = self.open_sources()?;
        if all.len() != 1 {
            return Err(format!(
                "--cam 은 이 툴에서 정확히 1개여야 함 (got {})",
                all.len()
            ));
        }
        return Ok(all.remove(0));
    }
}

pub fn resolve_cams(roles: &[CameraRole]) -> Result<Vec<ResolvedCam>, String> {
    if roles.is_empty() {
        return Err("--cam 이 비어 있음 (left|right)".into());
    }
    let rig = CamRigConfig::default();
    let mut out = Vec::with_capacity(roles.len());
    for &role in roles {
        let (device, camera_id) = rig.resolve(role);
        out.push(ResolvedCam {
            role,
            device,
            camera_id,
        });
    }
    return Ok(out);
}

pub fn parse_fourcc(value: &str) -> Result<[u8; 4], String> {
    let bytes = value.as_bytes();
    if bytes.len() != 4 {
        return Err(format!("FOURCC는 정확히 4글자여야 함: {value}"));
    }
    return Ok([bytes[0], bytes[1], bytes[2], bytes[3]]);
}
