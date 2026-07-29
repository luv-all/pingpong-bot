//! 카메라 CLI — clap 스키마·캡처 오케스트레이션.
//!
//! 앱 프리셋 [`Default`]는 [`crate::defaults::calib`]에 있다.

use clap::{Parser, ValueEnum};

use super::FrameSource;
use super::capture::{CaptureBackend, OpenCvCapture};
use super::clip::{ResolvedStereoOffline, resolve_mono_offline, resolve_stereo_offline};
use super::rig::{CamRigConfig, CameraRole};
use super::threaded::ThreadedCapture;
use crate::CameraId;
use crate::constants::camera::arducam_b0332;
use crate::defaults::calib::DEFAULT_STEREO_CAM_ROLES;
use std::path::PathBuf;

pub use crate::defaults::calib::{
    DEFAULT_FOV_Y_DEG, DEFAULT_STREAM_BACKEND, DEFAULT_STREAM_FOURCC, DEFAULT_STREAM_FPS,
    DEFAULT_STREAM_HEIGHT, DEFAULT_STREAM_THREADED, DEFAULT_STREAM_WIDTH,
};

/// 해상도 프리셋 — `--width/--height` 대신 대역 실험용.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum StreamPreset {
    /// B0332 네이티브 1280×800
    Full,
    /// 960×600
    Mid,
    /// 640×400 (hinguri 스테레오급)
    Low,
}

impl StreamPreset {
    pub fn size(self) -> (i32, i32) {
        return match self {
            Self::Full => (arducam_b0332::WIDTH, arducam_b0332::HEIGHT),
            Self::Mid => (arducam_b0332::WIDTH_MID, arducam_b0332::HEIGHT_MID),
            Self::Low => (arducam_b0332::WIDTH_LOW, arducam_b0332::HEIGHT_LOW),
        };
    }
}

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

/// 단일 캠 툴용. `--cam left|right` **필수** (기본값 없음 — 어느 쪽인지 헷갈리지 않게).
/// device는 [`CamRigConfig`]가 부여.
#[derive(Parser, Debug, Clone)]
pub struct CamCliArgs {
    /// 로봇 기준 역할. 예: `--cam left` (생략 불가)
    #[arg(long = "cam", value_enum, value_delimiter = ',')]
    pub cam: Vec<CameraRole>,

    #[command(flatten)]
    pub stream: CamStreamArgs,
}

/// 스테레오/멀티 선택용 (`left,right` 기본). `cam-preview`처럼 한 대만 열 수도 있는 툴.
#[derive(Parser, Debug, Clone)]
pub struct StereoCamCliArgs {
    /// 로봇 기준 역할. 예: `--cam left,right`
    #[arg(
        long = "cam",
        value_enum,
        value_delimiter = ',',
        default_values_t = DEFAULT_STEREO_CAM_ROLES
    )]
    pub cam: Vec<CameraRole>,

    #[command(flatten)]
    pub stream: CamStreamArgs,
}

/// 양쪽 캠 **필수** 툴용 — `--cam` 없음. 항상 left+right.
#[derive(Parser, Debug, Clone)]
pub struct StereoPairCliArgs {
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

impl StereoPairCliArgs {
    pub fn as_cam_cli(&self) -> CamCliArgs {
        return CamCliArgs {
            cam: DEFAULT_STEREO_CAM_ROLES.to_vec(),
            stream: self.stream.clone(),
        };
    }
}

/// 스테레오 오프라인 입력 (`--clip`). 없으면 라이브.
#[derive(Parser, Debug, Clone, Default)]
pub struct StereoOfflineArgs {
    /// `data/clips` 클립 이름 또는 디렉터리 (`fly_01`)
    #[arg(long, value_name = "NAME|DIR")]
    pub clip: Option<PathBuf>,
}

impl StereoOfflineArgs {
    pub fn resolve(&self) -> Result<Option<ResolvedStereoOffline>, String> {
        return resolve_stereo_offline(self.clip.as_deref());
    }

    pub fn has_offline(&self) -> bool {
        return self.clip.is_some();
    }
}

/// 단안 오프라인 입력 (`--clip`). 없으면 라이브.
#[derive(Parser, Debug, Clone, Default)]
pub struct MonoOfflineArgs {
    /// `data/clips` 클립 (`fly_01`) — `--cam` 쪽 left/right 자동
    #[arg(long, value_name = "NAME|DIR")]
    pub clip: Option<PathBuf>,
}

impl MonoOfflineArgs {
    pub fn resolve(&self, role: CameraRole) -> Result<Option<PathBuf>, String> {
        return resolve_mono_offline(self.clip.as_deref(), role);
    }

    pub fn has_offline(&self) -> bool {
        return self.clip.is_some();
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
            let mut cap = OpenCvCapture::from_device_with_backend(r.camera_id, r.device, backend)?;
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

    /// 파일 경로들을 `--cam` 역할 순서의 `CameraId`로 연다.
    pub fn open_file_sources(
        &self,
        paths: &[PathBuf],
        timeline_fps: Option<f64>,
    ) -> Result<Vec<Box<dyn FrameSource>>, String> {
        let roles = self.resolve()?;
        let mut out = Vec::with_capacity(paths.len());
        for (i, path) in paths.iter().enumerate() {
            let id = roles
                .get(i)
                .map(|r| r.camera_id)
                .unwrap_or(CameraId(i as u8));
            let mut cap = OpenCvCapture::from_path(id, path)?;
            if let Some(fps) = timeline_fps {
                cap.set_timeline_fps(fps);
            }
            out.push(Box::new(cap) as Box<dyn FrameSource>);
        }
        return Ok(out);
    }

    /// 스테레오: `--clip`이면 파일, 없으면 라이브.
    /// 반환 timeline_fps = CLI 덮어쓰기 또는 clip `meas_fps`.
    pub fn open_stereo_input(
        &self,
        offline: &StereoOfflineArgs,
        timeline_fps: Option<f64>,
    ) -> Result<(Vec<Box<dyn FrameSource>>, Option<f64>), String> {
        if let Some(resolved) = offline.resolve()? {
            resolved.log();
            let fps = timeline_fps.or(resolved.meas_fps);
            if let Some(f) = fps {
                if timeline_fps.is_some() {
                    println!("timeline_fps={f:.2} (cli)");
                } else {
                    println!("timeline_fps={f:.2}");
                }
            }
            return Ok((self.open_file_sources(&resolved.paths(), fps)?, fps));
        }
        let sources = self.open_sources()?.into_iter().map(|(_, s)| s).collect();
        return Ok((sources, None));
    }

    /// 단안: `--clip`이면 파일, 없으면 라이브.
    pub fn open_mono_input(
        &self,
        offline: &MonoOfflineArgs,
    ) -> Result<Box<dyn FrameSource>, String> {
        let resolved = self.resolve_one()?;
        if let Some(path) = offline.resolve(resolved.role)? {
            println!(
                "clip {} → {}",
                offline
                    .clip
                    .as_ref()
                    .map(|c| c.display().to_string())
                    .unwrap_or_default(),
                path.display()
            );
            return Ok(Box::new(OpenCvCapture::from_path(
                resolved.camera_id,
                &path,
            )?));
        }
        return Ok(self.open_one()?.1);
    }
}

pub(crate) fn resolve_cams(roles: &[CameraRole]) -> Result<Vec<ResolvedCam>, String> {
    if roles.is_empty() {
        return Err("--cam 필수 (left|right) — 단일 캠 툴은 생략 불가".into());
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

pub(crate) fn parse_fourcc(value: &str) -> Result<[u8; 4], String> {
    let bytes = value.as_bytes();
    if bytes.len() != 4 {
        return Err(format!("FOURCC는 정확히 4글자여야 함: {value}"));
    }
    return Ok([bytes[0], bytes[1], bytes[2], bytes[3]]);
}
