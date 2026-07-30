//! 단일 캠 툴용 CLI.

use std::path::PathBuf;

use clap::Parser;

use super::cam_stream_args::CamStreamArgs;
use super::mono_offline_args::MonoOfflineArgs;
use super::resolved_cam::{ResolvedCam, resolve_cams};
use super::stereo_offline_args::StereoOfflineArgs;
use crate::camera;
use crate::camera::io::FrameSource;
use crate::camera::io::capture::OpenCvCapture;
use crate::camera::io::threaded::ThreadedCapture;

/// 단일 캠 툴용. `--cam left|right` **필수** (기본값 없음 — 어느 쪽인지 헷갈리지 않게).
/// device는 [`crate::camera::CamRigConfig`]가 부여.
#[derive(Parser, Debug, Clone)]
pub struct CamCliArgs {
    /// 로봇 기준 역할. 예: `--cam left` (생략 불가)
    #[arg(long = "cam", value_enum, value_delimiter = ',')]
    pub cam: Vec<camera::Role>,

    #[command(flatten)]
    pub stream: CamStreamArgs,
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
    pub fn camera_id(&self) -> Result<camera::Id, String> {
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

    /// 파일 경로들을 `--cam` 역할 순서의 `camera::Id`로 연다.
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
                .unwrap_or(camera::Id(i as u8));
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
