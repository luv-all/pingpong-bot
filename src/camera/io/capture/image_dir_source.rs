use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use opencv::prelude::*;

use crate::camera;

use super::{Frame, FrameSource};

/// 디렉터리의 이미지를 정렬된 순서로 한 장씩 낸다 (`detect_*` 실험용).
pub struct ImageDirSource {
    camera_id: camera::Id,
    paths: Vec<PathBuf>,
    index: usize,
    epoch: Instant,
    /// 이미지 시퀀스용 가상 FPS
    fps: f64,
}

impl ImageDirSource {
    pub fn open(camera_id: camera::Id, dir: &Path) -> Result<Self, String> {
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

    fn camera_id(&self) -> camera::Id {
        return self.camera_id;
    }
}
