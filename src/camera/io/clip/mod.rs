//! `data/clips/{scene}_{nn}/` 오프라인 클립 경로 해석.

mod clip_meta_file;
mod resolved_stereo_offline;
mod stereo_clip;

pub use resolved_stereo_offline::ResolvedStereoOffline;
pub(crate) use resolved_stereo_offline::{resolve_mono_offline, resolve_stereo_offline};
pub use stereo_clip::StereoClip;
#[cfg(test)]
pub(crate) use stereo_clip::resolve_stereo_clip_under;

/// `record-stereo` 오프라인 클립 루트.
pub const DEFAULT_CLIPS_DIR: &str = "data/clips";

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::{Path, PathBuf};

    /// 이름(`fly_01`)이 클립 루트 아래로 해석되는지. 프로세스 CWD를 바꾸면
    /// 병렬 실행되는 다른 테스트의 상대경로 로드를 깨뜨리므로 루트를 주입한다.
    #[test]
    fn resolves_name_under_clips_root() {
        let clips_root = tempfile_dir().join("data").join("clips");
        let clip = clips_root.join("fly_01");
        write_clip(&clip);

        let got = resolve_stereo_clip_under(Path::new("fly_01"), &clips_root).unwrap();

        assert!(got.left.ends_with("left.avi"));
        assert!(got.right.ends_with("right.avi"));
        assert!((got.meas_fps.unwrap() - 40.5).abs() < 1e-9);
    }

    /// 이미 디렉터리인 경로는 루트와 무관하게 그대로 쓴다.
    #[test]
    fn resolves_explicit_dir_for_offline() {
        let clip = tempfile_dir().join("fly_01");
        write_clip(&clip);

        let offline = resolve_stereo_offline(Some(&clip))
            .unwrap()
            .expect("offline");

        assert!(offline.left.ends_with("left.avi"));
        assert!(offline.right.ends_with("right.avi"));
        assert!((offline.meas_fps.unwrap() - 40.5).abs() < 1e-9);
    }

    #[test]
    fn live_when_no_clip() {
        assert!(resolve_stereo_offline(None).unwrap().is_none());
    }

    fn write_clip(dir: &Path) {
        fs::create_dir_all(dir).unwrap();
        fs::write(dir.join("left.avi"), b"x").unwrap();
        fs::write(dir.join("right.avi"), b"y").unwrap();
        fs::write(dir.join("meta.json"), r#"{"meas_fps":40.5}"#).unwrap();
    }

    fn tempfile_dir() -> PathBuf {
        let mut d = std::env::temp_dir();
        d.push(format!(
            "pingpong-clip-test-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        fs::create_dir_all(&d).unwrap();
        return d;
    }
}
