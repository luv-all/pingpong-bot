//! `data/clips/{scene}_{nn}/` 오프라인 클립 경로 해석.

mod resolved_stereo_offline;
mod stereo_clip;

pub use resolved_stereo_offline::ResolvedStereoOffline;
pub(crate) use resolved_stereo_offline::{resolve_mono_offline, resolve_stereo_offline};
pub use stereo_clip::StereoClip;
#[cfg(test)]
pub(crate) use stereo_clip::resolve_stereo_clip;

/// `record-stereo` 오프라인 클립 루트.
pub const DEFAULT_CLIPS_DIR: &str = "data/clips";

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::{Path, PathBuf};

    #[test]
    fn resolves_name_under_default_clips() {
        let root = tempfile_dir();
        let clips = root.join("data").join("clips").join("fly_01");
        fs::create_dir_all(&clips).unwrap();
        fs::write(clips.join("left.avi"), b"x").unwrap();
        fs::write(clips.join("right.avi"), b"y").unwrap();
        fs::write(clips.join("meta.json"), r#"{"meas_fps":40.5}"#).unwrap();

        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(&root).unwrap();
        let got = resolve_stereo_clip(Path::new("fly_01")).unwrap();
        let offline = resolve_stereo_offline(Some(Path::new("fly_01")))
            .unwrap()
            .expect("offline");
        std::env::set_current_dir(prev).unwrap();

        assert!(got.left.ends_with("left.avi"));
        assert!(got.right.ends_with("right.avi"));
        assert!((got.meas_fps.unwrap() - 40.5).abs() < 1e-9);
        assert!((offline.meas_fps.unwrap() - 40.5).abs() < 1e-9);
    }

    #[test]
    fn live_when_no_clip() {
        assert!(resolve_stereo_offline(None).unwrap().is_none());
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
