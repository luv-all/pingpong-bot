use std::path::Path;

use anyhow::{Context, Result};

use crate::defaults::{STILL_HIT_RADIUS_PX, ensure_parent_dir};

use super::still_item::StillItem;

/// 스틸 GT 번들 — `label-stills`가 쓰고 `eval-colormask`가 읽는다.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StillsManifest {
    /// hit 판정 반경 [px].
    pub hit_radius_px: f64,
    pub items: Vec<StillItem>,
}

impl Default for StillsManifest {
    fn default() -> Self {
        return Self {
            hit_radius_px: STILL_HIT_RADIUS_PX,
            items: Vec::new(),
        };
    }
}

impl StillsManifest {
    /// 파일이 없으면 빈 manifest.
    pub fn load_or_default(path: &Path) -> Result<Self> {
        if !path.exists() {
            return Ok(Self::default());
        }
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("stills manifest 읽기: {}", path.display()))?;
        return serde_json::from_str(&text)
            .with_context(|| format!("stills manifest JSON: {}", path.display()));
    }

    /// 같은 `path`면 교체, 없으면 추가.
    pub fn upsert(&mut self, item: StillItem) {
        if let Some(slot) = self.items.iter_mut().find(|i| i.path == item.path) {
            *slot = item;
            return;
        }
        self.items.push(item);
    }

    /// 유공 장 수.
    pub fn ball_count(&self) -> usize {
        return self.items.iter().filter(|i| i.has_ball()).count();
    }

    /// 무공 장 수.
    pub fn empty_count(&self) -> usize {
        return self.items.len() - self.ball_count();
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        ensure_parent_dir(path)?;
        let text = serde_json::to_string_pretty(self)?;
        std::fs::write(path, text)
            .with_context(|| format!("stills manifest 쓰기: {}", path.display()))?;
        return Ok(());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera;

    fn item(path: &str, pixel: Option<[f64; 2]>) -> StillItem {
        return StillItem {
            path: path.to_string(),
            camera_id: camera::Id(0),
            clip: "fly_01".to_string(),
            frame: 48,
            pixel,
        };
    }

    #[test]
    fn upsert_replaces_same_path() {
        let mut manifest = StillsManifest::default();
        manifest.upsert(item("a.png", Some([1.0, 2.0])));
        manifest.upsert(item("a.png", None));
        assert_eq!(manifest.items.len(), 1);
        assert!(manifest.items[0].pixel.is_none());
    }

    #[test]
    fn roundtrip_keeps_null_pixel() {
        let mut manifest = StillsManifest::default();
        manifest.upsert(item("a.png", None));
        manifest.upsert(item("b.png", Some([3.5, 4.5])));
        let text = serde_json::to_string(&manifest).unwrap();
        assert!(text.contains("\"pixel\":null"), "{text}");
        let back: StillsManifest = serde_json::from_str(&text).unwrap();
        assert_eq!(back, manifest);
    }

    #[test]
    fn counts_split_ball_and_empty() {
        let mut manifest = StillsManifest::default();
        manifest.upsert(item("a.png", None));
        manifest.upsert(item("b.png", Some([3.5, 4.5])));
        manifest.upsert(item("c.png", Some([1.0, 1.0])));
        assert_eq!(manifest.ball_count(), 2);
        assert_eq!(manifest.empty_count(), 1);
    }

    #[test]
    fn load_or_default_on_missing_file() {
        let manifest = StillsManifest::load_or_default(Path::new("data/__no_such_stills.json"))
            .expect("missing file is not an error");
        assert!(manifest.items.is_empty());
        assert_eq!(manifest.hit_radius_px, STILL_HIT_RADIUS_PX);
    }
}
