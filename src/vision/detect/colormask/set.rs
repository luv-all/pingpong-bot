use anyhow::{Context, Result};

use crate::camera;

use super::{ColormaskBgr, ColormaskCam, ColormaskParams};

/// 멀티캠 colormask 번들 (`data/colormask.json`).
#[derive(Debug, Clone, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub struct ColormaskSet {
    pub cameras: Vec<ColormaskCam>,
}

impl ColormaskSet {
    pub fn params(&self, camera_id: camera::Id) -> Option<&ColormaskParams> {
        return self
            .cameras
            .iter()
            .find(|c| c.camera_id == camera_id)
            .map(|c| &c.params);
    }

    pub fn samples(&self, camera_id: camera::Id) -> Option<&[ColormaskBgr]> {
        return self
            .cameras
            .iter()
            .find(|c| c.camera_id == camera_id)
            .map(|c| c.samples.as_slice());
    }

    pub fn upsert(
        &mut self,
        camera_id: camera::Id,
        params: ColormaskParams,
        samples: Vec<ColormaskBgr>,
    ) {
        if let Some(slot) = self.cameras.iter_mut().find(|c| c.camera_id == camera_id) {
            slot.params = params;
            slot.samples = samples;
            return;
        }
        self.cameras.push(ColormaskCam {
            camera_id,
            params,
            samples,
        });
        self.cameras.sort_by_key(|c| c.camera_id);
    }
}

/// JSON에서 [`ColormaskSet`] 로드. 파일 없으면 에러.
pub fn load_colormask_set(path: &std::path::Path) -> Result<ColormaskSet> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("colormask 읽기: {}", path.display()))?;
    let set: ColormaskSet = serde_json::from_str(&text)
        .with_context(|| format!("colormask JSON: {}", path.display()))?;
    for cam in &set.cameras {
        cam.params.validate()?;
    }
    return Ok(set);
}

/// 있으면 로드, 없으면 빈 셋 (upsert 시작용).
pub fn load_colormask_set_or_empty(path: &std::path::Path) -> Result<ColormaskSet> {
    if !path.is_file() {
        return Ok(ColormaskSet::default());
    }
    return load_colormask_set(path);
}

/// [`ColormaskSet`]을 **compact** JSON으로 저장 (부모 dir 생성).
/// samples가 많아도 pretty로 수만 줄이 되지 않게 한 줄(+ trailing `\n`)로 쓴다.
pub fn save_colormask_set(path: &std::path::Path, set: &ColormaskSet) -> Result<()> {
    for cam in &set.cameras {
        cam.params.validate()?;
    }
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    let json = serde_json::to_string(set)?;
    std::fs::write(path, format!("{json}\n"))?;
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vision::detect::colormask::{ColorSpace, ColormaskParams};

    #[test]
    fn colormask_json_roundtrip_keeps_samples() {
        let mut set = ColormaskSet::default();
        set.upsert(
            camera::Id(0),
            ColormaskParams {
                space: ColorSpace::Hsv,
                c0_min: 10,
                c0_max: 20,
                c1_min: 30,
                c1_max: 40,
                c2_min: 50,
                c2_max: 60,
            },
            vec![[40u8, 120, 200]],
        );
        let json = serde_json::to_string(&set).unwrap();
        assert!(json.contains("\"samples\":[[40,120,200]]") || json.contains("[40, 120, 200]"));
        let loaded: ColormaskSet = serde_json::from_str(&json).unwrap();
        assert_eq!(loaded.samples(camera::Id(0)).unwrap(), &[[40u8, 120, 200]]);
        // 구포맷(samples 없음)도 로드
        let legacy = r#"{"cameras":[{"camera_id":1,"space":"ycrcb","c0_min":1,"c0_max":2,"c1_min":3,"c1_max":4,"c2_min":5,"c2_max":6}]}"#;
        let legacy_set: ColormaskSet = serde_json::from_str(legacy).unwrap();
        assert!(legacy_set.samples(camera::Id(1)).unwrap().is_empty());
    }

    #[test]
    fn save_colormask_writes_compact_single_line() {
        let mut set = ColormaskSet::default();
        set.upsert(
            camera::Id(0),
            ColormaskParams {
                space: ColorSpace::Hsv,
                c0_min: 1,
                c0_max: 2,
                c1_min: 3,
                c1_max: 4,
                c2_min: 5,
                c2_max: 6,
            },
            vec![[10, 20, 30], [40, 50, 60]],
        );
        let path =
            std::env::temp_dir().join(format!("pp_colormask_compact_{}.json", std::process::id()));
        save_colormask_set(&path, &set).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            text.ends_with('\n') && text.matches('\n').count() == 1,
            "expected one trailing newline only, got {} newlines",
            text.matches('\n').count()
        );
        assert!(text.contains("\"samples\":[[10,20,30],[40,50,60]]"));
    }
}
