//! 물리 계수. 앱 기본값은 [`crate::entry::competition_physics`].

use std::fs;
use std::io;
use std::path::Path;

use serde::{Deserialize, Serialize};
use toml_edit::{DocumentMut, Item, Table, value};

/// 해석된 물리 계수 (항상 concrete 값).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PhysicsParams {
    /// 반발 e
    pub restitution: f64,
    /// 접선 마찰 mu
    pub friction: f64,
    /// 이차 항력 k
    pub drag: f64,
}

/// TOML `[physics]` 섹션 - 필드별 optional (부분 갱신, measure 툴용).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PhysicsConfig {
    pub restitution: Option<f64>,
    pub friction: Option<f64>,
    pub drag: Option<f64>,
}

impl PhysicsConfig {
    pub fn is_empty(&self) -> bool {
        return self.restitution.is_none() && self.friction.is_none() && self.drag.is_none();
    }
}

impl PhysicsParams {
    pub fn with_overrides(self, c: &PhysicsConfig) -> Self {
        return Self {
            restitution: c.restitution.unwrap_or(self.restitution),
            friction: c.friction.unwrap_or(self.friction),
            drag: c.drag.unwrap_or(self.drag),
        };
    }
}

/// `path`의 `[physics]`에 측정값을 merge한다. 파일이 없으면 최소 config를 만든다.
pub fn merge_physics_into_config(
    path: impl AsRef<Path>,
    patch: &PhysicsConfig,
) -> io::Result<PhysicsConfig> {
    let path = path.as_ref();
    let mut doc = if path.exists() {
        let text = fs::read_to_string(path)?;
        text.parse::<DocumentMut>()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?
    } else {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let mut doc = DocumentMut::new();
        doc["mode"] = value("sim");
        doc["camera_count"] = value(3);
        doc["robot"] = value("competition");
        let mut intercept = Table::new();
        intercept["y_min"] = value(0.20);
        intercept["y_max"] = value(0.55);
        intercept["sample_step"] = value(0.05);
        doc["intercept"] = Item::Table(intercept);
        doc
    };

    let physics = doc["physics"].or_insert(Item::Table(Table::new()));
    let table = physics.as_table_mut().ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidData, "[physics] must be a table")
    })?;
    if let Some(v) = patch.restitution {
        table["restitution"] = value(v);
    }
    if let Some(v) = patch.friction {
        table["friction"] = value(v);
    }
    if let Some(v) = patch.drag {
        table["drag"] = value(v);
    }
    fs::write(path, doc.to_string())?;
    return Ok(load_physics_section(&fs::read_to_string(path)?).unwrap_or_default());
}

pub fn load_physics_from_config(path: impl AsRef<Path>) -> io::Result<PhysicsConfig> {
    let text = fs::read_to_string(path)?;
    return Ok(load_physics_section(&text).unwrap_or_default());
}

fn load_physics_section(text: &str) -> Option<PhysicsConfig> {
    #[derive(Deserialize)]
    struct File {
        physics: Option<PhysicsConfig>,
    }
    return toml::from_str::<File>(text).ok().and_then(|f| f.physics);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entry::competition_physics;

    #[test]
    fn competition_physics_values() {
        let p = competition_physics();
        assert!((p.restitution - 0.85).abs() < 1e-12);
    }

    #[test]
    fn with_overrides() {
        let p = competition_physics().with_overrides(&PhysicsConfig {
            restitution: Some(0.9),
            friction: None,
            drag: None,
        });
        assert!((p.restitution - 0.9).abs() < 1e-12);
        assert!((p.friction - 0.15).abs() < 1e-12);
    }
}
