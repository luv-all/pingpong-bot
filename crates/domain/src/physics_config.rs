//! 런타임·측정 툴이 공유하는 물리 계수 (`config.toml` `[physics]`).

use std::fs;
use std::io;
use std::path::Path;

use serde::{Deserialize, Serialize};
use toml_edit::{value, DocumentMut, Item, Table};

use crate::constants::{
    ball::{RESTITUTION, TABLE_BOUNCE_FRICTION},
    physics::DEFAULT_DRAG,
};

/// 해석된 물리 계수 (항상 concrete 값).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PhysicsParams {
    /// 반발 \(e\)
    pub restitution: f64,
    /// 접선 마찰 \(\mu\)
    pub friction: f64,
    /// 이차 항력 \(k\)
    pub drag: f64,
}

impl Default for PhysicsParams {
    fn default() -> Self {
        return Self {
            restitution: RESTITUTION,
            friction: TABLE_BOUNCE_FRICTION,
            // sim Rapier에는 이차 항력이 없음 — EKF 기본도 0
            drag: 0.0,
        };
    }
}

impl PhysicsParams {
    /// 컴파일 타임 상수 (실측 참고용 `DEFAULT_DRAG` 포함).
    pub fn from_constants() -> Self {
        return Self {
            restitution: RESTITUTION,
            friction: TABLE_BOUNCE_FRICTION,
            drag: DEFAULT_DRAG,
        };
    }
}

/// TOML `[physics]` 섹션 — 필드별 optional (부분 갱신).
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

    /// `None` 필드는 [`PhysicsParams::default`]로 채운다.
    pub fn to_params(&self) -> PhysicsParams {
        let d = PhysicsParams::default();
        return PhysicsParams {
            restitution: self.restitution.unwrap_or(d.restitution),
            friction: self.friction.unwrap_or(d.friction),
            drag: self.drag.unwrap_or(d.drag),
        };
    }
}

/// `path`의 `[physics]`에 측정값을 merge한다. 파일이 없으면 최소 config를 만든다.
///
/// 주석·다른 키는 `toml_edit`으로 최대한 보존한다.
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
        doc["hit_plane_y"] = value(0.30);
        doc["camera_count"] = value(3);
        doc["robot"] = value("competition");
        doc
    };

    if !doc.as_table().contains_key("physics") {
        doc["physics"] = Item::Table(Table::new());
    }
    let physics = doc["physics"].as_table_mut().expect("physics table");

    if let Some(e) = patch.restitution {
        physics["restitution"] = value(e);
    }
    if let Some(mu) = patch.friction {
        physics["friction"] = value(mu);
    }
    if let Some(k) = patch.drag {
        physics["drag"] = value(k);
    }

    fs::write(path, doc.to_string())?;
    return Ok(load_physics_section(&doc));
}

/// config 텍스트/파일에서 `[physics]`를 읽는다.
pub fn load_physics_from_config(path: impl AsRef<Path>) -> io::Result<PhysicsConfig> {
    let text = fs::read_to_string(path)?;
    let doc: DocumentMut = text
        .parse()
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    return Ok(load_physics_section(&doc));
}

fn load_physics_section(doc: &DocumentMut) -> PhysicsConfig {
    let Some(table) = doc.get("physics").and_then(|item| item.as_table()) else {
        return PhysicsConfig::default();
    };
    return PhysicsConfig {
        restitution: table.get("restitution").and_then(toml_float),
        friction: table.get("friction").and_then(toml_float),
        drag: table.get("drag").and_then(toml_float),
    };
}

fn toml_float(item: &Item) -> Option<f64> {
    return item.as_float().or_else(|| item.as_integer().map(|i| i as f64));
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn merge_writes_and_preserves_other_keys() {
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!("pingpong_physics_{stamp}.toml"));
        fs::write(
            &path,
            "hit_plane_y = 0.30\ncamera_count = 3\nrobot = \"competition\"\n",
        )
        .unwrap();

        merge_physics_into_config(
            &path,
            &PhysicsConfig {
                restitution: Some(0.84),
                friction: None,
                drag: Some(0.012),
            },
        )
        .unwrap();

        let text = fs::read_to_string(&path).unwrap();
        assert!(text.contains("hit_plane_y = 0.30"));
        assert!(text.contains("restitution"));
        assert!(text.contains("0.84"));
        assert!(text.contains("drag"));

        let loaded = load_physics_from_config(&path).unwrap();
        assert!((loaded.restitution.unwrap() - 0.84).abs() < 1e-9);
        assert!((loaded.drag.unwrap() - 0.012).abs() < 1e-9);

        merge_physics_into_config(
            &path,
            &PhysicsConfig {
                restitution: None,
                friction: Some(0.2),
                drag: None,
            },
        )
        .unwrap();
        let loaded = load_physics_from_config(&path).unwrap();
        assert!((loaded.restitution.unwrap() - 0.84).abs() < 1e-9);
        assert!((loaded.friction.unwrap() - 0.2).abs() < 1e-9);

        let _ = fs::remove_file(&path);
    }
}
