//! URDF visual geometry → kiss3d 프리미티브·mesh 경로.

use std::path::PathBuf;

use urdf_rs::{Geometry, Link, Robot};

/// link 1개의 visual 1개.
#[derive(Debug, Clone)]
pub struct UrdfLinkVisual {
    /// link 이름
    pub link_name: String,
    /// visual origin (link 로컬)
    pub origin_xyz: [f64; 3],
    pub origin_rpy: [f64; 3],
    /// geometry (mesh는 절대/해석된 파일 경로 포함)
    pub geometry: UrdfGeometry,
    /// RGBA 0..1
    pub color: [f32; 4],
}

/// kiss3d가 그릴 geometry.
#[derive(Debug, Clone)]
pub enum UrdfGeometry {
    Box {
        size: [f32; 3],
    },
    Cylinder {
        radius: f32,
        length: f32,
    },
    Sphere {
        radius: f32,
    },
    /// STL/OBJ mesh — `scale`은 URDF `<mesh scale="...">` (미터 단위 mesh 가정)
    Mesh {
        path: PathBuf,
        scale: [f32; 3],
    },
}

pub fn collect_link_visuals(
    robot: &Robot,
    resolve_path: impl Fn(&str) -> PathBuf,
) -> Vec<UrdfLinkVisual> {
    let mut out = Vec::new();
    for link in &robot.links {
        out.extend(link_visuals(link, &resolve_path));
    }
    return out;
}

fn link_visuals(link: &Link, resolve_path: &dyn Fn(&str) -> PathBuf) -> Vec<UrdfLinkVisual> {
    return link
        .visual
        .iter()
        .filter_map(|vis| {
            let geometry = parse_geometry(&vis.geometry, resolve_path)?;
            let color = vis
                .material
                .as_ref()
                .and_then(|m| m.color.as_ref())
                .map(|c| {
                    [
                        c.rgba[0] as f32,
                        c.rgba[1] as f32,
                        c.rgba[2] as f32,
                        c.rgba.get(3).copied().unwrap_or(1.0) as f32,
                    ]
                })
                .unwrap_or([0.72, 0.74, 0.78, 1.0]);
            return Some(UrdfLinkVisual {
                link_name: link.name.clone(),
                origin_xyz: *vis.origin.xyz,
                origin_rpy: *vis.origin.rpy,
                geometry,
                color,
            });
        })
        .collect();
}

fn parse_geometry(
    geometry: &Geometry,
    resolve_path: &dyn Fn(&str) -> PathBuf,
) -> Option<UrdfGeometry> {
    return match geometry {
        Geometry::Box { size } => Some(UrdfGeometry::Box {
            size: [size[0] as f32, size[1] as f32, size[2] as f32],
        }),
        Geometry::Cylinder { radius, length } => Some(UrdfGeometry::Cylinder {
            radius: *radius as f32,
            length: *length as f32,
        }),
        Geometry::Sphere { radius } => Some(UrdfGeometry::Sphere {
            radius: *radius as f32,
        }),
        Geometry::Mesh { filename, scale } => {
            let mesh_scale = scale
                .map(|s| [s[0] as f32, s[1] as f32, s[2] as f32])
                .unwrap_or([1.0, 1.0, 1.0]);
            return Some(UrdfGeometry::Mesh {
                path: resolve_path(filename),
                scale: mesh_scale,
            });
        }
        Geometry::Capsule { radius, length } => Some(UrdfGeometry::Cylinder {
            radius: *radius as f32,
            length: *length as f32,
        }),
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mesh_path_resolved_relative_to_base() {
        let base = PathBuf::from("/robot");
        let robot = urdf_rs::read_from_string(
            r#"<?xml version="1.0"?>
<robot name="r">
  <link name="a">
    <visual>
      <geometry><mesh filename="meshes/arm.stl" scale="1 1 1"/></geometry>
    </visual>
  </link>
</robot>"#,
        )
        .expect("urdf");
        let visuals = collect_link_visuals(&robot, |uri| base.join(uri));
        match &visuals[0].geometry {
            UrdfGeometry::Mesh { path, .. } => {
                assert_eq!(path, &PathBuf::from("/robot/meshes/arm.stl"));
            }
            _ => panic!("mesh expected"),
        }
    }
}
