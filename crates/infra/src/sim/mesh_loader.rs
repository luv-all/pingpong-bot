//! URDF mesh → kiss3d `SceneNode3d` (STL·OBJ).

use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use kiss3d::prelude::*;
use tracing::warn;

use crate::urdf::UrdfGeometry;

/// URDF visual geometry를 kiss3d 노드로 추가한다. 실패 시 작은 placeholder cube.
pub fn add_geometry(scene: &mut SceneNode3d, geometry: &UrdfGeometry, color: Color) -> SceneNode3d {
    return match geometry {
        UrdfGeometry::Box { size } => scene
            .add_cube(size[0], size[1], size[2])
            .set_color(color),
        UrdfGeometry::Cylinder { radius, length } => scene
            .add_cylinder(*radius, *length)
            .set_color(color),
        UrdfGeometry::Sphere { radius } => scene.add_sphere(*radius).set_color(color),
        UrdfGeometry::Mesh { path, scale } => load_mesh_or_placeholder(scene, path, *scale, color),
    };
}

fn load_mesh_or_placeholder(
    scene: &mut SceneNode3d,
    path: &Path,
    scale: [f32; 3],
    color: Color,
) -> SceneNode3d {
    if !path.exists() {
        warn!(path = %path.display(), "URDF mesh 파일 없음 — placeholder cube");
        return scene
            .add_cube(0.04, 0.04, 0.04)
            .set_color(Color::new(1.0, 0.2, 0.9, 1.0));
    }

    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();

    return match ext.as_str() {
        "stl" => match load_stl_trimesh(scene, path, scale, color) {
            Ok(node) => node,
            Err(error) => {
                warn!(path = %path.display(), %error, "STL 로드 실패");
                placeholder(scene)
            }
        },
        "obj" => load_obj(scene, path, scale, color),
        _ => {
            warn!(path = %path.display(), ext = %ext, "지원 mesh 형식: stl, obj");
            placeholder(scene)
        }
    };
}

fn load_stl_trimesh(
    scene: &mut SceneNode3d,
    path: &Path,
    scale: [f32; 3],
    color: Color,
) -> Result<SceneNode3d, String> {
    let file = File::open(path).map_err(|e| e.to_string())?;
    let mesh = stl_io::read_stl(&mut BufReader::new(file)).map_err(|e| e.to_string())?;
    let scale_vec = Vec3::new(scale[0], scale[1], scale[2]);

    let vertices: Vec<Vec3> = mesh
        .vertices
        .iter()
        .map(|v| Vec3::new(v[0] * scale_vec.x, v[1] * scale_vec.y, v[2] * scale_vec.z))
        .collect();

    let indices: Vec<[u32; 3]> = mesh
        .faces
        .iter()
        .map(|face| {
            [
                face.vertices[0] as u32,
                face.vertices[1] as u32,
                face.vertices[2] as u32,
            ]
        })
        .collect();

    if vertices.is_empty() || indices.is_empty() {
        return Err("STL이 비어 있습니다".to_string());
    }

    return Ok(scene
        .add_trimesh(vertices, indices, Vec3::ONE, true)
        .set_color(color));
}

fn load_obj(scene: &mut SceneNode3d, path: &Path, scale: [f32; 3], color: Color) -> SceneNode3d {
    let mtl_dir = path.parent().unwrap_or_else(|| Path::new("."));
    let scale_vec = Vec3::new(scale[0], scale[1], scale[2]);
    let mut node = scene.add_obj(path, mtl_dir, scale_vec);
    if color.a > 0.0 {
        node.set_color(color);
    }
    return node;
}

fn placeholder(scene: &mut SceneNode3d) -> SceneNode3d {
    return scene
        .add_cube(0.04, 0.04, 0.04)
        .set_color(Color::new(1.0, 0.2, 0.9, 1.0));
}
