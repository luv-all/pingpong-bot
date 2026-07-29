//! kiss3d가 그릴 geometry.

use std::path::PathBuf;

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
