//! 삼각측량된 한 샘플.

use crate::Point3;
use crate::camera;

#[derive(Debug, Clone)]
pub struct TrajPoint {
    pub t: f64,
    pub pos: Point3,
    pub pixels: Vec<(camera::Id, camera::Pixel)>,
}
