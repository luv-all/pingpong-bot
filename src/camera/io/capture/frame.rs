use std::time::Instant;

use opencv::core::Mat;

use crate::camera;

/// BGR 이미지 한 장 + 메타.
pub struct Frame {
    pub camera_id: camera::Id,
    pub image: Mat,
    pub timestamp: Instant,
}

impl Frame {
    pub fn new(camera_id: camera::Id, image: Mat, timestamp: Instant) -> Self {
        return Self {
            camera_id,
            image,
            timestamp,
        };
    }
}
