use std::time::Instant;

use opencv::core::Mat;
use opencv::{Result as CvResult, calib3d};

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

    /// 렌즈 왜곡을 편다. `dist`가 비어 있으면 그대로 돌려준다 — table-PnP 캘리브가
    /// 그렇고, 그 경우 프레임당 remap 을 통째로 건너뛴다.
    ///
    /// 소유권을 받는 이유는 왜곡이 없을 때 6 MB 를 복사하지 않기 위해서다.
    pub fn undistorted(self, params: &camera::Params) -> CvResult<Self> {
        if !params.has_distortion() {
            return Ok(self);
        }
        let k = Mat::from_slice_2d(&[
            &[params.fx, 0.0, params.cx],
            &[0.0, params.fy, params.cy],
            &[0.0, 0.0, 1.0],
        ])?;
        let dist = Mat::from_slice(&params.dist)?;
        let mut out = Mat::default();
        calib3d::undistort(&self.image, &mut out, &k, &dist, &opencv::core::no_array())?;
        return Ok(Self::new(self.camera_id, out, self.timestamp));
    }
}
