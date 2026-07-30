//! 역할 → device 인덱스 / 논리 `Id` 매핑.
//!
//! USB 장치 번호는 CLI에 노출하지 않는다.
//! [`Default`]·벤치 숫자는 [`crate::defaults::calib`].

use crate::camera;

/// 역할 → device 인덱스 / 논리 `Id`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CamRigConfig {
    pub left_device: i32,
    pub right_device: i32,
    pub left_id: camera::Id,
    pub right_id: camera::Id,
}

impl CamRigConfig {
    pub fn device(&self, role: camera::Role) -> i32 {
        return match role {
            camera::Role::Left => self.left_device,
            camera::Role::Right => self.right_device,
        };
    }

    pub fn camera_id(&self, role: camera::Role) -> camera::Id {
        return match role {
            camera::Role::Left => self.left_id,
            camera::Role::Right => self.right_id,
        };
    }

    pub fn resolve(&self, role: camera::Role) -> (i32, camera::Id) {
        return (self.device(role), self.camera_id(role));
    }
}
