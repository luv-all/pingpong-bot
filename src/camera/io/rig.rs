//! 역할 → device 인덱스 / 논리 `Id` 매핑.
//!
//! USB 장치 번호는 CLI에 노출하지 않는다.
//! [`Default`]·벤치 숫자는 [`crate::defaults::calib`].

use crate::camera::{Id, Role};

/// 역할 → device 인덱스 / 논리 `Id`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CamRigConfig {
    pub left_device: i32,
    pub right_device: i32,
    pub left_id: Id,
    pub right_id: Id,
}

impl CamRigConfig {
    pub fn device(&self, role: Role) -> i32 {
        return match role {
            Role::Left => self.left_device,
            Role::Right => self.right_device,
        };
    }

    pub fn camera_id(&self, role: Role) -> Id {
        return match role {
            Role::Left => self.left_id,
            Role::Right => self.right_id,
        };
    }

    pub fn resolve(&self, role: Role) -> (i32, Id) {
        return (self.device(role), self.camera_id(role));
    }
}
