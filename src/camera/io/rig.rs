//! 로봇 기준 카메라 역할 ↔ OS device / CameraId 매핑.
//!
//! USB 장치 번호는 CLI에 노출하지 않는다. 순서가 바뀌면 [`CamRigConfig`]만 고친다.

use clap::ValueEnum;

use crate::CameraId;

/// 로봇을 바라볼 때 왼쪽 / 오른쪽 캠.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, ValueEnum)]
pub enum CameraRole {
    /// 로봇 기준 왼쪽 → [`CamRigConfig::left_device`] / `CameraId(0)`.
    Left,
    /// 로봇 기준 오른쪽 → [`CamRigConfig::right_device`] / `CameraId(1)`.
    Right,
}

impl CameraRole {
    pub fn as_str(self) -> &'static str {
        return match self {
            Self::Left => "left",
            Self::Right => "right",
        };
    }
}

impl std::fmt::Display for CameraRole {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(self.as_str());
    }
}

/// 역할 → device 인덱스 / 논리 `CameraId`. USB 순서가 바뀌면 **여기만** 고친다.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CamRigConfig {
    pub left_device: i32,
    pub right_device: i32,
    pub left_id: CameraId,
    pub right_id: CameraId,
}

impl Default for CamRigConfig {
    fn default() -> Self {
        return Self {
            left_device: 0,
            right_device: 1,
            left_id: CameraId(0),
            right_id: CameraId(1),
        };
    }
}

impl CamRigConfig {
    pub fn device(&self, role: CameraRole) -> i32 {
        return match role {
            CameraRole::Left => self.left_device,
            CameraRole::Right => self.right_device,
        };
    }

    pub fn camera_id(&self, role: CameraRole) -> CameraId {
        return match role {
            CameraRole::Left => self.left_id,
            CameraRole::Right => self.right_id,
        };
    }

    pub fn resolve(&self, role: CameraRole) -> (i32, CameraId) {
        return (self.device(role), self.camera_id(role));
    }
}
