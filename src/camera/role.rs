//! 로봇 기준 카메라 역할 (왼쪽 / 오른쪽).

use clap::ValueEnum;

/// 로봇을 바라볼 때 왼쪽 / 오른쪽 캠.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, ValueEnum)]
pub enum Role {
    /// 로봇 기준 왼쪽 → [`crate::camera::CamRigConfig::left_device`] / `Id(0)`.
    Left,
    /// 로봇 기준 오른쪽 → [`crate::camera::CamRigConfig::right_device`] / `Id(1)`.
    Right,
}

impl Role {
    pub fn as_str(self) -> &'static str {
        return match self {
            Self::Left => "left",
            Self::Right => "right",
        };
    }
}

impl std::fmt::Display for Role {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(self.as_str());
    }
}
