//! 레일 x + 팔 관절각 스냅샷.

use super::Joints;

/// 레일 x + 팔 관절각 스냅샷 (`plan_swing` 입력).
#[derive(Debug, Clone, PartialEq)]
pub struct Pose {
    pub rail_x: f64,
    pub joints: Joints,
}

impl Pose {
    pub fn new(rail_x: f64, joints: Joints) -> Self {
        return Self { rail_x, joints };
    }
}
