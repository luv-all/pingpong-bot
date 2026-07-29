//! 제어용 `Arm` + (선택) URDF 메시·마운트.

use std::sync::Arc;

use crate::Arm;
use crate::robot::Joints;
use crate::robot::urdf::UrdfModel;

/// 제어용 `Arm` + (선택) URDF 메시·마운트.
///
/// - 단순 빌더 → `arm`만 (`urdf = None`)
/// - URDF 빌더 → `to_arm()`으로 만든 `arm` + 원본 `UrdfModel`
#[derive(Debug, Clone)]
pub struct Robot {
    /// plan_swing·관절 추종용 FK
    pub arm: Arc<Arm>,
    /// mesh 뷰어·URDF FK (없으면 primitive 렌더)
    pub urdf: Option<Arc<UrdfModel>>,
}

impl Robot {
    /// URDF 없이 primitive `Arm`만 가진 로봇.
    pub fn from_arm(arm: Arm) -> Self {
        return Self {
            arm: Arc::new(arm),
            urdf: None,
        };
    }

    /// 이미 `Arc`인 `Arm`으로 조립 (URDF 없음).
    pub fn from_shared_arm(arm: Arc<Arm>) -> Self {
        return Self { arm, urdf: None };
    }
}

impl Robot {
    pub fn obbs(&self, rail_x: f64, joints: &Joints) -> Vec<crate::planner::OrientedBox> {
        return crate::planner::collision::robot_obbs(&self.arm, rail_x, joints);
    }

    pub fn table_penetration(&self, rail_x: f64, joints: &Joints) -> f64 {
        return crate::planner::collision::table_penetration(&self.arm, rail_x, joints);
    }

    pub fn clamp_above_table(&self, rail_x: f64, joints: &Joints) -> Joints {
        return crate::planner::collision::clamp_above_table(&self.arm, rail_x, joints);
    }
}
