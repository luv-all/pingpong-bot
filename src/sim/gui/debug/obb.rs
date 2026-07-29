//! 뷰어용 OBB.

use crate::planner::collision::OrientedBox;

/// 뷰어용 OBB (중심·half extents·축).
#[derive(Debug, Clone, Copy)]
pub struct DebugObb {
    pub center: [f64; 3],
    pub half_extents: [f64; 3],
    /// 열-우선 9개: axes.column(0), column(1), column(2)
    pub axes: [[f64; 3]; 3],
}

impl From<&OrientedBox> for DebugObb {
    fn from(obb: &OrientedBox) -> Self {
        return Self {
            center: [obb.center.x, obb.center.y, obb.center.z],
            half_extents: [obb.half_extents.x, obb.half_extents.y, obb.half_extents.z],
            axes: [
                [
                    obb.axes.column(0).x,
                    obb.axes.column(0).y,
                    obb.axes.column(0).z,
                ],
                [
                    obb.axes.column(1).x,
                    obb.axes.column(1).y,
                    obb.axes.column(1).z,
                ],
                [
                    obb.axes.column(2).x,
                    obb.axes.column(2).y,
                    obb.axes.column(2).z,
                ],
            ],
        };
    }
}
