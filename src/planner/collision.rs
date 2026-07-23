//! 로봇/라켓 OBB <-> 탁구대 면 충돌 근사.
//!
//! 키네마틱 팔은 Rapier가 밀어내지 않으므로, 관절 명령 전에
//! 테이블 footprint 위 최저점 관통량을 재고 위로 올린다 (sim/real 공통).

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use crate::constants::{
    geometry::{
        LINK_FOREARM_RADIUS, RACKET_HALF_X, RACKET_HALF_Y, RACKET_HALF_Z, TABLE_CLAMP_ITERS,
        TABLE_CLEARANCE,
    },
    table,
};
use crate::robot::{Arm, RacketPose};
use crate::{Joints, Point3};

/// 월드 좌표계 oriented bounding box (중심 + 로컬 축 half-extents).
#[derive(Debug, Clone, Copy)]
pub struct OrientedBox {
    /// 박스 중심 (월드)
    pub center: Vector3<f64>,
    /// 열 = 로컬 X,Y,Z 축 (월드), 정규직교
    pub axes: Matrix3<f64>,
    /// 로컬 축별 half-extent [m]
    pub half_extents: Vector3<f64>,
}

impl OrientedBox {
    /// 링크 양 끝점으로 원통 근사 OBB (축 = from->to, 단면 = radius).
    pub fn from_segment(from: Vector3<f64>, to: Vector3<f64>, radius: f64) -> Option<Self> {
        let delta = to - from;
        let length = delta.norm();
        if length < 1e-9 {
            return None;
        }
        let axis = delta / length;
        let (x, z) = orthonormal_plane(axis);
        // 열: 단면X, 링크축Y, 단면Z - 뷰어 실린더(local Y)와 동일
        let axes = Matrix3::from_columns(&[x, axis, z]);
        return Some(Self {
            center: (from + to) * 0.5,
            axes,
            half_extents: Vector3::new(radius, length * 0.5, radius),
        });
    }

    /// 라켓 자세 -> OBB (local +Z = 면 법선).
    pub fn from_racket(pose: &RacketPose) -> Self {
        let axes = quat_to_axes(pose.orientation);
        return Self {
            center: pose.position.coords,
            axes,
            half_extents: Vector3::new(RACKET_HALF_X, RACKET_HALF_Y, RACKET_HALF_Z),
        };
    }

    /// 테이블 면 위에서 관통하는 최저점의 깊이 [m] (>=0이면 뚫음).
    pub fn table_penetration(&self) -> f64 {
        let floor = table::SURFACE_Z + TABLE_CLEARANCE;
        let mut worst = 0.0;
        for corner in self.corners() {
            if !over_table_xy(corner) {
                continue;
            }
            let depth = floor - corner.z;
            if depth > worst {
                worst = depth;
            }
        }
        return worst;
    }

    fn corners(&self) -> [Vector3<f64>; 8] {
        let hx = self.half_extents.x;
        let hy = self.half_extents.y;
        let hz = self.half_extents.z;
        let ax = self.axes.column(0);
        let ay = self.axes.column(1);
        let az = self.axes.column(2);
        let mut out = [Vector3::zeros(); 8];
        let mut i = 0;
        for sx in [-1.0, 1.0] {
            for sy in [-1.0, 1.0] {
                for sz in [-1.0, 1.0] {
                    out[i] = self.center + ax * (sx * hx) + ay * (sy * hy) + az * (sz * hz);
                    i += 1;
                }
            }
        }
        return out;
    }
}

/// 관절 자세의 테이블 충돌 OBB (전완/라켓).
///
/// 상완은 테이블 끝 마운트에 붙어 면과 겹칠 수 있어 제외한다.
pub fn robot_obbs(arm: &Arm, rail_x: f64, joints: &Joints) -> Vec<OrientedBox> {
    let Some(points) = arm.chain_points(rail_x, joints) else {
        return Vec::new();
    };
    let mut boxes = Vec::with_capacity(points.len());
    for segment in points.windows(2).skip(1) {
        if let Some(link) = OrientedBox::from_segment(segment[0], segment[1], LINK_FOREARM_RADIUS) {
            boxes.push(link);
        }
    }
    if let Some(pose) = arm.forward_kinematics_with_rail(rail_x, joints) {
        boxes.push(OrientedBox::from_racket(&pose));
    }
    return boxes;
}

/// 테이블 footprint 위 최대 관통 깊이 [m]. 0이면 여유 있음.
pub fn table_penetration(arm: &Arm, rail_x: f64, joints: &Joints) -> f64 {
    return robot_obbs(arm, rail_x, joints)
        .iter()
        .map(OrientedBox::table_penetration)
        .fold(0.0, f64::max);
}

/// 관통 시 EE를 들어 올려 재IK - 손목 open/yaw 힌트 유지.
pub fn clamp_above_table(arm: &Arm, rail_x: f64, joints: &Joints) -> Joints {
    let mut current = joints.clone();
    let wrist_index = arm.wrist_joint_index();
    let wrist = wrist_index
        .and_then(|index| joints.values.get(index))
        .copied()
        .unwrap_or(crate::defaults::control().racket_open_pitch);

    for _ in 0..TABLE_CLAMP_ITERS {
        let depth = table_penetration(arm, rail_x, &current);
        if depth <= 1e-4 {
            break;
        }
        let Some(pose) = arm.forward_kinematics_with_rail(rail_x, &current) else {
            break;
        };
        let lifted = Point3::new(
            pose.position.coords.x,
            pose.position.coords.y,
            pose.position.coords.z + depth + TABLE_CLEARANCE,
        );
        let Ok(mut solved) = (if let Some(rail) = &arm.rail {
            arm.inverse_kinematics_with_rail(rail, rail_x, lifted, Some(&current))
        } else {
            arm.inverse_kinematics_near(lifted, Some(&current))
        }) else {
            // IK 실패 시 어깨를 살짝 올려 폴백
            if current.values.len() > 1 {
                let raised = current.values[1] + 0.08;
                current.values[1] = arm
                    .joint_limit(1)
                    .map_or(raised, |limit| raised.clamp(limit.min, limit.max));
            }
            continue;
        };
        // 면이 테이블을 찌르면 open을 조금 줄여 모서리를 듦
        let mut open = wrist;
        if table_penetration(arm, rail_x, &solved) > 1e-4 {
            if let Some(index) = wrist_index {
                open = arm
                    .joint_limit(index)
                    .map_or(solved.values[index] - 0.1, |limit| {
                        (solved.values[index] - 0.1).max(limit.min)
                    });
            }
        }
        if let Ok(with_wrist) = arm.with_wrist_open(&solved, open) {
            solved = with_wrist;
        }
        current = solved;
    }
    return current;
}

fn over_table_xy(p: Vector3<f64>) -> bool {
    const MARGIN: f64 = 0.01;
    // 로봇 쪽 끝(y~=0)은 마운트/상완이 테이블과 맞닿음 - 플레이 영역만 검사
    const PLAY_Y_MIN: f64 = 0.08;
    return p.x >= -MARGIN
        && p.x <= table::WIDTH_X + MARGIN
        && p.y >= PLAY_Y_MIN
        && p.y <= table::LENGTH_Y + MARGIN;
}

fn quat_to_axes(q: [f64; 4]) -> Matrix3<f64> {
    let uq = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]));
    return *uq.to_rotation_matrix().matrix();
}

fn orthonormal_plane(axis: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let helper = if axis.z.abs() < 0.9 {
        Vector3::new(0.0, 0.0, 1.0)
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };
    let x = axis.cross(&helper).normalize();
    let z = axis.cross(&x).normalize();
    return (x, z);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deep_racket_penetrates_table() {
        let arm = crate::defaults::primitive_4dof().expect("arm").arm;
        let rail = arm.rail.expect("rail");
        let rail_x = rail.home_x();
        // 프로파일 마운트가 테이블 위 20cm라서 임의 관절 스윕이 관통을 못 찾을 수 있다.
        // 테이블 면 아래로 접은 자세로 클램프 경로를 검증한다.
        let joints = Joints::from_slice(&[0.0, 0.0, 1.4, 1.5]);
        let before = table_penetration(&arm, rail_x, &joints);
        let joints = if before > 0.0 {
            joints
        } else {
            let below =
                crate::Point3::new(rail_x, table::DEFAULT_HIT_PLANE_Y, table::SURFACE_Z - 0.05);
            arm.inverse_kinematics_with_rail(&rail, rail_x, below, Some(&arm.default_joints))
                .unwrap_or(joints)
        };
        let before = table_penetration(&arm, rail_x, &joints);
        assert!(before > 0.0, "의도적 저자세는 관통해야 함: {before}");
        let clamped = clamp_above_table(&arm, rail_x, &joints);
        let after = table_penetration(&arm, rail_x, &clamped);
        assert!(
            after < 1e-3,
            "클램프 후 관통 ~=0 이어야 함: before={before} after={after}"
        );
    }

    #[test]
    fn default_pose_clears_table() {
        let arm = crate::defaults::primitive_4dof().expect("arm").arm;
        let rail_x = arm.rail.as_ref().map(|r| r.home_x()).unwrap_or(0.0);
        let depth = table_penetration(&arm, rail_x, &arm.default_joints);
        assert!(depth <= 1e-4, "기본 자세는 테이블 위: {depth}");
    }
}
