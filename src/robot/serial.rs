//! 임의 revolute 직렬 체인의 기구학 표현.

use std::fmt;

use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

/// 한 revolute 관절의 URDF 기하.
///
/// 변환 순서는 URDF와 동일하게 `origin * rotation(axis, q)`다.
#[derive(Debug, Clone, PartialEq)]
pub struct SerialJoint {
    pub origin: Isometry3<f64>,
    pub axis: Unit<Vector3<f64>>,
}

impl SerialJoint {
    pub fn new(origin: Isometry3<f64>, axis: Vector3<f64>) -> Result<Self, SerialChainError> {
        if !axis.iter().all(|v| v.is_finite()) || axis.norm_squared() < 1e-12 {
            return Err(SerialChainError::InvalidAxis);
        }
        // -0.0 제거 — 다운스트림 f32 조인트 기저가 손갈라지지 않게.
        let axis = Vector3::new(axis.x + 0.0, axis.y + 0.0, axis.z + 0.0);
        return Ok(Self {
            origin,
            axis: Unit::new_normalize(axis),
        });
    }
}

/// 루트부터 EE 링크까지 고정 변환을 보존한 revolute 직렬 체인.
#[derive(Debug, Clone, PartialEq)]
pub struct SerialChain {
    /// 로봇 루트를 월드 마운트 축에 맞추는 회전.
    pub mount_rotation: UnitQuaternion<f64>,
    /// 루트 → EE 순서의 actuated 관절.
    pub joints: Vec<SerialJoint>,
    /// 마지막 actuated 관절 뒤 EE 링크까지의 고정 변환.
    pub ee_transform: Isometry3<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialChainError {
    Empty,
    InvalidAxis,
}

impl fmt::Display for SerialChainError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return match self {
            Self::Empty => write!(f, "직렬 체인에 revolute 관절이 없습니다"),
            Self::InvalidAxis => write!(f, "revolute 관절 축이 유효하지 않습니다"),
        };
    }
}

impl std::error::Error for SerialChainError {}

impl SerialChain {
    pub fn new(
        mount_rotation: UnitQuaternion<f64>,
        joints: Vec<SerialJoint>,
        ee_transform: Isometry3<f64>,
    ) -> Result<Self, SerialChainError> {
        if joints.is_empty() {
            return Err(SerialChainError::Empty);
        }
        return Ok(Self {
            mount_rotation,
            joints,
            ee_transform,
        });
    }

    pub(crate) fn mount_isometry(&self, mount: Vector3<f64>) -> Isometry3<f64> {
        return Isometry3::from_parts(Translation3::from(mount), self.mount_rotation);
    }

    /// EE 변환과 각 관절의 월드 원점/축을 함께 계산한다.
    pub(crate) fn forward_with_joint_frames(
        &self,
        mount: Vector3<f64>,
        values: &[f64],
    ) -> Option<(Isometry3<f64>, Vec<(Vector3<f64>, Vector3<f64>)>)> {
        if values.len() != self.joints.len() {
            return None;
        }

        let mut transform = self.mount_isometry(mount);
        let mut frames = Vec::with_capacity(self.joints.len());
        for (joint, &angle) in self.joints.iter().zip(values) {
            transform *= joint.origin;
            let position = transform.translation.vector;
            let axis = transform.rotation * joint.axis.into_inner();
            frames.push((position, axis));
            transform *= Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_axis_angle(&joint.axis, angle),
            );
        }
        return Some((transform * self.ee_transform, frames));
    }

    pub(crate) fn approximate_reach(&self) -> f64 {
        let origins = self
            .joints
            .iter()
            .map(|joint| joint.origin.translation.vector.norm())
            .sum::<f64>();
        return origins + self.ee_transform.translation.vector.norm();
    }
}
