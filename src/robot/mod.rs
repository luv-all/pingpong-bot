//! 로봇 팔 기구학.
//!
//! `Arm`은 sim/real이 같이 쓰는 불변 기하 모델이다. 부팅 때 한 번 만들고
//! `Arc<Arm>`으로 넘긴다. FK/IK랑 스윙 계획은 이 타입만 본다.
//!
//! 조립은 `ArmBuilder`, 런타임 추종은 `RobotState`.

pub mod builder;
mod loader;
pub mod rail;
pub mod serial;
pub mod state;
pub mod urdf;

#[cfg(test)]
mod tests;

use nalgebra::{DMatrix, DVector, Isometry3, Matrix3, UnitQuaternion, Vector3};

pub use builder::{ArmBuildError, ArmBuilder};
pub use loader::{MountPreset, Robot, RobotBuildError, RobotBuilder};
pub use rail::{LinearRail, RailFrame};
pub use serial::{SerialChain, SerialChainError, SerialJoint};
pub use state::RobotState;
pub use urdf::{UrdfGeometry, UrdfLinkVisual, UrdfLoadError, UrdfModel};

use crate::error::SwingPlanError;
use crate::Point3;

/// revolute 관절각 [rad].
#[derive(Debug, Clone, PartialEq)]
pub struct Joints {
    pub values: Vec<f64>,
}

impl Joints {
    pub fn from_slice(values: &[f64]) -> Self {
        return Self {
            values: values.to_vec(),
        };
    }
}

/// 레일 x + 팔 관절각 스냅샷 (`plan_swing` 입력).
#[derive(Debug, Clone, PartialEq)]
pub struct RobotPose {
    pub rail_x: f64,
    pub joints: Joints,
}

impl RobotPose {
    pub fn new(rail_x: f64, joints: Joints) -> Self {
        return Self { rail_x, joints };
    }
}

/// revolute 관절 1축 허용 범위 [rad].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLimit {
    /// 최소 각도 [rad]
    pub min: f64,
    /// 최대 각도 [rad]
    pub max: f64,
}

impl JointLimit {
    /// [min, max] 범위를 만든다.
    pub const fn new(min: f64, max: f64) -> Self {
        return Self { min, max };
    }

    /// 각도가 허용 범위 안인지 확인한다.
    pub fn contains(self, angle: f64) -> bool {
        return angle >= self.min && angle <= self.max;
    }
}

/// 로봇 팔 불변 모델. sim/real/plan_swing이 같은 `Arm`을 참조한다.
#[derive(Debug, Clone, PartialEq)]
pub struct Arm {
    /// 베이스 원점 (월드 좌표) [m] - 리니어 레일이 있으면 y/z 마운트 기준, x는 무시
    pub base: Point3,
    /// X축 리니어 레일 (있으면 베이스 x는 `rail_x`로 이동)
    pub rail: Option<LinearRail>,
    /// revolute 축 순서대로의 링크 길이 [m] - `limits`/`default_joints`와 같은 길이
    pub link_lengths: Vec<f64>,
    /// 축별 관절 한계. `None`은 URDF continuous 관절.
    pub limits: Vec<Option<JointLimit>>,
    /// 부팅 시 초기 관절각
    pub default_joints: Joints,
    /// 관절 추종 최대 각속도 [rad/s]
    pub max_joint_speed: f64,
    /// FK/IK 구현. URDF 로봇은 원본 고정 변환과 축을 보존한 직렬 체인을 쓴다.
    pub(crate) chain: SerialChain,
}

/// 월드 좌표계 라켓 자세 - sim/real 동일 표현.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RacketPose {
    /// 라켓 중심 위치 (월드)
    pub position: Point3,
    /// 라켓 면 법선 (단위 벡터)
    pub normal: Vector3<f64>,
    /// Hamilton 단위 쿼터니언 (w, x, y, z) - 어댑터가 SDK 회전으로 변환
    pub orientation: [f64; 4],
}

impl Arm {
    /// 빈 `ArmBuilder`를 반환한다.
    pub fn builder() -> ArmBuilder {
        return ArmBuilder::new();
    }

    /// URDF 등에서 보존한 일반 revolute 직렬 체인으로 팔을 만든다.
    pub fn from_serial_chain(
        base: Point3,
        rail: Option<LinearRail>,
        chain: SerialChain,
        limits: Vec<Option<JointLimit>>,
        default_joints: Joints,
        max_joint_speed: f64,
    ) -> Result<Self, ArmBuildError> {
        let chain_count = chain.joints.len();
        if chain_count != limits.len() || chain_count != default_joints.values.len() {
            return Err(ArmBuildError::KinematicsJointCountMismatch {
                chain: chain_count,
                limits: limits.len(),
                defaults: default_joints.values.len(),
            });
        }
        for (joint_index, limit) in limits.iter().enumerate() {
            let Some(limit) = limit else {
                continue;
            };
            if limit.min > limit.max {
                return Err(ArmBuildError::InvalidJointLimit {
                    joint_index,
                    min: limit.min,
                    max: limit.max,
                });
            }
            let value = default_joints.values[joint_index];
            if !limit.contains(value) {
                return Err(ArmBuildError::DefaultJointOutOfRange {
                    joint_index,
                    value,
                    min: limit.min,
                    max: limit.max,
                });
            }
        }
        if max_joint_speed <= 0.0 {
            return Err(ArmBuildError::NonPositiveMaxJointSpeed {
                value: max_joint_speed,
            });
        }
        let link_lengths = chain
            .joints
            .iter()
            .map(|joint| joint.origin.translation.vector.norm())
            .collect();
        return Ok(Self {
            base,
            rail,
            link_lengths,
            limits,
            default_joints,
            max_joint_speed,
            chain,
        });
    }

    fn arm_length(&self) -> f64 {
        return self.chain.approximate_reach();
    }

    /// revolute 축(관절) 개수.
    pub fn joint_count(&self) -> usize {
        return self.limits.len();
    }

    pub fn joint_limit(&self, index: usize) -> Option<JointLimit> {
        return self.limits.get(index).copied().flatten();
    }

    /// 라켓 open을 담당하는 관절. 3축 위치 체인은 별도 손목이 없다.
    pub fn wrist_joint_index(&self) -> Option<usize> {
        if self.chain.joints.len() >= 4 {
            return Some(self.chain.joints.len() - 1);
        }
        return None;
    }

    /// `default_joints`로 초기화된 런타임 상태.
    pub fn initial_state(&self) -> RobotState {
        let rail_x = self
            .rail
            .as_ref()
            .map(|rail| rail.home_x())
            .unwrap_or(self.base.coords.x);
        return RobotState::new(self.default_joints.clone(), rail_x);
    }

    /// 모든 관절각이 한계 안인지 확인한다.
    pub fn joints_in_limits(&self, joints: &Joints) -> bool {
        if joints.values.len() != self.joint_count() {
            return false;
        }
        return self
            .limits
            .iter()
            .zip(joints.values.iter())
            .all(|(limit, &angle)| limit.is_none_or(|limit| limit.contains(angle)));
    }

    /// 순기구학 - 관절각 -> 라켓 끝점/면 방향.
    ///
    /// 4축: yaw + 2R(어깨/팔꿈치 접힘) + 손목 open.
    pub fn forward_kinematics(&self, joints: &Joints) -> Option<RacketPose> {
        return self.forward_kinematics_at(self.base, joints);
    }

    /// `rail_x`가 주어진 레일 위치에서 FK.
    pub fn forward_kinematics_with_rail(&self, rail_x: f64, joints: &Joints) -> Option<RacketPose> {
        let mount = self.mount_at_rail(rail_x);
        return self.forward_kinematics_at(mount, joints);
    }

    /// 주어진 마운트 원점에서 FK.
    pub fn forward_kinematics_at(&self, mount: Point3, joints: &Joints) -> Option<RacketPose> {
        let (ee, _) = self.chain.forward_with_joint_frames(mount.coords, &joints.values)?;
        return Some(racket_pose_from_isometry(ee));
    }

    /// 마운트부터 EE까지의 체인 점 - OBB/뷰어 공용.
    pub fn chain_points(&self, rail_x: f64, joints: &Joints) -> Option<Vec<Vector3<f64>>> {
        let mount = self.mount_at_rail(rail_x).coords;
        let (ee, frames) = self.chain.forward_with_joint_frames(mount, &joints.values)?;
        let mut points = Vec::with_capacity(frames.len() + 2);
        points.push(mount);
        for (position, _) in frames {
            if (position - *points.last().expect("mount")).norm_squared() > 1e-16 {
                points.push(position);
            }
        }
        let ee_position = ee.translation.vector;
        if (ee_position - *points.last().expect("mount")).norm_squared() > 1e-16 {
            points.push(ee_position);
        }
        return Some(points);
    }

    /// 손목 open [rad]을 한계 안으로 넣어 새 `Joints`를 만든다.
    pub fn with_wrist_open(&self, joints: &Joints, open: f64) -> Result<Joints, SwingPlanError> {
        if joints.values.len() != self.joint_count() {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
            });
        }
        let Some(wrist_index) = self.wrist_joint_index() else {
            return Ok(joints.clone());
        };
        let requested = -open;
        let clamped = self
            .joint_limit(wrist_index)
            .map_or(requested, |limit| requested.clamp(limit.min, limit.max));
        let mut values = joints.values.clone();
        values[wrist_index] = clamped;
        return Ok(Joints { values });
    }

    /// 리턴 속도 방향에 맞춘 손목 open [rad] (수평/수직 성분).
    pub fn wrist_open_for_return(v_out: Vector3<f64>) -> f64 {
        let horizontal = (v_out.x * v_out.x + v_out.y * v_out.y).sqrt().max(1e-6);
        return v_out.z.atan2(horizontal);
    }

    /// 역기구학 - 라켓 끝을 `target`에 두는 관절각.
    pub fn inverse_kinematics(&self, target: Point3) -> Result<Joints, SwingPlanError> {
        return self.inverse_kinematics_near(target, None);
    }

    /// `hint`에 가까운 IK 해를 고른다 (스윙 연속성용).
    pub fn inverse_kinematics_near(
        &self,
        target: Point3,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
        return self.inverse_kinematics_at_mount(self.base, target, hint);
    }

    /// 레일 x에서 IK - X는 레일이 맡고 팔은 Y/Z 평면.
    pub fn inverse_kinematics_with_rail(
        &self,
        rail: &LinearRail,
        rail_x: f64,
        target: Point3,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
        return self.inverse_kinematics_at_mount(rail.mount_point(rail_x), target, hint);
    }

    /// 레일과 관절을 함께 움직여 라켓 중심과 면 법선을 맞춘다.
    pub fn inverse_pose_with_rail(
        &self,
        target: Point3,
        target_normal: Vector3<f64>,
        hint: &RobotPose,
    ) -> Result<RobotPose, SwingPlanError> {
        const MAX_ITERS: usize = 250;
        const POSITION_TOLERANCE: f64 = 2e-4;
        const NORMAL_TOLERANCE: f64 = 1e-3;
        const STEP: f64 = 1e-6;
        const DAMPING: f64 = 1e-3;
        const MAX_UPDATE: f64 = 0.2;

        if target_normal.norm_squared() <= f64::EPSILON
            || hint.joints.values.len() != self.joint_count()
        {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: target.coords.x,
                target_y: target.coords.y,
                target_z: target.coords.z,
            });
        }
        let target_normal = target_normal.normalize();
        let reference = if target_normal.z.abs() < 0.9 {
            Vector3::z()
        } else {
            Vector3::x()
        };
        let tangent_a = target_normal.cross(&reference).normalize();
        let tangent_b = target_normal.cross(&tangent_a).normalize();
        let has_rail = self.rail.is_some();

        let make_values = |rail_x: f64, joints: &Joints| {
            let mut values = Vec::with_capacity(self.joint_count() + usize::from(has_rail));
            if has_rail {
                values.push(rail_x);
            }
            values.extend_from_slice(&joints.values);
            values
        };
        let decode = |values: &[f64]| {
            let (rail_x, offset) = if has_rail {
                (values[0], 1)
            } else {
                (hint.rail_x, 0)
            };
            (
                rail_x,
                Joints {
                    values: values[offset..].to_vec(),
                },
            )
        };
        let task = |pose: RacketPose| {
            let normal_error = target_normal - pose.normal;
            DVector::from_vec(vec![
                target.coords.x - pose.position.coords.x,
                target.coords.y - pose.position.coords.y,
                target.coords.z - pose.position.coords.z,
                normal_error.dot(&tangent_a),
                normal_error.dot(&tangent_b),
            ])
        };

        let mut seeds = vec![make_values(hint.rail_x, &hint.joints)];
        if let Some(rail) = &self.rail {
            seeds.push(make_values(rail.clamp_x(target.coords.x), &self.default_joints));
            seeds.push(make_values(rail.default_x(), &self.default_joints));
        }
        let mut best: Option<(f64, RobotPose)> = None;
        for mut values in seeds {
            for _ in 0..MAX_ITERS {
                let (rail_x, joints) = decode(&values);
                let Some(pose) = self.forward_kinematics_with_rail(rail_x, &joints) else {
                    break;
                };
                let position_error = (target.coords - pose.position.coords).norm();
                let normal_error = (target_normal - pose.normal).norm();
                let score = position_error + normal_error;
                if best
                    .as_ref()
                    .is_none_or(|(best_score, _)| score < *best_score)
                {
                    best = Some((score, RobotPose::new(rail_x, joints.clone())));
                }
                if position_error <= POSITION_TOLERANCE && normal_error <= NORMAL_TOLERANCE {
                    return Ok(RobotPose::new(rail_x, joints));
                }

                let error = task(pose);
                let mut jacobian = DMatrix::zeros(5, values.len());
                for index in 0..values.len() {
                    let joint_offset = usize::from(has_rail);
                    let difference = if has_rail
                        && index == 0
                        && self.rail.is_some_and(|rail| values[0] + STEP > rail.x_max)
                    {
                        -STEP
                    } else if index >= joint_offset
                        && self
                            .joint_limit(index - joint_offset)
                            .is_some_and(|limit| values[index] + STEP > limit.max)
                    {
                        -STEP
                    } else {
                        STEP
                    };
                    let mut perturbed = values.clone();
                    perturbed[index] += difference;
                    let (perturbed_rail, perturbed_joints) = decode(&perturbed);
                    let Some(perturbed_pose) =
                        self.forward_kinematics_with_rail(perturbed_rail, &perturbed_joints)
                    else {
                        continue;
                    };
                    let derivative = (task(pose) - task(perturbed_pose)) / difference;
                    jacobian.set_column(index, &derivative);
                }
                let regularized = &jacobian * jacobian.transpose()
                    + DMatrix::identity(5, 5) * (DAMPING * DAMPING);
                let Some(inverse) = regularized.try_inverse() else {
                    break;
                };
                let mut delta = jacobian.transpose() * (inverse * error);
                if delta.norm() > MAX_UPDATE {
                    delta *= MAX_UPDATE / delta.norm();
                }
                for (index, value) in values.iter_mut().enumerate() {
                    *value += delta[index];
                }
                let joint_offset = usize::from(has_rail);
                if let Some(rail) = &self.rail {
                    values[0] = rail.clamp_x(values[0]);
                }
                for index in 0..self.joint_count() {
                    if let Some(limit) = self.joint_limit(index) {
                        values[index + joint_offset] =
                            values[index + joint_offset].clamp(limit.min, limit.max);
                    }
                }
            }
        }
        if let Some((_, candidate)) = best {
            let pose = self
                .forward_kinematics_with_rail(candidate.rail_x, &candidate.joints)
                .expect("validated pose IK candidate");
            if (target.coords - pose.position.coords).norm() <= POSITION_TOLERANCE
                && (target_normal - pose.normal).norm() <= NORMAL_TOLERANCE
            {
                return Ok(candidate);
            }
        }
        return Err(SwingPlanError::InverseKinematicsNoSolution {
            target_x: target.coords.x,
            target_y: target.coords.y,
            target_z: target.coords.z,
        });
    }

    fn inverse_kinematics_at_mount(
        &self,
        mount: Point3,
        target: Point3,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
        const MAX_ITERS: usize = 300;
        const TOLERANCE: f64 = 1e-5;
        const DAMPING: f64 = 1e-3;
        const MAX_STEP: f64 = 0.25;

        let mut seeds = Vec::new();
        if let Some(hint) = hint.filter(|joints| joints.values.len() == self.joint_count()) {
            seeds.push(hint.values.clone());
        }
        seeds.push(self.default_joints.values.clone());
        seeds.push(
            self.limits
                .iter()
                .enumerate()
                .map(|(index, limit)| {
                    limit.map_or(self.default_joints.values[index], |limit| {
                        (limit.min + limit.max) * 0.5
                    })
                })
                .collect(),
        );
        for fraction in [0.25, 0.75] {
            seeds.push(
                self.limits
                    .iter()
                    .enumerate()
                    .map(|(index, limit)| {
                        limit.map_or(self.default_joints.values[index], |limit| {
                            limit.min + (limit.max - limit.min) * fraction
                        })
                    })
                    .collect(),
            );
        }

        let mut best_error = f64::INFINITY;
        let mut best = None;
        for mut values in seeds {
            for _ in 0..MAX_ITERS {
                let joints = Joints {
                    values: values.clone(),
                };
                let Some(pose) = self.forward_kinematics_at(mount, &joints) else {
                    break;
                };
                let error = target.coords - pose.position.coords;
                let error_norm = error.norm();
                if error_norm < best_error {
                    best_error = error_norm;
                    best = Some(values.clone());
                }
                if error_norm <= TOLERANCE {
                    return Ok(joints);
                }

                let Some(jacobian) = self.position_jacobian_at(mount, &joints) else {
                    break;
                };
                let regularized =
                    &jacobian * jacobian.transpose() + Matrix3::identity() * (DAMPING * DAMPING);
                let Some(inverse) = regularized.try_inverse() else {
                    break;
                };
                let mut delta = jacobian.transpose() * (inverse * error);
                let norm = delta.norm();
                if norm > MAX_STEP {
                    delta *= MAX_STEP / norm;
                }
                for (index, value) in values.iter_mut().enumerate() {
                    *value += delta[index];
                    if let Some(limit) = self.joint_limit(index) {
                        *value = (*value).clamp(limit.min, limit.max);
                    }
                }
            }
        }

        if best_error <= TOLERANCE {
            return Ok(Joints {
                values: best.expect("best IK seed"),
            });
        }
        return Err(SwingPlanError::InverseKinematicsNoSolution {
            target_x: target.coords.x,
            target_y: target.coords.y,
            target_z: target.coords.z,
        });
    }

    /// 레일 x에서의 마운트 원점.
    pub fn mount_at_rail(&self, rail_x: f64) -> Point3 {
        if let Some(rail) = &self.rail {
            return rail.mount_point(rail_x);
        }
        return self.base;
    }

    /// 리니어 레일 + 팔 도달 범위로 임팩트점을 보정한다.
    ///
    /// 가능하면 hit-plane y를 유지하고 xz(레일/높이)만 줄인다.
    /// 구면 투영만 하면 y가 로봇 쪽으로 당겨져 타이밍/접촉이 어긋난다.
    pub fn clamp_impact_for_rail(&self, rail: &LinearRail, target: Point3) -> (f64, Point3) {
        let rail_x = rail.clamp_x(target.coords.x);
        let mount = rail.mount_point(rail_x);
        return (
            rail_x,
            Self::clamp_preserving_y(mount, target, self.arm_length()),
        );
    }

    /// `y`(접수 깊이)를 우선 보존하며 도달 구 안으로 투영한다.
    fn clamp_preserving_y(mount: Point3, target: Point3, arm_length: f64) -> Point3 {
        let max_reach = (arm_length - 1e-3).max(0.0);
        let rel = target.coords - mount.coords;
        let distance = rel.norm();
        if distance <= max_reach || distance < f64::EPSILON {
            return target;
        }

        let y_comp = rel.y;
        let lateral_sq = max_reach * max_reach - y_comp * y_comp;
        if lateral_sq > 0.0 {
            let max_lat = lateral_sq.sqrt();
            let lateral = Vector3::new(rel.x, 0.0, rel.z);
            let lat_norm = lateral.norm();
            if lat_norm > 1e-9 {
                let scale = (max_lat / lat_norm).min(1.0);
                return Point3::from(
                    mount.coords + Vector3::new(lateral.x * scale, y_comp, lateral.z * scale),
                );
            }
            return Point3::from(mount.coords + Vector3::new(0.0, y_comp, 0.0));
        }

        // y 자체만으로도 도달 불능 - 구면 투영 폴백
        return Point3::from(mount.coords + rel * (max_reach / distance));
    }

    /// 라켓 위치에 대한 3xN 자코비안 `dp/dq` (마운트 기준).
    fn position_jacobian_at(&self, mount: Point3, joints: &Joints) -> Option<DMatrix<f64>> {
        let (ee, frames) = self.chain.forward_with_joint_frames(mount.coords, &joints.values)?;
        let ee_position = ee.translation.vector;
        let mut jacobian = DMatrix::zeros(3, frames.len());
        for (index, (joint_position, joint_axis)) in frames.iter().enumerate() {
            let column = joint_axis.cross(&(ee_position - joint_position));
            jacobian[(0, index)] = column.x;
            jacobian[(1, index)] = column.y;
            jacobian[(2, index)] = column.z;
        }
        return Some(jacobian);
    }

    /// 라켓 면 법선을 유지하는 레일·관절 속도 역산.
    pub fn velocities_for_racket_velocity(
        &self,
        pose: &RobotPose,
        racket_velocity: Vector3<f64>,
    ) -> Result<(f64, Vec<f64>), SwingPlanError> {
        const STEP: f64 = 1e-6;
        if pose.joints.values.len() != self.joint_count() {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: racket_velocity.x,
                target_y: racket_velocity.y,
                target_z: racket_velocity.z,
            });
        }
        let current = self
            .forward_kinematics_with_rail(pose.rail_x, &pose.joints)
            .ok_or(SwingPlanError::InverseKinematicsNoSolution {
                target_x: racket_velocity.x,
                target_y: racket_velocity.y,
                target_z: racket_velocity.z,
            })?;
        let reference = if current.normal.z.abs() < 0.9 {
            Vector3::z()
        } else {
            Vector3::x()
        };
        let tangent_a = current.normal.cross(&reference).normalize();
        let tangent_b = current.normal.cross(&tangent_a).normalize();
        let has_rail = self.rail.is_some();
        let mut values = Vec::with_capacity(self.joint_count() + usize::from(has_rail));
        if has_rail {
            values.push(pose.rail_x);
        }
        values.extend_from_slice(&pose.joints.values);
        let output = |racket: RacketPose| {
            DVector::from_vec(vec![
                racket.position.coords.x,
                racket.position.coords.y,
                racket.position.coords.z,
                racket.normal.dot(&tangent_a),
                racket.normal.dot(&tangent_b),
            ])
        };
        let base_output = output(current);
        let mut jacobian = DMatrix::zeros(5, values.len());
        for index in 0..values.len() {
            let joint_offset = usize::from(has_rail);
            let difference = if has_rail
                && index == 0
                && self.rail.is_some_and(|rail| values[0] + STEP > rail.x_max)
            {
                -STEP
            } else if index >= joint_offset
                && self
                    .joint_limit(index - joint_offset)
                    .is_some_and(|limit| values[index] + STEP > limit.max)
            {
                -STEP
            } else {
                STEP
            };
            let mut perturbed = values.clone();
            perturbed[index] += difference;
            let (rail_x, joint_values) = if has_rail {
                (perturbed[0], perturbed[1..].to_vec())
            } else {
                (pose.rail_x, perturbed)
            };
            let perturbed_pose = self
                .forward_kinematics_with_rail(
                    rail_x,
                    &Joints {
                        values: joint_values,
                    },
                )
                .ok_or(SwingPlanError::InverseKinematicsNoSolution {
                    target_x: racket_velocity.x,
                    target_y: racket_velocity.y,
                    target_z: racket_velocity.z,
                })?;
            jacobian.set_column(
                index,
                &((output(perturbed_pose) - &base_output) / difference),
            );
        }
        let target = DVector::from_vec(vec![
            racket_velocity.x,
            racket_velocity.y,
            racket_velocity.z,
            0.0,
            0.0,
        ]);
        let regularized =
            jacobian.transpose() * &jacobian + DMatrix::identity(values.len(), values.len()) * 1e-9;
        let Some(inverse) = regularized.try_inverse() else {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: racket_velocity.x,
                target_y: racket_velocity.y,
                target_z: racket_velocity.z,
            });
        };
        let velocities = inverse * jacobian.transpose() * target;
        let (rail_velocity, offset) = if has_rail {
            (velocities[0], 1)
        } else {
            (0.0, 0)
        };
        return Ok((
            rail_velocity,
            velocities.iter().skip(offset).copied().collect(),
        ));
    }
}

fn racket_pose_from_isometry(iso: Isometry3<f64>) -> RacketPose {
    // 4-dof CAD 라켓 링크 계약: local +Y=면 법선(얇은 mesh 축), +Z=위.
    // 기구학/Rapier 라켓 계약: local +Z=면 법선, +Y=위.
    let normal = iso.rotation * Vector3::y();
    let link_from_racket = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0)),
        std::f64::consts::PI,
    );
    let orientation = iso.rotation * link_from_racket;
    let q = orientation.quaternion();
    return RacketPose {
        position: Point3::from(iso.translation.vector),
        normal,
        orientation: [q.w, q.i, q.j, q.k],
    };
}
