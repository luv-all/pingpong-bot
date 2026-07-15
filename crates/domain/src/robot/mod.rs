//! 로봇 팔 기구학.
//!
//! `Arm`은 sim/real이 같이 쓰는 불변 기하 모델이다. 부팅 때 한 번 만들고
//! `Arc<Arm>`으로 넘긴다. FK/IK랑 스윙 계획은 이 타입만 본다.
//! Rapier/Dynamixel 쪽 변환은 infra 어댑터가 `RacketPose`로 한다.
//!
//! 조립은 `ArmBuilder`, 런타임 추종은 `RobotState`.

pub mod builder;
pub mod rail;
pub mod serial;
pub mod state;

#[cfg(test)]
mod tests;

use nalgebra::{DMatrix, Isometry3, Matrix3, UnitQuaternion, Vector3};

pub use builder::{ArmBuildError, ArmBuilder, SUPPORTED_FK_JOINTS};
pub use serial::{SerialChain, SerialChainError, SerialJoint};
pub use state::RobotState;

use self::rail::LinearRail;
use crate::constants::ARM_POSITION_LINKS;
use crate::error::SwingPlanError;
use crate::types::{Joints, Point3};

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ArmKinematics {
    Canonical4Dof,
    Serial(SerialChain),
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
    pub(crate) kinematics: ArmKinematics,
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

/// 2R planar 체인: 어깨/팔꿈치 각 -> (reach, height) 및 중간 점.
#[derive(Debug, Clone, Copy)]
struct PlanarPose {
    reach: f64,
    height: f64,
    elbow_reach: f64,
    elbow_height: f64,
}

fn planar_2r(l1: f64, l2: f64, a1: f64, a2: f64) -> PlanarPose {
    let elbow = a1 + a2;
    return PlanarPose {
        reach: l1 * a1.cos() + l2 * elbow.cos(),
        height: l1 * a1.sin() + l2 * elbow.sin(),
        elbow_reach: l1 * a1.cos(),
        elbow_height: l1 * a1.sin(),
    };
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
            kinematics: ArmKinematics::Serial(chain),
        });
    }

    /// 경진용 4DOF URDF 체인을 fixed-link 단위로 합성한 primitive 모델.
    ///
    /// 축·관절 순서·한계·EE offset은 `all-4-export.urdf`와 같고 mesh만 생략한다.
    ///
    /// 앱에서 쓰는 프리셋은 `pingpong_app::ROBOTS`.
    /// 여기는 domain/infra 테스트용으로 같은 체인을 둔다.
    pub fn competition() -> Result<Self, ArmBuildError> {
        use crate::constants::arm::{BASE_Y, MAX_JOINT_SPEED, RAIL_MAX_SPEED};
        use crate::constants::table;

        let joints = vec![
            SerialJoint::new(
                Isometry3::translation(-0.02575, 0.028, 0.0601),
                Vector3::new(-1.0, 0.0, 0.0),
            )
            .expect("4-dof q0 axis"),
            SerialJoint::new(
                Isometry3::translation(0.0255, 0.0, 0.0825),
                Vector3::new(0.0, 0.0, -1.0),
            )
            .expect("4-dof q1 axis"),
            SerialJoint::new(
                Isometry3::translation(0.0, 0.025, 0.1398),
                Vector3::new(-1.0, 0.0, 0.0),
            )
            .expect("4-dof q2 axis"),
            SerialJoint::new(
                Isometry3::translation(0.0, 0.1518, 0.0),
                Vector3::new(-1.0, 0.0, 0.0),
            )
            .expect("4-dof q3 axis"),
        ];
        let chain = SerialChain::new(
            UnitQuaternion::identity(),
            joints,
            Isometry3::translation(0.0, 0.0513, -0.034),
        )
        .expect("4-dof serial chain");
        return Self::builder()
            .base_xyz(0.0, BASE_Y, table::SURFACE_Z)
            .linear_rail(
                BASE_Y,
                table::SURFACE_Z,
                0.0,
                table::WIDTH_X,
                RAIL_MAX_SPEED,
            )
            .serial_chain(
                chain,
                vec![
                    None,
                    Some(JointLimit::new(-0.523599, 0.523599)),
                    Some(JointLimit::new(-2.007129, 1.48353)),
                    Some(JointLimit::new(-2.094395, 2.094395)),
                ],
                Joints::from_slice(&[0.0, 0.0, -0.2617995, 0.0]),
            )
            .max_joint_speed(MAX_JOINT_SPEED)
            .build();
    }

    fn planar_link_lengths(&self) -> (f64, f64) {
        return (self.link_lengths[0], self.link_lengths[1]);
    }

    fn arm_length(&self) -> f64 {
        if let ArmKinematics::Serial(chain) = &self.kinematics {
            return chain.approximate_reach();
        }
        let (l1, l2) = self.planar_link_lengths();
        return l1 + l2;
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
        return match &self.kinematics {
            ArmKinematics::Canonical4Dof => Some(3),
            ArmKinematics::Serial(chain) if chain.joints.len() >= 4 => Some(chain.joints.len() - 1),
            ArmKinematics::Serial(_) => None,
        };
    }

    /// `default_joints`로 초기화된 런타임 상태.
    pub fn initial_state(&self) -> RobotState {
        let rail_x = self
            .rail
            .as_ref()
            .map(|rail| rail.home_x())
            .unwrap_or(self.base.v.x);
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
        if let ArmKinematics::Serial(chain) = &self.kinematics {
            let (ee, _) = chain.forward_with_joint_frames(mount.v, &joints.values)?;
            return Some(racket_pose_from_isometry(ee));
        }
        if joints.values.len() != SUPPORTED_FK_JOINTS
            || self.link_lengths.len() < ARM_POSITION_LINKS
        {
            return None;
        }
        let yaw = joints.values[0];
        let a1 = joints.values[1];
        let a2 = joints.values[2];
        let wrist_open = joints.values[3];
        let (l1, l2) = self.planar_link_lengths();
        let planar = planar_2r(l1, l2, a1, a2);

        let offset = Vector3::new(
            planar.reach * yaw.sin(),
            planar.reach * yaw.cos(),
            planar.height,
        );
        let position = Point3::from_vector(mount.v + offset);
        let (normal, orientation) = racket_face_toward_opponent(yaw, wrist_open);

        return Some(RacketPose {
            position,
            normal,
            orientation,
        });
    }

    /// 마운트부터 EE까지의 체인 점 - OBB/뷰어 공용.
    pub fn chain_points(&self, rail_x: f64, joints: &Joints) -> Option<Vec<Vector3<f64>>> {
        if let ArmKinematics::Serial(chain) = &self.kinematics {
            let mount = self.mount_at_rail(rail_x).v;
            let (ee, frames) = chain.forward_with_joint_frames(mount, &joints.values)?;
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
        if joints.values.len() != SUPPORTED_FK_JOINTS
            || self.link_lengths.len() < ARM_POSITION_LINKS
        {
            return None;
        }
        let yaw = joints.values[0];
        let a1 = joints.values[1];
        let a2 = joints.values[2];
        let (l1, l2) = self.planar_link_lengths();
        let planar = planar_2r(l1, l2, a1, a2);
        let mount = self.mount_at_rail(rail_x).v;

        let to_world = |reach: f64, height: f64| -> Vector3<f64> {
            return mount + Vector3::new(reach * yaw.sin(), reach * yaw.cos(), height);
        };

        let base = mount;
        let elbow = to_world(planar.elbow_reach, planar.elbow_height);
        let wrist = to_world(planar.reach, planar.height);
        return Some(vec![base, elbow, wrist]);
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
        let requested = if matches!(&self.kinematics, ArmKinematics::Serial(_)) {
            -open
        } else {
            open
        };
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

    fn inverse_kinematics_at_mount(
        &self,
        mount: Point3,
        target: Point3,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
        if matches!(&self.kinematics, ArmKinematics::Serial(_)) {
            return self.inverse_kinematics_serial(mount, target, hint);
        }
        if self.joint_count() != SUPPORTED_FK_JOINTS {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: target.v.x,
                target_y: target.v.y,
                target_z: target.v.z,
            });
        }

        let rel = target.v - mount.v;
        let planar_reach = (rel.x * rel.x + rel.y * rel.y).sqrt();
        let planar_height = rel.z;
        let yaw = rel.x.atan2(rel.y);

        let (l1, l2) = self.planar_link_lengths();
        let d_sq = planar_reach * planar_reach + planar_height * planar_height;
        let reach = d_sq.sqrt();

        const EPS: f64 = 1e-6;
        let reach_max = l1 + l2;
        let reach_min = (l1 - l2).abs();
        if reach > reach_max + EPS || reach < reach_min - EPS {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: target.v.x,
                target_y: target.v.y,
                target_z: target.v.z,
            });
        }

        let wrist = hint
            .and_then(|h| h.values.get(3).copied())
            .unwrap_or(self.default_joints.values[3]);
        let wrist = self
            .joint_limit(3)
            .map_or(wrist, |limit| wrist.clamp(limit.min, limit.max));

        let cos_a2 = ((d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)).clamp(-1.0, 1.0);
        let a2_mag = cos_a2.acos();
        let alpha = planar_height.atan2(planar_reach);

        let mut candidates: Vec<Joints> = Vec::with_capacity(2);
        for &a2 in &[a2_mag, -a2_mag] {
            let a1 = alpha - (l2 * a2.sin()).atan2(l1 + l2 * a2.cos());
            candidates.push(Joints::from_slice(&[yaw, a1, a2, wrist]));
        }

        candidates.sort_by(|a, b| {
            let score_a = ik_hint_distance(a, hint);
            let score_b = ik_hint_distance(b, hint);
            score_a
                .partial_cmp(&score_b)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        for joints in &candidates {
            if self.joints_in_limits(joints) {
                return Ok(joints.clone());
            }
        }

        if let Some(joints) = candidates.first() {
            for (joint_index, (&angle, limit)) in
                joints.values.iter().zip(self.limits.iter()).enumerate()
            {
                if let Some(limit) = limit.filter(|limit| !limit.contains(angle)) {
                    return Err(SwingPlanError::JointLimit {
                        joint_index,
                        value: angle,
                        min: limit.min,
                        max: limit.max,
                    });
                }
            }
        }

        return Err(SwingPlanError::InverseKinematicsNoSolution {
            target_x: target.v.x,
            target_y: target.v.y,
            target_z: target.v.z,
        });
    }

    fn inverse_kinematics_serial(
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
                let error = target.v - pose.position.v;
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
            target_x: target.v.x,
            target_y: target.v.y,
            target_z: target.v.z,
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
        let rail_x = rail.clamp_x(target.v.x);
        let mount = rail.mount_point(rail_x);
        return (
            rail_x,
            Self::clamp_preserving_y(mount, target, self.arm_length()),
        );
    }

    /// 월드 목표를 팔 도달 반경 안으로 당긴다 (고정 베이스/레일 없을 때).
    pub fn clamp_to_reach(&self, target: Point3) -> Point3 {
        return Self::clamp_preserving_y(self.base, target, self.arm_length());
    }

    /// `y`(접수 깊이)를 우선 보존하며 도달 구 안으로 투영한다.
    fn clamp_preserving_y(mount: Point3, target: Point3, arm_length: f64) -> Point3 {
        let max_reach = (arm_length - 1e-3).max(0.0);
        let rel = target.v - mount.v;
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
                return Point3::from_vector(
                    mount.v + Vector3::new(lateral.x * scale, y_comp, lateral.z * scale),
                );
            }
            return Point3::from_vector(mount.v + Vector3::new(0.0, y_comp, 0.0));
        }

        // y 자체만으로도 도달 불능 - 구면 투영 폴백
        return Point3::from_vector(mount.v + rel * (max_reach / distance));
    }

    /// 라켓 위치에 대한 3xN 자코비안 `dp/dq`.
    pub fn position_jacobian(&self, joints: &Joints) -> Option<DMatrix<f64>> {
        return self.position_jacobian_at(self.base, joints);
    }

    fn position_jacobian_at(&self, mount: Point3, joints: &Joints) -> Option<DMatrix<f64>> {
        if let ArmKinematics::Serial(chain) = &self.kinematics {
            let (ee, frames) = chain.forward_with_joint_frames(mount.v, &joints.values)?;
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
        if joints.values.len() != SUPPORTED_FK_JOINTS {
            return None;
        }
        let yaw = joints.values[0];
        let a1 = joints.values[1];
        let a2 = joints.values[2];
        let elbow = a1 + a2;
        let (l1, l2) = self.planar_link_lengths();

        let dreach_da1 = -l1 * a1.sin() - l2 * elbow.sin();
        let dreach_da2 = -l2 * elbow.sin();
        let dheight_da1 = l1 * a1.cos() + l2 * elbow.cos();
        let dheight_da2 = l2 * elbow.cos();

        let planar = planar_2r(l1, l2, a1, a2);

        let dyaw = Vector3::new(planar.reach * yaw.cos(), -planar.reach * yaw.sin(), 0.0);
        let da1 = Vector3::new(yaw.sin() * dreach_da1, yaw.cos() * dreach_da1, dheight_da1);
        let da2 = Vector3::new(yaw.sin() * dreach_da2, yaw.cos() * dreach_da2, dheight_da2);

        let position = Matrix3::from_columns(&[dyaw, da1, da2]);
        let mut jacobian = DMatrix::zeros(3, SUPPORTED_FK_JOINTS);
        jacobian.view_mut((0, 0), (3, 3)).copy_from(&position);
        return Some(jacobian);
    }

    /// 엔드이펙터 선속도에서 감쇠 최소제곱 관절 각속도를 구한다.
    pub fn joint_velocities_for_ee_velocity(
        &self,
        joints: &Joints,
        ee_velocity: Vector3<f64>,
    ) -> Result<Vec<f64>, SwingPlanError> {
        let j =
            self.position_jacobian(joints)
                .ok_or(SwingPlanError::InverseKinematicsNoSolution {
                    target_x: 0.0,
                    target_y: 0.0,
                    target_z: 0.0,
                })?;
        let regularized = &j * j.transpose() + Matrix3::identity() * 1e-6;
        let Some(inverse) = regularized.try_inverse() else {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: ee_velocity.x,
                target_y: ee_velocity.y,
                target_z: ee_velocity.z,
            });
        };
        let q_dot = j.transpose() * (inverse * ee_velocity);
        return Ok(q_dot.iter().copied().collect());
    }
}

fn ik_hint_distance(joints: &Joints, hint: Option<&Joints>) -> f64 {
    let Some(hint) = hint else {
        return 0.0;
    };
    return joints
        .values
        .iter()
        .zip(hint.values.iter())
        .map(|(a, b)| (a - b).abs())
        .sum();
}

/// 라켓 면 법선/자세 - 상대(yaw 방향)를 보고 `open`만큼 연다.
///
/// sim 콜라이더/뷰어 큐브는 local +Z가 얇은 축(면 법선)이다.
fn racket_face_toward_opponent(yaw: f64, open: f64) -> (Vector3<f64>, [f64; 4]) {
    let cy = yaw.cos();
    let sy = yaw.sin();
    let cp = open.cos();
    let sp = open.sin();
    // yaw=0 -> +Y(슈터/상대), open -> +Z 성분
    let normal = Vector3::new(sy * cp, cy * cp, sp).normalize();
    // 면 위쪽(local +Y): 월드 대략 +Z에 가깝게
    let mut face_up = Vector3::new(-sy * sp, -cy * sp, cp);
    if face_up.norm() < 1e-9 {
        face_up = Vector3::new(0.0, 0.0, 1.0);
    } else {
        face_up = face_up.normalize();
    }
    let face_right = face_up.cross(&normal).normalize();
    let face_up = normal.cross(&face_right).normalize();
    return (normal, rotation_matrix_to_quat(face_right, face_up, normal));
}

fn racket_pose_from_isometry(iso: Isometry3<f64>) -> RacketPose {
    // 4-dof CAD 라켓 링크 계약: local +Y=면 법선(얇은 mesh 축), +Z=위.
    // domain/Rapier 라켓 계약: local +Z=면 법선, +Y=위.
    let normal = iso.rotation * Vector3::y();
    let link_from_racket = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0)),
        std::f64::consts::PI,
    );
    let orientation = iso.rotation * link_from_racket;
    let q = orientation.quaternion();
    return RacketPose {
        position: Point3::from_vector(iso.translation.vector),
        normal,
        orientation: [q.w, q.i, q.j, q.k],
    };
}

/// 열 (local X,Y,Z) -> 월드 기저로 가는 회전의 Hamilton 쿼터니언 (w,x,y,z).
fn rotation_matrix_to_quat(x: Vector3<f64>, y: Vector3<f64>, z: Vector3<f64>) -> [f64; 4] {
    let matrix = Matrix3::from_columns(&[x, y, z]);
    let q = UnitQuaternion::from_matrix(&matrix);
    let p = q.into_inner();
    return [p.w, p.i, p.j, p.k];
}
