//! 로봇 팔 기구학 (plan §7.2–§7.4).
//!
//! `Arm`은 sim·real 공통 **불변** 기하 모델이다. 부팅 시 한 번 만들어
//! `Arc<Arm>`으로 공유하고, FK/IK·스윙 계획은 전부 이 타입만 본다.
//! Rapier·Dynamixel 등은 `infra` 어댑터가 `RacketPose`를 각 SDK 형식으로 변환한다.
//!
//! 설계가 바뀔 때마다 [`ArmBuilder`]로 **base → link → revolute → …** 순서 선언한다.

use std::fmt;

use nalgebra::Vector3;

use crate::types::{Joints, Point3, World};

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

/// 로봇 팔 불변 모델. sim·real·plan_swing이 같은 `Arm`을 참조한다.
#[derive(Debug, Clone, PartialEq)]
pub struct Arm {
    /// 베이스 원점 (월드 좌표) [m]
    pub base: Point3<World>,
    /// revolute 축 순서대로의 링크 길이 [m] — `limits`·`default_joints`와 같은 길이
    pub link_lengths: Vec<f64>,
    /// 축별 관절 한계
    pub limits: Vec<JointLimit>,
    /// 부팅 시 초기 관절각
    pub default_joints: Joints,
    /// 관절 추종 최대 각속도 [rad/s]
    pub max_joint_speed: f64,
}

/// 월드 좌표계 라켓 자세 — sim·real 동일 표현.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RacketPose {
    /// 라켓 중심 위치 (월드)
    pub position: Point3<World>,
    /// 라켓 면 법선 (단위 벡터, plan §7.1)
    pub normal: Vector3<f64>,
    /// Hamilton 단위 쿼터니언 (w, x, y, z) — 어댑터가 SDK 회전으로 변환
    pub orientation: [f64; 4],
}

/// [`Arm`] 조립용 빌더 — base 이후 **link → revolute** 를 키네마틱 체인 순서로 선언한다.
#[derive(Debug, Clone)]
pub struct ArmBuilder {
    /// 베이스 위치 (미설정 시 build 실패)
    base: Option<Point3<World>>,
    /// revolute 축 순서대로 수집된 링크 길이 [m]
    link_lengths: Vec<f64>,
    /// revolute 축 순서대로 수집된 관절 한계
    limits: Vec<JointLimit>,
    /// 각 revolute의 초기 관절각 [rad]
    default_joint_values: Vec<f64>,
    /// 최대 관절 속도 (미설정 시 2.5 rad/s)
    max_joint_speed: Option<f64>,
    /// 체인 선언 순서 (build 시 검증)
    phase: ChainPhase,
    /// link/revolute 순서 위반 등 조립 중 첫 오류
    chain_error: Option<ArmBuildError>,
}

/// 빌더가 기대하는 다음 체인 요소.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ChainPhase {
    /// `.base` / `.base_xyz` 필요
    NeedBase,
    /// `.link` 필요
    NeedLink,
    /// `.revolute` 필요 (직전 link에 대한 joint)
    NeedJoint,
}

/// `ArmBuilder::build` 실패 이유.
#[derive(Debug, Clone, PartialEq)]
pub enum ArmBuildError {
    /// base 미설정
    MissingBase,
    /// link→revolute 쌍이 하나도 없음
    EmptyChain,
    /// 마지막 link 뒤에 revolute 없음
    IncompleteChain,
    /// `.link` 와야 하는데 다른 호출
    ExpectedLink,
    /// `.revolute` 와야 하는데 `.link` 호출
    ExpectedJoint,
    /// 링크 길이 ≤ 0
    InvalidLinkLength {
        link_index: usize,
        value: f64,
    },
    /// min > max
    InvalidJointLimit {
        joint_index: usize,
        min: f64,
        max: f64,
    },
    /// 기본 관절각이 한계 밖
    DefaultJointOutOfRange {
        joint_index: usize,
        value: f64,
        min: f64,
        max: f64,
    },
    /// FK 미지원 DOF
    UnsupportedKinematics {
        joint_count: usize,
        supported: usize,
    },
    /// max_joint_speed ≤ 0
    NonPositiveMaxJointSpeed {
        value: f64,
    },
}

impl fmt::Display for ArmBuildError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingBase => return write!(f, "베이스 위치(base)가 설정되지 않았습니다"),
            Self::EmptyChain => return write!(f, "link→revolute 체인이 비어 있습니다"),
            Self::IncompleteChain => {
                return write!(f, "마지막 link 뒤에 revolute 관절이 없습니다");
            }
            Self::ExpectedLink => return write!(f, "이 시점에는 .link()가 와야 합니다"),
            Self::ExpectedJoint => return write!(f, "이 시점에는 .revolute()가 와야 합니다"),
            Self::InvalidLinkLength { link_index, value } => {
                return write!(f, "링크 {link_index} 길이가 양수가 아닙니다: {value}");
            }
            Self::InvalidJointLimit {
                joint_index,
                min,
                max,
            } => return write!(
                f,
                "관절 {joint_index} 한계가 뒤집혔습니다: min={min}, max={max}"
            ),
            Self::DefaultJointOutOfRange {
                joint_index,
                value,
                min,
                max,
            } => return write!(
                f,
                "관절 {joint_index} 기본값 {value:.3} rad 가 허용 범위 [{min:.3}, {max:.3}] 밖"
            ),
            Self::UnsupportedKinematics {
                joint_count,
                supported,
            } => return write!(
                f,
                "현재 FK는 {supported}축만 지원합니다 (요청: {joint_count}축)"
            ),
            Self::NonPositiveMaxJointSpeed { value } => {
                return write!(f, "max_joint_speed는 양수여야 합니다: {value}");
            }
        }
    }
}

impl std::error::Error for ArmBuildError {}

/// 현재 FK 구현이 지원하는 revolute 축 수.
pub const SUPPORTED_FK_JOINTS: usize = 3;

impl ArmBuilder {
    /// 빈 빌더를 만든다.
    pub fn new() -> Self {
        return Self {
            base: None,
            link_lengths: Vec::new(),
            limits: Vec::new(),
            default_joint_values: Vec::new(),
            max_joint_speed: None,
            phase: ChainPhase::NeedBase,
            chain_error: None,
        };
    }

    fn record_chain_error(&mut self, err: ArmBuildError) {
        if self.chain_error.is_none() {
            self.chain_error = Some(err);
        }
    }

    /// 베이스 위치를 설정한다. 이후 `.link()` → `.revolute()` 순으로 체인을 선언한다.
    pub fn base(mut self, base: Point3<World>) -> Self {
        self.base = Some(base);
        self.phase = ChainPhase::NeedLink;
        return self;
    }

    /// 베이스 좌표 (x, y, z)를 설정한다.
    pub fn base_xyz(self, x: f64, y: f64, z: f64) -> Self {
        return self.base(Point3::new(x, y, z));
    }

    /// revolute 축 앞의 rigid link [m]. 직후 `.revolute()`가 와야 한다.
    pub fn link(mut self, length_m: f64) -> Self {
        if self.phase == ChainPhase::NeedBase {
            self.record_chain_error(ArmBuildError::MissingBase);
            return self;
        }
        if self.phase != ChainPhase::NeedLink {
            self.record_chain_error(ArmBuildError::ExpectedJoint);
            return self;
        }
        self.link_lengths.push(length_m);
        self.phase = ChainPhase::NeedJoint;
        return self;
    }

    /// 직전 link에 revolute 관절을 붙인다 (초기각 0 rad).
    pub fn revolute(self, min: f64, max: f64) -> Self {
        return self.revolute_at(min, max, 0.0);
    }

    /// 직전 link에 revolute 관절과 초기각을 붙인다.
    pub fn revolute_at(self, min: f64, max: f64, default: f64) -> Self {
        return self.revolute_limit_at(JointLimit::new(min, max), default);
    }

    /// `JointLimit`으로 revolute 관절을 붙인다 (초기각 0 rad).
    pub fn revolute_limit(self, limit: JointLimit) -> Self {
        return self.revolute_limit_at(limit, 0.0);
    }

    /// `JointLimit`과 초기각으로 revolute 관절을 붙인다.
    pub fn revolute_limit_at(mut self, limit: JointLimit, default: f64) -> Self {
        if self.phase == ChainPhase::NeedBase {
            self.record_chain_error(ArmBuildError::MissingBase);
            return self;
        }
        if self.phase != ChainPhase::NeedJoint {
            self.record_chain_error(ArmBuildError::ExpectedLink);
            return self;
        }
        self.limits.push(limit);
        self.default_joint_values.push(default);
        self.phase = ChainPhase::NeedLink;
        return self;
    }

    /// 최대 관절 각속도를 설정한다.
    pub fn max_joint_speed(mut self, rad_per_sec: f64) -> Self {
        self.max_joint_speed = Some(rad_per_sec);
        return self;
    }

    /// 검증 후 `Arm`을 만든다.
    pub fn build(self) -> Result<Arm, ArmBuildError> {
        if let Some(err) = self.chain_error {
            return Err(err);
        }

        let base = self.base.ok_or(ArmBuildError::MissingBase)?;

        if self.link_lengths.is_empty() {
            return Err(ArmBuildError::EmptyChain);
        }
        if self.phase == ChainPhase::NeedJoint {
            return Err(ArmBuildError::IncompleteChain);
        }

        let default_joints = Joints {
            values: self.default_joint_values,
        };

        if self.limits.len() != SUPPORTED_FK_JOINTS {
            return Err(ArmBuildError::UnsupportedKinematics {
                joint_count: self.limits.len(),
                supported: SUPPORTED_FK_JOINTS,
            });
        }

        for (link_index, &value) in self.link_lengths.iter().enumerate() {
            if value <= 0.0 {
                return Err(ArmBuildError::InvalidLinkLength { link_index, value });
            }
        }

        for (joint_index, limit) in self.limits.iter().enumerate() {
            if limit.min > limit.max {
                return Err(ArmBuildError::InvalidJointLimit {
                    joint_index,
                    min: limit.min,
                    max: limit.max,
                });
            }
            if !limit.contains(default_joints.values[joint_index]) {
                return Err(ArmBuildError::DefaultJointOutOfRange {
                    joint_index,
                    value: default_joints.values[joint_index],
                    min: limit.min,
                    max: limit.max,
                });
            }
        }

        let max_joint_speed = self.max_joint_speed.unwrap_or(2.5);
        if max_joint_speed <= 0.0 {
            return Err(ArmBuildError::NonPositiveMaxJointSpeed {
                value: max_joint_speed,
            });
        }

        return Ok(Arm {
            base,
            link_lengths: self.link_lengths,
            limits: self.limits,
            default_joints,
            max_joint_speed,
        });
    }
}

impl Default for ArmBuilder {
    fn default() -> Self {
        return Self::new();
    }
}

impl Arm {
    /// 빈 `ArmBuilder`를 반환한다.
    pub fn builder() -> ArmBuilder {
        return ArmBuilder::new();
    }

    /// revolute 축(관절) 개수.
    pub fn joint_count(&self) -> usize {
        return self.limits.len();
    }

    /// `default_joints`로 초기화된 런타임 상태.
    pub fn initial_state(&self) -> RobotState {
        return RobotState::new(self.default_joints.clone());
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
            .all(|(limit, &angle)| limit.contains(angle));
    }

    /// 순기구학 — 관절각 → 라켓 끝점·면 방향 (plan §7.2).
    ///
    /// 현재는 3축 revolute(yaw + 2-link planar)만 지원한다.
    pub fn forward_kinematics(&self, joints: &Joints) -> Option<RacketPose> {
        if joints.values.len() != SUPPORTED_FK_JOINTS {
            return None;
        }
        let yaw = joints.values[0];
        let a1 = joints.values[1];
        let a2 = joints.values[2];
        let elbow = a1 + a2;

        let l1 = self.link_lengths[0];
        let l2 = self.link_lengths[1];
        let l3 = self.link_lengths[2];

        let planar_reach = l1 * a1.cos() + l2 * elbow.cos() + l3 * elbow.cos();
        let planar_height = l1 * a1.sin() + l2 * elbow.sin() + l3 * elbow.sin();

        let offset = Vector3::new(
            planar_reach * yaw.sin(),
            planar_reach * yaw.cos(),
            planar_height,
        );
        let position = Point3::from_vector(self.base.v + offset);

        let normal = Vector3::new(
            elbow.sin() * yaw.sin(),
            elbow.sin() * yaw.cos(),
            elbow.cos(),
        )
        .normalize();
        let orientation = quaternion_from_yaw_elbow(yaw, elbow);

        return Some(RacketPose {
            position,
            normal,
            orientation,
        });
    }
}

/// 런타임 관절 상태 — sim `RobotState`·real encoder 읽기가 같은 타입을 채운다.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotState {
    /// 현재 관절각
    angles: Joints,
    /// 추종 목표 관절각
    targets: Joints,
}

impl RobotState {
    /// 초기 관절각으로 상태를 만든다.
    pub fn new(initial: Joints) -> Self {
        return Self {
            targets: initial.clone(),
            angles: initial,
        };
    }

    /// 현재 관절각.
    pub fn joints(&self) -> &Joints {
        return &self.angles;
    }

    /// 목표 관절각.
    pub fn targets(&self) -> &Joints {
        return &self.targets;
    }

    /// 목표 관절각을 직접 설정한다.
    pub fn set_targets(&mut self, targets: Joints) {
        self.targets = targets;
    }

    /// 스윙 궤적에서 목표 관절각을 가져온다.
    pub fn set_targets_from_trajectory(&mut self, trajectory: &crate::types::SwingTrajectory) {
        for (i, &value) in trajectory.joints.values.iter().enumerate() {
            if i < self.targets.values.len() {
                self.targets.values[i] = value;
            }
        }
    }

    /// 목표 관절각을 `max_speed` [rad/s]로 추종한다.
    pub fn step_toward_targets(&mut self, arm: &Arm, dt: f64) {
        let n = self.angles.values.len().min(self.targets.values.len());
        for i in 0..n {
            let diff = self.targets.values[i] - self.angles.values[i];
            let step = (arm.max_joint_speed * dt).min(diff.abs());
            self.angles.values[i] += diff.signum() * step;
        }
    }

    /// 현재 관절각으로 FK 라켓 자세를 계산한다.
    pub fn racket_pose(&self, arm: &Arm) -> Option<RacketPose> {
        return arm.forward_kinematics(&self.angles);
    }
}

/// yaw·elbow 각도로 단위 쿼터니언을 만든다.
fn quaternion_from_yaw_elbow(yaw: f64, elbow: f64) -> [f64; 4] {
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();
    let cx = (-elbow * 0.5).cos();
    let sx = (-elbow * 0.5).sin();
    return [cy * cx, cy * sx + sy * cx, sy * sx, sy * cx - cy * sx];
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::table;

    fn sample_three_dof_arm() -> Arm {
        return Arm::builder()
            .base_xyz(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z)
            .link(0.35)
            .revolute_at(-1.2, 1.2, 0.0)
            .link(0.30)
            .revolute_at(-0.2, 1.4, 0.6)
            .link(0.15)
            .revolute_at(-1.5, 0.5, -0.4)
            .max_joint_speed(2.5)
            .build()
            .expect("테스트용 3DOF arm");
    }

    #[test]
    fn builder_produces_three_dof_arm() {
        let arm = sample_three_dof_arm();
        assert_eq!(arm.joint_count(), 3);
    }

    #[test]
    fn builder_rejects_link_without_following_revolute() {
        let err = ArmBuilder::new()
            .base_xyz(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z)
            .link(0.3)
            .link(0.3)
            .build()
            .unwrap_err();
        assert!(matches!(err, ArmBuildError::ExpectedJoint));
    }

    #[test]
    fn builder_rejects_incomplete_chain_at_build() {
        let err = ArmBuilder::new()
            .base_xyz(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z)
            .link(0.3)
            .build()
            .unwrap_err();
        assert!(matches!(err, ArmBuildError::IncompleteChain));
    }

    #[test]
    fn builder_rejects_revolute_before_link() {
        let err = ArmBuilder::new()
            .base_xyz(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z)
            .revolute(-1.0, 1.0)
            .build()
            .unwrap_err();
        assert!(matches!(err, ArmBuildError::ExpectedLink));
    }

    #[test]
    fn builder_rejects_unsupported_dof() {
        let err = ArmBuilder::new()
            .base_xyz(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z)
            .link(0.3)
            .revolute(-1.0, 1.0)
            .link(0.3)
            .revolute(-1.0, 1.0)
            .link(0.1)
            .revolute(-1.0, 1.0)
            .link(0.1)
            .revolute(-1.0, 1.0)
            .build()
            .unwrap_err();
        assert!(matches!(err, ArmBuildError::UnsupportedKinematics { .. }));
    }

    #[test]
    fn default_arm_produces_racket_pose() {
        let arm = sample_three_dof_arm();
        let state = arm.initial_state();
        let pose = state.racket_pose(&arm).expect("FK");
        assert!(pose.position.v.y > arm.base.v.y);
        assert!(pose.position.v.z >= arm.base.v.z);
    }

    #[test]
    fn step_moves_angles_toward_targets() {
        let arm = sample_three_dof_arm();
        let mut state = arm.initial_state();
        state.set_targets(Joints::from_slice(&[0.5, 0.8, -0.2]));
        state.step_toward_targets(&arm, 0.1);
        assert_ne!(state.joints().values[0], 0.0);
    }

    #[test]
    fn rejects_wrong_joint_count_in_fk() {
        let arm = sample_three_dof_arm();
        assert!(
            arm.forward_kinematics(&Joints::from_slice(&[0.0]))
                .is_none()
        );
    }
}
