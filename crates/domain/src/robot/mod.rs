//! 로봇 팔 기구학 (plan §7.2–§7.4).
//!
//! `Arm`은 sim·real 공통 **불변** 기하 모델이다. 부팅 시 한 번 만들어
//! `Arc<Arm>`으로 공유하고, FK/IK·스윙 계획은 전부 이 타입만 본다.
//! Rapier·Dynamixel 등은 `infra` 어댑터가 `RacketPose`를 각 SDK 형식으로 변환한다.
//!
//! 설계가 바뀔 때마다 [`ArmBuilder`]로 **base → link → revolute → …** 순서 선언한다.

use std::fmt;

pub mod rail;

use nalgebra::{Matrix3, Vector3};

use self::rail::LinearRail;
use crate::error::SwingPlanError;

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
    /// 베이스 원점 (월드 좌표) [m] — 리니어 레일이 있으면 y·z 마운트 기준, x는 무시
    pub base: Point3<World>,
    /// X축 리니어 레일 (있으면 베이스 x는 `rail_x`로 이동)
    pub rail: Option<LinearRail>,
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
    /// X축 리니어 레일
    rail: Option<LinearRail>,
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
    InvalidLinkLength { link_index: usize, value: f64 },
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
    NonPositiveMaxJointSpeed { value: f64 },
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
            } => {
                return write!(
                    f,
                    "관절 {joint_index} 한계가 뒤집혔습니다: min={min}, max={max}"
                );
            }
            Self::DefaultJointOutOfRange {
                joint_index,
                value,
                min,
                max,
            } => {
                return write!(
                    f,
                    "관절 {joint_index} 기본값 {value:.3} rad 가 허용 범위 [{min:.3}, {max:.3}] 밖"
                );
            }
            Self::UnsupportedKinematics {
                joint_count,
                supported,
            } => {
                return write!(
                    f,
                    "현재 FK는 {supported}축만 지원합니다 (요청: {joint_count}축)"
                );
            }
            Self::NonPositiveMaxJointSpeed { value } => {
                return write!(f, "max_joint_speed는 양수여야 합니다: {value}");
            }
        }
    }
}

impl std::error::Error for ArmBuildError {}

pub use crate::constants::{ARM_POSITION_LINKS, SUPPORTED_FK_JOINTS};

impl ArmBuilder {
    /// 빈 빌더를 만든다.
    pub fn new() -> Self {
        return Self {
            base: None,
            rail: None,
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

    /// X축 리니어 레일을 설정한다.
    pub fn rail(mut self, rail: LinearRail) -> Self {
        self.rail = Some(rail);
        return self;
    }

    /// X축 리니어 레일 파라미터를 빌더에 직접 넣는다.
    pub fn linear_rail(
        self,
        mount_y: f64,
        mount_z: f64,
        x_min: f64,
        x_max: f64,
        max_speed: f64,
    ) -> Self {
        return self.rail(LinearRail {
            mount_y,
            mount_z,
            x_min,
            x_max,
            max_speed,
        });
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
            rail: self.rail,
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

    /// 경진용 4DOF + X축 리니어 — Dynamixel처럼 팔꿈치 접힘 가능.
    ///
    /// `q0` yaw · `q1` 어깨 · `q2` 팔꿈치 · `q3` 손목 open.
    /// 기하·한계는 [`crate::constants::arm`].
    ///
    /// app 배포 SSOT는 `pingpong_app::ROBOTS`이다.
    /// 이 메서는 domain/infra 단위 테스트용 편의 API로 같은 체인을 둔다.
    pub fn competition() -> Result<Self, ArmBuildError> {
        use crate::constants::arm::*;
        use crate::constants::{RACKET_OPEN_PITCH, table};
        return Self::builder()
            .base_xyz(0.0, BASE_Y, table::SURFACE_Z)
            .linear_rail(
                BASE_Y,
                table::SURFACE_Z,
                0.0,
                table::WIDTH_X,
                RAIL_MAX_SPEED,
            )
            .link(LINK_UPPER)
            .revolute_at(YAW_MIN, YAW_MAX, YAW_DEFAULT)
            .link(LINK_FOREARM)
            .revolute_at(SHOULDER_MIN, SHOULDER_MAX, SHOULDER_DEFAULT)
            .link(LINK_WRIST_STUB)
            .revolute_at(ELBOW_MIN, ELBOW_MAX, ELBOW_DEFAULT)
            .link(LINK_WRIST_STUB)
            .revolute_at(WRIST_MIN, WRIST_MAX, RACKET_OPEN_PITCH)
            .max_joint_speed(MAX_JOINT_SPEED)
            .build();
    }

    fn planar_link_lengths(&self) -> (f64, f64) {
        return (self.link_lengths[0], self.link_lengths[1]);
    }

    fn arm_length(&self) -> f64 {
        let (l1, l2) = self.planar_link_lengths();
        return l1 + l2;
    }

    /// revolute 축(관절) 개수.
    pub fn joint_count(&self) -> usize {
        return self.limits.len();
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
            .all(|(limit, &angle)| limit.contains(angle));
    }

    /// 순기구학 — 관절각 → 라켓 끝점·면 방향 (plan §7.2).
    ///
    /// 4축: yaw + 2R(어깨·팔꿈치 접힘) + 손목 open.
    pub fn forward_kinematics(&self, joints: &Joints) -> Option<RacketPose> {
        return self.forward_kinematics_at(self.base, joints);
    }

    /// `rail_x`가 주어진 레일 위치에서 FK.
    pub fn forward_kinematics_with_rail(&self, rail_x: f64, joints: &Joints) -> Option<RacketPose> {
        let mount = self.mount_at_rail(rail_x);
        return self.forward_kinematics_at(mount, joints);
    }

    /// 주어진 마운트 원점에서 FK.
    pub fn forward_kinematics_at(
        &self,
        mount: Point3<World>,
        joints: &Joints,
    ) -> Option<RacketPose> {
        if joints.values.len() != SUPPORTED_FK_JOINTS
            || self.link_lengths.len() < ARM_POSITION_LINKS
        {
            return None;
        }
        let yaw = joints.values[0];
        let a1 = joints.values[1];
        let a2 = joints.values[2];
        let wrist_open = joints.values[3];
        let elbow = a1 + a2;
        let (l1, l2) = self.planar_link_lengths();

        let planar_reach = l1 * a1.cos() + l2 * elbow.cos();
        let planar_height = l1 * a1.sin() + l2 * elbow.sin();

        let offset = Vector3::new(
            planar_reach * yaw.sin(),
            planar_reach * yaw.cos(),
            planar_height,
        );
        let position = Point3::from_vector(mount.v + offset);

        let (normal, orientation) = racket_face_toward_opponent(yaw, wrist_open);

        return Some(RacketPose {
            position,
            normal,
            orientation,
        });
    }

    /// 마운트·팔꿈치·손목(EE) 체인 점 — OBB·뷰어 공용.
    pub fn chain_points(
        &self,
        rail_x: f64,
        joints: &Joints,
    ) -> Option<(Vector3<f64>, Vector3<f64>, Vector3<f64>)> {
        if joints.values.len() != SUPPORTED_FK_JOINTS
            || self.link_lengths.len() < ARM_POSITION_LINKS
        {
            return None;
        }
        let yaw = joints.values[0];
        let a1 = joints.values[1];
        let a2 = joints.values[2];
        let elbow_a = a1 + a2;
        let (l1, l2) = self.planar_link_lengths();
        let mount = self.mount_at_rail(rail_x).v;

        let to_world = |reach: f64, height: f64| -> Vector3<f64> {
            return mount + Vector3::new(reach * yaw.sin(), reach * yaw.cos(), height);
        };

        let base = mount;
        let elbow = to_world(l1 * a1.cos(), l1 * a1.sin());
        let wrist = to_world(
            l1 * a1.cos() + l2 * elbow_a.cos(),
            l1 * a1.sin() + l2 * elbow_a.sin(),
        );
        return Some((base, elbow, wrist));
    }

    /// 손목 open [rad]을 한계 안으로 넣어 새 `Joints`를 만든다.
    pub fn with_wrist_open(&self, joints: &Joints, open: f64) -> Result<Joints, SwingPlanError> {
        if joints.values.len() != SUPPORTED_FK_JOINTS {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
            });
        }
        let limit = self.limits[3];
        let clamped = open.clamp(limit.min, limit.max);
        let mut values = joints.values.clone();
        values[3] = clamped;
        return Ok(Joints { values });
    }

    /// 리턴 속도 방향에 맞춘 손목 open [rad] (수평·수직 성분).
    pub fn wrist_open_for_return(v_out: Vector3<f64>) -> f64 {
        let horizontal = (v_out.x * v_out.x + v_out.y * v_out.y).sqrt().max(1e-6);
        return v_out.z.atan2(horizontal);
    }

    /// 역기구학 — 라켓 끝을 `target`에 두는 관절각 (plan §7.2).
    pub fn inverse_kinematics(&self, target: Point3<World>) -> Result<Joints, SwingPlanError> {
        return self.inverse_kinematics_near(target, None);
    }

    /// `hint`에 가까운 IK 해를 고른다 (스윙 연속성용).
    pub fn inverse_kinematics_near(
        &self,
        target: Point3<World>,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
        return self.inverse_kinematics_at_mount(self.base, target, hint);
    }

    /// 레일 x에서 IK — X는 레일이 맡고 팔은 Y·Z 평면.
    pub fn inverse_kinematics_with_rail(
        &self,
        rail: &LinearRail,
        rail_x: f64,
        target: Point3<World>,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
        return self.inverse_kinematics_at_mount(rail.mount_point(rail_x), target, hint);
    }

    fn inverse_kinematics_at_mount(
        &self,
        mount: Point3<World>,
        target: Point3<World>,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
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
        let wrist = wrist.clamp(self.limits[3].min, self.limits[3].max);

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
                if !limit.contains(angle) {
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

    /// 레일 x에서의 마운트 원점.
    pub fn mount_at_rail(&self, rail_x: f64) -> Point3<World> {
        if let Some(rail) = &self.rail {
            return rail.mount_point(rail_x);
        }
        return self.base;
    }

    /// 리니어 레일 + 팔 도달 범위로 임팩트점을 보정한다.
    ///
    /// 가능하면 **hit-plane y를 유지**하고 xz(레일·높이)만 줄인다.
    /// 구면 투영만 하면 y가 로봇 쪽으로 당겨져 타이밍·접촉이 어긋난다.
    pub fn clamp_impact_for_rail(
        &self,
        rail: &LinearRail,
        target: Point3<World>,
    ) -> (f64, Point3<World>) {
        let rail_x = rail.clamp_x(target.v.x);
        let mount = rail.mount_point(rail_x);
        return (
            rail_x,
            Self::clamp_preserving_y(mount, target, self.arm_length()),
        );
    }

    /// 월드 목표를 팔 도달 반경 안으로 당긴다 (고정 베이스·레일 없을 때).
    pub fn clamp_to_reach(&self, target: Point3<World>) -> Point3<World> {
        return Self::clamp_preserving_y(self.base, target, self.arm_length());
    }

    /// `y`(접수 깊이)를 우선 보존하며 도달 구 안으로 투영한다.
    fn clamp_preserving_y(
        mount: Point3<World>,
        target: Point3<World>,
        arm_length: f64,
    ) -> Point3<World> {
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

        // y 자체만으로도 도달 불능 — 구면 투영 폴백
        return Point3::from_vector(mount.v + rel * (max_reach / distance));
    }

    /// 라켓 위치에 대한 3×3 자코비안 `∂p/∂q` (yaw·a1·a2, plan §7.3).
    pub fn position_jacobian(&self, joints: &Joints) -> Option<Matrix3<f64>> {
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

        let planar_reach = l1 * a1.cos() + l2 * elbow.cos();

        let dyaw = Vector3::new(planar_reach * yaw.cos(), -planar_reach * yaw.sin(), 0.0);
        let da1 = Vector3::new(yaw.sin() * dreach_da1, yaw.cos() * dreach_da1, dheight_da1);
        let da2 = Vector3::new(yaw.sin() * dreach_da2, yaw.cos() * dreach_da2, dheight_da2);

        return Some(Matrix3::from_columns(&[dyaw, da1, da2]));
    }

    /// 엔드이펙터 선속도 → 관절 각속도 (`q̇ = J⁻¹ v`).
    /// 손목 각속도는 0 (선속도에 기여 없음).
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
        let det = j.determinant();
        if det.abs() < 1e-8 {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: ee_velocity.x,
                target_y: ee_velocity.y,
                target_z: ee_velocity.z,
            });
        }
        let q_dot = j.try_inverse().expect("invertible jacobian") * ee_velocity;
        return Ok(vec![q_dot.x, q_dot.y, q_dot.z, 0.0]);
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

/// 런타임 관절 상태 — sim `RobotState`·real encoder 읽기가 같은 타입을 채운다.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotState {
    /// 리니어 레일 x [m]
    rail_x: f64,
    /// 리니어 목표 x [m]
    rail_target: f64,
    /// 현재 관절각
    angles: Joints,
    /// 추종 목표 관절각 (궤적 없을 때)
    targets: Joints,
    /// quintic 스윙 재생
    active_swing: Option<SwingPlayback>,
}

#[derive(Debug, Clone, PartialEq)]
struct SwingPlayback {
    trajectory: crate::types::SwingTrajectory,
    elapsed: f64,
}

impl RobotState {
    /// 초기 관절각·레일 x로 상태를 만든다.
    pub fn new(initial: Joints, rail_x: f64) -> Self {
        return Self {
            rail_x,
            rail_target: rail_x,
            targets: initial.clone(),
            angles: initial,
            active_swing: None,
        };
    }

    /// 리니어 레일 x [m].
    pub fn rail_x(&self) -> f64 {
        return self.rail_x;
    }

    /// 스윙 궤적 재생 중인지.
    pub fn is_swinging(&self) -> bool {
        return self.active_swing.is_some();
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

    /// quintic 스윙 궤적을 시작한다 (이미 스윙 중이면 무시).
    pub fn begin_swing(&mut self, trajectory: crate::types::SwingTrajectory) {
        if self.active_swing.is_some() {
            return;
        }
        self.replace_swing(trajectory);
    }

    /// 스윙을 현재 포즈 기준 새 궤적으로 교체한다 (elapsed=0).
    pub fn replace_swing(&mut self, trajectory: crate::types::SwingTrajectory) {
        self.targets = trajectory.end.clone();
        self.rail_target = trajectory.rail.end;
        self.active_swing = Some(SwingPlayback {
            trajectory,
            elapsed: 0.0,
        });
    }

    /// 진행 중 스윙을 취소한다 (다음 공 발사 전).
    pub fn cancel_swing(&mut self) {
        self.active_swing = None;
    }

    /// 스윙 궤적에서 목표 관절각만 설정한다 (레거시·폴백).
    pub fn set_targets_from_trajectory(&mut self, trajectory: &crate::types::SwingTrajectory) {
        self.begin_swing(trajectory.clone());
    }

    /// quintic 궤적을 `dt`만큼 진행한다. 완료 시 `true`.
    ///
    /// 샘플 직후 테이블 OBB 클램프로 관통 자세를 올린다.
    pub fn advance_swing(&mut self, arm: &Arm, dt: f64) -> bool {
        let Some(playback) = &mut self.active_swing else {
            return false;
        };
        playback.elapsed += dt;
        let t = playback.elapsed.min(playback.trajectory.duration_secs);
        let sampled = playback.trajectory.sample_at(t);
        self.rail_x = playback.trajectory.sample_rail_at(t);
        self.angles = crate::planner::collision::clamp_above_table(arm, self.rail_x, &sampled);
        if playback.elapsed >= playback.trajectory.duration_secs {
            self.active_swing = None;
            return true;
        }
        return false;
    }

    /// 목표 관절각을 `max_speed` [rad/s]로 추종한다 (궤적 없을 때 폴백).
    pub fn step_toward_targets(&mut self, arm: &Arm, dt: f64) {
        if self.active_swing.is_some() {
            let _ = self.advance_swing(arm, dt);
            return;
        }
        if let Some(rail) = &arm.rail {
            let diff = self.rail_target - self.rail_x;
            let step = (rail.max_speed * dt).min(diff.abs());
            self.rail_x += diff.signum() * step;
        }
        let n = self.angles.values.len().min(self.targets.values.len());
        for i in 0..n {
            let diff = self.targets.values[i] - self.angles.values[i];
            let step = (arm.max_joint_speed * dt).min(diff.abs());
            self.angles.values[i] += diff.signum() * step;
        }
        self.angles = crate::planner::collision::clamp_above_table(arm, self.rail_x, &self.angles);
    }

    /// 현재 관절각으로 FK 라켓 자세를 계산한다.
    pub fn racket_pose(&self, arm: &Arm) -> Option<RacketPose> {
        if arm.rail.is_some() {
            return arm.forward_kinematics_with_rail(self.rail_x, &self.angles);
        }
        return arm.forward_kinematics(&self.angles);
    }
}

/// 라켓 면 법선·자세 — 상대(yaw 방향)를 보고 `open`만큼 연다.
///
/// sim 콜라이더/뷰어 큐브는 local +Z가 얇은 축(면 법선)이다.
fn racket_face_toward_opponent(yaw: f64, open: f64) -> (Vector3<f64>, [f64; 4]) {
    let cy = yaw.cos();
    let sy = yaw.sin();
    let cp = open.cos();
    let sp = open.sin();
    // yaw=0 → +Y(슈터/상대), open → +Z 성분
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

/// 열 (local X,Y,Z) → 월드 기저로 가는 회전의 Hamilton 쿼터니언 (w,x,y,z).
fn rotation_matrix_to_quat(x: Vector3<f64>, y: Vector3<f64>, z: Vector3<f64>) -> [f64; 4] {
    // Shepperd's method on R with columns = local axes in world
    let m00 = x.x;
    let m01 = y.x;
    let m02 = z.x;
    let m10 = x.y;
    let m11 = y.y;
    let m12 = z.y;
    let m20 = x.z;
    let m21 = y.z;
    let m22 = z.z;
    let trace = m00 + m11 + m22;
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        return [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s];
    }
    if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        return [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s];
    }
    if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        return [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s];
    }
    let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
    return [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s];
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::table;
    use crate::error::SwingPlanError;

    fn sample_three_dof_arm() -> Arm {
        return Arm::competition().expect("테스트용 4DOF arm");
    }

    #[test]
    fn wrist_open_tilts_racket_face() {
        let arm = sample_three_dof_arm();
        let flat = arm
            .with_wrist_open(&arm.default_joints, 0.1)
            .expect("wrist");
        let lofted = arm
            .with_wrist_open(&arm.default_joints, 0.9)
            .expect("wrist");
        let n_flat = arm.forward_kinematics(&flat).expect("FK").normal;
        let n_loft = arm.forward_kinematics(&lofted).expect("FK").normal;
        assert!(
            n_loft.z > n_flat.z,
            "open↑ → normal.z↑: flat={n_flat:?} loft={n_loft:?}"
        );
    }

    #[test]
    fn racket_face_points_toward_opponent_not_ceiling() {
        let arm = sample_three_dof_arm();
        let pose = arm.forward_kinematics(&arm.default_joints).expect("FK");
        assert!(
            pose.normal.y > 0.5,
            "면이 상대(+Y)를 봐야 함: normal={:?}",
            pose.normal
        );
        assert!(
            pose.normal.y > pose.normal.z,
            "면이 천장보다 상대 쪽: normal={:?}",
            pose.normal
        );
        // local +Z (얇은 축) ≈ normal
        let [w, x, y, z] = pose.orientation;
        let q = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z));
        let local_z = q * Vector3::new(0.0, 0.0, 1.0);
        assert!((local_z - pose.normal).norm() < 1e-5, "local_z={local_z:?}");
    }

    #[test]
    fn builder_produces_three_dof_arm() {
        let arm = sample_three_dof_arm();
        assert_eq!(arm.joint_count(), 4);
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
        state.set_targets(Joints::from_slice(&[0.5, 0.8, -0.2, 0.45]));
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

    #[test]
    fn inverse_kinematics_round_trips_forward_kinematics() {
        let arm = sample_three_dof_arm();
        let joints = Joints::from_slice(&[0.2, 0.9, -0.3, 0.45]);
        let pose = arm.forward_kinematics(&joints).expect("FK");
        let solved = arm.inverse_kinematics(pose.position).expect("IK");
        let again = arm.forward_kinematics(&solved).expect("FK again");
        assert!((again.position.v - pose.position.v).norm() < 1e-6);
    }

    #[test]
    fn inverse_kinematics_rejects_unreachable_target() {
        let arm = sample_three_dof_arm();
        let err = arm
            .inverse_kinematics(Point3::new(10.0, 10.0, 10.0))
            .unwrap_err();
        assert!(matches!(
            err,
            SwingPlanError::InverseKinematicsNoSolution { .. }
        ));
    }

    #[test]
    fn clamp_impact_preserves_hit_plane_y_when_possible() {
        use crate::constants::{LINK_FOREARM, LINK_UPPER};
        let arm = sample_three_dof_arm();
        let rail = arm.rail.as_ref().expect("competition rail");
        let far = Point3::new(0.76, 0.30, table::SURFACE_Z + 0.25);
        let (rail_x, clamped) = arm.clamp_impact_for_rail(rail, far);
        assert!(
            (clamped.v.y - 0.30).abs() < 1e-9,
            "도달 밖이어도 y는 hit plane 유지: {}",
            clamped.v.y
        );
        let mount = rail.mount_point(rail_x);
        let max_reach = LINK_UPPER + LINK_FOREARM;
        assert!((clamped.v - mount.v).norm() <= max_reach + 1e-6);
    }
}
