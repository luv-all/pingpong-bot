//! [`Arm`] 조립용 빌더 - base 이후 link -> revolute 순서 선언.

use std::fmt;

use super::{Arm, JointLimit};
use crate::robot::rail::LinearRail;
use crate::types::{Joints, Point3};

pub use crate::constants::SUPPORTED_FK_JOINTS;

/// [`Arm`] 조립용 빌더 - base 이후 link -> revolute 를 키네마틱 체인 순서로 선언한다.
#[derive(Debug, Clone)]
pub struct ArmBuilder {
    /// 베이스 위치 (미설정 시 build 실패)
    base: Option<Point3>,
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
    /// link->revolute 쌍이 하나도 없음
    EmptyChain,
    /// 마지막 link 뒤에 revolute 없음
    IncompleteChain,
    /// `.link` 와야 하는데 다른 호출
    ExpectedLink,
    /// `.revolute` 와야 하는데 `.link` 호출
    ExpectedJoint,
    /// 링크 길이 <= 0
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
    /// max_joint_speed <= 0
    NonPositiveMaxJointSpeed { value: f64 },
}

impl fmt::Display for ArmBuildError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingBase => return write!(f, "베이스 위치(base)가 설정되지 않았습니다"),
            Self::EmptyChain => return write!(f, "link->revolute 체인이 비어 있습니다"),
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

    /// 베이스 위치를 설정한다. 이후 `.link` -> `.revolute` 순으로 체인을 선언한다.
    pub fn base(mut self, base: Point3) -> Self {
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

    /// revolute 축 앞의 rigid link [m]. 직후 `.revolute`가 와야 한다.
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
