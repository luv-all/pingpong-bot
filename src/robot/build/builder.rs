//! [`Arm`] 조립용 빌더 - base + serial chain으로 조립한다.

use std::fmt;

use crate::robot::{Arm, JointLimit, SerialChain};
use crate::robot::rail::LinearRail;
use crate::{Joints, Point3};

/// [`Arm`] 조립용 빌더 - base 설정 후 `.serial_chain`으로 기구학을 채운다.
#[derive(Debug, Clone, Default)]
pub struct ArmBuilder {
    /// 베이스 위치 (미설정 시 build 실패)
    base: Option<Point3>,
    /// X축 리니어 레일
    rail: Option<LinearRail>,
    /// 최대 관절 속도 (미설정 시 2.5 rad/s)
    max_joint_speed: Option<f64>,
    /// arbitrary-axis 직렬 체인 (미설정 시 build 실패)
    serial_model: Option<(SerialChain, Vec<Option<JointLimit>>, Joints)>,
}

/// `ArmBuilder::build` 실패 이유.
#[derive(Debug, Clone, PartialEq)]
pub enum ArmBuildError {
    /// base 미설정
    MissingBase,
    /// `.serial_chain` 미설정
    MissingSerialChain,
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
    /// 체인·한계·기본 관절각 개수가 서로 다름
    KinematicsJointCountMismatch {
        chain: usize,
        limits: usize,
        defaults: usize,
    },
    /// max_joint_speed <= 0
    NonPositiveMaxJointSpeed { value: f64 },
}

impl fmt::Display for ArmBuildError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingBase => return write!(f, "베이스 위치(base)가 설정되지 않았습니다"),
            Self::MissingSerialChain => {
                return write!(f, "직렬 체인(.serial_chain)이 설정되지 않았습니다");
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
            Self::KinematicsJointCountMismatch {
                chain,
                limits,
                defaults,
            } => {
                return write!(
                    f,
                    "기구학 관절 개수가 다릅니다: chain={chain}, limits={limits}, defaults={defaults}"
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
        return Self::default();
    }

    /// 베이스 위치를 설정한다.
    pub fn base(mut self, base: Point3) -> Self {
        self.base = Some(base);
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

    /// 최대 관절 각속도를 설정한다.
    pub fn max_joint_speed(mut self, rad_per_sec: f64) -> Self {
        self.max_joint_speed = Some(rad_per_sec);
        return self;
    }

    /// 이미 검증된 일반 직렬 체인을 이 빌더의 기구학으로 사용한다.
    pub fn serial_chain(
        mut self,
        chain: SerialChain,
        limits: Vec<Option<JointLimit>>,
        default_joints: Joints,
    ) -> Self {
        self.serial_model = Some((chain, limits, default_joints));
        return self;
    }

    /// 검증 후 `Arm`을 만든다.
    pub fn build(self) -> Result<Arm, ArmBuildError> {
        let base = self.base.ok_or(ArmBuildError::MissingBase)?;
        let max_joint_speed = self.max_joint_speed.unwrap_or(2.5);
        if max_joint_speed <= 0.0 {
            return Err(ArmBuildError::NonPositiveMaxJointSpeed {
                value: max_joint_speed,
            });
        }
        let (chain, limits, default_joints) = self
            .serial_model
            .ok_or(ArmBuildError::MissingSerialChain)?;
        return Arm::from_serial_chain(base, self.rail, chain, limits, default_joints, max_joint_speed);
    }
}
