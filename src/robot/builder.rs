//! [`Arm`] 조립용 빌더 - base + serial chain으로 조립한다.

use std::fmt;

use super::{Arm, JointLimit, LinkInertial, SerialChain};
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
    serial_model: Option<(SerialChain, Vec<Option<JointLimit>>, Vec<LinkInertial>, Joints)>,
    /// 합성 강체 관성 (미설정 시 `link_inertials` 그대로 사용 - fixed 하위 링크 미합성)
    aggregated_inertials: Option<Vec<LinkInertial>>,
    /// per-joint 토크 한계 [N*m] (미설정 시 무제한 = `f64::INFINITY`)
    joint_torque_limits: Option<Vec<f64>>,
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
    /// 체인·한계·링크 관성·기본 관절각 개수가 서로 다름
    KinematicsJointCountMismatch {
        chain: usize,
        limits: usize,
        link_inertials: usize,
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
                link_inertials,
                defaults,
            } => {
                return write!(
                    f,
                    "기구학 관절 개수가 다릅니다: chain={chain}, limits={limits}, link_inertials={link_inertials}, defaults={defaults}"
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
        link_inertials: Vec<LinkInertial>,
        default_joints: Joints,
    ) -> Self {
        self.serial_model = Some((chain, limits, link_inertials, default_joints));
        return self;
    }

    /// fixed 하위 링크까지 합성한 per-joint 강체 관성을 설정한다.
    /// 미설정 시 build에서 `link_inertials`(원본 child link만)로 대체한다.
    pub fn aggregated_inertials(mut self, aggregated: Vec<LinkInertial>) -> Self {
        self.aggregated_inertials = Some(aggregated);
        return self;
    }

    /// per-joint 토크 한계 [N*m]를 설정한다. 미설정 시 무제한.
    pub fn joint_torque_limits(mut self, limits: Vec<f64>) -> Self {
        self.joint_torque_limits = Some(limits);
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
        let (chain, limits, link_inertials, default_joints) = self
            .serial_model
            .ok_or(ArmBuildError::MissingSerialChain)?;
        // 미설정이면 합성 관성은 원본 child link, 토크 한계는 무제한으로 둔다
        // (동역학을 안 쓰는 빌더 기반 테스트 arm이 그대로 동작하도록).
        let joint_count = chain.joints.len();
        let aggregated_inertials = self
            .aggregated_inertials
            .unwrap_or_else(|| link_inertials.clone());
        let joint_torque_limits = self
            .joint_torque_limits
            .unwrap_or_else(|| vec![f64::INFINITY; joint_count]);
        return Arm::from_serial_chain(
            base,
            self.rail,
            chain,
            limits,
            link_inertials,
            aggregated_inertials,
            joint_torque_limits,
            default_joints,
            max_joint_speed,
        );
    }
}
