//! `ArmBuilder::build` 실패 이유.

use std::fmt;

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
