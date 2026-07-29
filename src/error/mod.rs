//! 도메인 전용 에러 타입.
//!
//! variant마다 왜 실패했는지 필드로 담아 로그/디버깅에 바로 쓸 수 있게 한다.

mod domain;
mod hw;
mod observation;
mod swing_plan;

pub use domain::DomainError;
pub use hw::HwError;
pub use observation::ObservationError;
pub use swing_plan::SwingPlanError;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hard_unreachable_skips_only_insufficient_time() {
        assert!(
            SwingPlanError::InverseKinematicsNoSolution {
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
            }
            .is_hard_unreachable()
        );
        assert!(
            SwingPlanError::ReturnVelocityUnreachable {
                incoming_velocity: [0.0; 3],
                outgoing_velocity: [0.0; 3],
            }
            .is_hard_unreachable()
        );
        assert!(
            !SwingPlanError::InsufficientTime {
                time_to_impact_secs: 0.05,
                min_swing_secs: 0.1,
            }
            .is_hard_unreachable()
        );
        assert!(
            SwingPlanError::TablePenetration {
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
                depth: 0.01,
            }
            .is_hard_unreachable()
        );
        assert!(
            SwingPlanError::JointOrTorqueLimit {
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
            }
            .is_hard_unreachable()
        );
    }

    #[test]
    fn hw_error_display_includes_reason() {
        let err = HwError::ReadFailed {
            reason: "Present Position sync_read 실패".into(),
        };
        assert_eq!(
            err.to_string(),
            "하드웨어 상태 읽기 실패: Present Position sync_read 실패"
        );
    }

    #[test]
    fn domain_error_source_chains_to_swing_plan() {
        use std::error::Error as _;
        let inner = SwingPlanError::InsufficientTime {
            time_to_impact_secs: 0.05,
            min_swing_secs: 0.1,
        };
        let err = DomainError::InfeasibleSwing(inner.clone());
        assert_eq!(
            err.source().map(ToString::to_string),
            Some(inner.to_string())
        );
    }
}
