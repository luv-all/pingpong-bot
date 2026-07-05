//! 배포·실험별 로봇 팔 조립.
//!
//! domain은 `Arm`/`ArmBuilder`만 제공하고, **어떤 스펙으로 돌릴지**는
//! app(또는 bin·TOML)에서 결정한다.

use std::sync::Arc;

use pingpong_domain::{constants::table, Arm, ArmBuildError};

/// GIST 경진용 3DOF 팔.
pub fn competition_arm() -> Result<Arm, ArmBuildError> {
    return Arm::builder()
        .base_xyz(table::WIDTH_X * 0.15, 0.02, table::SURFACE_Z)
        .link(0.35)
        .revolute_at(-1.2, 1.2, 0.0)
        .link(0.30)
        .revolute_at(-0.2, 1.4, 0.6)
        .link(0.15)
        .revolute_at(-1.5, 0.5, -0.4)
        .max_joint_speed(2.5)
        .build();
}

/// 파이프라인·sim 세션에서 공유하는 경진용 `Arm`.
pub fn shared_competition_arm() -> Arc<Arm> {
    return Arc::new(
        competition_arm().expect("경진용 arm 프리셋은 항상 유효해야 합니다"),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn competition_arm_builds() {
        let arm = competition_arm().expect("프리셋");
        assert_eq!(arm.joint_count(), 3);
    }
}
