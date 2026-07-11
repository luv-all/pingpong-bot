//! URDF → domain `Arm` (3 revolute 체인 전용).

use pingpong_domain::Arm;

use super::{UrdfLoadError, UrdfRobot};

const SUPPORTED: usize = pingpong_domain::SUPPORTED_FK_JOINTS;

pub fn try_into_arm(urdf: &UrdfRobot, max_joint_speed: f64) -> Result<Arm, UrdfLoadError> {
    if urdf.joint_count() != SUPPORTED {
        return Err(UrdfLoadError::ArmConversion {
            reason: format!(
                "현재 `Arm` FK는 {SUPPORTED}축만 지원합니다 (URDF actuated={})",
                urdf.joint_count()
            ),
        });
    }

    let defaults = urdf.default_joints();
    let limits = urdf.joint_limits();

    let template = Arm::competition().map_err(|e| UrdfLoadError::ArmConversion {
        reason: format!("레일·베이스 템플릿: {e}"),
    })?;
    let rail = template.rail.expect("competition arm은 레일 포함");
    let mut builder =
        Arm::builder()
            .rail(rail)
            .base_xyz(template.base.v.x, template.base.v.y, template.base.v.z);

    for i in 0..SUPPORTED {
        let limit = limits[i];
        let length = urdf.joint_origin_length(i);
        builder =
            builder
                .link(length.max(0.05))
                .revolute_at(limit.min, limit.max, defaults.values[i]);
    }

    return builder
        .max_joint_speed(max_joint_speed)
        .build()
        .map_err(|e| UrdfLoadError::ArmConversion {
            reason: format!("{e}"),
        });
}
