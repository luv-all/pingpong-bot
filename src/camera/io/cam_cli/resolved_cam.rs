//! resolve된 한 대 (device는 rig에서만).

use crate::camera::io::rig::CamRigConfig;
use crate::camera::{Id, Role};

/// resolve된 한 대 (device는 rig에서만).
#[derive(Debug, Clone, Copy)]
pub struct ResolvedCam {
    pub role: Role,
    pub device: i32,
    pub camera_id: Id,
}

pub(crate) fn resolve_cams(roles: &[Role]) -> Result<Vec<ResolvedCam>, String> {
    if roles.is_empty() {
        return Err("--cam 필수 (left|right) — 단일 캠 툴은 생략 불가".into());
    }
    let rig = CamRigConfig::default();
    let mut out = Vec::with_capacity(roles.len());
    for &role in roles {
        let (device, camera_id) = rig.resolve(role);
        out.push(ResolvedCam {
            role,
            device,
            camera_id,
        });
    }
    return Ok(out);
}
