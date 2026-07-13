//! 제어 `Joints` → URDF actuated 관절 매핑.

/// 제어 관절각을 URDF actuated 슬롯 길이로 변환한다.
///
/// - `sources[i] = Some(j)` → `urdf_q[i] = control[j]` (없으면 `fill`)
/// - `sources[i] = None` → `urdf_q[i] = fill`
///
/// `sources.len()`이 URDF actuated 수와 같아야 한다 (호출 측에서 검증).
pub fn map_control_joints_to_urdf(
    control: &[f64],
    sources: &[Option<usize>],
    fill: f64,
) -> Vec<f64> {
    return sources
        .iter()
        .map(|src| match src {
            Some(j) => control.get(*j).copied().unwrap_or(fill),
            None => fill,
        })
        .collect();
}

/// 명시 매핑이 있으면 사용하고, 없으면 앞쪽 `urdf_count`개를 truncate/pad.
pub fn map_control_joints_or_truncate(
    control: &[f64],
    urdf_count: usize,
    sources: Option<&[Option<usize>]>,
    fill: f64,
) -> Vec<f64> {
    if let Some(map) = sources {
        debug_assert_eq!(
            map.len(),
            urdf_count,
            "control_to_urdf 길이({}) != URDF actuated({})",
            map.len(),
            urdf_count
        );
        return map_control_joints_to_urdf(control, map, fill);
    }
    return (0..urdf_count)
        .map(|i| control.get(i).copied().unwrap_or(fill))
        .collect();
}

/// 카탈로그 매핑이 URDF·제어 DOF와 맞는지 검사한다.
pub fn validate_control_to_urdf_map(
    sources: &[Option<usize>],
    urdf_joint_count: usize,
    control_joint_count: usize,
) -> Result<(), String> {
    if sources.len() != urdf_joint_count {
        return Err(format!(
            "control_to_urdf 길이 {} != URDF actuated {}",
            sources.len(),
            urdf_joint_count
        ));
    }
    for (slot, src) in sources.iter().enumerate() {
        if let Some(j) = *src {
            if j >= control_joint_count {
                return Err(format!(
                    "control_to_urdf[{slot}] = {j} >= control DOF {control_joint_count}"
                ));
            }
        }
    }
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_4dof_control_to_3_urdf_axes() {
        let control = [0.1, 0.2, 0.3, 0.4];
        let sources = [Some(0), Some(1), Some(2)];
        let q = map_control_joints_to_urdf(&control, &sources, 0.0);
        assert_eq!(q, vec![0.1, 0.2, 0.3]);
    }

    #[test]
    fn none_slot_uses_fill() {
        let control = [1.0, 2.0, 3.0];
        let sources = [Some(0), None, Some(2)];
        let q = map_control_joints_to_urdf(&control, &sources, -9.0);
        assert_eq!(q, vec![1.0, -9.0, 3.0]);
    }

    #[test]
    fn truncate_fallback_pads_and_clips() {
        assert_eq!(
            map_control_joints_or_truncate(&[1.0, 2.0], 3, None, 0.0),
            vec![1.0, 2.0, 0.0]
        );
        assert_eq!(
            map_control_joints_or_truncate(&[1.0, 2.0, 3.0, 4.0], 3, None, 0.0),
            vec![1.0, 2.0, 3.0]
        );
    }

    #[test]
    fn validate_rejects_length_and_oob() {
        assert!(validate_control_to_urdf_map(&[Some(0), Some(1)], 3, 4).is_err());
        assert!(validate_control_to_urdf_map(&[Some(0), Some(1), Some(9)], 3, 4).is_err());
        assert!(validate_control_to_urdf_map(&[Some(0), Some(1), Some(2)], 3, 4).is_ok());
    }
}
