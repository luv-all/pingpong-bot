//! 슈터 설정 → 접수 평면 도달 예측.

use anyhow::{Result, ensure};
use nalgebra::Vector3;
use pingpong_bot::defaults::PhysicsParams;
use pingpong_bot::estimator::{HitPlane, Kinematics, Prediction};
use pingpong_bot::sim::launch;

/// 슈터 설정으로 발사한 공이 `hit_plane_y` 평면에 도달하는 지점·속도.
///
/// 실제 파이프라인과 같은 예측기를 쓴다 — 테이블 바운스를 포함해 적분하고,
/// 네트 미달·테이블 구름·리드 시간 밖이면 실패한다. 그래서 도달점과 입사
/// 속도가 항상 물리적으로 성립하는 짝이 된다.
pub fn predict(settings: &launch::Settings, hit_plane_y: f64) -> Result<Prediction> {
    ensure!(hit_plane_y.is_finite(), "접수 평면 y가 유한해야 합니다");
    let m = settings.muzzle_position();
    let v = settings.launch_velocity();
    let w = settings.launch_angular_velocity();
    return Kinematics::predict_to(
        Vector3::new(f64::from(m.x), f64::from(m.y), f64::from(m.z)),
        Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z)),
        Vector3::new(f64::from(w.x), f64::from(w.y), f64::from(w.z)),
        HitPlane { y: hit_plane_y },
        &PhysicsParams::default(),
    )
    .ok_or_else(|| {
        anyhow::anyhow!(
            "이 슈터 설정으로는 y={hit_plane_y:.3} m 평면에 도달하는 공이 없습니다 \
             (네트 미달 · 너무 낮음 · 리드 시간 밖)"
        )
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::constants::table;

    #[test]
    fn default_shooter_reaches_default_hit_plane() {
        let pred = predict(&launch::Settings::default(), table::DEFAULT_HIT_PLANE_Y)
            .expect("기본 슈터는 접수 평면에 도달해야 한다");
        assert!(pred.time_to_impact_secs > 0.0);
        assert!(pred.incoming_velocity.y < 0.0, "로봇 쪽으로 와야 한다");
        assert!(
            pred.impact_position.coords.z > table::SURFACE_Z,
            "테이블 면 위여야 한다: {}",
            pred.impact_position.coords.z
        );
    }

    #[test]
    fn low_flat_shot_is_unreachable() {
        let settings = launch::Settings {
            pitch_deg: 0.0,
            height_offset_m: -0.35,
            speed_mps: 12.0,
            ..Default::default()
        };
        let err = predict(&settings, table::DEFAULT_HIT_PLANE_Y).unwrap_err();
        assert!(
            format!("{err:#}").contains("도달"),
            "사유가 사람이 읽을 수 있어야 함: {err:#}"
        );
    }

    #[test]
    fn non_finite_plane_is_rejected() {
        let err = predict(&launch::Settings::default(), f64::NAN).unwrap_err();
        assert!(format!("{err:#}").contains("접수 평면"), "{err:#}");
    }
}
