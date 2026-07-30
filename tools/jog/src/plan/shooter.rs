//! 슈터 설정 → (시뮬과 같은) 커밋 시점 예측 묶음.

use anyhow::{Result, ensure};
use nalgebra::Vector3;
use pingpong_bot::defaults::{EstimatorParams, PhysicsParams};
use pingpong_bot::estimator::{Kinematics, Prediction};
use pingpong_bot::robot::motion::{InterceptWindow, physics as motion_physics};
use pingpong_bot::sim::launch;

/// 커밋 게이트까지 굴리는 적분 스텝 상한 (`integrate_dt` 기준 ≈ 4 s).
const MAX_ROLL_STEPS: usize = 4_000;

/// 슈터가 쏜 공을 **시뮬이 스윙을 커밋하는 시점**까지 굴린 뒤, 접수 창의 모든
/// hit plane에 대한 예측을 만든다.
///
/// 시뮬은 접수 평면을 사람이 고르지 않는다 — `InterceptWindow`의 평면들을 전부
/// 예측해 넘기고, `plan_best_swing`이 그중 가장 좋은 타점을 고른다. jog도 같은
/// 입력을 만들어 같은 planner에 넣는다.
///
/// 발사구에서 바로 예측하면 리드 시간이 0.5 s쯤이라 커밋 창
/// (`in_swing_commit_window`) 밖이어서 planner가 후보를 전부 버린다. 그래서
/// 시뮬과 같은 게이트(`ball_past_midcourt_for_commit`)까지 먼저 굴린다.
pub fn commit_predictions(settings: &launch::Settings) -> Result<Vec<Prediction>> {
    let physics = PhysicsParams::default();
    let est = EstimatorParams::default();

    let m = settings.muzzle_position();
    let v = settings.launch_velocity();
    let w = settings.launch_angular_velocity();
    let mut position = Vector3::new(f64::from(m.x), f64::from(m.y), f64::from(m.z));
    let mut velocity = Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z));
    let mut omega = Vector3::new(f64::from(w.x), f64::from(w.y), f64::from(w.z));

    let mut steps = 0;
    while !motion_physics::ball_past_midcourt_for_commit(position.y) {
        let (np, nv, nw) = Kinematics::step(position, velocity, omega, est.integrate_dt, &physics);
        position = np;
        velocity = nv;
        omega = nw;
        steps += 1;
        ensure!(
            steps < MAX_ROLL_STEPS,
            "이 슈터 설정으로는 공이 로봇 코트로 넘어오지 않습니다"
        );
    }

    let predictions: Vec<Prediction> = InterceptWindow::default()
        .hit_planes()
        .into_iter()
        .filter_map(|plane| Kinematics::predict_to(position, velocity, omega, plane, &physics))
        .collect();
    ensure!(
        !predictions.is_empty(),
        "이 슈터 설정으로는 접수 창에 도달하는 공이 없습니다 \
         (네트 미달 · 너무 낮음 · 리드 시간 밖)"
    );
    return Ok(predictions);
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::constants::table;

    #[test]
    fn default_shooter_yields_commit_predictions() {
        let preds = commit_predictions(&launch::Settings::default())
            .expect("기본 슈터는 접수 창에 도달해야 한다");
        assert!(!preds.is_empty());
        for p in &preds {
            assert!(p.incoming_velocity.y < 0.0, "로봇 쪽으로 와야 한다");
            assert!(
                p.impact_position.coords.z > table::SURFACE_Z,
                "테이블 면 위여야 한다: {}",
                p.impact_position.coords.z
            );
        }
    }

    /// 커밋 시점까지 굴렸으므로 리드 시간이 시뮬 커밋 창 안이어야 한다.
    #[test]
    fn commit_predictions_are_inside_the_commit_window() {
        let preds = commit_predictions(&launch::Settings::default()).expect("예측");
        assert!(
            preds
                .iter()
                .any(|p| motion_physics::in_swing_commit_window(p.time_to_impact_secs)),
            "커밋 창 안 후보가 하나도 없다: {:?}",
            preds
                .iter()
                .map(|p| p.time_to_impact_secs)
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn low_flat_shot_never_reaches_the_robot_court() {
        let settings = launch::Settings {
            pitch_deg: 0.0,
            height_offset_m: -0.35,
            speed_mps: 12.0,
            ..Default::default()
        };
        let err = commit_predictions(&settings).unwrap_err();
        assert!(format!("{err:#}").contains("도달") || format!("{err:#}").contains("넘어오지"));
    }
}
