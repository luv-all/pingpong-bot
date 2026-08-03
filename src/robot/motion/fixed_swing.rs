//! 사전 정의된 스윙 딕셔너리 — IK 없이 시작/끝 관절각만으로 스윙한다.
//!
//! `Planner::plan_best_swing`/`robot::control::PositionController`와 달리 라켓
//! 위치·자세를 IK로 풀지 않는다. 스윙 "모양"은 [`FIXED_SWING_START_DEG`] →
//! [`FIXED_SWING_END_DEG`]로 고정이고, 호출부는 레일 x(기하만, IK 없음)와
//! 스윙을 시작할 타이밍만 정한다.

use crate::defaults;
use crate::error::{DomainError, SwingPlanError};
use crate::robot::{Arm, Joints, LinearRail, Pose};

use super::rail::Rail;
use super::{Planner, Trajectory};

/// 스윙 시작(백스윙/준비) 자세 [deg] — j0 yaw, j1 shoulder, j2 elbow, j3 wrist.
pub const FIXED_SWING_START_DEG: [f64; 4] = [-10.0, 0.0, 50.0, -30.0];
/// 스윙 끝(임팩트) 자세 [deg] — 관절 순서는 시작과 동일.
pub const FIXED_SWING_END_DEG: [f64; 4] = [40.0, 0.0, -12.0, -70.0];

pub fn fixed_swing_start_joints() -> Joints {
    return Joints::from_slice(&FIXED_SWING_START_DEG.map(f64::to_radians));
}

pub fn fixed_swing_end_joints() -> Joints {
    return Joints::from_slice(&FIXED_SWING_END_DEG.map(f64::to_radians));
}

/// 고정 스윙의 관절 타이밍 모양 — 사용자가 GUI에서 실시간 비교할 두 선택지.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwingShapeStrategy {
    /// 4관절이 같은 시간축을 공유하는 단일 quintic(기존 `move_to_fastest`).
    /// 라켓 방향은 이미 정확하지만(실측 96-100%가 전진 방향), 근위·원위
    /// 관절이 동시에 각자의 한계까지만 움직여 라켓 최고 속력이 낮다
    /// (2026-08-03 실측: 0.879 m/s).
    Synchronized,
    /// 근위(j0/j1)가 먼저 움직이기 시작해 멈추고, 원위(j2/j3)가 그 뒤에
    /// 시작해 임팩트 순간 "스냅"되는 채찍형 — 각 관절은 여전히 정지→정지
    /// quintic이지만 구간이 서로 어긋나 겹친다(채찍/골프 스윙의 kinetic
    /// chain과 같은 원리).
    Staggered,
}

/// 사용자가 GUI에서 두 전략을 비교하는 동안의 기본값 — 고친 쪽(Staggered)을
/// 기본으로 둔다.
pub const DEFAULT_SWING_SHAPE_STRATEGY: SwingShapeStrategy = SwingShapeStrategy::Staggered;

/// 근위→원위 순서로 어긋난 구간 — 궤적 전체 시간에 대한 분수 `(시작, 끝)`.
/// j0/j1이 먼저 시작해 먼저 끝나고, j2가 그 위에 걸쳐 움직이다가, j3가
/// 가장 늦게 시작해 임팩트 순간 스냅한다. 실측으로 조정 가능한 시작값
/// (`fixed_swing_impact_time_secs`처럼 이후 실측 기반 재조정 여지가 있다).
const STAGGERED_PHASE_FRACTIONS: [(f64, f64); 4] = [
    (0.0, 0.55),  // j0 yaw
    (0.0, 0.65),  // j1 shoulder
    (0.25, 0.85), // j2 elbow (0.25~0.85)
    (0.30, 1.00), // j3 wrist (0.30~1.00) — 실측 스윕(2026-08-03)으로 고른 값,
                  // j3가 이보다 늦게 시작하면(예: 0.45) 임팩트까지 남은 자기
                  // 구간이 너무 짧아 최고 속력이 오히려 낮아진다.
];

/// 레일 `rail_x`에 고정한 채, IK 없이 시작→끝 관절각을 모터 한계(속도·가속·
/// 토크) 100%로 잇는 quintic — `shape`로 관절 타이밍을 고른다.
pub fn plan_fixed_swing(
    arm: &Arm,
    rail_x: f64,
    shape: SwingShapeStrategy,
) -> Result<Trajectory, DomainError> {
    return match shape {
        SwingShapeStrategy::Synchronized => {
            let start = Pose::new(rail_x, fixed_swing_start_joints());
            Planner::move_to_fastest(arm, &start, fixed_swing_end_joints(), rail_x)
        }
        SwingShapeStrategy::Staggered => plan_staggered_fixed_swing(arm, rail_x),
    };
}

/// [`SwingShapeStrategy::Staggered`] 빌더 — 동기화형(`move_to_fastest`)이 낸
/// 실현가능 소요 시간을 기준선으로 잡고, 그 위에 [`STAGGERED_PHASE_FRACTIONS`]
/// 비율로 관절별 구간을 어긋나게 둔 뒤, 이 모양 자체의 속도·토크 실현가능성을
/// (동기화형과 별개로) 확인한다 — 같은 각도 변화를 더 짧은 자기 구간에
/// 눌러넣으므로 관절별 각속도가 동기화형보다 커져, 기준선이 실현 가능했다고
/// 이 모양도 자동으로 실현 가능한 건 아니다. 안 되면 기준 소요 시간을 늘려
/// 재시도한다(`move_to_fastest`/`plan_return_to_center`와 같은 성장 탐색 정신).
fn plan_staggered_fixed_swing(arm: &Arm, rail_x: f64) -> Result<Trajectory, DomainError> {
    let baseline = {
        let start = Pose::new(rail_x, fixed_swing_start_joints());
        Planner::move_to_fastest(arm, &start, fixed_swing_end_joints(), rail_x)?
    };
    let start_joints = fixed_swing_start_joints();
    let end_joints = fixed_swing_end_joints();
    let n = start_joints.values.len();

    let mut duration = baseline.duration_secs;
    const MAX_DURATION_SECS: f64 = 3.0;
    const GROWTH: f64 = 1.2;
    let mut last_error: Option<DomainError> = None;
    while duration <= MAX_DURATION_SECS {
        let offsets: Vec<(f64, f64)> = STAGGERED_PHASE_FRACTIONS
            .iter()
            .take(n)
            .map(|(start_fraction, end_fraction)| {
                let offset = start_fraction * duration;
                (offset, (end_fraction - start_fraction) * duration)
            })
            .collect();
        let candidate = Trajectory::new(
            start_joints.clone(),
            end_joints.clone(),
            vec![0.0; n],
            vec![0.0; n],
            duration,
            Rail::fixed(rail_x),
        )
        .with_phase_offsets(offsets);

        match staggered_feasibility(arm, &candidate) {
            Ok(()) => return Ok(candidate),
            Err(error) => {
                last_error = Some(error);
                duration *= GROWTH;
            }
        }
    }
    return Err(last_error.unwrap_or(DomainError::InfeasibleSwing(
        SwingPlanError::InverseKinematicsNoSolution {
            target_x: rail_x,
            target_y: 0.0,
            target_z: 0.0,
        },
    )));
}

/// `candidate`가 관절 속도·토크 한계 안인지 직접 샘플링으로 확인한다 —
/// `physics.rs`의 `peak_torque_utilization`/`kinematic_limit_violation`은
/// 관절마다 **같은** 로컬 시간을 공유한다고 가정해 위상이 어긋난 이
/// 궤적에는 안 맞는다(모듈 문서 참고). `Trajectory::sample_at` 계열은
/// (Task 5b에서) 위상 오프셋을 이미 반영하므로, 이 함수는 그것만으로 검사한다.
fn staggered_feasibility(arm: &Arm, candidate: &Trajectory) -> Result<(), DomainError> {
    if candidate.peak_joint_speed() > arm.max_joint_speed {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::TrajectoryExceedsLimits {
                rail_end_x: candidate.rail.end,
                violated: "관절 속도",
            },
        ));
    }
    if arm.joint_torque_limits.iter().all(|limit| !limit.is_finite()) {
        return Ok(());
    }
    const SAMPLES: usize = 40;
    let mut worst = 0.0_f64;
    for index in 0..=SAMPLES {
        let t = candidate.duration_secs * index as f64 / SAMPLES as f64;
        let q = candidate.sample_at(t);
        let qd = candidate.sample_velocity_at(t);
        let qdd = candidate.sample_acceleration_at(t);
        let Some(torques) = arm.required_torque_with_rotor(&q.values, &qd, &qdd) else {
            continue;
        };
        for (torque, &limit) in torques.iter().zip(arm.joint_torque_limits.iter()) {
            if limit.is_finite() && limit > 0.0 {
                worst = worst.max(torque.abs() / limit);
            }
        }
    }
    if worst > 1.0 {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::TrajectoryExceedsTorque {
                rail_end_x: candidate.rail.end,
                utilization: worst,
            },
        ));
    }
    return Ok(());
}

/// 예측 임팩트 x를 레일 사거리 안으로 자른다 — IK 없이 기하만으로 리니어
/// 목표를 정한다.
pub fn fixed_swing_rail_target(rail: &LinearRail, predicted_impact_x: f64) -> f64 {
    return rail.clamp_x(predicted_impact_x);
}

/// 고정 스윙 내부에서 라켓이 공과 만난다고 가정하는 시각을 고르는 전략.
///
/// 라켓은 START→END를 실시간으로 스윕하므로, 공은 그 스윕 **도중** 만나야
/// 한다 — 스윙이 끝나는 순간(= END 자세 도달)을 임팩트로 보면 안 된다
/// (2026-08-03 실측 회귀: 기본 슈터 샷에서 스윙 전체 소요 시간이 로봇
/// 접수창의 어떤 평면에 대해서도 남은 비행 시간보다 길어, "지금 시작해도
/// 이미 늦음"이 발사 첫 틱부터 참이었다).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImpactTimeStrategy {
    /// 스윙 소요 시간의 정확히 절반.
    Midpoint,
    /// 라켓 중심의 순간 속력(FK 유한차분)이 최대가 되는 시각.
    PeakRacketSpeed,
}

/// 두 전략을 비교 중이라 기본값은 더 단순하고 예측 가능한 쪽으로 둔다.
pub const DEFAULT_IMPACT_TIME_STRATEGY: ImpactTimeStrategy = ImpactTimeStrategy::Midpoint;

/// `strategy`에 따라 고정 스윙 내부의 가정 임팩트 시각 [s]을 고른다
/// (`0 < 반환값 < trajectory.duration_secs`).
///
/// 레일은 스윙 도중 고정이라(`plan_fixed_swing`이 `rail_x`를 시작=끝으로 둔다)
/// 라켓의 시간에 따른 **속도** 형태는 `rail_x`와 무관하다 — 위치만
/// 평행이동한다. 그래서 이 계산은 궤적당 한 번만 하면 되고, 레일 위치가
/// 바뀌어도 다시 스윕할 필요가 없다(다만 인터페이스는 `rail_x`를 그대로
/// 받아 FK 위치를 실제로 구한다 — 속도만 rail_x 불변이라는 뜻).
pub fn fixed_swing_impact_time_secs(
    arm: &Arm,
    rail_x: f64,
    trajectory: &Trajectory,
    strategy: ImpactTimeStrategy,
) -> f64 {
    return match strategy {
        ImpactTimeStrategy::Midpoint => trajectory.duration_secs * 0.5,
        ImpactTimeStrategy::PeakRacketSpeed => {
            peak_racket_speed_time(arm, rail_x, trajectory)
        }
    };
}

/// 라켓 중심 속력이 최대인 시각을 유한차분으로 찾는다 — 균등 표본
/// 중심차분, 표본 수는 정확도와 계산량의 실용적 절충.
fn peak_racket_speed_time(arm: &Arm, rail_x: f64, trajectory: &Trajectory) -> f64 {
    const SAMPLES: usize = 64;
    let duration = trajectory.duration_secs;
    if duration <= 0.0 {
        return 0.0;
    }
    let step = duration / SAMPLES as f64;
    let position_at = |t: f64| -> Option<nalgebra::Vector3<f64>> {
        return arm
            .forward_kinematics_with_rail(rail_x, &trajectory.sample_at(t))
            .map(|pose| pose.position.coords);
    };
    let mut best_time = duration * 0.5;
    let mut best_speed = -1.0_f64;
    for index in 0..=SAMPLES {
        let t = step * index as f64;
        let before = (t - step * 0.5).max(0.0);
        let after = (t + step * 0.5).min(duration);
        let span = (after - before).max(1e-9);
        if let (Some(p0), Some(p1)) = (position_at(before), position_at(after)) {
            let speed = (p1 - p0).norm() / span;
            if speed > best_speed {
                best_speed = speed;
                best_time = t;
            }
        }
    }
    return best_time;
}

/// 남은 시간이 스윙 **내부의 가정 임팩트 시각**([`fixed_swing_impact_time_secs`]) 이하가
/// 되는 즉시 스윙을 시작해야 한다 — 스윙 전체 소요 시간이 아니다. 스윙은
/// START→END를 실시간으로 스윕하므로, 공은 그 스윕 도중 만나야 한다.
pub fn should_start_fixed_swing(time_to_impact_secs: f64, impact_time_secs: f64) -> bool {
    return time_to_impact_secs.is_finite()
        && time_to_impact_secs > defaults::MIN_TIME_TO_GO_SECS
        && time_to_impact_secs <= impact_time_secs;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dictionary_joints_convert_degrees_to_radians() {
        let start = fixed_swing_start_joints();
        let end = fixed_swing_end_joints();
        for (actual, expected_deg) in start.values.iter().zip(FIXED_SWING_START_DEG) {
            assert!((actual.to_degrees() - expected_deg).abs() < 1e-9);
        }
        for (actual, expected_deg) in end.values.iter().zip(FIXED_SWING_END_DEG) {
            assert!((actual.to_degrees() - expected_deg).abs() < 1e-9);
        }
    }

    #[test]
    fn plan_fixed_swing_starts_and_ends_at_the_dictionary_poses_with_rail_held() {
        let robot = crate::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail");
        let rail_x = rail.default_x();

        let trajectory = plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Synchronized)
            .expect("fixed swing plan");

        for (actual, expected) in trajectory
            .start
            .values
            .iter()
            .zip(fixed_swing_start_joints().values)
        {
            assert!((actual - expected).abs() < 1e-9);
        }
        for (actual, expected) in trajectory
            .goal_joints()
            .values
            .iter()
            .zip(fixed_swing_end_joints().values)
        {
            assert!((actual - expected).abs() < 1e-9);
        }
        assert!((trajectory.rail.start - rail_x).abs() < 1e-12);
        assert!((trajectory.rail.end - rail_x).abs() < 1e-12);
        assert!(trajectory.duration_secs > 0.0);
    }

    #[test]
    fn fixed_swing_rail_target_clamps_to_rail_range() {
        let robot = crate::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail");

        assert!((fixed_swing_rail_target(&rail, rail.x_min - 1.0) - rail.x_min).abs() < 1e-12);
        assert!((fixed_swing_rail_target(&rail, rail.x_max + 1.0) - rail.x_max).abs() < 1e-12);
        let mid = (rail.x_min + rail.x_max) * 0.5;
        assert!((fixed_swing_rail_target(&rail, mid) - mid).abs() < 1e-12);
    }

    #[test]
    fn should_start_fixed_swing_fires_only_inside_the_duration_window() {
        let swing_duration = 0.30;
        assert!(!should_start_fixed_swing(0.50, swing_duration), "too early");
        assert!(should_start_fixed_swing(0.30, swing_duration), "exactly at duration");
        assert!(should_start_fixed_swing(0.10, swing_duration), "inside window");
        assert!(
            !should_start_fixed_swing(defaults::MIN_TIME_TO_GO_SECS * 0.5, swing_duration),
            "degenerate tti"
        );
        assert!(!should_start_fixed_swing(f64::NAN, swing_duration), "non-finite");
    }

    #[test]
    fn midpoint_strategy_is_exactly_half_duration() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let trajectory = plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Synchronized)
            .expect("fixed swing plan");
        let impact_time = fixed_swing_impact_time_secs(
            &robot.arm,
            rail_x,
            &trajectory,
            ImpactTimeStrategy::Midpoint,
        );
        assert!((impact_time - trajectory.duration_secs * 0.5).abs() < 1e-9);
    }

    #[test]
    fn peak_speed_strategy_picks_a_time_strictly_inside_the_swing() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let trajectory = plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Synchronized)
            .expect("fixed swing plan");
        let impact_time = fixed_swing_impact_time_secs(
            &robot.arm,
            rail_x,
            &trajectory,
            ImpactTimeStrategy::PeakRacketSpeed,
        );
        assert!(impact_time > 0.0, "impact_time={impact_time}");
        assert!(
            impact_time < trajectory.duration_secs,
            "impact_time={impact_time} duration={}",
            trajectory.duration_secs
        );
    }

    #[test]
    fn should_start_fixed_swing_now_gates_on_impact_time_not_full_duration() {
        // 회귀 방지: `duration_secs`(전체 소요)가 아니라 그보다 짧은
        // `impact_time_secs`(스윙 내부 임팩트 시각)를 기준으로 삼아야, 스윙이
        // "끝나는" 시점이 아니라 "공을 맞히는" 시점에 남은 시간을 맞춘다.
        let duration_secs = 0.53;
        let impact_time_secs = duration_secs * 0.5;
        // 남은 시간이 절반(임팩트 시각)보다 크면 아직 시작하면 안 된다 — 예전
        // 로직(`duration_secs` 기준)이었다면 이 값에서 이미 시작했을 것이다.
        assert!(!should_start_fixed_swing(0.45, impact_time_secs));
        assert!(should_start_fixed_swing(impact_time_secs, impact_time_secs));
        assert!(should_start_fixed_swing(0.10, impact_time_secs));
    }

    #[test]
    fn synchronized_shape_matches_the_original_move_to_fastest_behavior() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let via_shape =
            plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Synchronized).expect("sync");
        assert!(via_shape.joint_phase_offsets.is_none());
        for (actual, expected) in via_shape.start.values.iter().zip(fixed_swing_start_joints().values) {
            assert!((actual - expected).abs() < 1e-9);
        }
    }

    #[test]
    fn staggered_shape_sets_distinct_per_joint_windows_and_stays_feasible() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let staggered =
            plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Staggered).expect("staggered");
        let offsets = staggered
            .joint_phase_offsets
            .clone()
            .expect("staggered trajectory must set phase offsets");
        assert_eq!(offsets.len(), 4);
        // 근위(j0/j1)가 원위(j3)보다 먼저 시작해야 한다 — 채찍 순서 확인.
        assert!(offsets[0].0 <= offsets[3].0, "j0 시작 {} > j3 시작 {}", offsets[0].0, offsets[3].0);
        assert!(offsets[1].0 <= offsets[3].0, "j1 시작 {} > j3 시작 {}", offsets[1].0, offsets[3].0);
        // 각 관절 구간은 궤적 전체 시간 안에 들어와야 한다.
        for (index, (offset, duration)) in offsets.iter().enumerate() {
            assert!(*offset >= 0.0, "joint {index} offset={offset}");
            assert!(
                offset + duration <= staggered.duration_secs + 1e-6,
                "joint {index}: {offset}+{duration} > {}",
                staggered.duration_secs
            );
        }
    }

    #[test]
    fn staggered_shape_reaches_a_higher_peak_racket_speed_than_synchronized() {
        // 이 테스트가 곧 이 기능의 존재 이유다: 채찍형이 동기화형보다
        // 라켓 중심 최고 속력을 실제로 더 내야 한다.
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let sync =
            plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Synchronized).expect("sync");
        let staggered =
            plan_fixed_swing(&robot.arm, rail_x, SwingShapeStrategy::Staggered).expect("staggered");

        let peak_speed = |trajectory: &Trajectory| -> f64 {
            const SAMPLES: usize = 80;
            let step = trajectory.duration_secs / SAMPLES as f64;
            let mut best = 0.0_f64;
            for index in 0..=SAMPLES {
                let t = step * index as f64;
                let dt = (step * 0.5).max(1e-6);
                let before = (t - dt).max(0.0);
                let after = (t + dt).min(trajectory.duration_secs);
                let p0 = robot
                    .arm
                    .forward_kinematics_with_rail(rail_x, &trajectory.sample_at(before))
                    .expect("fk")
                    .position
                    .coords;
                let p1 = robot
                    .arm
                    .forward_kinematics_with_rail(rail_x, &trajectory.sample_at(after))
                    .expect("fk")
                    .position
                    .coords;
                let speed = (p1 - p0).norm() / (after - before).max(1e-9);
                best = best.max(speed);
            }
            return best;
        };

        let sync_peak = peak_speed(&sync);
        let staggered_peak = peak_speed(&staggered);
        assert!(
            staggered_peak > sync_peak,
            "채찍형 최고속={staggered_peak} 동기화형 최고속={sync_peak} — 개선이 없음"
        );
    }
}
