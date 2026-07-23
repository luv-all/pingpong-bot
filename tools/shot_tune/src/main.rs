//! shot-tune: 슈터 발사 기하(`speed_mps` × `pitch_deg` × `height_offset_m`)를
//! **실제 Rapier 시뮬레이션**으로 채점하는 스윕 도구.
//!
//! `tools/mount_search`는 임팩트 위치/속도를 해석적으로 직접 정의해
//! `swing_feasibility`만 본다 — 실제 슈터가 그런 임팩트를 만들어내는지는
//! 모른다. 2026-07-23 조사에서 이 해석 모델과 실제 Rapier 탄도(발사 →
//! 상대 코트 바운스 → 로봇 쪽 도달)가 만드는 임팩트 조성이 크게 달라
//! 예측 실현가능률이 서로 안 맞는 문제가 확인됐다
//! (`.omc/research/known-regressions-realistic-joint-speed.md` §4).
//!
//! 이 도구는 그 간극을 없앤다: `SimWorld`를 ground-truth 자동 스윙 모드로
//! 그대로 돌려서(= GUI 앱과 완전히 같은 경로: `predict_impact` →
//! `plan_best_swing` → `RobotState` 추종) 실제로 라켓에 맞고 네트를 넘겨
//! 리턴하는지를 센다. 중간 모델 없이 최종 사용자 관점 성공률이 곧 점수다.
//!
//! 사용법 (반드시 저장소 루트에서 — URDF 상대경로가 cwd 기준):
//!   cargo run -p shot-tune --release -- --robot 4-dof
//!   cargo run -p shot-tune --release -- --robot 4-dof --json --top-n 10

use anyhow::{Context, Result, anyhow};
use clap::Parser;
use pingpong_bot::sim::{BallShooterSettings, SimWorld};
use pingpong_bot::{Arm, MountPreset, RobotBuilder, RobotPose, find_robot, robot_ids_csv};
use rand::SeedableRng;
use rand::rngs::StdRng;
use serde::Serialize;
use std::sync::Arc;

/// 한 샷당 최대 물리 스텝 수 (1kHz 기준 4초).
/// `sim::world` 랠리 테스트들이 쓰는 4_000과 같은 예산.
const MAX_STEPS: usize = 4_000;
const DT: f64 = 1.0 / 1000.0;

#[derive(Parser, Debug)]
#[command(
    about = "슈터 발사 기하(speed × pitch × height_offset)를 실제 Rapier 랠리 성공률로 스윕한다"
)]
struct Args {
    /// 카탈로그 로봇 id (`competition` | `urdf-test` | `4-dof`).
    #[arg(long, default_value = "4-dof")]
    robot: String,

    #[arg(long, default_value_t = 5.0)]
    speed_min: f64,
    #[arg(long, default_value_t = 11.0)]
    speed_max: f64,
    #[arg(long, default_value_t = 7)]
    speed_steps: usize,

    #[arg(long, allow_hyphen_values = true, default_value_t = -12.0)]
    pitch_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 4.0)]
    pitch_max: f64,
    #[arg(long, default_value_t = 9)]
    pitch_steps: usize,

    #[arg(long, allow_hyphen_values = true, default_value_t = 0.0)]
    height_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.30)]
    height_max: f64,
    #[arg(long, default_value_t = 7)]
    height_steps: usize,

    /// 로봇 베이스 y [m] 스윕 (URDF 로봇 전용 — `SimRobotMount::
    /// rep103_z_up_at_table_end_with_mount`). 기본값은 코드의 현재 마운트
    /// (`REP103_BASE_Y`/`REP103_HEIGHT_OFFSET_M`)와 같게 맞춰 둔다 — 플래그
    /// 없이 돌리면 "지금 코드가 실제로 하는 동작"이 그대로 측정된다.
    #[arg(long, allow_hyphen_values = true, default_value_t = -0.02)]
    base_y_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = -0.02)]
    base_y_max: f64,
    #[arg(long, default_value_t = 1)]
    base_y_steps: usize,

    /// 로봇 베이스의 탁구대 면 대비 높이 오프셋 [m] 스윕.
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.05)]
    mount_height_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.05)]
    mount_height_max: f64,
    #[arg(long, default_value_t = 1)]
    mount_height_steps: usize,

    /// 후보마다 돌릴 랜덤 샷(좌우 위치·yaw) 개수 — `BallShooterSettings::randomized`
    /// 와 같은 분포. 0이면 정면(lateral=0, yaw=0) 한 발만 본다.
    #[arg(long, default_value_t = 12)]
    shots: usize,

    /// 속도를 스윕값으로 덮어쓰지 않고 `BallShooterSettings::randomized`가 뽑은
    /// 값(= `RANDOM_SHOT_SPEED_MIN/MAX` 상수)을 그대로 쓴다 — 실제 게임처럼
    /// 매 샷 속도가 달라지는 조건에서 최종 기본값을 검증할 때.
    #[arg(long)]
    use_random_speed: bool,

    /// 랜덤 샷 시드 (재현성).
    #[arg(long, default_value_t = 20260723)]
    seed: u64,

    /// 레일 시작 위치를 중앙이 아니라 테이블 중앙(WIDTH_X*0.5)으로 둘지 —
    /// 실제 GUI에서 이전 샷 뒤 로봇이 멈춰 있는 위치 재현.
    #[arg(long)]
    start_from_table_center: bool,

    /// 들어오는 공이 전부 정상 랠리 샷(네트 통과 + 로봇 코트 바운스)인
    /// 후보만 남긴다. 끄면 적법성과 무관하게 `legal` 열로 분포를 관찰만 한다.
    #[arg(long)]
    require_legal: bool,

    /// `success` 대신 `legal` 기준으로 정렬 — 적법한 샷이 존재하는 영역
    /// 자체를 먼저 찾을 때.
    #[arg(long)]
    sort_by_legal: bool,

    /// 스윕 대신 휴지(ready) 자세 탐색을 돌린다 — 대표 임팩트 자세들까지의
    /// 최악 관절공간 이동거리 Δq를 최소화하는 자세를 찾아 출력한다.
    #[arg(long)]
    rest_pose_search: bool,

    /// 스윕 대신 단일 후보 한 발을 돌리며 commit 창의 매 재시도마다
    /// `plan_best_swing`이 실제로 어떤 오류로 실패하는지 출력한다.
    #[arg(long)]
    explain: bool,

    #[arg(long)]
    json: bool,

    #[arg(long, default_value_t = 12)]
    top_n: usize,
}

/// `main.rs::load_robot`와 같은 카탈로그 dispatch. `mount`가 `Some`이면
/// 카탈로그 기본 마운트 대신 그 위치로 URDF 로봇을 올린다(마운트 스윕용).
fn resolve_arm(
    robot_id: &str,
    mount: Option<(f64, f64)>,
) -> Result<(Arc<Arm>, Option<Arc<pingpong_bot::UrdfRobot>>)> {
    let entry = find_robot(robot_id).ok_or_else(|| {
        anyhow!(
            "알 수 없는 robot id `{robot_id}` — 사용 가능: {}",
            robot_ids_csv()
        )
    })?;
    if let Some(rel) = entry.urdf_rel {
        let path = std::env::current_dir().context("현재 작업 디렉터리")?.join(rel);
        let mut builder = RobotBuilder::new()
            .urdf(&path)
            .ee_link_opt(entry.ee_link)
            .max_joint_speed(entry.max_joint_speed);
        builder = match mount {
            Some((base_y, height_offset_m)) => builder.mount(
                pingpong_bot::SimRobotMount::rep103_z_up_at_table_end_with_mount(
                    base_y,
                    height_offset_m,
                ),
            ),
            None => builder.mount_preset(MountPreset::Rep103AtTableEnd),
        };
        let built = builder
            .build()
            .with_context(|| format!("로봇 빌드 실패: {}", path.display()))?;
        return Ok((built.arm, built.urdf));
    }
    let arm = entry
        .primitive_arm()
        .ok_or_else(|| anyhow!("robot `{robot_id}` primitive 빌더 누락"))?;
    return Ok((arm, None));
}

/// 한 발의 결과 — GUI 사용자가 실제로 보는 단계 그대로.
#[derive(Debug, Default, Clone, Copy)]
struct ShotOutcome {
    /// **들어오는** 공이 정상적인 랠리 샷인가 — 네트를 넘어와서 로봇 쪽
    /// 코트(0 < y < LENGTH_Y/2) 테이블 면에 한 번 바운스했는가.
    ///
    /// 이 가드가 없으면 스윕이 "팔이 치기 쉬운" 방향으로만 최적화돼, 테이블
    /// 위를 아예 넘어가버리는 높은 로브(바운스 없음)나 네트에 맞는 샷을
    /// 정답으로 고를 수 있다 — 둘 다 실제 탁구 랠리가 아니다.
    incoming_valid: bool,
    /// `plan_best_swing`이 실제로 커밋됐는가 (= "로봇이 얼어붙지 않았는가").
    committed: bool,
    /// 공–라켓 활성 접촉이 있었는가.
    contact: bool,
    /// 접촉 뒤 공이 +y(상대편)로 되돌아갔는가.
    returned: bool,
    /// 리턴 공이 네트 윗면 위로 통과했는가.
    cleared_net: bool,
    /// 리턴 공이 **상대 코트 테이블 면에 실제로 떨어졌는가**.
    ///
    /// `cleared_net`만으로는 부족하다: 네트를 62cm 높이로 넘어 테이블 끝
    /// (y=2.74)을 지나 2.89까지 날아가는 로브도 `cleared_net`이 켜진다
    /// (2026-07-23 실측, `ground_truth_rally_contacts_racket_clears_net_and_
    /// bounces_near_center`가 잡아낸 실제 사례). 그건 랠리가 이어지는
    /// 리턴이 아니라 아웃이다.
    returned_in: bool,
    /// commit 창(네트 통과 후) 동안 관측된 `peak_joint_speed_ratio`의 최소값.
    ///
    /// `committed`가 격자 전역에서 0이면 이진 점수로는 어느 방향이 더
    /// 나은지 알 수 없다(기울기가 없음). 이 연속값은 "얼마나 모자라는가"를
    /// 준다 — `NEAR_SINGULARITY_SPEED_RATIO`(2.5) 이하로 내려가야 실제로
    /// commit이 가능해진다.
    best_peak_ratio: f64,
}

fn run_shot(
    arm: &Arc<Arm>,
    urdf: &Option<Arc<pingpong_bot::UrdfRobot>>,
    settings: &BallShooterSettings,
    start_from_table_center: bool,
) -> ShotOutcome {
    use pingpong_bot::constants::{BALL_RADIUS, table};

    let mut world = SimWorld::new(arm.clone(), urdf.clone());
    world.set_use_ground_truth(true);
    // `SimWorld::with_physics`가 세팅하는 기본 접수 창과 같은 값 — 진단용
    // `swing_feasibility` 샘플링이 실제 commit 경로와 같은 평면을 보게 한다.
    let intercept = pingpong_bot::InterceptWindow {
        y_min: 0.20,
        y_max: 0.55,
        sample_step: 0.05,
    };
    if start_from_table_center {
        world.robot_mut().set_rail_target(table::WIDTH_X * 0.5);
    }

    let ball_collider = world
        .collider_set
        .iter()
        .find_map(|(handle, collider)| (collider.parent() == Some(world.ball_handle)).then_some(handle))
        .expect("ball collider");
    let racket_collider = world
        .collider_set
        .iter()
        .find_map(|(handle, collider)| {
            (collider.parent() == Some(world.racket_handle)).then_some(handle)
        })
        .expect("racket collider");
    let table_collider = world
        .collider_set
        .iter()
        .find_map(|(handle, collider)| {
            let cuboid = collider.shape().as_cuboid()?;
            ((f64::from(cuboid.half_extents.x) - table::WIDTH_X * 0.5).abs() < 1e-5
                && (f64::from(cuboid.half_extents.y) - table::LENGTH_Y * 0.5).abs() < 1e-5)
                .then_some(handle)
        })
        .expect("table collider");

    world.shoot_ball(settings);

    let mut outcome = ShotOutcome {
        best_peak_ratio: f64::INFINITY,
        ..ShotOutcome::default()
    };
    let net_y = (table::LENGTH_Y * 0.5) as f32;
    let net_top_z = (table::SURFACE_Z + table::NET_HEIGHT + BALL_RADIUS) as f32;
    let mut previous_y = world.ball_position().y;
    let mut incoming_crossed_net = false;

    for step in 0..MAX_STEPS {
        world.step(DT, None);
        outcome.committed |= world.swing_committed();
        let position = world.ball_position();
        let velocity = world.ball_velocity();

        // 들어오는 공(아직 라켓에 안 닿은 상태)의 적법성 판정.
        if !outcome.contact {
            if previous_y > net_y && position.y <= net_y {
                incoming_crossed_net = position.z > net_top_z;
            }
            if incoming_crossed_net
                && !outcome.incoming_valid
                && position.y > 0.0
                && position.y < net_y
                && world
                    .narrow_phase
                    .contact_pair(ball_collider, table_collider)
                    .is_some_and(|pair| pair.has_any_active_contact())
            {
                outcome.incoming_valid = true;
            }
        }

        // `try_auto_swing`과 같은 조건(비행 중 + 네트 통과 후)에서 같은
        // 예측(`predict_impact` × intercept 창)을 만들어 실현가능성을 잰다.
        // `SWING_RETRY_THROTTLE_SECS`(0.02s)와 같은 주기로만 샘플링한다 —
        // 실제 플래너도 그 간격으로만 재시도하므로 정보 손실이 없고,
        // 매 스텝(1kHz) IK를 도는 것보다 20배 빠르다.
        if step % 20 == 0
            && !outcome.contact
            && world.ball_state == pingpong_bot::BallState::InFlight
            && pingpong_bot::ball_past_midcourt_for_commit(f64::from(position.y))
        {
            let start = RobotPose::new(world.robot().rail_x(), world.robot().joints().clone());
            for plane in intercept.hit_planes() {
                let Some(prediction) = pingpong_bot::sim::predict_impact(&world, plane) else {
                    continue;
                };
                if let Some(f) = pingpong_bot::swing_feasibility(arm, &prediction, &start) {
                    outcome.best_peak_ratio = outcome.best_peak_ratio.min(f.peak_joint_speed_ratio);
                }
            }
        }

        // 공의 바운스/네트 통과 지점을 그대로 찍어 궤적 자체를 확인한다
        // (`legal` 판정이 왜 그렇게 나왔는지 눈으로 보기 위함).
        if std::env::var("SHOT_TUNE_DEBUG").is_ok() {
            let on_table = world
                .narrow_phase
                .contact_pair(ball_collider, table_collider)
                .is_some_and(|pair| pair.has_any_active_contact());
            if on_table || (previous_y > net_y) != (position.y > net_y) {
                eprintln!(
                    "  y={:.3} z={:.3} vy={:.2} on_table={on_table} crossed_net={incoming_crossed_net} contact={}",
                    position.y, position.z, velocity.y, outcome.contact
                );
            }
        }

        if world
            .narrow_phase
            .contact_pair(ball_collider, racket_collider)
            .is_some_and(|pair| pair.has_any_active_contact())
        {
            outcome.contact = true;
        }
        if outcome.contact && velocity.y > 0.0 {
            outcome.returned = true;
        }
        if outcome.returned && previous_y < net_y && position.y >= net_y {
            outcome.cleared_net = position.z > net_top_z;
        }
        // 네트를 넘긴 뒤 상대 코트(net_y < y < LENGTH_Y) 테이블 면에 닿아야
        // 진짜 "들어간" 리턴이다. 여기서 break하지 않고 착지까지 지켜본다.
        if outcome.cleared_net
            && !outcome.returned_in
            && position.y > net_y
            && f64::from(position.y) < table::LENGTH_Y
            && world
                .narrow_phase
                .contact_pair(ball_collider, table_collider)
                .is_some_and(|pair| pair.has_any_active_contact())
        {
            outcome.returned_in = true;
            break;
        }
        previous_y = position.y;
    }

    return outcome;
}

/// 이 팔이 실제로 마주칠 대표 임팩트 예측들 — 레일이 담당하는 x는 테이블
/// 폭 전역, y는 접수 창(`InterceptWindow` 기본 0.20~0.55), 높이는 1차 조사가
/// 찾아낸 "실현 가능 대역"(테이블 위 10~30cm)을 격자로 훑는다.
///
/// 입사속도는 `tools/shot_tune` 실측에서 실제 랠리가 만들어내는 조성
/// (수평 6~8 m/s에 완만한 상하 성분)을 대표값으로 쓴다.
fn rest_pose_scenarios() -> Vec<pingpong_bot::Prediction> {
    use pingpong_bot::constants::table;
    use pingpong_bot::{Point3, Prediction};

    let mut out = Vec::new();
    for &x_frac in &[0.1, 0.3, 0.5, 0.7, 0.9] {
        for &y in &[0.20, 0.30, 0.40, 0.55] {
            for &z_off in &[0.10, 0.18, 0.26, 0.30] {
                for &(speed, descend) in &[(6.0, -0.15), (7.5, 0.10), (6.5, 0.30)] {
                    out.push(Prediction {
                        // 순간 IK 자세만 보므로 대표값 고정 (궤적 생성 없음).
                        time_to_impact_secs: 0.15,
                        impact_position: Point3::new(
                            table::WIDTH_X * x_frac,
                            y,
                            table::SURFACE_Z + z_off,
                        ),
                        incoming_velocity: nalgebra::Vector3::new(0.0, -speed, speed * descend),
                    });
                }
            }
        }
    }
    return out;
}

/// 대표 임팩트 자세들까지의 **최악** 관절 이동거리를 최소화하는 휴지 자세.
///
/// 비용은 `max_scenario max_joint |q*_j - q_j^s|` 인데, 두 max는 교환 가능해
/// `max_j (max_s q_j^s - min_s q_j^s)/2` 로 분리된다 — 즉 관절마다 독립적으로
/// "그 관절이 가야 하는 각도 구간의 중점"(1D Chebyshev 중심)이 정확한 최적해다.
/// 평균이 아니라 최악값을 줄여야 하는 이유: quintic 소요시간은 가장 많이
/// 움직이는 한 관절이 결정하므로, 나머지가 아무리 가까워도 소용이 없다.
///
/// IK 힌트가 `arm.default_joints`라 해집합이 휴지 자세에 의존한다(고정점
/// 문제) — 그래서 몇 번 반복해 수렴시킨다.
fn rest_pose_search(arm: &Arm, iterations: usize) {
    let scenarios = rest_pose_scenarios();
    let n = arm.default_joints.values.len();
    let mut arm = arm.clone();

    for iteration in 0..iterations {
        let mut lo = vec![f64::INFINITY; n];
        let mut hi = vec![f64::NEG_INFINITY; n];
        let mut solved = 0usize;
        for prediction in &scenarios {
            let Some(pose) = pingpong_bot::plan_coarse_track(&arm, std::slice::from_ref(prediction))
            else {
                continue;
            };
            solved += 1;
            for (j, &q) in pose.joints.values.iter().enumerate() {
                lo[j] = lo[j].min(q);
                hi[j] = hi[j].max(q);
            }
        }
        if solved == 0 {
            println!("IK 해가 있는 시나리오가 없음 — 탐색 불가");
            return;
        }

        let mut candidate = Vec::with_capacity(n);
        for j in 0..n {
            let mid = (lo[j] + hi[j]) * 0.5;
            let clamped = match arm.joint_limit(j) {
                Some(limit) => mid.clamp(limit.min, limit.max),
                None => mid,
            };
            candidate.push(clamped);
        }
        let worst_dq = (0..n)
            .map(|j| (hi[j] - candidate[j]).max(candidate[j] - lo[j]))
            .fold(0.0_f64, f64::max);
        let need_secs = 1.875 * worst_dq / arm.max_joint_speed;

        println!(
            "iter {iteration}: 해결 {solved}/{} | 휴지자세 [{}] | 최악 Δq={worst_dq:.3} rad → 필요시간 {need_secs:.3}s",
            scenarios.len(),
            candidate
                .iter()
                .map(|v| format!("{v:.4}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
        for j in 0..n {
            println!(
                "    joint {j}: 임팩트 각도 범위 [{:.3}, {:.3}] → 중심 {:.4}",
                lo[j], hi[j], candidate[j]
            );
        }
        arm.default_joints = pingpong_bot::Joints::from_slice(&candidate);
    }
}

/// 한 발을 돌리며 commit 창에서 `plan_best_swing`이 실제로 어떤 오류로
/// 실패하는지 그대로 출력한다 — "왜 안 되는가"를 추측 대신 확인하기 위함.
fn explain_one(
    arm: &Arc<Arm>,
    urdf: &Option<Arc<pingpong_bot::UrdfRobot>>,
    settings: &BallShooterSettings,
) {
    use pingpong_bot::constants::table;

    let mut world = SimWorld::new(arm.clone(), urdf.clone());
    // ground truth를 켜야 rough 단계의 레일 선추종(`plan_coarse_track`)이
    // 실제 게임플레이와 같이 동작한다 — 끄면 레일이 중앙에 머물러 IK가
    // 실제보다 불리하게 평가된다. 커밋에 성공하면 그 시점에 COMMIT이 찍히고
    // 이후 월드가 알아서 스윙하므로, 실패가 계속되는 경우만 길게 출력된다.
    world.set_use_ground_truth(true);
    let intercept = pingpong_bot::InterceptWindow {
        y_min: 0.20,
        y_max: 0.55,
        sample_step: 0.05,
    };
    world.shoot_ball(settings);

    println!(
        "explain — speed={:.2} pitch={:.2} height={:.3}",
        settings.speed_mps, settings.pitch_deg, settings.height_offset_m
    );
    for step in 0..MAX_STEPS {
        world.step(DT, None);
        if step % 20 != 0 || world.ball_state != pingpong_bot::BallState::InFlight {
            continue;
        }
        let ball_y = f64::from(world.ball_position().y);
        if !pingpong_bot::ball_past_midcourt_for_commit(ball_y) {
            continue;
        }
        let predictions: Vec<_> = intercept
            .hit_planes()
            .into_iter()
            .filter_map(|plane| pingpong_bot::sim::predict_impact(&world, plane))
            .collect();
        let start = RobotPose::new(world.robot().rail_x(), world.robot().joints().clone());
        // 평면별로 "시간 창(`in_swing_commit_window`)"과 "관절속도 비율
        // (`NEAR_SINGULARITY_SPEED_RATIO`)" 중 무엇이 실제 병목인지 나눠 본다.
        let per_plane: Vec<String> = predictions
            .iter()
            .map(|p| {
                let t = p.time_to_impact_secs;
                let in_window = pingpong_bot::in_swing_commit_window(t);
                let ratio = match pingpong_bot::swing_feasibility(arm, p, &start) {
                    Some(f) => format!("{:.1}", f.peak_joint_speed_ratio),
                    None => "IK✗".to_string(),
                };
                // `plan_best_swing`은 실패해도 어느 평면 때문인지 안 알려주고
                // 마지막 오류만 남기므로, 평면별 `plan_swing`을 직접 부른다.
                // 여기서 Ok인데 `plan_best_swing`이 실패하면 범인은
                // `plan_best_swing`의 접촉오차 필터(MAX_CONTACT_ERROR)다.
                let plan = match pingpong_bot::plan_swing(arm, *p, &start) {
                    Ok(_) => "ok".to_string(),
                    Err(e) => format!("{e}"),
                };
                // 관절공간 이동거리 Δq와, 그걸 quintic(피크 계수 1.875)으로
                // 관절속도 한계 안에서 소화하는 데 필요한 최소 시간.
                let travel = pingpong_bot::swing_feasibility(arm, p, &start)
                    .and(
                        pingpong_bot::plan_coarse_track(arm, std::slice::from_ref(p))
                            .map(|target| {
                                let dq = target
                                    .joints
                                    .values
                                    .iter()
                                    .zip(&start.joints.values)
                                    .map(|(a, b)| (a - b).abs())
                                    .fold(0.0_f64, f64::max);
                                let need = 1.875 * dq / arm.max_joint_speed;
                                return format!(" dq={dq:.2}rad need>={need:.2}s");
                            }),
                    )
                    .unwrap_or_default();
                let plan = format!("{plan}{travel}");
                return format!(
                    "\n        y={:.2} t={t:.3}{} r={ratio} impact=({:.3},{:.3},{:.3}) v_in=({:.2},{:.2},{:.2}) -> {plan}",
                    p.impact_position.v.y,
                    if in_window { "" } else { " ✗win" },
                    p.impact_position.v.x,
                    p.impact_position.v.y,
                    p.impact_position.v.z,
                    p.incoming_velocity.x,
                    p.incoming_velocity.y,
                    p.incoming_velocity.z
                );
            })
            .collect();
        let outcome = match pingpong_bot::plan_best_swing(arm, &predictions, &start) {
            Ok(_) => "COMMIT".to_string(),
            Err(e) => format!("{e}"),
        };
        println!(
            "  t={:.3} ball_y={ball_y:.3} z={:.3}\n      {}\n      -> {outcome}",
            step as f64 * DT,
            world.ball_position().z,
            per_plane.join("  ")
        );
        if ball_y < 0.10 {
            break;
        }
    }
    println!("  (table SURFACE_Z={:.3})", table::SURFACE_Z);
}

#[derive(Debug, Serialize)]
struct CandidateResult {
    base_y: f64,
    mount_height_offset_m: f64,
    speed_mps: f64,
    pitch_deg: f64,
    height_offset_m: f64,
    shots: usize,
    /// 들어오는 공이 정상 랠리 샷이었던 횟수 — 이게 `shots`보다 작으면 그
    /// 후보는 애초에 탁구가 아니다(로브/네트 아웃).
    incoming_valid: usize,
    committed: usize,
    contact: usize,
    returned: usize,
    cleared_net: usize,
    /// 최종 점수 — 로봇이 **실제로 스윙을 커밋하고** 그 결과 공이 네트를
    /// 넘어갔는가. `cleared_net`만 보면 안 되는 이유: 스윙을 못 해 가만히
    /// 있는 라켓에 공이 우연히 맞고 튕겨 나가도 `contact`/`returned`/
    /// `cleared_net`이 모두 켜진다(2026-07-23 실측: 기존 기본값
    /// speed=5.0/pitch=-2.0/height=0.19에서 commit=0인데 cleared_net=9/12).
    /// 사용자가 보고한 "로봇이 얼어붙는다"가 정확히 이 상태다.
    /// `incoming_valid`까지 함께 요구해 "칠 수 없는 공을 안 쏘는" 방향이
    /// 아니라 "정상 랠리 공을 실제로 받아친다"만 점수로 인정한다.
    /// 리턴도 네트 통과만이 아니라 **상대 코트에 실제로 떨어질 것**을 요구한다.
    success: usize,
    /// 리턴이 상대 코트에 실제로 떨어진 횟수.
    returned_in: usize,
    /// 배터리 전체에서 관측된 최선의 `peak_joint_speed_ratio` (낮을수록 좋음,
    /// 2.5 이하여야 commit 가능).
    best_peak_ratio: f64,
    /// 샷별 최선 비율의 중앙값 — 한 발만 운 좋은 경우와 전반적으로 좋은
    /// 경우를 구분한다.
    median_peak_ratio: f64,
}

fn linspace(min: f64, max: f64, steps: usize) -> Vec<f64> {
    if steps <= 1 {
        return vec![min];
    }
    return (0..steps)
        .map(|i| min + (max - min) * i as f64 / (steps - 1) as f64)
        .collect();
}

fn main() -> Result<()> {
    let args = Args::parse();

    // 좌우 위치·yaw 배터리는 후보마다 **같은** 시드로 뽑아 공정 비교한다.
    let base = BallShooterSettings::default();
    let mut rng = StdRng::seed_from_u64(args.seed);
    let battery: Vec<BallShooterSettings> = if args.shots == 0 {
        vec![base.clone()]
    } else {
        (0..args.shots).map(|_| base.randomized(&mut rng)).collect()
    };

    if args.rest_pose_search {
        let (arm, _) = resolve_arm(
            &args.robot,
            Some((args.base_y_min, args.mount_height_min)),
        )?;
        rest_pose_search(&arm, 5);
        return Ok(());
    }

    if args.explain {
        let (arm, urdf) = resolve_arm(
            &args.robot,
            Some((args.base_y_min, args.mount_height_min)),
        )?;
        explain_one(
            &arm,
            &urdf,
            &BallShooterSettings {
                speed_mps: args.speed_min,
                pitch_deg: args.pitch_min,
                height_offset_m: args.height_min,
                ..BallShooterSettings::default()
            },
        );
        return Ok(());
    }

    let mut grid = Vec::new();
    for base_y in linspace(args.base_y_min, args.base_y_max, args.base_y_steps) {
        for mount_h in linspace(args.mount_height_min, args.mount_height_max, args.mount_height_steps)
        {
            for speed_mps in linspace(args.speed_min, args.speed_max, args.speed_steps) {
                for pitch_deg in linspace(args.pitch_min, args.pitch_max, args.pitch_steps) {
                    for height_offset_m in
                        linspace(args.height_min, args.height_max, args.height_steps)
                    {
                        grid.push((base_y, mount_h, speed_mps, pitch_deg, height_offset_m));
                    }
                }
            }
        }
    }

    // 후보끼리 완전히 독립이라 코어 수만큼 나눠 병렬로 돈다(격자 하나가
    // 수천 번의 4초 물리 시뮬이라 단일 스레드로는 수십 분 단위).
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get());
    let chunk = grid.len().div_ceil(threads).max(1);
    let mut results: Vec<CandidateResult> = std::thread::scope(|scope| {
        let handles: Vec<_> = grid
            .chunks(chunk)
            .map(|slice| {
                let battery = &battery;
                let robot_id = args.robot.as_str();
                let from_table_center = args.start_from_table_center;
                let random_speed = args.use_random_speed;
                scope.spawn(move || {
                    let mut out = Vec::with_capacity(slice.len());
                    let mut cached: Option<(f64, f64, Arc<Arm>, Option<Arc<pingpong_bot::UrdfRobot>>)> =
                        None;
                    for &(base_y, mount_h, speed_mps, pitch_deg, height_offset_m) in slice {
                        // 마운트가 바뀔 때만 URDF를 다시 로드한다(파일 파싱이
                        // 후보당 수천 번 반복되면 스윕 시간을 지배함).
                        if !cached
                            .as_ref()
                            .is_some_and(|(y, h, _, _)| *y == base_y && *h == mount_h)
                        {
                            let (arm, urdf) =
                                resolve_arm(robot_id, Some((base_y, mount_h))).expect("로봇 빌드");
                            cached = Some((base_y, mount_h, arm, urdf));
                        }
                        let (_, _, arm, urdf) = cached.as_ref().expect("캐시된 로봇");
                        let mut result = CandidateResult {
                            base_y,
                            mount_height_offset_m: mount_h,
                            speed_mps,
                            pitch_deg,
                            height_offset_m,
                            shots: battery.len(),
                            incoming_valid: 0,
                            committed: 0,
                            contact: 0,
                            returned: 0,
                            cleared_net: 0,
                            returned_in: 0,
                            success: 0,
                            best_peak_ratio: f64::INFINITY,
                            median_peak_ratio: f64::INFINITY,
                        };
                        let mut ratios = Vec::with_capacity(battery.len());
                        for shot in battery {
                            let settings = BallShooterSettings {
                                speed_mps: if random_speed { shot.speed_mps } else { speed_mps },
                                pitch_deg,
                                height_offset_m,
                                ..shot.clone()
                            };
                            let outcome = run_shot(arm, urdf, &settings, from_table_center);
                            result.incoming_valid += usize::from(outcome.incoming_valid);
                            result.committed += usize::from(outcome.committed);
                            result.contact += usize::from(outcome.contact);
                            result.returned += usize::from(outcome.returned);
                            result.cleared_net += usize::from(outcome.cleared_net);
                            result.returned_in += usize::from(outcome.returned_in);
                            result.success += usize::from(
                                outcome.incoming_valid && outcome.committed && outcome.returned_in,
                            );
                            ratios.push(outcome.best_peak_ratio);
                        }
                        ratios.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                        result.best_peak_ratio = ratios.first().copied().unwrap_or(f64::INFINITY);
                        result.median_peak_ratio =
                            ratios.get(ratios.len() / 2).copied().unwrap_or(f64::INFINITY);
                        out.push(result);
                    }
                    return out;
                })
            })
            .collect();
        return handles
            .into_iter()
            .flat_map(|h| h.join().expect("sweep 스레드"))
            .collect();
    });

    if args.require_legal {
        results.retain(|r| r.incoming_valid == r.shots);
    }

    // `success`(적법한 샷을 커밋된 스윙으로 네트 너머로 리턴)가 최종 기준.
    results.sort_by(|a, b| {
        if args.sort_by_legal {
            return b.incoming_valid.cmp(&a.incoming_valid).then_with(|| {
                b.success.cmp(&a.success)
            });
        }
        return b
            .success
            .cmp(&a.success)
            .then_with(|| b.incoming_valid.cmp(&a.incoming_valid))
            .then_with(|| b.committed.cmp(&a.committed))
            // commit이 전역 0인 구간에서는 이 연속값이 유일한 방향 신호다.
            .then_with(|| {
                a.median_peak_ratio
                    .partial_cmp(&b.median_peak_ratio)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .then_with(|| b.cleared_net.cmp(&a.cleared_net));
    });

    if args.json {
        println!(
            "{}",
            serde_json::to_string_pretty(&results[..results.len().min(args.top_n)])?
        );
    } else {
        println!(
            "shot-tune — robot `{}`, {} 후보 x {} 샷 (각 샷 = 실제 Rapier ground-truth 랠리)",
            args.robot,
            results.len(),
            battery.len()
        );
        println!(
            "{:>7} {:>7} {:>7} {:>7} {:>8} {:>6} {:>7} {:>7} {:>6} {:>7} {:>7}",
            "base_y", "mnt_h", "speed", "pitch", "height", "legal", "commit", "cleared",
            "IN", "SUCCESS", "med_r"
        );
        for r in results.iter().take(args.top_n) {
            println!(
                "{:>7.3} {:>7.3} {:>7.2} {:>7.2} {:>8.3} {:>6} {:>7} {:>7} {:>6} {:>7} {:>7.2}",
                r.base_y,
                r.mount_height_offset_m,
                r.speed_mps,
                r.pitch_deg,
                r.height_offset_m,
                r.incoming_valid,
                r.committed,
                r.cleared_net,
                r.returned_in,
                r.success,
                r.median_peak_ratio
            );
        }
    }

    return Ok(());
}
