//! swing-bench: quintic 스윙 모양 제약 없이, 순수 토크 한계 안에서 이
//! 로봇팔이 특정 임팩트에 실제로 얼마나 빨리·안정적으로 도달할 수 있는지
//! 측정하는 오프라인 벤치마크/프로파일링 도구.
//!
//! `plan_swing`(실제 게임플레이 경로, quintic 궤적)은 건드리지 않는다 — 이
//! 도구는 그 대신 매 스텝 (`planner::dynamics::mass_matrix`/`forward_dynamics`)
//! 로 실제 강체 동역학을 적분하면서, 관절마다 시간최적(bang-bang) 스위칭
//! 곡선으로 토크를 명령한다. 사전에 정해둔 궤적 "모양"이 없다 — 매 틱 현재
//! 상태에서 다시 스위칭을 계산하는 폐루프라 관절 간 결합(coupling)에도
//! 스스로 보정된다.
//!
//! 사용법 (하이브리드: TOML 시나리오 파일 + CLI 오버라이드):
//!   cargo run -p swing-bench -- --scenario scenarios/example.toml
//!   cargo run -p swing-bench -- --robot 4-dof --impact-x 0.76 --impact-y 0.30 \
//!       --impact-z 0.78 --incoming-vx 0.0 --incoming-vy -5.0 --incoming-vz -0.2
//! (반드시 저장소 루트에서 실행 — `--robot`의 URDF 상대경로가 현재 디렉터리
//! 기준이다, `config/*.toml`과 동일한 관례.)

use std::path::PathBuf;

use anyhow::{Context, Result, anyhow, bail};
use clap::Parser;
use nalgebra::Vector3;
use pingpong_bot::planner::dynamics::{forward_dynamics, mass_matrix};
use pingpong_bot::{
    Arm, Joints, MountPreset, Point3, RobotBuilder, RobotPose, find_robot,
    rally_return_velocity, required_racket_velocity, robot_ids_csv,
};
use serde::{Deserialize, Serialize};

/// 실기 AXL 레일 가속/감속 [m/s^2].
/// 출처: `config/real-hardware.toml`의 `[hardware.rail]` accel/decel = 12.0.
const RAIL_ACCEL_M_S2: f64 = 12.0;

/// 수렴 판정 허용 오차 — `RobotState::is_at_center`의 관례(1e-3)를 따른다.
const POSITION_TOLERANCE_RAD_OR_M: f64 = 1e-3;
/// 라켓 속도 크기 허용오차(목표 대비 비율) — 목표의 [1-tol, 1+tol] 안이면 OK.
///
/// 관절 공간 목표속도(`target.joint_velocities`)를 칼같이 맞추라고 하면,
/// 같은 라켓 속도를 내는 다른 관절 조합이 있어도 무시하고 하나의 특정
/// 조합만 강요하게 돼 불필요한 백스윙성 왕복이 "필수"인 것처럼 나온다.
/// 실제로 필요한 건 임팩트 순간 라켓의 방향·크기지 특정 관절 속도 벡터가
/// 아니라서, 수렴 판정은 FK로 역산한 실제 라켓 속도 대 목표로 한다.
const RACKET_SPEED_RATIO_TOLERANCE: f64 = 0.15;
/// 라켓 속도 방향 허용오차 [deg].
const RACKET_DIRECTION_TOLERANCE_DEG: f64 = 15.0;

#[derive(Parser, Debug)]
#[command(
    about = "quintic 스윙 모양 제약 없이 순수 토크 한계로 임팩트 도달 시간을 측정한다"
)]
struct Args {
    /// TOML 시나리오 파일. 여기 값들을 아래 CLI 플래그가 덮어쓴다.
    #[arg(long)]
    scenario: Option<PathBuf>,

    /// 카탈로그 로봇 id (`competition` | `urdf-test` | `4-dof`).
    #[arg(long)]
    robot: Option<String>,

    /// 시작 레일 x [m]. 생략하면 레일 중앙(`default_x()`).
    #[arg(long)]
    start_rail_x: Option<f64>,

    #[arg(long, allow_hyphen_values = true)]
    impact_x: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    impact_y: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    impact_z: Option<f64>,

    #[arg(long, allow_hyphen_values = true)]
    incoming_vx: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    incoming_vy: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    incoming_vz: Option<f64>,

    /// 참고용 — 실제 예측이라면 이 안에 들어와야 할 여유 시간 [s]. 결과 판정에는
    /// 안 쓰고, 리포트에서 achieved_time과 나란히 비교만 한다.
    #[arg(long)]
    time_budget_secs: Option<f64>,

    /// 적분 스텝 [s].
    #[arg(long, default_value_t = 0.001)]
    dt: f64,

    /// 수렴하지 않을 때 포기하는 최대 시뮬레이션 시간 [s].
    #[arg(long, default_value_t = 2.0)]
    max_time_secs: f64,

    /// 사람이 읽는 표 대신 JSON으로 출력.
    #[arg(long)]
    json: bool,
}

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    robot: Option<String>,
    start_rail_x: Option<f64>,
    impact: Option<[f64; 3]>,
    incoming_velocity: Option<[f64; 3]>,
    time_budget_secs: Option<f64>,
}

#[derive(Debug, Serialize)]
struct Report {
    robot: String,
    /// 목표 라켓 속도가 관절/레일 속도 한계를 넘어 실행 전에 잘렸는지.
    /// true면 아래 결과는 "이상적인 리턴 파워"가 아니라 "낼 수 있는 최대
    /// 속도로 한 번 자른 뒤" 기준이라는 뜻 — 조용히 숨기지 않는다.
    target_speed_clamped: bool,
    feasible: bool,
    achieved_time_secs: f64,
    /// 목표 관절속도까지는 못 맞춰도, 위치만 허용오차 안에 처음 들어온 시각
    /// [s] — 라켓이 "제자리"에는 도착했는지(타점 자체는 맞았는지)를 목표
    /// 속도 달성 여부와 분리해서 본다. 끝까지 도달 못 하면 `None`.
    position_reached_time_secs: Option<f64>,
    max_time_secs: f64,
    time_budget_secs: Option<f64>,
    within_time_budget: Option<bool>,
    position_error: f64,
    /// 종료 시점 실제 라켓 속도 크기 [m/s] (FK 유한차분 역산).
    achieved_racket_speed_m_s: f64,
    /// 목표 라켓 속도 크기 [m/s] (`target_speed_clamped`면 잘린 뒤 값).
    target_racket_speed_m_s: f64,
    /// 종료 시점 라켓 속도 방향과 목표 방향의 각도차 [deg].
    racket_direction_error_deg: f64,
    peak_joint_torque_utilization: Vec<f64>,
    peak_joint_speed_rad_s: Vec<f64>,
    peak_joint_speed_ratio_to_cap: Vec<f64>,
    peak_rail_speed_m_s: f64,
    peak_rail_speed_ratio_to_cap: f64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let scenario = match &args.scenario {
        Some(path) => {
            let text = std::fs::read_to_string(path)
                .with_context(|| format!("시나리오 파일 읽기 실패: {}", path.display()))?;
            toml::from_str(&text)
                .with_context(|| format!("시나리오 TOML 파싱 실패: {}", path.display()))?
        }
        None => Scenario::default(),
    };

    let robot_id = args
        .robot
        .clone()
        .or(scenario.robot.clone())
        .unwrap_or_else(|| "4-dof".to_string());

    let mut impact = scenario.impact.unwrap_or([f64::NAN; 3]);
    if let Some(x) = args.impact_x {
        impact[0] = x;
    }
    if let Some(y) = args.impact_y {
        impact[1] = y;
    }
    if let Some(z) = args.impact_z {
        impact[2] = z;
    }
    if impact.iter().any(|v| v.is_nan()) {
        bail!(
            "impact 위치가 필요합니다 — --scenario의 [impact] 또는 --impact-x/y/z를 모두 지정하세요"
        );
    }

    let mut incoming = scenario.incoming_velocity.unwrap_or([f64::NAN; 3]);
    if let Some(vx) = args.incoming_vx {
        incoming[0] = vx;
    }
    if let Some(vy) = args.incoming_vy {
        incoming[1] = vy;
    }
    if let Some(vz) = args.incoming_vz {
        incoming[2] = vz;
    }
    if incoming.iter().any(|v| v.is_nan()) {
        bail!(
            "incoming_velocity가 필요합니다 — --scenario의 [incoming_velocity] 또는 --incoming-vx/vy/vz를 모두 지정하세요"
        );
    }

    let time_budget_secs = args.time_budget_secs.or(scenario.time_budget_secs);
    let start_rail_x_override = args.start_rail_x.or(scenario.start_rail_x);

    let arm = resolve_arm(&robot_id)?;
    let rail = arm
        .rail
        .as_ref()
        .ok_or_else(|| anyhow!("robot `{robot_id}`에 레일이 없음 — swing-bench는 레일 있는 로봇 전용"))?;
    let start_rail_x = start_rail_x_override.unwrap_or_else(|| rail.default_x());

    let start = RobotPose::new(start_rail_x, arm.default_joints.clone());
    let target = compute_target(
        &arm,
        &start,
        Point3::new(impact[0], impact[1], impact[2]),
        Vector3::new(incoming[0], incoming[1], incoming[2]),
    )?;

    let mut target = target;
    let target_speed_clamped = clamp_target_to_speed_caps(&arm, &mut target);

    let mut report = simulate(&arm, &start, &target, args.dt, args.max_time_secs, time_budget_secs);
    report.robot = robot_id.clone();
    report.target_speed_clamped = target_speed_clamped;

    if args.json {
        println!("{}", serde_json::to_string_pretty(&report)?);
    } else {
        print_human(&robot_id, &report);
    }

    return Ok(());
}

/// `main.rs::load_robot`와 같은 카탈로그 dispatch(URDF vs primitive) — GUI/캘리브레이션은 필요 없어 그 부분만 뺐다.
fn resolve_arm(robot_id: &str) -> Result<std::sync::Arc<Arm>> {
    let entry = find_robot(robot_id).ok_or_else(|| {
        anyhow!(
            "알 수 없는 robot id `{robot_id}` — 사용 가능: {}",
            robot_ids_csv()
        )
    })?;
    if let Some(rel) = entry.urdf_rel {
        let workspace = std::env::current_dir().context("현재 작업 디렉터리")?;
        let path = workspace.join(rel);
        let built = RobotBuilder::new()
            .urdf(&path)
            .ee_link_opt(entry.ee_link)
            .mount_preset(MountPreset::Rep103AtTableEnd)
            .max_joint_speed(entry.max_joint_speed)
            .build()
            .with_context(|| format!("로봇 빌드 실패: {}", path.display()))?;
        return Ok(built.arm);
    }
    let arm = entry
        .primitive_arm()
        .ok_or_else(|| anyhow!("robot `{robot_id}` primitive 빌더 누락"))?;
    return Ok(arm);
}

struct Target {
    rail_x: f64,
    joints: Joints,
    rail_velocity: f64,
    joint_velocities: Vec<f64>,
    /// 임팩트 순간 라켓이 실제로 내야 하는 속도(월드) — `required_racket_velocity`
    /// 원본. 수렴 판정은 이걸 기준으로 한다(관절 공간 속도 벡터를 칼같이
    /// 맞추면 불필요한 백스윙성 왕복이 "최적해"로 나올 수 있어서 — 관절
    /// 속도는 여러 조합이 같은 라켓 속도를 낼 수 있는데 그중 하나만 목표로
    /// 못박으면 과잉제약이다).
    racket_velocity: Vector3<f64>,
}

/// 목표 속도가 실제 관절/레일 속도 한계를 넘으면 그 한계로 자른다.
///
/// bang-bang 스위칭 곡선은 목표 (위치,속도)를 상대좌표 원점으로 모는데,
/// 목표 속도 자체가 실기에서 낼 수 없는 값이면(예: 원하는 라켓 속도가
/// 특이점 근처 관절 속도로 환산돼 한계를 훌쩍 넘는 경우) 속도 오차가
/// 절대 줄지 않아 컨트롤러가 위치는 무시한 채 그 방향으로 계속 전력
/// 가속만 하다 위치를 지나쳐 발산한다. 여기서 미리 달성 가능한 범위로
/// 잘라 스위칭 곡선이 실제로 수렴 가능한 목표를 보게 한다. 잘랐다는
/// 사실 자체가 "이상적인 라켓 속도는 이 로봇 한계 밖"이라는 유의미한
/// 정보라 `Report`에 남겨 조용히 숨기지 않는다.
fn clamp_target_to_speed_caps(arm: &Arm, target: &mut Target) -> bool {
    let mut clamped = false;
    for v in target.joint_velocities.iter_mut() {
        let capped = v.clamp(-arm.max_joint_speed, arm.max_joint_speed);
        if (capped - *v).abs() > 1e-12 {
            clamped = true;
        }
        *v = capped;
    }
    if let Some(rail) = &arm.rail {
        let capped = target.rail_velocity.clamp(-rail.max_speed, rail.max_speed);
        if (capped - target.rail_velocity).abs() > 1e-12 {
            clamped = true;
        }
        target.rail_velocity = capped;
    }
    return clamped;
}

/// `plan_swing`과 같은 임팩트 설정(목표 라켓 자세/속도 → 관절각·관절속도
/// 역산)을 재사용한다. 여기서 갈라지는 지점은 이다음이다 — `plan_swing`은
/// 이 목표를 quintic에 넣지만, 여기서는 `simulate`가 순수 토크 적분으로
/// 도달 시간 자체를 구한다.
fn compute_target(
    arm: &Arm,
    start: &RobotPose,
    impact: Point3,
    incoming_velocity: Vector3<f64>,
) -> Result<Target> {
    let v_in = incoming_velocity;
    let v_out = rally_return_velocity(impact, v_in);
    let desired_normal = (v_out - v_in).normalize();

    let ik_hint = arm
        .with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out - v_in))
        .map_err(|e| anyhow!("wrist-open IK 힌트 실패: {e}"))?;
    let racket_center = Point3::from(
        impact.v
            - desired_normal
                * (pingpong_bot::constants::BALL_RADIUS
                    + pingpong_bot::constants::geometry::RACKET_HALF_Z),
    );
    let solved = arm
        .inverse_pose_with_rail(
            racket_center,
            desired_normal,
            &RobotPose::new(start.rail_x, ik_hint),
        )
        .map_err(|e| anyhow!("임팩트 IK 실패: {e}"))?;
    let pose = arm
        .forward_kinematics_with_rail(solved.rail_x, &solved.joints)
        .ok_or_else(|| anyhow!("IK 해에서 FK 실패"))?;

    let v_r = required_racket_velocity(
        v_in,
        v_out,
        pose.normal,
        pingpong_bot::constants::RACKET_EFFECTIVE_RESTITUTION,
    )
    .map_err(|e| anyhow!("목표 라켓 속도 계산 실패: {e}"))?;
    let (rail_velocity, joint_velocities) = arm
        .velocities_for_racket_velocity(&solved, v_r)
        .map_err(|e| anyhow!("목표 관절속도 역산 실패: {e}"))?;

    return Ok(Target {
        rail_x: solved.rail_x,
        joints: solved.joints,
        rail_velocity,
        joint_velocities,
        racket_velocity: v_r,
    });
}

/// 현재 관절/레일 위치·속도에서 실제로 나오는 라켓(월드) 속도 추정.
///
/// `Arm::velocities_for_racket_velocity`와 같은 유한차분 스타일(그 함수의
/// `STEP=1e-6`과 동일한 크기)로 FK를 두 번 불러 만든다 — 전용 순방향
/// 자코비안 API가 없어 직접 만든다.
fn racket_velocity_estimate(
    arm: &Arm,
    rail_x: f64,
    rail_velocity: f64,
    joints: &[f64],
    joint_velocities: &[f64],
) -> Option<Vector3<f64>> {
    const STEP: f64 = 1e-6;
    let base = arm.forward_kinematics_with_rail(rail_x, &Joints::from_slice(joints))?;
    let perturbed_joints: Vec<f64> = joints
        .iter()
        .zip(joint_velocities)
        .map(|(q, v)| q + v * STEP)
        .collect();
    let perturbed = arm.forward_kinematics_with_rail(
        rail_x + rail_velocity * STEP,
        &Joints::from_slice(&perturbed_joints),
    )?;
    return Some((perturbed.position.v - base.position.v) / STEP);
}

/// 1차원 이중적분기를 원점(목표)으로 모는 시간최적 bang-bang 스위칭.
///
/// `x`/`v`는 목표 기준 상대 위치/속도 오차(`현재 - 목표`), `a_max`는 이
/// 축이 낼 수 있는 최대 가속. 표준 최소시간 스위칭 곡선
/// `x + v|v|/(2 a_max) = 0` 기준 부호로 ±a_max를 고른다.
fn bang_bang_accel(x: f64, v: f64, a_max: f64) -> f64 {
    let switch = x + v * v.abs() / (2.0 * a_max);
    if switch.abs() < 1e-12 {
        return 0.0;
    }
    return -a_max * switch.signum();
}

fn simulate(
    arm: &Arm,
    start: &RobotPose,
    target: &Target,
    dt: f64,
    max_time_secs: f64,
    time_budget_secs: Option<f64>,
) -> Report {
    let n = start.joints.values.len();
    let mut q = start.joints.values.clone();
    let mut qdot = vec![0.0; n];
    let mut rail_x = start.rail_x;
    let mut rail_v = 0.0;

    let rail_max_speed = arm.rail.as_ref().map_or(f64::INFINITY, |r| r.max_speed);

    let mut peak_util: Vec<f64> = vec![0.0; n];
    let mut peak_speed: Vec<f64> = vec![0.0; n];
    let mut peak_rail_speed = 0.0f64;

    let mut t = 0.0;
    let mut pos_err = f64::INFINITY;
    let mut achieved_speed = 0.0;
    let target_speed = target.racket_velocity.norm();
    let mut direction_error_deg = 180.0;
    let mut racket_velocity_ok = false;
    let mut position_reached_time_secs: Option<f64> = None;

    while t < max_time_secs {
        let m = mass_matrix(arm, &Joints::from_slice(&q));
        let mut torque_cmd = vec![0.0; n];
        for i in 0..n {
            let effective_inertia = m[(i, i)].max(1e-9);
            let a_max = (arm.joint_torque_limits[i] / effective_inertia).max(1e-6);
            let x = q[i] - target.joints.values[i];
            let v = qdot[i] - target.joint_velocities[i];
            let a_cmd = bang_bang_accel(x, v, a_max);
            torque_cmd[i] =
                (a_cmd * effective_inertia).clamp(-arm.joint_torque_limits[i], arm.joint_torque_limits[i]);
        }

        let Some(accel) = forward_dynamics(arm, &Joints::from_slice(&q), &qdot, &torque_cmd) else {
            break;
        };
        for i in 0..n {
            qdot[i] += accel[i] * dt;
            qdot[i] = qdot[i].clamp(-arm.max_joint_speed, arm.max_joint_speed);
            q[i] += qdot[i] * dt;
            peak_util[i] = peak_util[i].max(torque_cmd[i].abs() / arm.joint_torque_limits[i]);
            peak_speed[i] = peak_speed[i].max(qdot[i].abs());
        }
        if std::env::var("SWING_BENCH_DEBUG").is_ok() && (t % 0.05) < dt {
            eprintln!(
                "t={t:.3} q={q:?} qdot={qdot:?} target_q={:?} target_qdot={:?}",
                target.joints.values, target.joint_velocities
            );
        }

        {
            let x = rail_x - target.rail_x;
            let v = rail_v - target.rail_velocity;
            let a_cmd = bang_bang_accel(x, v, RAIL_ACCEL_M_S2);
            rail_v += a_cmd * dt;
            rail_v = rail_v.clamp(-rail_max_speed, rail_max_speed);
            rail_x += rail_v * dt;
            peak_rail_speed = peak_rail_speed.max(rail_v.abs());
        }

        t += dt;

        pos_err = q
            .iter()
            .zip(&target.joints.values)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max)
            .max((rail_x - target.rail_x).abs());

        let achieved_racket_velocity =
            racket_velocity_estimate(arm, rail_x, rail_v, &q, &qdot).unwrap_or(Vector3::zeros());
        achieved_speed = achieved_racket_velocity.norm();
        let speed_ratio = if target_speed > f64::EPSILON {
            achieved_speed / target_speed
        } else {
            1.0
        };
        direction_error_deg = if target_speed > f64::EPSILON && achieved_speed > f64::EPSILON {
            let cos_angle = (achieved_racket_velocity.dot(&target.racket_velocity)
                / (achieved_speed * target_speed))
                .clamp(-1.0, 1.0);
            cos_angle.acos().to_degrees()
        } else {
            0.0
        };
        racket_velocity_ok = (1.0 - RACKET_SPEED_RATIO_TOLERANCE
            ..=1.0 + RACKET_SPEED_RATIO_TOLERANCE)
            .contains(&speed_ratio)
            && direction_error_deg <= RACKET_DIRECTION_TOLERANCE_DEG;

        if position_reached_time_secs.is_none() && pos_err < POSITION_TOLERANCE_RAD_OR_M {
            position_reached_time_secs = Some(t);
        }
        if pos_err < POSITION_TOLERANCE_RAD_OR_M && racket_velocity_ok {
            break;
        }
    }

    let feasible = pos_err < POSITION_TOLERANCE_RAD_OR_M && racket_velocity_ok;
    let within_time_budget = time_budget_secs.map(|budget| feasible && t <= budget);

    return Report {
        robot: String::new(),               // main()에서 채움
        target_speed_clamped: false,        // main()에서 채움
        feasible,
        achieved_time_secs: t,
        position_reached_time_secs,
        max_time_secs,
        time_budget_secs,
        within_time_budget,
        position_error: pos_err,
        achieved_racket_speed_m_s: achieved_speed,
        target_racket_speed_m_s: target_speed,
        racket_direction_error_deg: direction_error_deg,
        peak_joint_torque_utilization: peak_util,
        peak_joint_speed_rad_s: peak_speed.clone(),
        peak_joint_speed_ratio_to_cap: peak_speed
            .iter()
            .map(|s| s / arm.max_joint_speed)
            .collect(),
        peak_rail_speed_m_s: peak_rail_speed,
        peak_rail_speed_ratio_to_cap: peak_rail_speed / rail_max_speed,
    };
}

fn print_human(robot_id: &str, report: &Report) {
    println!("swing-bench — robot `{robot_id}`");
    if report.target_speed_clamped {
        println!(
            "  [주의] 목표 라켓 속도가 이 로봇의 실제 속도 한계를 넘어 사전에 잘렸음 \
             (이상적인 리턴 파워가 아니라 낼 수 있는 최대 속도 기준)"
        );
    }
    println!(
        "  feasible: {} ({}s elapsed, cutoff {}s)",
        report.feasible, report.achieved_time_secs, report.max_time_secs
    );
    match report.position_reached_time_secs {
        Some(t) => println!(
            "  position-only reached at: {t:.4}s (목표 라켓 속도까지는 못 맞췄을 수 있음)"
        ),
        None => println!("  position-only reached at: 도달 못 함 (cutoff까지 못 감)"),
    }
    if let (Some(budget), Some(within)) = (report.time_budget_secs, report.within_time_budget) {
        println!(
            "  time budget: {budget}s → {}",
            if within { "충분함" } else { "부족함" }
        );
    }
    println!("  position error: {:.6}", report.position_error);
    println!(
        "  racket speed: {:.3} m/s (target {:.3} m/s, {:.1}%), direction error: {:.1}°",
        report.achieved_racket_speed_m_s,
        report.target_racket_speed_m_s,
        if report.target_racket_speed_m_s > f64::EPSILON {
            report.achieved_racket_speed_m_s / report.target_racket_speed_m_s * 100.0
        } else {
            100.0
        },
        report.racket_direction_error_deg
    );
    println!("  per-joint peak torque utilization (|τ|/limit):");
    for (i, u) in report.peak_joint_torque_utilization.iter().enumerate() {
        println!("    joint {i}: {:.1}%", u * 100.0);
    }
    println!("  per-joint peak speed vs cap:");
    for (i, (speed, ratio)) in report
        .peak_joint_speed_rad_s
        .iter()
        .zip(&report.peak_joint_speed_ratio_to_cap)
        .enumerate()
    {
        println!("    joint {i}: {speed:.3} rad/s ({:.1}% of cap)", ratio * 100.0);
    }
    println!(
        "  rail peak speed: {:.3} m/s ({:.1}% of cap)",
        report.peak_rail_speed_m_s,
        report.peak_rail_speed_ratio_to_cap * 100.0
    );
}
