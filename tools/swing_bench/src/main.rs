//! swing-bench: quintic 스윙 모양 제약 없이, 순수 토크 한계 안에서 이
//! 로봇팔이 특정 임팩트에 실제로 얼마나 빨리·안정적으로 도달할 수 있는지
//! 측정하는 오프라인 벤치마크/프로파일링 도구.
//!
//! `plan_swing`(실제 게임플레이 경로, quintic 궤적)은 건드리지 않는다 — 이
//! 도구는 GUI 디버그 경로(`plan_bang_bang_swing`)가 매 스텝 쓰는 것과 **같은**
//! [`pingpong_bot::step_racket_guidance`](라켓 task-space ZEM/ZEV 유도 +
//! 토크 가중 자코비안 역산)를 그대로 호출한다 — 예전에는 이 도구가 자체
//! `bang_bang_accel`(관절별 독립 bang-bang) 복사본을 따로 갖고 있어서, 그
//! 스위칭 곡선에 있던 버그를 한쪽만 고치고 다른 쪽은 실측 결과가 갈라진 적이
//! 있었다(`.omc/progress.txt`). 사전에 정해둔 궤적 "모양"이 없다 — 매 틱
//! 현재 상태에서 다시 유도를 계산하는 폐루프라 관절 간 결합(coupling)에도
//! 스스로 보정된다.
//!
//! ZEM/ZEV는 "남은 시간(`Tg`) 안에 도달"을 목표로 계산하므로, 이 도구가 매
//! 시나리오를 시간 무제한으로 "얼마나 빨리 가는지" 재는 대신, `--max-time-secs`
//! (기본 2.0s)를 그 `Tg`의 원본 마감으로 써서 "이 예산 안에 실제로
//! 도달하는가"를 측정한다 — 실제 GUI/게임플레이 경로가 실제 임팩트까지 남은
//! 시간을 그 마감으로 쓰는 것과 같은 구조다.
//!
//! 사용법 (하이브리드: TOML 시나리오 파일 + CLI 오버라이드):
//!   cargo run -p swing-bench -- --scenario scenarios/example.toml
//!   cargo run -p swing-bench -- --robot 4-dof --impact-x 0.76 --impact-y 0.30 \
//!       --impact-z 0.78 --incoming-vx 0.0 --incoming-vy -5.0 --incoming-vz -0.2
//! (반드시 저장소 루트에서 실행 — `--robot`의 URDF 상대경로가 현재 디렉터리
//! 기준이다, `config/*.toml`과 동일한 관례.)

mod args;
mod report;
mod scenario;
mod target;

use anyhow::{Context, Result, anyhow, bail};
use clap::Parser;
use nalgebra::Vector3;
use pingpong_bot::Point3;
use pingpong_bot::defaults;
use pingpong_bot::hardware::dynamixel::DYNAMIXEL_MAX_JOINT_SPEED_RAD_S;
use pingpong_bot::planner::Impact;
use pingpong_bot::robot::{self, Arm, Joints, MountPreset, RobotBuilder};
use pingpong_bot::sim::SimWorld;
use pingpong_bot::sim::launch;
use pingpong_bot::swing;
use serde::Serialize;

use args::Args;
use report::Report;
use scenario::Scenario;
use target::Target;

/// 수렴 판정 허용 오차 — `robot::State::is_at_center`의 관례(1e-3)를 따른다.
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

/// `--sim-verify` 대기 예산 — 스윙 커밋까지. 기본 슈터 설정의 비행시간(<1s)
/// 보다 넉넉하게 잡아 스윙이 늦게 커밋돼도 놓치지 않는다.
const SIM_VERIFY_MAX_WAIT_STEPS: usize = 4_000;
/// `--sim-verify` 대기 예산 — 커밋 이후 실제 접촉까지. `impact_time_secs`는
/// 보통 이보다 훨씬 짧지만(수백 ms), PD 지연으로 늦게 맞는 경우까지 커버.
const SIM_VERIFY_MAX_CONTACT_STEPS: usize = 3_000;

/// `--sim-verify` 관절별 결과 한 줄.
#[derive(Debug, Serialize)]
struct ContactVerifyJointRow {
    joint: usize,
    tracking_error_at_contact_rad: Option<f64>,
    tracking_error_at_planned_impact_rad: Option<f64>,
    peak_commanded_speed_rad_s: f64,
}

/// `--sim-verify` 결과.
#[derive(Debug, Serialize)]
struct ContactVerifyReport {
    swing_committed: bool,
    contact_detected: bool,
    planned_impact_time_secs: f64,
    contact_elapsed_secs: Option<f64>,
    contact_vs_planned_delta_secs: Option<f64>,
    joints: Vec<ContactVerifyJointRow>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    if args.sim_verify {
        return run_sim_verify(args.dt, args.json);
    }

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
    let rail = arm.rail.as_ref().ok_or_else(|| {
        anyhow!("robot `{robot_id}`에 레일이 없음 — swing-bench는 레일 있는 로봇 전용")
    })?;
    let start_rail_x = start_rail_x_override.unwrap_or_else(|| rail.default_x());

    let start = robot::Pose::new(start_rail_x, arm.default_joints.clone());
    let target = compute_target(
        &arm,
        &start,
        Point3::new(impact[0], impact[1], impact[2]),
        Vector3::new(incoming[0], incoming[1], incoming[2]),
    )?;

    let mut target = target;
    let target_speed_clamped = clamp_target_to_speed_caps(&arm, &mut target);

    let mut report = simulate(
        &arm,
        &start,
        &target,
        args.dt,
        args.max_time_secs,
        time_budget_secs,
    );
    report.robot = robot_id.clone();
    report.target_speed_clamped = target_speed_clamped;

    if args.json {
        println!("{}", serde_json::to_string_pretty(&report)?);
    } else {
        print_human(&robot_id, &report);
    }

    return Ok(());
}

/// `defaults` 프리셋 dispatch(URDF vs primitive) — GUI/캘리브레이션은 필요 없어 그 부분만 뺐다.
fn resolve_arm(robot_id: &str) -> Result<std::sync::Arc<Arm>> {
    let urdf_rel: &str = match robot_id {
        "4-dof" | "primitive" | "competition" => {
            let robot = defaults::primitive_4dof()
                .map_err(|e| anyhow!("primitive 4-dof 빌드 실패: {e}"))?;
            return Ok(robot.arm);
        }
        "urdf-4-dof" | "urdf" => "assets/robots/4-dof/urdf/all-4-export.urdf",
        "urdf-test" => "assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf",
        other => {
            return Err(anyhow!(
                "알 수 없는 robot id `{other}` — 사용 가능: 4-dof, primitive, urdf-4-dof, urdf-test"
            ));
        }
    };
    let workspace = std::env::current_dir().context("현재 작업 디렉터리")?;
    let path = workspace.join(urdf_rel);
    let built = RobotBuilder::new()
        .urdf(&path)
        .ee_link_opt(Some("pingpong_paddle_v5_1"))
        .mount_preset(MountPreset::Rep103AtTableEnd)
        .max_joint_speed(DYNAMIXEL_MAX_JOINT_SPEED_RAD_S)
        .build()
        .with_context(|| format!("로봇 빌드 실패: {}", path.display()))?;
    return Ok(built.arm);
}

/// 목표 관절/레일 속도가 실제 한계를 넘으면 그 한계로 자른다 — 하지만 이제
/// `simulate`가 이 클램프된 값을 직접 쓰지는 않는다(`step_racket_guidance`가
/// 라켓 task-space 목표만 보고, 실현 가능성은 내부에서 토크/관절가속 클램프로
/// 알아서 처리한다). 그래도 "IK가 요구한 이상적인 관절/레일 속도가 이 로봇의
/// 원시 속도 한계를 얼마나 넘는가"는 여전히 유의미한 진단 정보라
/// `target_speed_clamped`로 `Report`에 남긴다 — 잘랐다는 사실 자체가 "이상적인
/// 라켓 속도는 이 로봇 한계 밖"이라는 신호이지, 시뮬레이션 결과에 영향을
/// 주지는 않는다.
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
    start: &robot::Pose,
    impact: Point3,
    incoming_velocity: Vector3<f64>,
) -> Result<Target> {
    let v_in = incoming_velocity;
    let v_out = Impact::rally_return(impact, v_in);
    let desired_normal = (v_out - v_in).normalize();

    let ik_hint = arm
        .with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out - v_in))
        .map_err(|e| anyhow!("wrist-open IK 힌트 실패: {e}"))?;
    let racket_center = Point3::from(
        impact.coords
            - desired_normal
                * (pingpong_bot::constants::BALL_RADIUS
                    + pingpong_bot::constants::geometry::RACKET_HALF_Z),
    );
    let solved = arm
        .inverse_pose_with_rail(
            racket_center,
            desired_normal,
            &robot::Pose::new(start.rail_x, ik_hint),
        )
        .map_err(|e| anyhow!("임팩트 IK 실패: {e}"))?;
    let pose = arm
        .forward_kinematics_with_rail(solved.rail_x, &solved.joints)
        .ok_or_else(|| anyhow!("IK 해에서 FK 실패"))?;

    let v_r = Impact::required_racket_velocity(
        v_in,
        v_out,
        pose.normal,
        defaults::ImpactParams::default().racket_effective_restitution,
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
    return Some((perturbed.position.coords - base.position.coords) / STEP);
}

fn simulate(
    arm: &Arm,
    start: &robot::Pose,
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

    // `plan_bang_bang_for`(GUI 디버그 경로)와 같은 함수를 쓰므로, 목표도 그
    // 함수가 기대하는 라켓(3D) 좌표계로 맞춘다 — 관절 공간 목표(`target.joints`)가
    // 아니라 그 관절각의 FK 위치.
    let target_racket_position = arm
        .forward_kinematics_with_rail(target.rail_x, &target.joints)
        .expect("target FK — compute_target이 이미 IK로 검증한 자세")
        .position
        .coords;
    let mut scratch = swing::RacketGuidanceScratch::new(n);

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
        // ZEM/ZEV의 `Tg`(목표 도달까지 남은 시간)로 이 시뮬레이션 자체의
        // 예산(`max_time_secs`)을 쓴다 — 모듈 문서 참고.
        let remaining_secs = max_time_secs - t;
        let Some(step) = swing::Planner::step_racket_guidance(
            arm,
            &mut q,
            &mut qdot,
            &mut rail_x,
            &mut rail_v,
            target_racket_position,
            target.racket_velocity,
            remaining_secs,
            dt,
            &mut scratch,
        ) else {
            break;
        };
        for i in 0..n {
            peak_util[i] = peak_util[i].max(step.torque_cmd[i].abs() / arm.joint_torque_limits[i]);
            peak_speed[i] = peak_speed[i].max(qdot[i].abs());
        }
        peak_rail_speed = peak_rail_speed.max(rail_v.abs());
        if std::env::var("SWING_BENCH_DEBUG").is_ok() && (t % 0.05) < dt {
            eprintln!(
                "t={t:.3} q={q:?} qdot={qdot:?} racket_accel_desired={:?}",
                step.racket_accel_desired
            );
        }

        t += dt;

        let Some(current_pose) = arm.forward_kinematics_with_rail(rail_x, &Joints::from_slice(&q))
        else {
            break;
        };
        pos_err = (current_pose.position.coords - target_racket_position).norm();

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
        robot: String::new(),        // main()에서 채움
        target_speed_clamped: false, // main()에서 채움
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
        peak_joint_speed_ratio_to_cap: peak_speed.iter().map(|s| s / arm.max_joint_speed).collect(),
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
        Some(t) => {
            println!("  position-only reached at: {t:.4}s (목표 라켓 속도까지는 못 맞췄을 수 있음)")
        }
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
        println!(
            "    joint {i}: {speed:.3} rad/s ({:.1}% of cap)",
            ratio * 100.0
        );
    }
    println!(
        "  rail peak speed: {:.3} m/s ({:.1}% of cap)",
        report.peak_rail_speed_m_s,
        report.peak_rail_speed_ratio_to_cap * 100.0
    );
}

/// `--sim-verify`: 기본 4-dof 로봇으로 기본 슈터 설정 스윙 하나를 실제
/// Rapier `SimWorld`(ground-truth 자동 스윙 경로, `plan_best_swing`)로 진짜
/// ball-paddle 접촉까지 물리 스텝을 돌려, 계획된 quintic 궤적 대비 관절별
/// PD 추종 오차를 실측한다.
///
/// 위 `simulate`(기본 벤치 경로)는 `step_racket_guidance`의 이상적인
/// ZEM/ZEV 폐루프를 순수 토크로 적분할 뿐 Rapier PD 모터 모델을 전혀 거치지
/// 않는다 — 그래서 PD 추종 지연(base/shoulder가 실제 임팩트 순간 명령각에
/// 못 미치는 문제)을 볼 수 없다. 이 모드가 그 blind spot을 메운다.
fn run_sim_verify(dt: f64, json: bool) -> Result<()> {
    let robot =
        defaults::primitive_4dof().map_err(|e| anyhow!("기본 4-dof 로봇 빌드 실패: {e}"))?;
    let mut world = SimWorld::new(robot);
    world.set_use_ground_truth(true);
    world.shoot_ball(&launch::Settings::default());

    let mut committed_trajectory = None;
    for _ in 0..SIM_VERIFY_MAX_WAIT_STEPS {
        world.step(dt, None);
        if world.robot().is_swinging()
            && let Some(trajectory) = world.robot().active_trajectory()
        {
            committed_trajectory = Some(trajectory.clone());
            break;
        }
    }
    let Some(trajectory) = committed_trajectory else {
        bail!(
            "sim-verify: {SIM_VERIFY_MAX_WAIT_STEPS}스텝 안에 스윙이 커밋되지 않음 \
             (기본 슈터 설정이 바뀌었거나 자동 스윙 로직에 회귀가 있을 수 있음)"
        );
    };

    let n = trajectory.start.values.len();
    let mut contact_frame: Option<(f64, Vec<f64>, Vec<f64>)> = None;
    let mut planned_frame: Option<(f64, Vec<f64>, Vec<f64>)> = None;
    let mut elapsed = 0.0;

    for _ in 0..SIM_VERIFY_MAX_CONTACT_STEPS {
        world.step(dt, None);
        elapsed += dt;
        let actual = world.robot().joints().values.clone();
        let commanded = trajectory.sample_at(elapsed).values;

        if planned_frame.is_none() && elapsed >= trajectory.impact_time_secs {
            planned_frame = Some((elapsed, actual.clone(), commanded.clone()));
        }
        if contact_frame.is_none() && world.ball_racket_contact_active() {
            contact_frame = Some((elapsed, actual, commanded));
        }
        if contact_frame.is_some() && planned_frame.is_some() {
            break;
        }
    }

    let peak_commanded_speed = trajectory.peak_joint_speeds();
    let tracking_error = |frame: &Option<(f64, Vec<f64>, Vec<f64>)>, joint: usize| {
        frame
            .as_ref()
            .map(|(_, actual, commanded)| (actual[joint] - commanded[joint]).abs())
    };

    let report = ContactVerifyReport {
        swing_committed: true,
        contact_detected: contact_frame.is_some(),
        planned_impact_time_secs: trajectory.impact_time_secs,
        contact_elapsed_secs: contact_frame.as_ref().map(|(t, ..)| *t),
        contact_vs_planned_delta_secs: contact_frame
            .as_ref()
            .map(|(t, ..)| *t - trajectory.impact_time_secs),
        joints: (0..n)
            .map(|joint| ContactVerifyJointRow {
                joint,
                tracking_error_at_contact_rad: tracking_error(&contact_frame, joint),
                tracking_error_at_planned_impact_rad: tracking_error(&planned_frame, joint),
                peak_commanded_speed_rad_s: peak_commanded_speed.get(joint).copied().unwrap_or(0.0),
            })
            .collect(),
    };

    if json {
        println!("{}", serde_json::to_string_pretty(&report)?);
    } else {
        print_sim_verify_human(&report);
    }
    return Ok(());
}

fn print_sim_verify_human(report: &ContactVerifyReport) {
    println!("swing-bench --sim-verify — 실제 Rapier PD 추종 vs 계획된 quintic 궤적");
    println!("  swing committed: {}", report.swing_committed);
    println!(
        "  planned impact_time_secs: {:.4}s",
        report.planned_impact_time_secs
    );
    match (
        report.contact_elapsed_secs,
        report.contact_vs_planned_delta_secs,
    ) {
        (Some(t), Some(delta)) => {
            let timing = if delta > 1e-4 {
                "계획보다 늦음"
            } else if delta < -1e-4 {
                "계획보다 이름"
            } else {
                "계획과 거의 동시"
            };
            println!("  actual contact at: {t:.4}s ({delta:+.4}s, {timing})");
        }
        _ => println!(
            "  [주의] 접촉 대기창({SIM_VERIFY_MAX_CONTACT_STEPS}스텝) 안에 실제 \
             ball-racket ContactPair를 감지하지 못함"
        ),
    }
    println!("  --- per-joint tracking error |q_actual - q_commanded| (real Rapier PD sim) ---");
    for row in &report.joints {
        let at_contact = row
            .tracking_error_at_contact_rad
            .map_or_else(|| "n/a".to_string(), |v| format!("{v:.5} rad"));
        let at_planned = row
            .tracking_error_at_planned_impact_rad
            .map_or_else(|| "n/a".to_string(), |v| format!("{v:.5} rad"));
        println!(
            "    joint {}: at real contact={at_contact}, at planned impact={at_planned}, \
             peak commanded speed={:.3} rad/s",
            row.joint, row.peak_commanded_speed_rad_s
        );
    }
}
