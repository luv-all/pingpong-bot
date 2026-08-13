//! `--mode real` 라켓 헤드·리니어 레일 제어 진입점 — 조립 · 메인 루프 · 요약.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, ensure};
use crossbeam_channel::{Receiver, Sender, bounded, unbounded};
use pingpong_bot::camera::{
    Calibration, CamCliArgs, CamStreamArgs, PreviewAction, StereoOfflineArgs,
};
use pingpong_bot::defaults::vision::detector_for;
use pingpong_bot::defaults::{self, DEFAULT_STEREO_CAM_ROLES, camera_params_for, robot};
use pingpong_bot::hardware::dynamixel::DynamixelConfig;
use pingpong_bot::hardware::rail::{AxlRail, RailCalibration, RailConfig, RailEnd};
use pingpong_bot::hardware::{Hardware, RealHardware};
use tracing::{debug, info, warn};

use crate::cli::Args;

use super::camera_worker::{self, CameraStats};
use super::estimator_worker::{self, EstimatorStats};
use super::fmt::{f2, f2_slice};
use super::{
    Options, PacedSource, PreviewEvent, PreviewWindow, RuntimeEvent, ShutdownGuard, TestControl,
    control_worker, shutdown_channel,
};

/// 카메라 → 추정 버퍼. 실시간이라 크게 잡을 이유가 없다 (밀리면 어차피 버린다).
const VISION_CAPACITY: usize = 8;
const PREVIEW_CAPACITY: usize = 2;
/// 프리뷰가 없을 때 메인 루프 tick.
const IDLE_TICK: Duration = Duration::from_millis(5);
/// 실기 시작 홈잉은 기존 +X 왕복과 같은 논리 +X 엔드스톱을 사용한다.
const STARTUP_RAIL_HOME_END: RailEnd = RailEnd::Max;
/// 스케일 점검은 육안으로 방향과 거리를 확인할 수 있게 천천히 움직인다.
const STARTUP_RAIL_SCALE_CHECK_MOVE_SECS: f64 = 2.0;
/// 마지막 실측 자세에서 라켓 최하단과 상판 사이에 남길 실제 목표 간격 [m].
const FINAL_RACKET_TABLE_CLEARANCE_M: f64 = 0.010;
/// 모델 1cm 목표에서 실물이 4.3cm였던 실측 차이 [m].
/// 전용 측정 자세의 라켓 높이에만 적용하고 메인 타격·링크 충돌 모델은 바꾸지 않는다.
const FINAL_RACKET_OBSERVED_HEIGHT_ERROR_M: f64 = 0.033;
/// 실물 1cm를 만들기 위해 FK 모델에서 노려야 하는 라켓 끝 간격 [m].
const FINAL_RACKET_MODEL_CLEARANCE_M: f64 =
    FINAL_RACKET_TABLE_CLEARANCE_M - FINAL_RACKET_OBSERVED_HEIGHT_ERROR_M;
/// 홈 자세에서 테이블 근접 자세로 천천히 내려가는 시간.
const FINAL_RACKET_APPROACH_SECS: f64 = 6.0;
/// 최종 기준 자세에서 라켓 장축이 수직선과 이루어도 되는 최대 각도.
const FINAL_RACKET_VERTICAL_TOLERANCE_DEG: f64 = 3.0;

/// 보정된 공 접촉점에 라켓을 맞추고 상대편 반코트의 무게중심을 조준한다.
/// 종료는 ESC·`q`(preview) 또는 제어 워커 `Done`이다.
pub fn run(args: &Args) -> Result<()> {
    let options = Options::from_args(args);
    let robot = robot().context("defaults::robot")?;
    let arm = Arc::clone(&robot.arm);

    if options.rail_scale_check {
        return run_rail_scale_check_command(&options, &arm);
    }

    if options.home && !options.dry_run {
        calibrate_rail_on_startup()?;
    }
    let mut hardware = open_hardware(&options)?;
    // 카메라·추정·제어 스레드를 시작하기 전에 실기 자세부터 확정한다. 초기화 중에
    // 공을 잘못 추적하거나, 정렬 전 포즈를 sim에 보내는 일을 막는다.
    if options.home {
        info!("시작 자세 초기화 — 레일·관절을 준비 자세로 이동");
        let pose = control_worker::initialize_pose(&mut hardware, &arm)
            .map_err(|error| anyhow::anyhow!("시작 자세 초기화 실패: {error}"))?;
        info!(
            rail_x = f2(pose.rail_x),
            joints = %f2_slice(&pose.joints.values),
            "시작 자세 초기화 완료"
        );
    }
    let calibration = load_calibration()?;
    let sources = open_cameras(&options)?;
    ensure!(
        sources.len() >= calibration.min_cameras_for_triangulation(),
        "새 픽셀 궤적 적합에 카메라 {}대가 필요한데 {}대만 열렸다",
        calibration.min_cameras_for_triangulation(),
        sources.len()
    );

    let (guard, shutdown) = shutdown_channel();
    let (vision_tx, vision_rx) = bounded(VISION_CAPACITY);
    let vision_evict_rx = vision_rx.clone();
    // 제어 워커가 계획 중일 때는 이전 요청을 버리고 최신 한 건만 남긴다.
    let (commit_tx, commit_rx) = bounded(1);
    let commit_evict_rx = commit_rx.clone();
    let (event_tx, event_rx) = unbounded();
    let (test_control_tx, test_control_rx) = unbounded();
    let (preview_tx, preview_rx) = if options.preview {
        let (tx, rx) = bounded(PREVIEW_CAPACITY);
        (Some(tx), Some(rx))
    } else {
        (None, None)
    };
    // 실기 모드는 카메라·추정·하드웨어 제어만 실행한다. 관전용 3D sim 자식
    // 프로세스는 CPU/GPU를 점유하고 제어 지연 측정을 흐리므로 생성하지 않는다.

    let mut camera_handles: Vec<JoinHandle<CameraStats>> = Vec::with_capacity(sources.len());
    for (resolved, source) in sources {
        let camera_id = resolved.camera_id;
        let detector =
            detector_for(camera_id).with_context(|| format!("detector_for cam{}", camera_id.0))?;
        let params = camera_params_for(camera_id)
            .with_context(|| format!("camera_params_for cam{}", camera_id.0))?;
        camera_handles.push(camera_worker::spawn(
            source,
            Box::new(detector),
            params,
            vision_tx.clone(),
            vision_evict_rx.clone(),
            shutdown.clone(),
        ));
    }
    // 원본 sender를 놓아야 카메라가 모두 끝났을 때 추정 워커가 Disconnected를 본다.
    drop(vision_tx);

    let estimator_handle = estimator_worker::spawn(
        vision_rx,
        calibration,
        commit_tx,
        commit_evict_rx,
        preview_tx,
        None,
        event_tx.clone(),
        shutdown.clone(),
    );
    let control_handle = control_worker::spawn(
        Box::new(hardware),
        Arc::clone(&arm),
        commit_rx,
        test_control_rx,
        None,
        event_tx,
        shutdown,
    );

    let outcome = main_loop(&options, &event_rx, preview_rx, test_control_tx, guard);

    let camera_stats: Vec<CameraStats> = camera_handles
        .drain(..)
        .filter_map(|handle| handle.join().ok())
        .collect();
    let estimator_stats = estimator_handle.join().ok();
    if control_handle.join().is_err() {
        warn!("제어 워커 패닉");
    }
    log_summary(&outcome, &camera_stats, estimator_stats.as_ref());
    return Ok(());
}

/// 실기 기동 시 calib-rail과 같은 방식으로 논리 +X(max) 엔드스톱을 찾아 영점을 저장한다.
/// `AxlRail`만 먼저 열어 팔의 Dynamixel 정렬 상태와 무관하게 홈잉하고, 홈잉 내부에서
/// 중앙 준비 위치로 복귀한 뒤 `open_hardware`가 방금 저장한 영점을 다시 읽는다.
fn calibrate_rail_on_startup() -> Result<()> {
    let end = STARTUP_RAIL_HOME_END;
    let rail_config = RailConfig::default();
    info!(
        dll_path = %rail_config.dll_path.display(),
        end = ?end,
        logical_direction = "+X",
        "실기 시작 레일 홈잉 — +X 물리적 엔드스톱까지 저속 이동"
    );
    let mut rail = AxlRail::open(rail_config).context("시작 레일 초기화 실패")?;
    let result = rail.home(end).context("시작 레일 홈잉 실패")?;

    let measured_unix_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let calibration = RailCalibration::from_home(
        end,
        result.board_position_m,
        result.board_zero_domain_m,
        measured_unix_secs,
    );
    let path = defaults::rail::rail_calibration_path();
    calibration
        .save(&path)
        .with_context(|| format!("시작 레일 캘리브레이션 저장: {}", path.display()))?;
    info!(
        path = %path.display(),
        board_position_m = result.board_position_m,
        board_zero_domain_m = result.board_zero_domain_m,
        end = ?end,
        "실기 시작 레일 홈잉·중앙 복귀·캘리브레이션 저장 완료"
    );
    return Ok(());
}

/// 카메라·공 제어를 시작하지 않고 레일 스케일과 최종 홈 자세의 테이블 간격만 점검한다.
fn run_rail_scale_check_command(options: &Options, arm: &pingpong_bot::robot::Arm) -> Result<()> {
    ensure!(
        !options.dry_run,
        "--rail-scale-check는 실제 레일의 이동 스케일을 확인하는 명령이므로 --dry-run과 함께 사용할 수 없습니다"
    );
    ensure!(
        !options.release_torque,
        "--rail-scale-check 측정 중 자세를 유지해야 하므로 --release-torque와 함께 사용할 수 없습니다"
    );
    info!("레일 스케일 점검 명령 시작 — 카메라·공 제어는 실행하지 않음");
    calibrate_rail_on_startup()?;
    let mut hardware = open_hardware(options)?;
    let home = control_worker::initialize_pose(&mut hardware, arm)
        .map_err(|error| anyhow::anyhow!("레일 스케일 점검 홈 자세 초기화 실패: {error}"))?;
    run_startup_rail_scale_check(&mut hardware, arm)?;
    let returned_home = hardware
        .read_pose()
        .context("레일 스케일 점검 최종 홈 자세 읽기 실패")?;
    move_to_racket_table_reference(&mut hardware, arm, &returned_home)?;
    let final_home = hardware
        .read_pose()
        .context("라켓·탁구대 기준 자세 도착값 읽기 실패")?;
    log_home_racket_table_clearance(arm, &final_home)?;
    info!(
        start_rail_x = f2(home.rail_x),
        final_rail_x = f2(final_home.rail_x),
        return_error_m = f2(final_home.rail_x - home.rail_x),
        "레일 스케일 점검 동작 완료"
    );
    println!("\n최종 수직 라켓 측정 자세를 유지합니다. 측정을 마친 뒤 Enter를 누르면 종료합니다.");
    let mut input = String::new();
    std::io::stdin()
        .read_line(&mut input)
        .context("레일 스케일 점검 종료 입력 읽기 실패")?;
    info!("사용자 확인 — 레일 스케일 점검 명령 종료");
    return Ok(());
}

/// 홈 관절 가지를 유지하면서 라켓 최하단이 상판 위 1cm가 되는 기준 자세를 찾아
/// 6초 동안 천천히 이동한다. 일반 플래너의 3cm 안전 여유보다 가까운 전용 진단이라
/// 궤적 전 구간을 별도로 샘플해 실제 상판 관통·관절·속도·가속도·토크를 검사한다.
fn move_to_racket_table_reference(
    hardware: &mut dyn Hardware,
    arm: &pingpong_bot::robot::Arm,
    start: &pingpong_bot::robot::Pose,
) -> Result<()> {
    let target_joints = racket_table_reference_joints(arm, start)?;
    let zeros = vec![0.0; target_joints.values.len()];
    let trajectory = pingpong_bot::robot::motion::Trajectory::new(
        start.joints.clone(),
        target_joints,
        zeros.clone(),
        zeros,
        FINAL_RACKET_APPROACH_SECS,
        pingpong_bot::robot::motion::Rail::fixed(start.rail_x),
    );
    validate_racket_table_reference_trajectory(arm, &trajectory)?;
    info!(
        target_clearance_m = FINAL_RACKET_TABLE_CLEARANCE_M,
        duration_secs = FINAL_RACKET_APPROACH_SECS,
        "최종 라켓·탁구대 기준 자세로 저속 이동"
    );
    hardware
        .command_joints(&trajectory)
        .context("최종 라켓·탁구대 기준 자세 명령 실패")?;
    while hardware.is_busy() {
        thread::sleep(Duration::from_millis(5));
    }
    return Ok(());
}

fn racket_table_reference_joints(
    arm: &pingpong_bot::robot::Arm,
    start: &pingpong_bot::robot::Pose,
) -> Result<pingpong_bot::robot::Joints> {
    arm.rail
        .context("라켓·탁구대 기준 자세에 레일 모델이 없습니다")?;
    let mut joints = start.joints.clone();
    let start_racket = arm
        .forward_kinematics_with_rail(start.rail_x, &start.joints)
        .context("라켓·탁구대 기준 시작 FK 실패")?;
    let mut best_score = racket_table_reference_score(arm, start, &start_racket, &joints)?;
    // 위치 IK는 라켓 중심만 맞추므로 장축 롤을 고정하지 못한다. 홈 관절 가지 주변에서
    // 각 축을 점차 작은 각도로 훑어 수직도와 상판 간격을 동시에 맞춘다.
    for step_deg in [20.0_f64, 10.0, 5.0, 2.0, 1.0, 0.5, 0.2, 0.1] {
        let step = step_deg.to_radians();
        for _ in 0..32 {
            let mut improved = false;
            for index in 0..joints.values.len() {
                for direction in [-1.0, 1.0] {
                    let mut candidate = joints.clone();
                    candidate.values[index] += direction * step;
                    if !arm.joints_in_limits(&candidate) {
                        continue;
                    }
                    let score =
                        racket_table_reference_score(arm, start, &start_racket, &candidate)?;
                    if score + 1e-12 < best_score {
                        joints = candidate;
                        best_score = score;
                        improved = true;
                    }
                }
            }
            if !improved {
                break;
            }
        }
    }
    let racket = arm
        .forward_kinematics_with_rail(start.rail_x, &joints)
        .context("라켓·탁구대 기준 자세 최종 FK 실패")?;
    let model_clearance = racket_tip_clearance_m(&racket);
    let corrected_clearance = model_clearance + FINAL_RACKET_OBSERVED_HEIGHT_ERROR_M;
    ensure!(
        (corrected_clearance - FINAL_RACKET_TABLE_CLEARANCE_M).abs() <= 0.002,
        "라켓 끝 실측 보정 간격 수렴 실패: 실물 목표={:.4}m 모델={model_clearance:.4}m 보정후={corrected_clearance:.4}m",
        FINAL_RACKET_TABLE_CLEARANCE_M
    );
    let horizontal_drift_m =
        (racket.position.coords.xy() - start_racket.position.coords.xy()).norm();
    ensure!(
        horizontal_drift_m <= 0.050,
        "라켓을 내리며 너무 멀리 뻗는 자세입니다: 홈 대비 수평 이동={horizontal_drift_m:.4}m"
    );
    let vertical_error_deg = racket_blade_vertical_error_deg(&racket);
    ensure!(
        vertical_error_deg <= FINAL_RACKET_VERTICAL_TOLERANCE_DEG,
        "라켓 장축 수직 자세 수렴 실패: 수직 오차={vertical_error_deg:.2}°"
    );
    return Ok(joints);
}

fn racket_table_reference_score(
    arm: &pingpong_bot::robot::Arm,
    start: &pingpong_bot::robot::Pose,
    start_racket: &pingpong_bot::robot::RacketPose,
    joints: &pingpong_bot::robot::Joints,
) -> Result<f64> {
    let racket = arm
        .forward_kinematics_with_rail(start.rail_x, joints)
        .context("라켓·탁구대 기준 후보 FK 실패")?;
    let clearance_error =
        (racket_tip_clearance_m(&racket) - FINAL_RACKET_MODEL_CLEARANCE_M) / 0.002;
    let vertical_error =
        racket_blade_vertical_error_deg(&racket) / FINAL_RACKET_VERTICAL_TOLERANCE_DEG;
    // 홈 위치를 적극적으로 유지해 라켓을 앞쪽으로 길게 뻗으며 내리는 해를 피한다.
    let horizontal_drift =
        (racket.position.coords.xy() - start_racket.position.coords.xy()).norm() / 0.01;
    let joint_drift = joints
        .values
        .iter()
        .zip(&start.joints.values)
        .map(|(candidate, home)| ((candidate - home) / 20.0_f64.to_radians()).powi(2))
        .sum::<f64>();
    return Ok(clearance_error * clearance_error
        + vertical_error * vertical_error
        + 0.18 * horizontal_drift * horizontal_drift
        + 0.02 * joint_drift);
}

fn validate_racket_table_reference_trajectory(
    arm: &pingpong_bot::robot::Arm,
    trajectory: &pingpong_bot::robot::motion::Trajectory,
) -> Result<()> {
    ensure!(
        trajectory.peak_joint_speed() <= arm.max_joint_speed,
        "라켓·탁구대 기준 자세 궤적이 관절 속도 한계를 초과합니다"
    );
    ensure!(
        trajectory.peak_joint_acceleration()
            <= pingpong_bot::defaults::ControlParams::default().max_joint_accel,
        "라켓·탁구대 기준 자세 궤적이 관절 가속도 한계를 초과합니다"
    );
    for index in 0..=120 {
        let time = trajectory.duration_secs * index as f64 / 120.0;
        let joints = trajectory.sample_at(time);
        ensure!(
            arm.joints_in_limits(&joints),
            "라켓·탁구대 기준 자세 궤적이 관절 한계를 벗어납니다: t={time:.3}s"
        );
        let rail_x = trajectory.sample_rail_at(time);
        let boxes = pingpong_bot::robot::collision::robot_obbs(arm, rail_x, &joints);
        let link_depth = boxes
            .iter()
            .take(boxes.len().saturating_sub(1))
            .map(pingpong_bot::robot::OrientedBox::table_penetration)
            .fold(0.0_f64, f64::max);
        ensure!(
            link_depth <= 0.001,
            "라켓 접근 중 전완 링크가 기존 3cm 안전영역을 침범합니다: t={time:.3}s safety_depth={link_depth:.4}m"
        );
        let racket = arm
            .forward_kinematics_with_rail(rail_x, &joints)
            .context("라켓 접근 궤적 FK 실패")?;
        let corrected_clearance =
            racket_tip_clearance_m(&racket) + FINAL_RACKET_OBSERVED_HEIGHT_ERROR_M;
        ensure!(
            corrected_clearance >= FINAL_RACKET_TABLE_CLEARANCE_M - 0.002,
            "실측 보정 기준 라켓이 상판에 너무 가까워집니다: t={time:.3}s corrected_clearance={corrected_clearance:.4}m"
        );
        let velocity = trajectory.sample_velocity_at(time);
        let acceleration = trajectory.sample_acceleration_at(time);
        let torque = arm
            .required_torque_with_rotor(&joints.values, &velocity, &acceleration)
            .context("라켓·탁구대 기준 자세 토크 계산 실패")?;
        ensure!(
            pingpong_bot::robot::Arm::torque_feasible(&torque, &arm.joint_torque_limits),
            "라켓·탁구대 기준 자세 궤적이 토크 한계를 초과합니다: t={time:.3}s"
        );
    }
    return Ok(());
}

/// 홈잉·시작 자세 초기화가 끝난 레일을 -X/+X 안전 마진 끝으로
/// 이동하고 양쪽 마진 끝에서 사용자가 Enter를 누를 때까지 유지한다.
/// 마지막에는 중앙으로 돌아온다.
fn run_startup_rail_scale_check(
    hardware: &mut dyn Hardware,
    arm: &pingpong_bot::robot::Arm,
) -> Result<()> {
    let rail = arm
        .rail
        .context("시작 레일 스케일 점검에 레일 모델이 없습니다")?;
    let start_x = hardware
        .read_pose()
        .context("시작 레일 스케일 점검 기준 위치 읽기 실패")?
        .rail_x;
    let (negative_target_x, center_target_x, positive_target_x) =
        startup_rail_margin_targets(start_x, rail.x_min, rail.x_max)?;
    let mut from_x = start_x;

    for (direction, target_x, wait_for_measurement) in [
        ("-X 안전 마진 끝", negative_target_x, true),
        ("+X 중앙 복귀", center_target_x, false),
        ("+X 안전 마진 끝", positive_target_x, true),
        ("-X 중앙 복귀", center_target_x, false),
    ] {
        info!(
            direction,
            start_x = f2(from_x),
            target_x = f2(target_x),
            distance_m = (target_x - from_x).abs(),
            move_secs = STARTUP_RAIL_SCALE_CHECK_MOVE_SECS,
            "시작 레일 스케일 점검 이동"
        );
        let applied_x = hardware
            .command_rail(target_x, STARTUP_RAIL_SCALE_CHECK_MOVE_SECS)
            .with_context(|| format!("시작 레일 마진 점검 {direction} 이동 실패"))?;
        ensure!(
            (applied_x - target_x).abs() <= 1e-6,
            "시작 레일 스케일 점검 목표가 안전 범위에서 변경됨: 요청={target_x:.4}m 적용={applied_x:.4}m"
        );
        thread::sleep(Duration::from_secs_f64(STARTUP_RAIL_SCALE_CHECK_MOVE_SECS));
        let measured_x = hardware
            .read_pose()
            .with_context(|| format!("시작 레일 스케일 점검 {direction} 도착 위치 읽기 실패"))?
            .rail_x;
        info!(
            direction,
            target_x = f2(target_x),
            measured_x = f2(measured_x),
            error_m = f2(measured_x - target_x),
            wait_for_measurement,
            "시작 레일 마진 점검 도착"
        );
        from_x = measured_x;
        if wait_for_measurement {
            println!(
                "\n{direction} 도착: 레일 마진 끝과 탁구대 끝 거리를 측정한 뒤 Enter를 누르세요."
            );
            let mut input = String::new();
            std::io::stdin()
                .read_line(&mut input)
                .with_context(|| format!("레일 마진 점검 {direction} 확인 입력 읽기 실패"))?;
        }
    }
    return Ok(());
}

/// 최종 홈 자세에서 라켓 OBB의 물리적 최하단과 탁구대 윗면 간격을 계산한다.
fn log_home_racket_table_clearance(
    arm: &pingpong_bot::robot::Arm,
    pose: &pingpong_bot::robot::Pose,
) -> Result<()> {
    let racket = arm
        .forward_kinematics_with_rail(pose.rail_x, &pose.joints)
        .context("최종 홈 자세 라켓 FK 실패")?;
    let model_clearance_m = racket_tip_clearance_m(&racket);
    let corrected_clearance_m = model_clearance_m + FINAL_RACKET_OBSERVED_HEIGHT_ERROR_M;
    let table_z = pingpong_bot::constants::table::SURFACE_Z;
    let racket_tip_z = table_z + model_clearance_m;
    let robot_table_penetration_m =
        pingpong_bot::robot::collision::table_penetration(arm, pose.rail_x, &pose.joints);
    info!(
        rail_x = f2(pose.rail_x),
        joints_deg = %format!("{:?}", pose.joints.values.iter().map(|angle| angle.to_degrees()).collect::<Vec<_>>()),
        racket_center_z_m = f2(racket.position.z),
        racket_tip_z_m = f2(racket_tip_z),
        table_surface_z_m = f2(table_z),
        racket_tip_model_clearance_m = f2(model_clearance_m),
        racket_tip_corrected_clearance_m = f2(corrected_clearance_m),
        observed_height_error_m = FINAL_RACKET_OBSERVED_HEIGHT_ERROR_M,
        racket_tip_contacts_table = corrected_clearance_m <= 0.0,
        racket_blade_vertical_error_deg = f2(racket_blade_vertical_error_deg(&racket)),
        robot_table_safety_penetration_m = f2(robot_table_penetration_m),
        "최종 홈 자세 라켓 끝·탁구대 충돌 계산"
    );
    return Ok(());
}

fn racket_blade_vertical_error_deg(racket: &pingpong_bot::robot::RacketPose) -> f64 {
    let [w, x, y, z] = racket.orientation;
    let rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z));
    let blade_axis = rotation * nalgebra::Vector3::y();
    return blade_axis.z.abs().clamp(-1.0, 1.0).acos().to_degrees();
}

fn racket_tip_clearance_m(racket: &pingpong_bot::robot::RacketPose) -> f64 {
    let [w, x, y, z] = racket.orientation;
    let rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z));
    let axis_x = rotation * nalgebra::Vector3::x();
    let axis_y = rotation * nalgebra::Vector3::y();
    let axis_z = rotation * nalgebra::Vector3::z();
    let vertical_half_extent = axis_x.z.abs() * pingpong_bot::constants::geometry::RACKET_HALF_X
        + axis_y.z.abs() * pingpong_bot::constants::geometry::RACKET_HALF_Y
        + axis_z.z.abs() * pingpong_bot::constants::geometry::RACKET_HALF_Z;
    return racket.position.z - vertical_half_extent - pingpong_bot::constants::table::SURFACE_Z;
}

fn startup_rail_margin_targets(start_x: f64, x_min: f64, x_max: f64) -> Result<(f64, f64, f64)> {
    ensure!(start_x.is_finite(), "시작 레일 위치가 유한값이 아닙니다");
    ensure!(
        x_min.is_finite() && x_max.is_finite() && x_min < x_max,
        "레일 안전 범위가 잘못됐습니다: [{x_min:.4}, {x_max:.4}]m"
    );
    ensure!(
        start_x >= x_min && start_x <= x_max,
        "시작 레일 위치 {start_x:.4}m가 안전 범위 [{x_min:.4}, {x_max:.4}]m 밖입니다"
    );
    return Ok((x_min, start_x, x_max));
}

#[cfg(test)]
mod startup_rail_tests {
    use super::*;

    #[test]
    fn startup_homing_targets_positive_x_end() {
        assert_eq!(STARTUP_RAIL_HOME_END, RailEnd::Max);
    }

    #[test]
    fn startup_scale_check_moves_to_both_safe_margin_ends() {
        let (negative, returned, positive) = startup_rail_margin_targets(
            defaults::RAIL_READY_X_M,
            defaults::RAIL_X_MIN_M,
            defaults::RAIL_X_MAX_M,
        )
        .expect("scale check targets");
        assert!((negative - defaults::RAIL_X_MIN_M).abs() < 1e-12);
        assert!((returned - defaults::RAIL_READY_X_M).abs() < 1e-12);
        assert!((positive - defaults::RAIL_X_MAX_M).abs() < 1e-12);
    }

    #[test]
    fn startup_scale_check_rejects_start_outside_safe_range() {
        let error = startup_rail_margin_targets(
            defaults::RAIL_X_MAX_M - 0.10,
            defaults::RAIL_X_MIN_M,
            defaults::RAIL_X_MAX_M - 0.20,
        )
        .expect_err("unsafe start");
        assert!(error.to_string().contains("시작 레일 위치"));
    }

    #[test]
    fn final_reference_pose_places_racket_tip_one_centimeter_above_table() {
        let robot = defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let start = pingpong_bot::robot::Pose::new(
            rail_x,
            pingpong_bot::robot::Joints::from_slice(&pingpong_bot::defaults::READY_JOINTS_4DOF),
        );
        let joints = racket_table_reference_joints(&robot.arm, &start).expect("reference joints");
        let racket = robot
            .arm
            .forward_kinematics_with_rail(rail_x, &joints)
            .expect("reference FK");
        let model_clearance = racket_tip_clearance_m(&racket);
        let corrected_clearance = model_clearance + FINAL_RACKET_OBSERVED_HEIGHT_ERROR_M;
        assert!(
            (corrected_clearance - FINAL_RACKET_TABLE_CLEARANCE_M).abs() <= 0.002,
            "model={model_clearance:.4} corrected={corrected_clearance:.4}"
        );
        assert!(racket_blade_vertical_error_deg(&racket) <= FINAL_RACKET_VERTICAL_TOLERANCE_DEG);
        let zeros = vec![0.0; joints.values.len()];
        let trajectory = pingpong_bot::robot::motion::Trajectory::new(
            start.joints.clone(),
            joints,
            zeros.clone(),
            zeros,
            FINAL_RACKET_APPROACH_SECS,
            pingpong_bot::robot::motion::Rail::fixed(rail_x),
        );
        validate_racket_table_reference_trajectory(&robot.arm, &trajectory)
            .expect("reference trajectory must remain physically safe");
    }
}

/// 세션 요약용 — 추적한 공과 보낸 제어 명령 수.
struct Outcome {
    tracks_seen: u64,
    commands_sent: u64,
    last: LastState,
}

enum LastState {
    None,
    Commanded,
    Failed(String),
    TimedOut,
    Quit,
}

impl Outcome {
    fn label(&self) -> String {
        let last = match &self.last {
            LastState::None => "없음".to_owned(),
            LastState::Commanded => "레일·라켓 조준 명령".to_owned(),
            LastState::Failed(reason) => format!("실패 - {reason}"),
            LastState::TimedOut => "타임아웃 - 공이 오지 않음".to_owned(),
            LastState::Quit => "사용자 종료".to_owned(),
        };
        return format!(
            "tracks={} commands={} last={last}",
            self.tracks_seen, self.commands_sent
        );
    }
}

/// 런타임 이벤트를 찍고 프리뷰를 돌린다.
fn main_loop(
    options: &Options,
    event_rx: &Receiver<RuntimeEvent>,
    preview_rx: Option<Receiver<PreviewEvent>>,
    test_control_tx: Sender<TestControl>,
    guard: ShutdownGuard,
) -> Outcome {
    let mut preview = options.preview.then(|| PreviewWindow::new("real shot"));
    let mut guard = Some(guard);
    let mut wait_deadline = Instant::now() + Duration::from_secs_f64(options.timeout_secs);
    let mut outcome = Outcome {
        tracks_seen: 0,
        commands_sent: 0,
        last: LastState::None,
    };
    let mut timed_out_warned = false;

    let result = loop {
        let mut control_done = false;
        while let Ok(event) = event_rx.try_recv() {
            log_event(&event);
            if let Some(preview) = &mut preview
                && let Some(lines) = result_lines(&event)
            {
                preview.set_result(lines);
            }
            match &event {
                RuntimeEvent::Ready { .. } => {
                    wait_deadline = Instant::now() + Duration::from_secs_f64(options.timeout_secs);
                    timed_out_warned = false;
                }
                RuntimeEvent::Tracking { track_seq, .. } => {
                    outcome.tracks_seen = outcome.tracks_seen.max(*track_seq);
                    wait_deadline = Instant::now() + Duration::from_secs_f64(options.timeout_secs);
                    timed_out_warned = false;
                }
                RuntimeEvent::Commanded { .. } => {
                    outcome.commands_sent += 1;
                    outcome.last = LastState::Commanded;
                }
                RuntimeEvent::ControlState { state } => {
                    if let Some(preview) = &mut preview {
                        preview.set_control_state(*state);
                    }
                }
                RuntimeEvent::TestZoneChanged {
                    zone,
                    home_rail_x,
                    filtering,
                } => {
                    if let Some(preview) = &mut preview {
                        preview.set_zone(*zone, *home_rail_x, *filtering);
                    }
                }
                RuntimeEvent::Failed { reason, .. } => {
                    outcome.last = LastState::Failed(reason.clone());
                }
                RuntimeEvent::Done => control_done = true,
            }
        }

        if control_done {
            if !options.preview {
                break outcome;
            }
            // preview: Done이어도 창이 있으면 ESC까지 화면 유지. 워커는 이미 끝.
        }

        if !timed_out_warned && Instant::now() >= wait_deadline {
            warn!(
                timeout_secs = f2(options.timeout_secs),
                "공을 기다리다 시간 초과 — 세션은 유지"
            );
            outcome.last = LastState::TimedOut;
            timed_out_warned = true;
        }

        match &mut preview {
            Some(preview) => {
                if let Some(rx) = &preview_rx {
                    while let Ok(event) = rx.try_recv() {
                        preview.push(event);
                    }
                }
                match preview.show() {
                    PreviewAction::Quit => {
                        outcome.last = LastState::Quit;
                        break outcome;
                    }
                    PreviewAction::Key(key) => {
                        if let Some(control) = TestControl::from_key(key) {
                            let _ = test_control_tx.send(control);
                        }
                    }
                    PreviewAction::Continue => {}
                }
            }
            None => {
                if control_done {
                    break outcome;
                }
                thread::sleep(IDLE_TICK);
            }
        }
    };

    drop(guard.take());
    if let Some(preview) = &preview {
        preview.close();
    }
    return result;
}

/// 최근 제어 결과 HUD (ASCII — Hershey 폰트 제약).
fn result_lines(event: &RuntimeEvent) -> Option<Vec<String>> {
    return match event {
        RuntimeEvent::Commanded {
            track_seq,
            target,
            rail_x,
            aim_rad,
        } => Some(vec![
            format!("COMMAND track {track_seq}"),
            format!(
                "target x{} y{} z{}",
                f2(target.coords.x),
                f2(target.coords.y),
                f2(target.coords.z)
            ),
            format!("rail {}  aim {} deg", f2(*rail_x), f2(aim_rad.to_degrees())),
        ]),
        RuntimeEvent::Failed { track_seq, reason } => {
            Some(vec![format!("FAILED track {track_seq:?}"), reason.clone()])
        }
        _ => None,
    };
}

fn open_hardware(options: &Options) -> Result<RealHardware> {
    let mut dxl = DynamixelConfig::default();
    if let Some(port) = &options.dxl_port {
        dxl.port = port.clone();
    }
    dxl.hold_torque_on_close = !options.release_torque;
    let mut rail = RailConfig::default();
    let calibration_path = defaults::rail::rail_calibration_path();
    if let Some(calibration) = RailCalibration::load(&calibration_path) {
        info!(
            path = %calibration_path.display(),
            board_zero_domain_m = calibration.board_zero_domain_m,
            "레일 캘리브레이션 파일 적용"
        );
        calibration.apply_to(&mut rail);
    }
    info!(
        port = %dxl.port,
        dry_run = options.dry_run,
        rail_enabled = rail.enabled,
        hold_torque_on_close = dxl.hold_torque_on_close,
        "real 하드웨어 (mirror ID1↔ID2)"
    );
    let hardware = if options.dry_run {
        RealHardware::dry_run(dxl, Some(rail))
    } else {
        RealHardware::new(dxl, Some(rail))
    };
    return hardware.context("하드웨어 초기화");
}

fn load_calibration() -> Result<Calibration> {
    let path = defaults::calibration_path();
    let calibration = Calibration::load_json(&path)
        .map_err(anyhow::Error::msg)
        .with_context(|| format!("calibration 로드: {}", path.display()))?;
    info!(
        cameras = calibration.camera_count(),
        path = %path.display(),
        "calibration"
    );
    return Ok(calibration);
}

type OpenedCameras = Vec<(
    pingpong_bot::camera::ResolvedCam,
    Box<dyn pingpong_bot::camera::FrameSource>,
)>;

/// 라이브 캠, 또는 `--clip`이면 녹화 클립.
///
/// 클립은 **녹화 당시 fps로 페이싱**해서 재생한다 ([`PacedSource`]) — 그래야 계획 스로틀·
/// 요청 신선도·하드웨어 `stream_hz` 같은 벽시계 로직이 라이브와 같은 조건에서 돈다.
fn open_cameras(options: &Options) -> Result<OpenedCameras> {
    let cams = CamCliArgs {
        cam: DEFAULT_STEREO_CAM_ROLES.to_vec(),
        stream: CamStreamArgs::default(),
    };
    let Some(clip) = &options.clip else {
        return cams
            .open_sources()
            .map_err(anyhow::Error::msg)
            .context("실캠 열기");
    };

    let offline = StereoOfflineArgs {
        clip: Some(clip.clone()),
    };
    let resolved = offline
        .resolve()
        .map_err(anyhow::Error::msg)
        .context("클립 해석")?
        .context("클립을 찾지 못했다")?;
    resolved.log();
    info!(
        dir = %resolved.dir.display(),
        meas_fps = resolved.meas_fps.map(f2),
        "클립 재생 — 라이브 캠 대신"
    );

    // 파일 소스는 `--cam` 역할 순서대로 camera::Id를 받는다 (left → Id(0), right → Id(1)).
    let sources = cams
        .open_file_sources(&resolved.paths(), resolved.meas_fps)
        .map_err(anyhow::Error::msg)
        .context("클립 열기")?;
    let resolved_cams = cams.resolve().map_err(anyhow::Error::msg)?;
    return Ok(resolved_cams
        .into_iter()
        .zip(sources)
        .map(|(cam, source)| {
            let paced: Box<dyn pingpong_bot::camera::FrameSource> =
                Box::new(PacedSource::new(source));
            (cam, paced)
        })
        .collect());
}

fn log_event(event: &RuntimeEvent) {
    match event {
        RuntimeEvent::Ready { pose } => info!(
            rail_x = f2(pose.rail_x),
            joints = f2_slice(&pose.joints.values),
            "실기 단순 제어 준비"
        ),
        RuntimeEvent::Tracking {
            track_seq,
            position,
            speed,
        } => info!(
            track = track_seq,
            x = f2(position.coords.x),
            y = f2(position.coords.y),
            z = f2(position.coords.z),
            speed = f2(*speed),
            "공 궤적 추적 시작"
        ),
        RuntimeEvent::Commanded {
            track_seq,
            target,
            rail_x,
            aim_rad,
        } => info!(
            track = track_seq,
            target_x = f2(target.coords.x),
            target_y = f2(target.coords.y),
            target_z = f2(target.coords.z),
            rail_x = f2(*rail_x),
            aim_deg = f2(aim_rad.to_degrees()),
            "레일·라켓 조준 명령 전송"
        ),
        RuntimeEvent::ControlState { state } => debug!(?state, "제어 상태 전이"),
        RuntimeEvent::TestZoneChanged {
            zone,
            home_rail_x,
            filtering,
        } => info!(
            ?zone,
            home_rail_x = f2(*home_rail_x),
            filtering,
            "제어 모드 변경 — 준비 자세 레일 x·존 필터 갱신"
        ),
        RuntimeEvent::Failed { track_seq, reason } => {
            warn!(track = track_seq, reason, "실기 단순 제어 실패")
        }
        RuntimeEvent::Done => {}
    }
}

fn log_summary(outcome: &Outcome, cameras: &[CameraStats], estimator: Option<&EstimatorStats>) {
    for stats in cameras {
        info!(
            cam = stats.camera_id,
            frames = stats.frames,
            detections = stats.detections,
            detection_rate = f2(stats.detection_rate()),
            dropped = stats.dropped,
            undistort_failures = stats.undistort_failures,
            "real shot: end — 카메라"
        );
    }
    if let Some(stats) = estimator {
        info!(
            accepted = stats.accepted,
            rejected = stats.rejected,
            seeded = stats.seeded,
            commit_dropped = stats.commit_dropped,
            preview_dropped = stats.preview_dropped,
            "real shot: end — 추정"
        );
    }
    info!(outcome = outcome.label(), "real shot: end");
}
