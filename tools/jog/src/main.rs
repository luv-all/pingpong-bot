//! 인터랙티브 조그 REPL — Dynamixel + AXL 레일, FK/IK/임팩트 속도 스윙.
//!
//! planner(`plan_swing`)는 쓰지 않는다. 목표 pose → quintic → `Hardware::command`.

use std::io::{self, Write};
use std::path::PathBuf;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result, bail, ensure};
use clap::Parser;
use nalgebra::Vector3;
use pingpong_bot::{
    Arm, Hardware, Joints, Point3, RailMotion, RealHardware, RobotPose, SwingTrajectory, control,
    dynamixel, rail, robot,
};

#[derive(Parser, Debug)]
#[command(name = "jog", about = "관절·레일 인터랙티브 조그 REPL")]
struct Args {
    /// Dynamixel 시리얼 포트 (`defaults::dynamixel().port` 덮어씀).
    #[arg(long)]
    port: Option<String>,
    /// AXL.dll 경로 (`defaults::rail().dll_path` 덮어씀).
    #[arg(long)]
    dll_path: Option<PathBuf>,
    /// 시리얼·DLL 없이 변환·IK·executor만.
    #[arg(long)]
    dry_run: bool,
}

struct Session {
    arm: Arc<Arm>,
    hardware: RealHardware,
    duration_secs: f64,
    max_delta_deg: f64,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive("info".parse().unwrap_or_default()),
        )
        .init();
    return run(Args::parse());
}

fn run(args: Args) -> Result<()> {
    let mut dxl = dynamixel();
    if let Some(port) = &args.port {
        dxl.port = port.clone();
    }
    let mut rail_cfg = rail();
    if let Some(dll_path) = args.dll_path {
        rail_cfg.dll_path = dll_path;
    }

    let hardware = if args.dry_run {
        RealHardware::dry_run(dxl, Some(rail_cfg))
    } else {
        RealHardware::new(dxl, Some(rail_cfg))
    }
    .context("하드웨어 초기화 실패")?;

    let robot = robot().context("defaults::robot")?;
    let mut session = Session {
        arm: robot.arm,
        hardware,
        duration_secs: 1.0,
        max_delta_deg: 15.0,
    };

    println!("jog REPL — help 로 명령 목록. q 로 종료.");
    print_status(&mut session)?;

    let stdin = io::stdin();
    loop {
        print!("jog> ");
        io::stdout().flush()?;
        let mut line = String::new();
        if stdin.read_line(&mut line)? == 0 {
            break;
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        match handle_line(&mut session, line) {
            Ok(true) => break,
            Ok(false) => {}
            Err(err) => eprintln!("error: {err:#}"),
        }
    }
    return Ok(());
}

/// `true`면 종료.
fn handle_line(session: &mut Session, line: &str) -> Result<bool> {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    let cmd = tokens[0].to_ascii_lowercase();
    match cmd.as_str() {
        "q" | "quit" | "exit" => return Ok(true),
        "help" | "h" | "?" => {
            print_help();
            return Ok(false);
        }
        "status" | "s" => {
            print_status(session)?;
            return Ok(false);
        }
        "duration" => {
            ensure!(tokens.len() == 2, "usage: duration <secs>");
            let secs: f64 = tokens[1].parse().context("duration")?;
            ensure!(secs.is_finite() && secs > 0.0, "duration > 0");
            session.duration_secs = secs;
            println!("duration_secs={}", session.duration_secs);
            return Ok(false);
        }
        "maxdelta" => {
            ensure!(tokens.len() == 2, "usage: maxdelta <deg>");
            let deg: f64 = tokens[1].parse().context("maxdelta")?;
            ensure!(deg.is_finite() && deg > 0.0, "maxdelta > 0");
            session.max_delta_deg = deg;
            println!("max_delta_deg={}", session.max_delta_deg);
            return Ok(false);
        }
        "j" => {
            ensure!(tokens.len() == 3, "usage: j <index> <deg>");
            let index: usize = tokens[1].parse().context("joint index")?;
            let deg: f64 = tokens[2].parse().context("angle deg")?;
            ensure!(deg.is_finite(), "angle finite");
            let start = session.hardware.read_pose().context("read_pose")?;
            ensure!(index < start.joints.values.len(), "joint out of range");
            let mut target = start.joints.values.clone();
            target[index] = deg.to_radians();
            move_joints_rail(session, &start, Joints::from_slice(&target), start.rail_x)?;
            print_status(session)?;
            return Ok(false);
        }
        "angles" => {
            ensure!(tokens.len() == 2, "usage: angles a0,a1,a2,a3");
            let degs = parse_csv_f64(tokens[1])?;
            let start = session.hardware.read_pose().context("read_pose")?;
            ensure!(
                degs.len() == start.joints.values.len(),
                "need {} angles",
                start.joints.values.len()
            );
            let rads: Vec<f64> = degs.iter().map(|d| d.to_radians()).collect();
            move_joints_rail(session, &start, Joints::from_slice(&rads), start.rail_x)?;
            print_status(session)?;
            return Ok(false);
        }
        "r" => {
            ensure!(tokens.len() == 2, "usage: r <x_m>");
            let x: f64 = tokens[1].parse().context("rail x")?;
            ensure!(x.is_finite(), "rail finite");
            let start = session.hardware.read_pose().context("read_pose")?;
            move_joints_rail(session, &start, start.joints.clone(), x)?;
            print_status(session)?;
            return Ok(false);
        }
        "rd" => {
            ensure!(tokens.len() == 2, "usage: rd <delta_m>");
            let dx: f64 = tokens[1].parse().context("rail delta")?;
            ensure!(dx.is_finite(), "delta finite");
            let start = session.hardware.read_pose().context("read_pose")?;
            move_joints_rail(session, &start, start.joints.clone(), start.rail_x + dx)?;
            print_status(session)?;
            return Ok(false);
        }
        "ik" => {
            ensure!(tokens.len() == 4, "usage: ik <x> <y> <z>");
            let target = parse_point3(&tokens[1..4])?;
            let start = session.hardware.read_pose().context("read_pose")?;
            let linear = session
                .arm
                .rail
                .ok_or_else(|| anyhow::anyhow!("arm has no linear rail"))?;
            let joints = session
                .arm
                .inverse_kinematics_with_rail(&linear, start.rail_x, target, Some(&start.joints))
                .context("ik")?;
            // 레일은 현재 x 유지; 필요하면 사용자가 r로 맞춤.
            move_joints_rail(session, &start, joints, start.rail_x)?;
            print_status(session)?;
            return Ok(false);
        }
        "pose" => {
            ensure!(tokens.len() == 7, "usage: pose <x> <y> <z> <nx> <ny> <nz>");
            let target = parse_point3(&tokens[1..4])?;
            let normal = parse_vec3(&tokens[4..7])?;
            let start = session.hardware.read_pose().context("read_pose")?;
            let solved = session
                .arm
                .inverse_pose_with_rail(target, normal, &start)
                .context("pose ik")?;
            move_joints_rail(session, &start, solved.joints.clone(), solved.rail_x)?;
            print_status(session)?;
            return Ok(false);
        }
        "swing" => {
            run_swing(session, &tokens[1..])?;
            print_status(session)?;
            return Ok(false);
        }
        other => bail!("unknown command `{other}` — help"),
    }
}

fn run_swing(session: &mut Session, args: &[&str]) -> Result<()> {
    // swing x y z [nx ny nz] speed <v>
    ensure!(
        args.len() == 5 || args.len() == 8,
        "usage: swing <x> <y> <z> [nx ny nz] speed <m/s>"
    );
    let target = parse_point3(&args[0..3])?;
    let (normal_opt, speed_tok) = if args.len() == 8 {
        (Some(parse_vec3(&args[3..6])?), &args[6..])
    } else {
        (None, &args[3..])
    };
    ensure!(speed_tok.len() == 2 && speed_tok[0].eq_ignore_ascii_case("speed"), "need speed <v>");
    let speed: f64 = speed_tok[1].parse().context("speed")?;
    ensure!(speed.is_finite() && speed > 0.0, "speed > 0");

    let start = session.hardware.read_pose().context("read_pose")?;
    let impact = if let Some(normal) = normal_opt {
        session
            .arm
            .inverse_pose_with_rail(target, normal, &start)
            .context("swing pose ik")?
    } else {
        let linear = session
            .arm
            .rail
            .ok_or_else(|| anyhow::anyhow!("arm has no linear rail"))?;
        let joints = session
            .arm
            .inverse_kinematics_with_rail(&linear, start.rail_x, target, Some(&start.joints))
            .context("swing ik")?;
        RobotPose::new(start.rail_x, joints)
    };

    let racket = session
        .arm
        .forward_kinematics_with_rail(impact.rail_x, &impact.joints)
        .context("fk at impact")?;
    let normal = racket.normal.normalize();
    let v_r = normal * speed;
    let (rail_impact_vel, joint_impact_vel) = session
        .arm
        .velocities_for_racket_velocity(&impact, v_r)
        .context("joint velocities for racket speed")?;

    ensure_max_delta(session, &start.joints, &impact.joints)?;

    let follow = control().swing_follow_through_secs.max(0.02);
    let approach = session.duration_secs.max(follow + 0.05);
    let impact_time = (approach - follow).max(0.05);
    let duration = impact_time + follow;

    let n = impact.joints.values.len();
    let mut follow_joints = Vec::with_capacity(n);
    for i in 0..n {
        follow_joints.push(impact.joints.values[i] + joint_impact_vel[i] * follow);
    }
    let follow_rail = impact.rail_x + rail_impact_vel * follow;
    let start_vel = vec![0.0; n];
    let follow_vel = vec![0.0; n];

    let trajectory = SwingTrajectory::with_follow_through(
        start.joints.clone(),
        impact.joints.clone(),
        Joints::from_slice(&follow_joints),
        start_vel,
        joint_impact_vel,
        follow_vel,
        impact_time,
        duration,
        RailMotion {
            start: start.rail_x,
            end: impact.rail_x,
            start_velocity: 0.0,
            end_velocity: rail_impact_vel,
        },
        follow_rail,
        0.0,
    );

    println!(
        "swing impact_t={impact_time:.3}s duration={duration:.3}s speed={speed:.3} m/s along normal"
    );
    session
        .hardware
        .command(&trajectory)
        .context("command swing")?;
    wait_idle(session)?;
    return Ok(());
}

fn move_joints_rail(
    session: &mut Session,
    start: &RobotPose,
    target_joints: Joints,
    target_rail: f64,
) -> Result<()> {
    ensure_max_delta(session, &start.joints, &target_joints)?;
    let n = target_joints.values.len();
    let trajectory = SwingTrajectory::new(
        start.joints.clone(),
        target_joints,
        vec![0.0; n],
        vec![0.0; n],
        session.duration_secs,
        RailMotion {
            start: start.rail_x,
            end: target_rail,
            start_velocity: 0.0,
            end_velocity: 0.0,
        },
    );
    session
        .hardware
        .command(&trajectory)
        .context("command move")?;
    wait_idle(session)?;
    return Ok(());
}

fn ensure_max_delta(session: &Session, from: &Joints, to: &Joints) -> Result<()> {
    let max_delta = session.max_delta_deg.to_radians();
    for (index, (a, b)) in from.values.iter().zip(&to.values).enumerate() {
        ensure!(
            (b - a).abs() <= max_delta,
            "joint {index} Δ {:.1}° > maxdelta {}",
            (b - a).abs().to_degrees(),
            session.max_delta_deg
        );
    }
    return Ok(());
}

fn wait_idle(session: &mut Session) -> Result<()> {
    while session.hardware.is_busy() {
        thread::sleep(Duration::from_millis(10));
    }
    return Ok(());
}

fn print_status(session: &mut Session) -> Result<()> {
    let pose = session.hardware.read_pose().context("read_pose")?;
    let fk = session
        .arm
        .forward_kinematics_with_rail(pose.rail_x, &pose.joints);
    let degs: Vec<String> = pose
        .joints
        .values
        .iter()
        .map(|r| format!("{:.2}", r.to_degrees()))
        .collect();
    println!(
        "rail_x={:.4} m  joints_deg=[{}]  duration={}s  maxdelta={}°",
        pose.rail_x,
        degs.join(", "),
        session.duration_secs,
        session.max_delta_deg
    );
    if let Some(racket) = fk {
        println!(
            "fk pos=({:.4}, {:.4}, {:.4})  n=({:.3}, {:.3}, {:.3})",
            racket.position.coords.x,
            racket.position.coords.y,
            racket.position.coords.z,
            racket.normal.x,
            racket.normal.y,
            racket.normal.z
        );
    } else {
        println!("fk: unreachable");
    }
    return Ok(());
}

fn print_help() {
    println!(
        "\n\
commands:\n\
  status                 현재 관절·레일·FK\n\
  j <i> <deg>            단축 목표 [deg]\n\
  angles a0,a1,a2,a3     전축 목표 [deg]\n\
  r <x_m> / rd <dx>      레일 절대 / 상대 [m]\n\
  ik <x> <y> <z>         위치 IK (레일 x 유지)\n\
  pose x y z nx ny nz    위치+법선 IK\n\
  swing x y z [n…] speed <v>   임팩트 속도 [m/s] 스윙\n\
  duration <s>           기본 이동 시간\n\
  maxdelta <deg>         관절 최대 Δ (안전)\n\
  help / q\n"
    );
}

fn parse_csv_f64(s: &str) -> Result<Vec<f64>> {
    return s
        .split(',')
        .map(|part| {
            let v: f64 = part.trim().parse().context("csv f64")?;
            ensure!(v.is_finite(), "finite");
            Ok(v)
        })
        .collect();
}

fn parse_point3(parts: &[&str]) -> Result<Point3> {
    let v = parse_vec3(parts)?;
    return Ok(Point3::new(v.x, v.y, v.z));
}

fn parse_vec3(parts: &[&str]) -> Result<Vector3<f64>> {
    ensure!(parts.len() == 3, "need 3 numbers");
    let x: f64 = parts[0].parse().context("x")?;
    let y: f64 = parts[1].parse().context("y")?;
    let z: f64 = parts[2].parse().context("z")?;
    ensure!(x.is_finite() && y.is_finite() && z.is_finite(), "finite");
    return Ok(Vector3::new(x, y, z));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_angles_csv() {
        let v = parse_csv_f64("0, -15, 10.5, 0").unwrap();
        assert_eq!(v.len(), 4);
        assert!((v[1] + 15.0).abs() < 1e-12);
    }
}
