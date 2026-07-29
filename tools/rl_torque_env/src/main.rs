//! JSON-lines bridge between the Rust Rapier environment and Gymnasium/SB3.

use std::io::{BufRead, Write};

use anyhow::{Context, Result};
use clap::Parser;
use pingpong_bot::{BallShooterSettings, TorqueResidualEnv, defaults};
use rand::SeedableRng;
use rand::rngs::StdRng;
use serde::{Deserialize, Serialize};

#[derive(Parser, Debug)]
#[command(about = "residual-torque RL environment JSONL bridge")]
struct Args {
    #[arg(long, default_value_t = 20260729)]
    seed: u64,
    /// 한 정책 액션 동안 실행할 1 kHz 물리 스텝 수. 10이면 정책 100 Hz.
    #[arg(long, default_value_t = 10)]
    action_repeat: usize,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
enum Command {
    Spaces,
    Reset {
        #[serde(default = "default_true")]
        randomize: bool,
    },
    Step {
        action: Vec<f64>,
    },
    Close,
}

fn default_true() -> bool {
    return true;
}

#[derive(Serialize)]
struct SpacesResponse {
    kind: &'static str,
    observation_size: usize,
    action_size: usize,
}

#[derive(Serialize)]
struct ResetResponse {
    kind: &'static str,
    observation: Vec<f64>,
}

#[derive(Serialize)]
struct StepResponse {
    kind: &'static str,
    observation: Vec<f64>,
    reward: f64,
    terminated: bool,
    truncated: bool,
    info: pingpong_bot::TorqueEpisodeInfo,
}

#[derive(Serialize)]
struct ErrorResponse<'a> {
    kind: &'static str,
    message: &'a str,
}

#[derive(Serialize)]
struct CloseResponse {
    kind: &'static str,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let robot = defaults::primitive_4dof().context("4-dof robot")?;
    let mut env = TorqueResidualEnv::new(robot);
    let mut rng = StdRng::seed_from_u64(args.seed);
    let base = BallShooterSettings::default();

    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    let mut stdout = std::io::BufWriter::new(stdout.lock());
    for line in stdin.lock().lines() {
        let line = line.context("stdin")?;
        let command: Command = match serde_json::from_str(&line) {
            Ok(command) => command,
            Err(error) => {
                let message = error.to_string();
                write_json(
                    &mut stdout,
                    &ErrorResponse {
                        kind: "error",
                        message: &message,
                    },
                )?;
                continue;
            }
        };
        match command {
            Command::Spaces => write_json(
                &mut stdout,
                &SpacesResponse {
                    kind: "spaces",
                    observation_size: env.observation_size(),
                    action_size: env.action_size(),
                },
            )?,
            Command::Reset { randomize } => {
                let settings = if randomize {
                    base.randomized(&mut rng)
                } else {
                    base.clone()
                };
                let observation = env.reset(&settings).flattened();
                write_json(
                    &mut stdout,
                    &ResetResponse {
                        kind: "reset",
                        observation,
                    },
                )?;
            }
            Command::Step { action } => {
                let step = env.step(&action, args.action_repeat);
                write_json(
                    &mut stdout,
                    &StepResponse {
                        kind: "step",
                        observation: step.observation.flattened(),
                        reward: step.reward,
                        terminated: step.terminated,
                        truncated: step.truncated,
                        info: step.info,
                    },
                )?;
            }
            Command::Close => {
                write_json(&mut stdout, &CloseResponse { kind: "closed" })?;
                break;
            }
        }
    }
    return Ok(());
}

fn write_json(writer: &mut impl Write, value: &impl Serialize) -> Result<()> {
    serde_json::to_writer(&mut *writer, value).context("serialize response")?;
    writer.write_all(b"\n").context("write newline")?;
    writer.flush().context("flush response")?;
    return Ok(());
}
