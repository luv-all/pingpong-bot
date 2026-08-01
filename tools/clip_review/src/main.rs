//! 클립 리뷰어 — 카메라 2창(OpenCV) + sim 창(kiss3d)을 0.1x로 돌려 **예측 궤적이
//! 실제 궤적으로 수렴하는지**를 눈으로 본다.
//!
//! 세 창 모두 같은 색 규칙을 쓴다 — **초록** 실제 궤적(죽인 초록 = 아직 안 온 구간),
//! **회색** 매 프레임 예측, **자홍** 커밋 순간에 얼린 예측, **노랑** 검출 픽셀.
//!
//! 실제와 커밋 예측이 벌어졌을 때 **왜** 벌어졌는지는 카메라 창에서 갈린다 — 노란
//! 동그라미가 공 위에 있으면 물리·필터가 틀린 것이고, 바닥에 떨어진 공이나 팔에 가 있으면
//! 검출이 틀린 것이다.
//!
//! 검출은 시작할 때 **한 번만** 돈다. 되감아도 값이 다시 계산되지 않으므로 앞뒤로 오가며
//! 같은 프레임을 몇 번이든 다시 봐도 숫자가 흔들리지 않는다.
//!
//! 클립 대부분은 공이 아직 발사되지 않은 프리롤이다. **첫 삼각측량 전까지는 1배속**으로
//! 지나가고, 거기서부터 `--speed`가 걸린다.
//!
//! ```bash
//! cargo run --release -p clip-review -- --clip fly_04
//! cargo run --release -p clip-review -- --clip fly_02 --speed 0.05
//! ```
//!
//! 키: `Space` 일시정지 · `←`/`→` 또는 `,`/`.` 한 프레임 · `[`/`]` 10프레임 · `0` 처음으로 · `q` 종료

mod msg;
mod overlay;
mod sim_child;
mod track;

use std::io::Write;
use std::process::{Child, ChildStdin, Command, Stdio};

use anyhow::{Context, Result, bail};
use clap::Parser;
use opencv::highgui;
use opencv::prelude::*;
use pingpong_bot::camera::{
    self, Calibration, Frame, FrameSource, OpenCvCapture, Preview, StereoOfflineArgs,
};
use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::estimator::Decision;

use msg::SceneMsg;
use track::{FrameState, Reviewed};

/// 수렴 오차를 보여줄 리드타임 [s].
const LEADS_SECS: [f64; 3] = [0.1, 0.2, 0.3];

const WINDOW_CAM0: &str = "clip-review cam0";
const WINDOW_CAM1: &str = "clip-review cam1";

#[derive(Parser, Debug)]
#[command(about = "클립 리뷰 — 실제 궤적 vs 예측 궤적 (카메라 2창 + sim 창)")]
struct Args {
    #[command(flatten)]
    offline: StereoOfflineArgs,

    /// 재생 배속. 0.1 = 10배 느리게. 첫 공이 잡히기 전까지는 무시된다 (1배속).
    #[arg(long, default_value_t = 0.1)]
    speed: f64,

    /// 카메라 창 축소 배율.
    #[arg(long, default_value_t = 0.5)]
    scale: f64,

    /// 시작 프레임.
    #[arg(long, default_value_t = 0)]
    start: usize,

    /// sim 창 자식 프로세스 (내부용).
    #[arg(long, hide = true)]
    sim_child: bool,
}

/// 표시용 프레임 공급 — 순차 재생이면 그냥 읽고, 되감으면 seek한다.
struct Player {
    left: OpenCvCapture,
    right: OpenCvCapture,
    /// 다음 `next_frame()`이 낼 인덱스.
    next: usize,
}

impl Player {
    fn at(&mut self, index: usize) -> Option<(Frame, Frame)> {
        if index != self.next {
            self.left.seek_frame(index as u64).ok()?;
            self.right.seek_frame(index as u64).ok()?;
        }
        let left = self.left.next_frame()?;
        let right = self.right.next_frame()?;
        self.next = index + 1;
        return Some((left, right));
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.sim_child {
        return sim_child::run();
    }
    return run(&args);
}

fn run(args: &Args) -> Result<()> {
    let Some(clip) = args.offline.resolve().map_err(anyhow::Error::msg)? else {
        bail!("--clip 이 필요하다 (예: --clip fly_04)");
    };
    clip.log();

    let fps = clip.meas_fps.unwrap_or(30.0);
    println!("검출·삼각측량·EKF 재생 중 — 클립을 한 번 훑는다 …");
    let reviewed = track::review(&clip.left, &clip.right, fps).map_err(anyhow::Error::msg)?;
    if reviewed.len() == 0 {
        bail!("프레임을 하나도 못 읽었다");
    }
    let calibration = Calibration::load_json(&defaults::calibration_path())
        .map_err(anyhow::Error::msg)
        .context("calibration")?;
    let params: Vec<camera::Params> = [camera::Id(0), camera::Id(1)]
        .iter()
        .map(|id| {
            calibration
                .params(*id)
                .cloned()
                .with_context(|| format!("cam{} params 없음", id.0))
        })
        .collect::<Result<_>>()?;

    // 프리롤은 공이 아직 없다 — 여기까지는 1배속으로 지나간다.
    let first_ball = reviewed.observed.first().map(|o| o.frame);
    println!(
        "frames={} observed={} ({:.0}% 삼각측량) fps={fps:.1} 첫 공={}",
        reviewed.len(),
        reviewed.observed.len(),
        100.0 * reviewed.observed.len() as f64 / reviewed.len() as f64,
        first_ball.map_or("없음".to_owned(), |f| f.to_string())
    );
    println!("keys: Space 일시정지 | ←/→ or ,/. 한 프레임 | [/] 10프레임 | 0 처음 | q 종료");

    let mut player = Player {
        left: OpenCvCapture::from_path(camera::Id(0), &clip.left).map_err(anyhow::Error::msg)?,
        right: OpenCvCapture::from_path(camera::Id(1), &clip.right).map_err(anyhow::Error::msg)?,
        next: 0,
    };
    // sim 창이 안 떠도 본 재생은 계속한다 — 관전용이다.
    let mut sim = spawn_sim_child()
        .map_err(|error| eprintln!("sim 창 띄우기 실패 — 카메라 창만으로 진행: {error}"))
        .ok();

    let speed = args.speed.max(1e-3);
    let slow_wait_ms = ((1000.0 / fps / speed).round() as i32).max(1);
    let live_wait_ms = ((1000.0 / fps).round() as i32).max(1);
    let mut index = args.start.min(reviewed.len() - 1);
    let mut paused = false;
    // 실제 궤적은 pass 1이 클립을 통째로 훑어 **이미 다 안다**. 커밋 예측이 이후로 어디로
    // 갔는지와 나란히 보려면 미래도 있어야 하므로 잘라 주지 않는다 — 대신 현재 프레임을
    // 기준으로 과거·미래를 나눠 그려서 "지금 아는 것"과 구분되게 한다.

    loop {
        // 끝(또는 디코드 실패)에서 **종료하지 않는다** — 멈춰 서서 되감을 수 있어야 한다.
        let Some((left, right)) = player.at(index) else {
            paused = true;
            let last = index.saturating_sub(1);
            match Step::of(highgui::wait_key_ex(0)?, &mut paused) {
                Step::Quit => break,
                Step::Delta(delta) => index = shift(last, delta, reviewed.len()),
                Step::Home => index = 0,
                _ => index = last,
            }
            continue;
        };
        let state = &reviewed.frames[index];
        let (actual_past, actual_future) = reviewed.observed_split(index);
        let predicted: Vec<_> = state.predicted.iter().map(|s| s.position).collect();
        // 커밋 예측은 얼려 둔 것 — 프레임이 지나도 안 바뀐다. 아직 커밋 전이면 비어 있다.
        let committed: Vec<_> = reviewed
            .commit
            .as_ref()
            .filter(|commit| index >= commit.frame)
            .map(|commit| commit.predicted.iter().map(|s| s.position).collect())
            .unwrap_or_default();
        let fast = first_ball.is_some_and(|first| index < first);

        let shared = shared_hud(&reviewed, state, index, speed, paused, fast);
        for (slot, frame) in [(0usize, &left), (1usize, &right)] {
            let mut hud = camera_hud(state, slot);
            hud.extend(shared.iter().cloned());
            let panel = overlay::draw(
                &frame.image,
                &params[slot],
                &overlay::Tracks {
                    actual_past: &actual_past,
                    actual_future: &actual_future,
                    live: &predicted,
                    committed: &committed,
                },
                state.pixels[slot],
                state.ekf_position,
                &format!("cam{slot}"),
                &hud,
            )?;
            let shown = Preview::fit_bgr_downscale(
                &panel,
                (f64::from(panel.cols()) * args.scale).round() as i32,
                (f64::from(panel.rows()) * args.scale).round() as i32,
            )?;
            highgui::imshow(
                if slot == 0 { WINDOW_CAM0 } else { WINDOW_CAM1 },
                &shown.image,
            )?;
        }

        if let Some((_, stdin)) = &mut sim {
            let message = SceneMsg {
                ekf: state.ekf_position.map(Into::into),
                raw: state.observed.map(|o| o.point.into()),
                observed: actual_past.iter().copied().map(Into::into).collect(),
                observed_future: actual_future.iter().copied().map(Into::into).collect(),
                predicted: predicted.iter().copied().map(Into::into).collect(),
                committed: committed.iter().copied().map(Into::into).collect(),
            };
            if writeln!(stdin, "{}", message.to_line()).is_err() || stdin.flush().is_err() {
                // 사용자가 sim 창을 닫았다 — 조용히 그만 보낸다.
                sim = None;
            }
        }

        // 일시정지면 0 — 키가 올 때까지 블록한다. 프리롤은 1배속으로 지나간다.
        let wait = match (paused, fast) {
            (true, _) => 0,
            (false, true) => live_wait_ms,
            (false, false) => slow_wait_ms,
        };
        let key = highgui::wait_key_ex(wait)?;
        match Step::of(key, &mut paused) {
            Step::Quit => break,
            Step::Delta(delta) => index = shift(index, delta, reviewed.len()),
            Step::Home => index = 0,
            Step::Stay => {}
            Step::Advance => {
                // 끝에 닿으면 **멈춰 선다**. 자동 종료는 `q`/ESC로만.
                if index + 1 >= reviewed.len() {
                    paused = true;
                } else {
                    index += 1;
                }
            }
        }
    }

    for window in [WINDOW_CAM0, WINDOW_CAM1] {
        Preview::destroy_window(window);
    }
    if let Some((mut child, stdin)) = sim {
        drop(stdin);
        let _ = child.wait();
    }
    return Ok(());
}

fn spawn_sim_child() -> Result<(Child, ChildStdin)> {
    let exe = std::env::current_exe().context("current_exe")?;
    let mut child = Command::new(exe)
        .arg("--sim-child")
        .stdin(Stdio::piped())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .spawn()
        .context("spawn sim child")?;
    let stdin = child.stdin.take().context("sim child stdin")?;
    return Ok((child, stdin));
}

/// 키 하나가 재생 위치에 하는 일.
enum Step {
    Quit,
    /// 프레임 이동 (일시정지 상태로 들어간다).
    Delta(i64),
    Home,
    /// 아무 것도 안 함 (일시정지 중 다른 키).
    Stay,
    /// 다음 프레임으로.
    Advance,
}

impl Step {
    fn of(key: i32, paused: &mut bool) -> Self {
        if key == 27 || key == i32::from(b'q') || key == i32::from(b'Q') {
            return Self::Quit;
        }
        if key == i32::from(b' ') {
            *paused = !*paused;
            return Self::Stay;
        }
        if let Some((dx, _)) = Preview::arrow_delta(key)
            && dx != 0
        {
            *paused = true;
            return Self::Delta(i64::from(dx));
        }
        let delta = match key {
            k if k == i32::from(b'.') => 1,
            k if k == i32::from(b',') => -1,
            k if k == i32::from(b']') => 10,
            k if k == i32::from(b'[') => -10,
            k if k == i32::from(b'0') => return Self::Home,
            _ => 0,
        };
        if delta != 0 {
            *paused = true;
            return Self::Delta(delta);
        }
        if *paused {
            return Self::Stay;
        }
        return Self::Advance;
    }
}

fn shift(index: usize, delta: i64, len: usize) -> usize {
    let next = index as i64 + delta;
    return next.clamp(0, len as i64 - 1) as usize;
}

/// 그 카메라에서만 나오는 줄.
fn camera_hud(state: &FrameState, slot: usize) -> Vec<String> {
    let mut lines = vec![match state.pixels[slot] {
        Some(p) => format!("detect ({:.1}, {:.1})", p.x, p.y),
        None => "detect miss".to_owned(),
    }];
    lines.push(match state.observed {
        Some(o) => format!(
            "3d  x{:+.2} y{:+.2} z{:+.2}  reproj {:.1}px",
            o.point.x, o.point.y, o.point.z, o.reprojection_px
        ),
        None => "3d  none (한쪽만 잡았거나 재투영 게이트)".to_owned(),
    });
    return lines;
}

/// 두 카메라 창에 똑같이 붙는 줄 — 어느 쪽을 보고 있든 같은 숫자가 보이게.
fn shared_hud(
    reviewed: &Reviewed,
    state: &FrameState,
    index: usize,
    speed: f64,
    paused: bool,
    fast: bool,
) -> Vec<String> {
    let now = reviewed.time_of(index);
    let rate = if fast {
        "1.00x (preroll)".to_owned()
    } else {
        format!("{speed:.2}x")
    };
    let mut lines = vec![format!(
        "frame {index}/{}  t={now:.3}s  {rate}{}",
        reviewed.len() - 1,
        if paused { "  [PAUSED]" } else { "" }
    )];

    lines.push(if state.tracking {
        match (state.ekf_position, state.ekf_speed) {
            (Some(p), Some(v)) => {
                format!("ekf  x{:+.2} y{:+.2} z{:+.2}  |v|{v:.1}", p.x, p.y, p.z)
            }
            _ => "ekf  tracking".to_owned(),
        }
    } else {
        "ekf  not tracking".to_owned()
    });
    if let Some(gate) = state.gate {
        lines.push(match state.gate_d2 {
            Some(d2) => format!("gate {gate:?}  d2={d2:.1}"),
            None => format!("gate {gate:?}"),
        });
    }

    // 실기와 같은 게이트 — 왜 아직 안 넘겼는지가 여기 그대로 나온다.
    if let Some(decision) = state.decision {
        let sigma = state
            .impact_sigma
            .map_or("--".to_owned(), |s| format!("{:.0}cm", s * 100.0));
        lines.push(match decision {
            Decision::Attempt => format!("gate  ATTEMPT  sigma {sigma}"),
            Decision::Wait(reason) => format!("gate  {}  sigma {sigma}", reason.label()),
        });
    }

    // 매 프레임 다시 굴린 예측이 얼마나 앞을 맞히나 — 수렴 여부.
    let errors: Vec<String> = LEADS_SECS
        .iter()
        .map(|lead| {
            match track::convergence_error(
                &state.predicted,
                &reviewed.observed,
                now,
                *lead,
                reviewed.fps,
            ) {
                Some(error) => format!("+{lead:.1}s {:.1}cm", error * 100.0),
                None => format!("+{lead:.1}s --"),
            }
        })
        .collect();
    lines.push(format!("live      {}", errors.join("  ")));

    // 본론 — 실기가 하드웨어에 넘겼을 그 예측이 실제와 얼마나 어긋났나.
    lines.extend(commit_lines(reviewed, index));

    if let (Some(sp), Some(sv)) = (state.position_sigma, state.velocity_sigma) {
        lines.push(format!(
            "sigma  p {:.1}cm  v {:.0}cm/s",
            sp * 100.0,
            sv * 100.0
        ));
    }
    return lines;
}

/// 커밋 순간과 그 결과. 아직 커밋 전이면 그렇다고 말한다 — 빈 줄로 두면 "커밋이 없었다"와
/// "아직 안 왔다"가 구분이 안 된다.
fn commit_lines(reviewed: &Reviewed, index: usize) -> Vec<String> {
    let Some(commit) = &reviewed.commit else {
        return vec!["COMMIT  never (게이트를 끝내 못 넘음)".to_owned()];
    };
    if index < commit.frame {
        return vec![format!("COMMIT  at frame {} (아직 전)", commit.frame)];
    }

    let plane = table::DEFAULT_HIT_PLANE_Y;
    let mut lines = vec![
        format!(
            "COMMIT  frame {} t={:.3}s  tti {:.2}s  sigma {:.0}cm",
            commit.frame,
            commit.t,
            commit.time_to_impact,
            commit.impact_sigma * 100.0
        ),
        // 실기가 "여기를 치겠다"고 넘겼을 대표 후보.
        format!(
            "  target x{:+.2} y{:+.2} z{:+.2}",
            commit.impact.x, commit.impact.y, commit.impact.z
        ),
    ];

    // 얼린 예측의 평면 통과점 vs 실제 통과점 — 이 한 줄이 "예측이 맞았나"의 답이다.
    let guess = track::crossing_y(&commit.predicted, plane);
    let truth = reviewed.observed_crossing_y(plane);
    lines.push(match (guess, truth) {
        (Some(g), Some(t)) => format!(
            "  at y={plane:.2}  pred x{:+.2} z{:+.2} |v|{:.1}  real x{:+.2} z{:+.2}  MISS {:.1}cm",
            g.position.x,
            g.position.z,
            g.velocity.norm(),
            t.x,
            t.z,
            (g.position - t).norm() * 100.0
        ),
        (Some(g), None) => format!(
            "  at y={plane:.2}  pred x{:+.2} z{:+.2} |v|{:.1}  real -- (실제가 평면을 안 지남)",
            g.position.x,
            g.position.z,
            g.velocity.norm()
        ),
        (None, Some(t)) => format!(
            "  at y={plane:.2}  pred -- (예측이 평면을 안 지남)  real x{:+.2} z{:+.2}",
            t.x, t.z
        ),
        (None, None) => format!("  at y={plane:.2}  둘 다 평면을 안 지남"),
    });
    return lines;
}
