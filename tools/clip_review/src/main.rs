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
mod score;
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

use msg::SceneMsg;
use pingpong_bot::vision::Outcome;
use score::{LEADS_SECS, Score};
use track::{FrameState, Reviewed};

/// 수렴 오차를 보여줄 리드타임 [s].

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
    let score = Score::of(&reviewed);
    score.print(
        clip.dir
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("clip"),
    );
    print_commit_summary(&reviewed);
    println!("keys: Space 일시정지 | ←/→ or ,/. 한 프레임 | [/] 10프레임 | 0 처음 | q 종료");
    println!(
        "frame  t     | cam0 cam1 | 3d x y z reproj | ekf x y z |v| sigma_p sigma_v | gate0 gate1 | err +.1/+.2/+.3s [cm]"
    );

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
    let mut printed: Option<usize> = None;
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
        // 그리는 건 로봇 마운트까지만 — 그 뒤는 로봇이 이미 지나친 자리다. 예측 자체는
        // 여전히 끝까지 적분한다 (평면 통과·MISS 계산에 필요하다).
        let (past, future) = reviewed.observed_split(index);
        let actual_past = track::clip_to_mount(&past);
        let actual_future = track::clip_to_mount(&future);
        // 아래 둘은 제어로 나가는 Trajectory 그 자체에서 뽑는다. 툴이 따로 모은 상태로
        // 그리면 계약이 무엇을 담고 있는지가 아니라 툴이 무엇을 봤는지를 보게 된다.
        // 트리거 전에는 계약이 없어 둘 다 비어 있다 — 그때는 제어도 아무것도 못 받는다.
        let started = reviewed
            .contract
            .as_ref()
            .is_some_and(|contract| index >= contract.frame);
        let filtered = if started {
            track::clip_to_mount(&reviewed.measured_to(index))
        } else {
            Vec::new()
        };
        let committed = if started {
            track::clip_to_mount(
                &reviewed
                    .predicted()
                    .iter()
                    .map(|state| state.position)
                    .collect::<Vec<_>>(),
            )
        } else {
            Vec::new()
        };
        let fast = first_ball.is_some_and(|first| index < first);

        // 추적 중인 프레임만, 그리고 같은 프레임을 두 번 찍지 않는다.
        if state.tracking && printed != Some(index) {
            printed = Some(index);
            println!("{}", console_line(&reviewed, state, index));
        }

        for (slot, frame) in [(0usize, &left), (1usize, &right)] {
            let hud = overlay_hud(&reviewed, &score, state, index, slot, speed, paused, fast);
            let panel = overlay::draw(
                &frame.image,
                &params[slot],
                &overlay::Tracks {
                    actual_past: &actual_past,
                    actual_future: &actual_future,
                    filtered: &filtered,
                    committed: &committed,
                },
                state.pixels[slot],
                state.filtered.map(|s| s.position),
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
                ekf: state.filtered.map(|s| s.position.into()),
                raw: state.observed.map(|o| o.point.into()),
                observed: actual_past.iter().copied().map(Into::into).collect(),
                observed_future: actual_future.iter().copied().map(Into::into).collect(),
                filtered: filtered.iter().copied().map(Into::into).collect(),
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

/// 화면에 남기는 것은 **두 줄**뿐이다 — 지금 몇 번째 프레임인지, 이 카메라가 뭘 잡았는지.
///
/// 숫자를 화면에 다 쌓으면 정작 봐야 할 궤적을 가린다. 나머지는 [`console_line`]으로
/// 콘솔에 흘린다 — 거기서는 스크롤해서 앞뒤 프레임을 비교할 수도 있다.
#[allow(clippy::too_many_arguments)]
fn overlay_hud(
    reviewed: &Reviewed,
    score: &Score,
    state: &FrameState,
    index: usize,
    slot: usize,
    speed: f64,
    paused: bool,
    fast: bool,
) -> Vec<String> {
    let rate = if fast {
        "1.00x preroll".to_owned()
    } else {
        format!("{speed:.2}x")
    };
    return vec![
        format!(
            "f{index}/{}  t={:.3}s  {rate}{}",
            reviewed.len() - 1,
            reviewed.time_of(index),
            if paused { "  PAUSED" } else { "" }
        ),
        match (state.pixels[slot], state.residual_px[slot]) {
            (Some(p), Some(px)) => format!("detect ({:.0}, {:.0})  resid {px:.1}px", p.x, p.y),
            (Some(p), None) => format!("detect ({:.0}, {:.0})", p.x, p.y),
            (None, _) => "detect MISS".to_owned(),
        },
        score.hud_line(),
    ];
}

/// 프레임 하나를 한 줄로 — 콘솔용. 추적 중일 때만 찍는다 (프리롤은 볼 게 없다).
fn console_line(reviewed: &Reviewed, state: &FrameState, index: usize) -> String {
    let now = reviewed.time_of(index);
    let px = |slot: usize| match state.pixels[slot] {
        Some(p) => format!("{:.0},{:.0}", p.x, p.y),
        None => "----".to_owned(),
    };
    let three_d = match state.observed {
        Some(o) => format!(
            "3d {:+.2} {:+.2} {:+.2} rp{:.1}",
            o.point.x, o.point.y, o.point.z, o.reprojection_px
        ),
        None => "3d ----".to_owned(),
    };
    let ekf = match state.filtered {
        Some(s) => format!(
            "ekf {:+.2} {:+.2} {:+.2} |v|{:.1}",
            s.position.x,
            s.position.y,
            s.position.z,
            s.velocity.norm()
        ),
        None => "ekf ----".to_owned(),
    };
    // 축별로 찍는다. 하나로 합치면 잘 관측되는 축이 나쁜 축을 가린다.
    let sigma = match state.filtered {
        Some(s) => format!(
            "sp{:.0}/{:.0}/{:.0} sv{:.0}/{:.0}/{:.0}",
            s.sigma_position.x * 100.0,
            s.sigma_position.y * 100.0,
            s.sigma_position.z * 100.0,
            s.sigma_velocity.x * 100.0,
            s.sigma_velocity.y * 100.0,
            s.sigma_velocity.z * 100.0
        ),
        None => "sp-- sv--".to_owned(),
    };
    let gate = |slot: usize| match state.outcomes[slot] {
        Some(Outcome::Accepted) => "ok".to_owned(),
        Some(Outcome::Rejected { px }) => format!("rej{px:.0}"),
        Some(Outcome::Seeded) => "seed".to_owned(),
        Some(Outcome::Idle) | None => "--".to_owned(),
    };
    // 얼린 예측이 리드타임마다 얼마나 빗나갔나. 커밋 전이면 잴 게 없다.
    let error: Vec<String> = LEADS_SECS
        .iter()
        .map(|lead| {
            let measured = reviewed.contract.as_ref().and_then(|contract| {
                track::convergence_error(
                    &contract.at_trigger.predicted,
                    &reviewed.observed,
                    now,
                    *lead,
                    reviewed.fps,
                )
            });
            return measured.map_or("--".to_owned(), |e| format!("{:.0}", e * 100.0));
        })
        .collect();
    return format!(
        "f{index:<4} t={now:6.3} | {} {} | {three_d} | {ekf} {sigma} | {} {} | err {}",
        px(0),
        px(1),
        gate(0),
        gate(1),
        error.join("/")
    );
}

/// 커밋 결과 — 이 툴의 결론이다. pass 1이 끝나자마자 한 번 찍는다.
///
/// 실기가 하드웨어에 넘겼을 그 예측이 실제와 얼마나 어긋났는가. `MISS`가 그 숫자다.
fn print_commit_summary(reviewed: &Reviewed) {
    let plane = table::DEFAULT_HIT_PLANE_Y;
    let Some(contract) = &reviewed.contract else {
        let tracked = reviewed.frames.iter().filter(|f| f.tracking).count();
        println!("COMMIT  never — 트리거를 끝내 못 넘었다 (추적 프레임 {tracked}개)");
        return;
    };
    // 트리거 순간의 상태. 계약이 제어에게 "이만큼은 못 믿는다"고 말한 값이다.
    let at_trigger = contract
        .at_trigger
        .predicted
        .first()
        .copied()
        .unwrap_or_default();
    println!(
        "COMMIT  frame {} t={:.3}s  measured {}개  predicted {}개",
        contract.frame,
        contract.t,
        contract.latest.measured.len(),
        contract.at_trigger.predicted.len()
    );
    println!(
        "  sigma   p {:.0}/{:.0}/{:.0} cm   v {:.1}/{:.1}/{:.1} m/s",
        at_trigger.sigma_position.x * 100.0,
        at_trigger.sigma_position.y * 100.0,
        at_trigger.sigma_position.z * 100.0,
        at_trigger.sigma_velocity.x,
        at_trigger.sigma_velocity.y,
        at_trigger.sigma_velocity.z
    );
    let guess = contract.at_trigger.predicted.at_plane(plane);
    let truth = reviewed.observed_crossing_y(plane);
    match (guess, truth) {
        (Some(g), Some(t)) => println!(
            "  at y={plane:.2}  pred x{:+.2} z{:+.2} |v|{:.1}   real x{:+.2} z{:+.2}   MISS {:.1}cm",
            g.position.x,
            g.position.z,
            g.velocity.norm(),
            t.x,
            t.z,
            (g.position - t).norm() * 100.0
        ),
        (Some(g), None) => println!(
            "  at y={plane:.2}  pred x{:+.2} z{:+.2} |v|{:.1}   real -- (실제가 평면을 안 지남)",
            g.position.x,
            g.position.z,
            g.velocity.norm()
        ),
        (None, Some(t)) => println!(
            "  at y={plane:.2}  pred -- (예측이 평면을 안 지남)   real x{:+.2} z{:+.2}",
            t.x, t.z
        ),
        (None, None) => println!("  at y={plane:.2}  둘 다 평면을 안 지난다"),
    }
}
