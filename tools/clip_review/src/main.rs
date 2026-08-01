//! 클립 리뷰어 — 카메라 2창 + 월드 2면도를 0.1x로 돌려 **예측 궤적이 실제 궤적으로
//! 수렴하는지**를 눈으로 본다.
//!
//! 흰 선 = 검출·삼각측량이 말하는 실제 궤적. 하늘색 선 = 그 시점 EKF의 예측 궤적.
//! 둘이 붙으면 예측이 맞은 것이고, 벌어지면 그 프레임이 어디서 틀렸는지 카메라 화면에서
//! 바로 보인다 (검출이 튀었나, 3D가 밀렸나, 물리가 안 맞나).
//!
//! 검출은 시작할 때 **한 번만** 돈다. 되감아도 값이 다시 계산되지 않으므로 앞뒤로 오가며
//! 같은 프레임을 몇 번이든 다시 봐도 숫자가 흔들리지 않는다.
//!
//! ```bash
//! cargo run --release -p clip-review -- --clip fly_04
//! cargo run --release -p clip-review -- --clip fly_02 --speed 0.05
//! ```
//!
//! 키: `Space` 일시정지 · `←`/`→` 또는 `,`/`.` 한 프레임 · `[`/`]` 10프레임 · `0` 처음으로 · `q` 종료

mod overlay;
mod plot;
mod track;

use anyhow::{Context, Result, bail};
use clap::Parser;
use opencv::highgui;
use opencv::prelude::*;
use pingpong_bot::camera::{
    self, Calibration, Frame, FrameSource, OpenCvCapture, Preview, StereoOfflineArgs,
};
use pingpong_bot::constants::table;
use pingpong_bot::defaults;

use track::{FrameState, Reviewed};

/// 수렴 오차를 보여줄 리드타임 [s].
const LEADS_SECS: [f64; 3] = [0.1, 0.2, 0.3];

const WINDOW_CAM0: &str = "clip-review cam0";
const WINDOW_CAM1: &str = "clip-review cam1";
const WINDOW_WORLD: &str = "clip-review world";

#[derive(Parser, Debug)]
#[command(about = "클립 리뷰 — 실제 궤적 vs 예측 궤적 (카메라 2창 + 월드 2면도)")]
struct Args {
    #[command(flatten)]
    offline: StereoOfflineArgs,

    /// 재생 배속. 0.1 = 10배 느리게.
    #[arg(long, default_value_t = 0.1)]
    speed: f64,

    /// 카메라 창 축소 배율.
    #[arg(long, default_value_t = 0.5)]
    scale: f64,

    /// 월드 2면도 가로 폭 [px].
    #[arg(long, default_value_t = 900)]
    plot_width: i32,

    /// 시작 프레임.
    #[arg(long, default_value_t = 0)]
    start: usize,
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

    println!(
        "frames={} observed={} ({:.0}% 삼각측량) fps={fps:.1}",
        reviewed.len(),
        reviewed.observed.len(),
        100.0 * reviewed.observed.len() as f64 / reviewed.len() as f64
    );
    println!("keys: Space 일시정지 | ←/→ or ,/. 한 프레임 | [/] 10프레임 | 0 처음 | q 종료");

    let mut player = Player {
        left: OpenCvCapture::from_path(camera::Id(0), &clip.left).map_err(anyhow::Error::msg)?,
        right: OpenCvCapture::from_path(camera::Id(1), &clip.right).map_err(anyhow::Error::msg)?,
        next: 0,
    };
    let world = plot::WorldPlot::new(args.plot_width);

    let speed = args.speed.max(1e-3);
    let play_wait_ms = ((1000.0 / fps / speed).round() as i32).max(1);
    let mut index = args.start.min(reviewed.len() - 1);
    let mut paused = false;

    loop {
        let Some((left, right)) = player.at(index) else {
            break;
        };
        let state = &reviewed.frames[index];
        let observed = reviewed.observed_upto(index);
        let predicted: Vec<_> = state.predicted.iter().map(|s| s.position).collect();

        for (slot, frame) in [(0usize, &left), (1usize, &right)] {
            let panel = overlay::draw(
                &frame.image,
                &params[slot],
                &observed,
                &predicted,
                state.pixels[slot],
                state.ekf_position,
                &format!("cam{slot}"),
                &camera_hud(&reviewed, state, index, slot),
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

        let world_panel = world.render(
            &observed,
            &predicted,
            state.ekf_position,
            state.observed.map(|o| o.point),
            &world_hud(&reviewed, state, index, speed, paused),
        )?;
        highgui::imshow(WINDOW_WORLD, &world_panel)?;

        // 일시정지면 0 — 키가 올 때까지 블록한다.
        let wait = if paused { 0 } else { play_wait_ms };
        let key = highgui::wait_key_ex(wait)?;
        match Step::of(key, &mut paused) {
            Step::Quit => break,
            Step::Delta(delta) => {
                index = shift(index, delta, reviewed.len());
            }
            Step::Home => index = 0,
            Step::Stay => {}
            Step::Advance => {
                if index + 1 >= reviewed.len() {
                    paused = true;
                } else {
                    index += 1;
                }
            }
        }
    }

    for window in [WINDOW_CAM0, WINDOW_CAM1, WINDOW_WORLD] {
        Preview::destroy_window(window);
    }
    return Ok(());
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

fn camera_hud(reviewed: &Reviewed, state: &FrameState, index: usize, slot: usize) -> Vec<String> {
    let mut lines = vec![format!(
        "frame {index}/{}  t={:.3}s",
        reviewed.len() - 1,
        reviewed.time_of(index)
    )];
    lines.push(match state.pixels[slot] {
        Some(p) => format!("detect ({:.1}, {:.1})", p.x, p.y),
        None => "detect miss".to_owned(),
    });
    lines.push(match state.observed {
        Some(o) => format!(
            "3d  x{:+.2} y{:+.2} z{:+.2}  reproj {:.1}px",
            o.point.x, o.point.y, o.point.z, o.reprojection_px
        ),
        None => "3d  none (한쪽만 잡았거나 재투영 게이트)".to_owned(),
    });
    if let Some(gate) = state.gate {
        lines.push(match state.gate_d2 {
            Some(d2) => format!("gate {gate:?}  d2={d2:.1}"),
            None => format!("gate {gate:?}"),
        });
    }
    return lines;
}

fn world_hud(
    reviewed: &Reviewed,
    state: &FrameState,
    index: usize,
    speed: f64,
    paused: bool,
) -> Vec<String> {
    let now = reviewed.time_of(index);
    let mut lines = vec![format!(
        "frame {index}/{}  t={now:.3}s  {speed:.2}x{}",
        reviewed.len() - 1,
        if paused { "  [PAUSED]" } else { "" }
    )];

    lines.push(if state.tracking {
        match (state.ekf_position, state.ekf_speed) {
            (Some(p), Some(v)) => format!("ekf  x{:+.2} y{:+.2} z{:+.2}  |v|{v:.1}", p.x, p.y, p.z),
            _ => "ekf  tracking".to_owned(),
        }
    } else {
        "ekf  not tracking".to_owned()
    });

    // 본론 — 그때 한 예측이 실제와 얼마나 벌어졌나.
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
    lines.push(format!("converge  {}", errors.join("  ")));

    // 제어측이 실제로 받을 값 — 접수 평면 통과 상태 (위치 + 속도).
    if let Some(crossing) = track::crossing_y(&state.predicted, table::DEFAULT_HIT_PLANE_Y) {
        lines.push(format!(
            "hit y={:.2}  x{:+.2} z{:+.2}  |v|{:.1}  in {:.2}s",
            table::DEFAULT_HIT_PLANE_Y,
            crossing.position.x,
            crossing.position.z,
            crossing.velocity.norm(),
            crossing.t - now
        ));
    }

    if let (Some(sp), Some(sv)) = (state.position_sigma, state.velocity_sigma) {
        lines.push(format!(
            "sigma  p {:.1}cm  v {:.0}cm/s",
            sp * 100.0,
            sv * 100.0
        ));
    }
    lines.push("white=observed  cyan=predicted  green=ekf  yellow=this frame".to_owned());
    return lines;
}
