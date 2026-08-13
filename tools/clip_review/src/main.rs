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
mod report;
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

/// 카메라 창 하나. 둘로 띄우면 어느 쪽이 포커스인지에 따라 키가 안 먹는다.
const WINDOW: &str = "clip-review";
/// 일시정지 중 키를 기다리는 주기 [ms]. `waitKey(0)` 은 macOS 에서 키를 못 받는다.
const PAUSED_POLL_MS: i32 = 20;

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

    /// 창을 안 띄우고 `data/clips` 아래 클립을 전부 훑어 성적표와 그림을 남긴다.
    #[arg(long)]
    all: bool,

    /// `--all` 결과를 넣을 폴더.
    #[arg(long, default_value = "data/review")]
    out: std::path::PathBuf,

    /// 창 없이 `--clip`의 한 프레임에 월드 격자를 얹어 `{name}_grid.png`로 남긴다 —
    /// 캘리브가 지금 카메라 자세와 맞는지 눈으로 확인하는 용도.
    #[arg(long)]
    grid: bool,

    /// 창을 안 띄우고 `data/clips` 전 클립에 e·mu·drag를 공유로 좌표하강 탐색한다 —
    /// 매 후보마다 재빌드 없이 실제 `Fit`/트리거로 다시 채점한다(`review_with_physics`).
    #[arg(long)]
    calibrate: bool,

    /// 트리거(σ 문턱·평면 비율)를 격자로 훑어 "리드타임을 얼마나 벌면서 정확도를
    /// 얼마나 잃는가"의 프런티어를 낸다. 물리는 지금 기본값 고정.
    #[arg(long)]
    trigger_sweep: bool,
}

/// `--grid` — 격자가 탁구대·네트에 붙는지로 캘리브 자세를 확인한다. 공은 필요 없어서
/// 프리롤 프레임(0)을 그냥 쓴다 — 카메라가 고정이라 어차피 매 프레임 같은 배경이다.
fn run_grid(args: &Args) -> Result<()> {
    let Some(clip) = args.offline.resolve().map_err(anyhow::Error::msg)? else {
        bail!("--clip 이 필요하다 (예: --clip fly_21 --grid)");
    };
    let name = clip
        .dir
        .file_name()
        .and_then(|n| n.to_str())
        .context("클립 이름")?
        .to_owned();
    let calibration = Calibration::load_json(&defaults::calibration_path())
        .map_err(anyhow::Error::msg)
        .context("calibration")?;
    std::fs::create_dir_all(&args.out).with_context(|| format!("mkdir {}", args.out.display()))?;

    let mut panels = Vec::with_capacity(2);
    for (slot, path) in [(0usize, &clip.left), (1usize, &clip.right)] {
        let mut source = camera::OpenCvCapture::from_path(camera::Id(slot as u8), path)
            .map_err(anyhow::Error::msg)?;
        use pingpong_bot::camera::FrameSource;
        let frame = source.next_frame().context("첫 프레임 못 읽음")?;
        let mut panel = frame.image.try_clone().context("프레임 clone")?;
        let params = calibration
            .params(camera::Id(slot as u8))
            .with_context(|| format!("cam{slot} params 없음"))?;
        camera::Preview::draw_world_grid(&mut panel, params, &camera::WorldGridParams::default())?;
        camera::Preview::draw_cam_label(
            &mut panel,
            &format!("cam{slot}  grid"),
            opencv::core::Scalar::new(230.0, 230.0, 230.0, 0.0),
        )?;
        panels.push(panel);
    }
    let mosaic = camera::Preview::hstack_bgr(&panels)?;
    let path = args.out.join(format!("{name}_grid.png"));
    opencv::imgcodecs::imwrite(
        path.to_str().context("경로에 UTF-8 아닌 글자")?,
        &mosaic,
        &opencv::core::Vector::new(),
    )
    .with_context(|| format!("imwrite {}", path.display()))?;
    println!("{}", path.display());
    return Ok(());
}

/// 클립 하나 — 성적표 한 줄.
fn run_one(root: &std::path::Path, name: &str, out: &std::path::Path) -> Result<String> {
    let clip = camera::StereoOfflineArgs {
        clip: Some(root.join(name)),
    }
    .resolve()
    .map_err(anyhow::Error::msg)?
    .context("클립 해석 실패")?;
    return Ok(report::summary_row(name, &report::one(&clip, name, out)?));
}

/// 클립 전부를 훑어 그림과 표를 남긴다. 창은 안 띄운다.
fn run_all(out: &std::path::Path) -> Result<()> {
    std::fs::create_dir_all(out).with_context(|| format!("mkdir {}", out.display()))?;
    // 폴더를 훑지 않는다. 카메라를 옮긴 뒤로 옛 클립은 지금 캘리브와 안 맞아서, 같이 돌리면
    // 지표가 조용히 거짓말을 한다. 하나만 보려면 `--clip` 으로 명시하면 된다.
    let root = std::path::Path::new("data/clips");
    let names: Vec<String> = defaults::CURRENT_RIG_CLIPS
        .iter()
        .map(|name| (*name).to_owned())
        .filter(|name| root.join(name).join("left.avi").is_file())
        .collect();
    if names.is_empty() {
        bail!(
            "지금 리그 클립이 하나도 없다 ({:?}) — 카메라를 옮긴 뒤로 찍은 클립이 필요하다",
            defaults::CURRENT_RIG_CLIPS
        );
    }

    let mut lines = vec![report::summary_header()];
    println!("{}", lines[0]);
    for name in &names {
        // 한 클립이 나빠도 나머지는 돌린다 — 배치가 통째로 멎으면 아무 표도 안 나온다.
        match run_one(root, name, out) {
            Ok(row) => {
                println!("{row}");
                lines.push(row);
            }
            Err(error) => {
                let row = format!("{name:<8} 건너뜀 — {error:#}");
                println!("{row}");
                lines.push(row);
            }
        }
    }
    lines.push(String::new());
    lines.push(
        "동시=두 캠 동시 검출 프레임, 잔차=적합 재투영 p50 [px], 관측=트리거 시점 관측 수".into(),
    );
    lines.push(
        "RMSEx·y·z=얼린 예측의 축별 RMSE — 합이 아니라 어느 방향이 나쁜지를 본다".into(),
    );
    lines.push("RMSE·리드타임 오차는 cm, 전부 생 삼각측량 기준".into());
    lines.push("y0.08~y0.35 = 접수 창의 평면들 — 그 자리에서 라켓이 빗나가는 거리 [cm]".into());
    let summary = out.join("summary.txt");
    std::fs::write(&summary, lines.join("\n") + "\n")
        .with_context(|| format!("write {}", summary.display()))?;
    println!("\n{} 개 클립 → {}", names.len(), out.display());
    return Ok(());
}

/// 중심값 근처를 촘촘히 훑어 `try_value`가 가장 작아지는 자리로 옮긴다.
fn sweep(
    center: f64,
    radius: f64,
    steps: usize,
    (lo, hi): (f64, f64),
    mut try_value: impl FnMut(f64) -> Option<f64>,
) -> f64 {
    let mut best = (center, try_value(center).unwrap_or(f64::INFINITY));
    for i in 0..steps {
        let frac = i as f64 / (steps - 1) as f64 * 2.0 - 1.0; // -1..1
        let candidate = (center + frac * radius).clamp(lo, hi);
        if let Some(score) = try_value(candidate)
            && score < best.1
        {
            best = (candidate, score);
        }
    }
    return best.0;
}

/// e·mu·drag를 9클립 공유로 좌표하강 탐색한다. 매 후보를 재빌드 없이 **실제**
/// `Vision`→`Fit`→트리거→`Score` 파이프라인으로 다시 채점한다(`review_with_physics`).
///
/// `tests/diag_physics.rs::calibrate_shared_physics_across_clips`가 같은 질문을
/// 손으로 짠 별도 궤적 적합기로 물어봤는데 결과가 실측과 어긋났다(항력 0으로 수렴 —
/// 실측 `clip-review`는 drag=0.126이 크게 이겼다). 그 도구는 트리거·관측 창·아웃라이어
/// 배제 없이 raw 궤적을 통째로 다시 맞히는 거라 로봇이 실제로 겪는 "본 것으로 못 본
/// 걸 맞히는" 문제와 다른 걸 재고 있었다 — 여기는 진짜 파이프라인을 그대로 써서
/// 그 간극을 없앤다.
///
/// 점수는 클립별 0.2s 리드 타점 오차의 **중앙값**이다(`min_swing_secs=0.20`이 로봇이
/// 실제로 쓸 수 있는 하한 — `src/defaults/control.rs`). 평균 대신 중앙값을 쓰는 이유는
/// fly_49류(검출 하나가 튀어 그 클립만 구조적으로 나쁜 경우)가 평균을 통째로 흔드는 걸
/// 막기 위해서다.
///
/// **검출은 클립당 한 번만.** 첫 버전은 후보마다 `review_with_physics`(비디오 디코드+
/// 검출 캐스케이드까지 포함)를 통째로 다시 돌려서, 후보 1개 채점이 `--all` 전체보다도
/// 오래 걸렸다(실측 2026-08-12: 시작값 하나 채점에 31분+, 킬 전까지 라운드 하나도
/// 못 끝냄). 검출은 물리와 무관하니 [`track::detect_all`]로 클립당 한 번만 캐싱하고,
/// 후보마다는 [`track::replay_with_physics`](비디오·검출 없이 `Fit`만 다시 굴림)만
/// 돈다 — 총 비용이 "클립 수 × 검출 1회" + "후보 수 × (클립 수 × 재적합, 훨씬 쌈)"으로
/// 바뀐다.
fn run_calibrate() -> Result<()> {
    let root = std::path::Path::new("data/clips");
    let names: Vec<String> = defaults::CURRENT_RIG_CLIPS
        .iter()
        .map(|name| (*name).to_owned())
        .filter(|name| root.join(name).join("left.avi").is_file())
        .collect();
    if names.is_empty() {
        bail!(
            "지금 리그 클립이 하나도 없다 ({:?}) — 카메라를 옮긴 뒤로 찍은 클립이 필요하다",
            defaults::CURRENT_RIG_CLIPS
        );
    }

    let start = std::time::Instant::now();
    let mut detected = Vec::with_capacity(names.len());
    for (i, name) in names.iter().enumerate() {
        let clip = camera::StereoOfflineArgs {
            clip: Some(root.join(name)),
        }
        .resolve()
        .map_err(anyhow::Error::msg)?
        .with_context(|| format!("{name} 클립 해석 실패"))?;
        let Some(fps) = clip.meas_fps else {
            println!("{name}: meas_fps 없음 — 건너뜀");
            continue;
        };
        print!(
            "검출 {}/{} — {name} ... ",
            i + 1,
            names.len()
        );
        std::io::stdout().flush().ok();
        let t0 = std::time::Instant::now();
        let cache = track::detect_all(&clip.left, &clip.right, fps).map_err(anyhow::Error::msg)?;
        println!("{:.1}s", t0.elapsed().as_secs_f64());
        detected.push(cache);
    }
    if detected.is_empty() {
        bail!("검출된 클립이 없다");
    }
    println!(
        "검출 캐싱 끝 — {}개 클립, {:.1}초 (탐색은 이제부터 비디오를 안 연다)",
        detected.len(),
        start.elapsed().as_secs_f64()
    );

    let score = |e: f64, mu: f64, drag: f64| -> Option<f64> {
        let physics = defaults::PhysicsParams {
            restitution: e,
            friction: mu,
            drag,
            ..defaults::PhysicsParams::default()
        };
        let mut errors: Vec<f64> = Vec::new();
        for cache in &detected {
            let Ok(reviewed) = track::replay_with_physics(cache, physics) else {
                continue;
            };
            if let Some(err) = Score::of(&reviewed).lead_error.get(2).copied().flatten() {
                errors.push(err);
            }
        }
        if errors.is_empty() {
            return None;
        }
        errors.sort_by(|a, b| a.partial_cmp(b).expect("유한"));
        return Some(errors[errors.len() / 2]);
    };

    let default = defaults::PhysicsParams::default();
    let (mut e, mut mu, mut drag) = (
        default.restitution,
        default.friction,
        defaults::vision::fit::DRAG,
    );
    let t0 = std::time::Instant::now();
    println!(
        "시작(지금 기본값): e={e:.3} mu={mu:.3} drag={drag:.3}  0.2s 리드 중앙값={:.2}cm  ({:.1}s)",
        score(e, mu, drag).unwrap_or(f64::INFINITY) * 100.0,
        t0.elapsed().as_secs_f64()
    );

    const ROUNDS: [f64; 3] = [0.10, 0.04, 0.015];
    for (round, &radius) in ROUNDS.iter().enumerate() {
        let t0 = std::time::Instant::now();
        e = sweep(e, radius, 7, (0.5, 0.95), |cand| score(cand, mu, drag));
        mu = sweep(mu, radius, 7, (0.0, 0.7), |cand| score(e, cand, drag));
        drag = sweep(drag, radius, 7, (0.0, 0.30), |cand| score(e, mu, cand));
        println!(
            "라운드 {round} (반경 {radius:.3}): e={e:.4} mu={mu:.4} drag={drag:.4}  0.2s 리드 중앙값={:.3}cm  ({:.1}s)",
            score(e, mu, drag).unwrap_or(f64::INFINITY) * 100.0,
            t0.elapsed().as_secs_f64()
        );
    }

    println!(
        "\n{}",
        pingpong_bot::physics::PhysicsIdentify::format_physics_for_defaults(
            Some(e),
            Some(mu),
            Some(drag),
        )
    );
    return Ok(());
}

/// 트리거(σ 문턱·평면 비율) 격자 — 리드타임 이득과 정확도 손실의 프런티어를 낸다.
///
/// 물리는 지금 커밋된 기본값(`RESTITUTION`·`FRICTION`·`DRAG`)으로 고정 — 이건 트리거
/// 실험이지 물리 재탐색이 아니다. 검출은 [`detect_all`]로 클립당 한 번만 캐싱하고,
/// 후보(σ, y_frac)마다 [`track::replay_with`]로 `Fit`만 다시 굴린다.
///
/// **점수 두 개**를 같이 본다 — 리드타임만 보면 트리거가 이를수록 무조건 이겨서
/// 최악의 설정(맨 처음 관측에서 바로 커밋)이 "우승"해 버린다.
/// - 리드타임 이득 = 기본 트리거 대비 `contract.t`가 얼마나 당겨졌나 [ms] (클수록 좋음).
/// - 정확도 = 커밋 순간에 얼린 예측이 접수 평면들에서 실제와 벌어진 거리
///   (`Score::plane_error`, cm, 작을수록 좋음) — 트리거 이후 재적합 전에 로봇이
///   그 순간 뭘 볼지라 이게 맞는 지표다. `lead_error`(여러 시각에서 재본 것)를 쓰면
///   트리거 이른 것과 무관한 나중 재적합까지 섞여 든다.
///
/// 후보는 "기본값보다 나쁘지 않은 정확도"로 거르고, 그 안에서 리드타임 이득이
/// 가장 큰 것을 고른다 — 정확도를 먼저 안 정하면(예: "10% 손해까지 허용") 계속
/// 임의적이라, 대신 "지금과 비슷하거나 나은 것"만 후보로 남긴다.
fn run_trigger_sweep() -> Result<()> {
    let root = std::path::Path::new("data/clips");
    let names: Vec<String> = defaults::CURRENT_RIG_CLIPS
        .iter()
        .map(|name| (*name).to_owned())
        .filter(|name| root.join(name).join("left.avi").is_file())
        .collect();
    if names.is_empty() {
        bail!(
            "지금 리그 클립이 하나도 없다 ({:?}) — 카메라를 옮긴 뒤로 찍은 클립이 필요하다",
            defaults::CURRENT_RIG_CLIPS
        );
    }

    let mut detected = Vec::with_capacity(names.len());
    for (i, name) in names.iter().enumerate() {
        let clip = camera::StereoOfflineArgs {
            clip: Some(root.join(name)),
        }
        .resolve()
        .map_err(anyhow::Error::msg)?
        .with_context(|| format!("{name} 클립 해석 실패"))?;
        let Some(fps) = clip.meas_fps else {
            println!("{name}: meas_fps 없음 — 건너뜀");
            continue;
        };
        print!("검출 {}/{} — {name} ... ", i + 1, names.len());
        std::io::stdout().flush().ok();
        let t0 = std::time::Instant::now();
        let cache = track::detect_all(&clip.left, &clip.right, fps).map_err(anyhow::Error::msg)?;
        println!("{:.1}s", t0.elapsed().as_secs_f64());
        // 이름을 같이 들고 간다 — meas_fps 없는 클립을 건너뛰므로 `names`와 인덱스가
        // 어긋난다(실측 fly_54). 진단표가 엉뚱한 클립 이름을 달면 안 된다.
        detected.push((name.clone(), cache));
    }
    if detected.is_empty() {
        bail!("검출된 클립이 없다");
    }

    let physics = defaults::PhysicsParams {
        restitution: defaults::vision::fit::RESTITUTION,
        friction: defaults::vision::fit::FRICTION,
        drag: defaults::vision::fit::DRAG,
        ..defaults::PhysicsParams::default()
    };
    let calibration = Calibration::load_json(&defaults::calibration_path())
        .map_err(anyhow::Error::msg)
        .context("calibration")?;
    let base_max_lead = defaults::EstimatorParams::default().max_lead;

    // 조립 모양이 `defaults::vision::trigger()`와 **같아야** 한다 — 도구가 생산과 다른
    // 트리거를 재면 여기서 좋아 보이는 값이 실기에서 재현되지 않는다. 실제로 그 함정을
    // 한 번 밟았다: 게이트를 넣기 전 이 스윕은 맨 `PlaneCrossing`을 썼고, 그래서 "0.75보다
    // 이르게 가면 절벽(8.6cm -> 22.6cm)"이라는 결론이 나왔다. 그 절벽의 원인이 바로
    // 게이트가 막는 실패(한쪽 캠이 죽은 단안 구간에서 확신 없이 얼어붙는 것)였으므로,
    // 게이트를 낀 지금은 프런티어 자체가 달라진다.
    let base_stereo = defaults::vision::MIN_STEREO_SAMPLES;
    let base_y_frac = defaults::vision::ALIGNMENT_TRIGGER_TABLE_Y_FRAC;
    // `defaults::vision::trigger()`와 **같은 모양**이어야 한다 — 도구가 생산과 다른
    // 트리거를 재면 여기서 좋아 보이는 값이 실기에서 재현되지 않는다.
    let make_trigger = |stereo: usize, y_frac: f64| -> Box<dyn pingpong_bot::vision::Trigger> {
        return Box::new(pingpong_bot::vision::triggers::All(vec![
            Box::new(pingpong_bot::vision::triggers::StereoSamples {
                min_samples: stereo,
            }),
            Box::new(pingpong_bot::vision::triggers::PlaneCrossing {
                y: table::LENGTH_Y * y_frac,
            }),
        ]));
    };

    /// 트리거 하나로 클립을 다시 굴린다 — (트리거 시각, 도달 시각, 접수 평면 오차 [cm]).
    ///
    /// 도달 시각을 같이 내는 이유: 리드타임을 기준값 대비 **상대**로만 재면 "그래서
    /// 제어가 몇 ms를 손에 쥐나"를 끝내 모른다. `impact_t - contract.t`가 그 절대값이다.
    fn run(
        cache: &track::DetectedFrames,
        calibration: &Calibration,
        physics: defaults::PhysicsParams,
        trigger: Box<dyn pingpong_bot::vision::Trigger>,
    ) -> Option<(f64, Option<f64>, Vec<f64>)> {
        return run_full(cache, calibration, physics, trigger).map(|(t, i, e, _, _)| (t, i, e));
    }

    /// [`run`]에 "그 순간 스테레오 표본이 몇 개였나"를 덧붙인다 — 조건이 걸린 자리를
    /// 표본 수로 환산해 보면 서로 다른 축인지 같은 축인지가 드러난다.
    fn run_full(
        cache: &track::DetectedFrames,
        calibration: &Calibration,
        physics: defaults::PhysicsParams,
        trigger: Box<dyn pingpong_bot::vision::Trigger>,
    ) -> Option<(f64, Option<f64>, Vec<f64>, usize, bool)> {
        let fit = pingpong_bot::vision::Fit::with_physics(calibration, trigger, physics);
        let reviewed = track::replay_with(cache, fit).ok()?;
        let contract_frame = reviewed.contract.as_ref()?.frame;
        let pairs = reviewed.frames[contract_frame].stereo_samples;
        let t = reviewed.contract.as_ref()?.t;
        let score = Score::of(&reviewed);
        let errs = score
            .plane_error
            .iter()
            .filter_map(|(_, e)| e.map(|(hypot, _, _)| hypot * 100.0))
            .collect();
        return Some((t, score.impact_t, errs, pairs, score.solved_spin.is_some()));
    }

    // 클립별 (contract.t, impact_t, plane_error) — 지금 생산 트리거 기준.
    let baseline: Vec<Option<(f64, Option<f64>, Vec<f64>)>> = detected
        .iter()
        .map(|(_, cache)| run(cache, &calibration, physics, defaults::vision::trigger()))
        .collect();
    let median = |values: &mut Vec<f64>| -> f64 {
        values.sort_by(|a, b| a.partial_cmp(b).expect("유한"));
        return values.get(values.len() / 2).copied().unwrap_or(f64::INFINITY);
    };
    let base_plane_median = {
        let mut all: Vec<f64> = baseline
            .iter()
            .flatten()
            .flat_map(|(_, _, e)| e.clone())
            .collect();
        median(&mut all)
    };
    println!("기본 트리거 — 접수 평면 오차 중앙값 {base_plane_median:.2}cm");

    // 어느 경로가 리드타임의 병목인가.
    //
    // 트리거는 `Any`라 두 경로 중 **빠른 쪽**이 건다. 그러니 "더 일찍 걸고 싶다"의
    // 지렛대는 지금 실제로 먼저 걸리는 경로 쪽에 있다. 평면 경로가 늘 먼저라면 y_frac이
    // 지렛대고, sigma 경로가 먼저라면 문턱을 푸는 것 말고도 **sigma가 더 빨리 떨어지게
    // 만드는 것**(= 검출을 덜 놓쳐 관측이 빨리 쌓이게 하는 것)이 지렛대가 된다 —
    // 후자는 정확도를 깎지 않고 리드타임만 버는 유일한 방향이라 값이 다르다.
    // 평면이 "그냥 더 높은 표본 문턱"인지 다른 축인지 — 각 조건이 걸린 순간의 표본
    // 수로 환산해서 본다. 같은 축이면 평면 시점 표본 수가 클립마다 비슷해야 한다.
    println!(
        "\n{:<9} {:>10} {:>10} {:>8}   조건이 걸린 순간의 스테레오 표본 수",
        "clip", "표본문턱", "평면", "도달초"
    );
    let plane_only: Box<dyn Fn() -> Box<dyn pingpong_bot::vision::Trigger>> =
        Box::new(|| Box::new(pingpong_bot::vision::triggers::PlaneCrossing {
            y: table::LENGTH_Y * defaults::vision::ALIGNMENT_TRIGGER_TABLE_Y_FRAC,
        }));
    let mut plane_pairs = Vec::new();
    for ((name, cache), base) in detected.iter().zip(&baseline) {
        let at = |trigger| run_full(cache, &calibration, physics, trigger).map(|(t, _, _, n, _)| (t, n));
        let stereo = at(Box::new(pingpong_bot::vision::triggers::StereoSamples {
            min_samples: base_stereo,
        }));
        let plane = at(plane_only());
        if let Some((_, n)) = plane {
            plane_pairs.push(n);
        }
        let cell = |v: Option<(f64, usize)>| {
            v.map_or("--".to_owned(), |(t, n)| format!("{n}개@{t:.2}s"))
        };
        let impact = base.as_ref().and_then(|(_, i, _)| *i);
        println!(
            "{name:<9} {:>10} {:>10} {:>8}",
            cell(stereo),
            cell(plane),
            impact.map_or("--".to_owned(), |v| format!("{v:.2}")),
        );
    }
    plane_pairs.sort_unstable();
    println!(
        "  평면이 걸린 순간의 표본 수: 최소 {} 중앙 {} 최대 {} — 흩어지면 다른 축이다",
        plane_pairs.first().copied().unwrap_or(0),
        plane_pairs.get(plane_pairs.len() / 2).copied().unwrap_or(0),
        plane_pairs.last().copied().unwrap_or(0),
    );

    // (절대 리드타임 평균 ms, 기준 대비 이득 ms, 접수 평면 오차 중앙값 cm,
    //  계약 선 클립 수, 전체 클립 수)
    let eval = |stereo: usize, y_frac: f64| -> Option<(f64, f64, f64, usize, usize)> {
        let mut lead_ms = Vec::new();
        let mut gain_ms = Vec::new();
        let mut plane_errs = Vec::new();
        let n_total = detected.len();
        let mut n_triggered = 0usize;
        for ((_, cache), base) in detected.iter().zip(&baseline) {
            let Some((base_t, _, _)) = base else { continue };
            let Some((t, impact, errs)) = run(
                cache,
                &calibration,
                physics,
                make_trigger(stereo, y_frac),
            ) else {
                continue;
            };
            n_triggered += 1;
            gain_ms.push((base_t - t) * 1000.0);
            if let Some(impact) = impact {
                lead_ms.push((impact - t) * 1000.0);
            }
            plane_errs.extend(errs);
        }
        if gain_ms.is_empty() {
            return None;
        }
        let mean = |values: &[f64]| -> f64 {
            return values.iter().sum::<f64>() / values.len().max(1) as f64;
        };
        return Some((
            mean(&lead_ms),
            mean(&gain_ms),
            median(&mut plane_errs),
            n_triggered,
            n_total,
        ));
    };

    // `vision::fit`의 e·mu·drag는 측정값이 아니라 **`ω=0` 가정을 흡수하려고** 올려 둔
    // 보정치다(e=0.8617 vs 참값 0.72). 스핀 발동률이 오르면 그 보정이 필요한 몫도 줄어야
    // 정상이라고 문서가 예고해 뒀는데, 지금이 그 시점이다 — 정직한 값으로 되돌렸을 때
    // 정말 나아지는지 클립별로 본다(평균 하나로는 어느 클립이 움직였는지 안 보인다).
    let honest = defaults::PhysicsParams::default();
    let configs: [(&str, defaults::PhysicsParams); 3] = [
        ("지금(보정)", physics),
        (
            "e만 참값",
            defaults::PhysicsParams { restitution: honest.restitution, ..physics },
        ),
        ("전부 참값", defaults::PhysicsParams { drag: physics.drag.min(honest.drag), ..honest }),
    ];
    println!("\n하방 물리별 — 클립마다 접수 평면 오차 중앙값 [cm], 괄호는 스핀 풀림 여부");
    print!("{:<12}", "");
    for (name, _) in &detected {
        print!("{name:>10}");
    }
    println!("{:>8}", "평균");
    for (label, candidate) in &configs {
        print!("{label:<12}");
        let mut all = Vec::new();
        for (_, cache) in &detected {
            let cell = match run_full(cache, &calibration, *candidate, defaults::vision::trigger()) {
                Some((_, _, mut errs, _, spin)) if !errs.is_empty() => {
                    let value = median(&mut errs);
                    all.push(value);
                    format!("{value:.1}{}", if spin { "*" } else { "" })
                }
                _ => "--".to_owned(),
            };
            print!("{cell:>10}");
        }
        println!("{:>8.2}", all.iter().sum::<f64>() / all.len().max(1) as f64);
    }
    println!("  * = 그 클립에서 바운스 스핀이 풀렸다는 뜻");

    // 조립의 두 축이 곧 이 격자다. 둘은 서로 다른 걸 재므로(표본 수 = 정보량,
    // 평면 = 외삽 거리) 한쪽만 훑으면 다른 쪽이 병목인 클립을 못 본다.
    const STEREOS: [usize; 6] = [0, 4, 6, 8, 10, 12];
    const Y_FRACS: [f64; 5] = [0.65, 0.70, 0.75, 0.82, 0.90];

    println!(
        "\n{:>7} {:>7} {:>8} {:>8} {:>10} {:>8}  (기준보다 이르면서 안 나빠지면 *)",
        "stereo", "y_frac", "리드ms", "이득ms", "평면오차cm", "걸린수"
    );
    // 목표가 제어에게 시간을 벌어 주는 것이므로 정확도는 기준선을 안 깎는 것만 조건으로
    // 걸고, 그 안에서 리드타임을 최대화한다.
    let mut best: Option<(usize, f64, f64, f64)> = None; // (stereo, y_frac, lead, err)
    for &stereo in &STEREOS {
        for &y_frac in &Y_FRACS {
            let Some((lead, gain, err, n_triggered, n_total)) = eval(stereo, y_frac) else {
                continue;
            };
            let ok = n_triggered == n_total && err <= base_plane_median && gain > 0.0;
            println!(
                "{stereo:>7} {y_frac:>7.2} {lead:>8.0} {gain:>8.0} {err:>10.2}    {n_triggered}/{n_total}  {}",
                if ok { "*" } else { "" },
            );
            if ok && best.is_none_or(|(_, _, top, _)| lead > top) {
                best = Some((stereo, y_frac, lead, err));
            }
        }
    }

    match best {
        Some((stereo, y_frac, lead, err)) => println!(
            "\n최선(정확도 기준선 유지하면서 가장 이른 트리거): stereo={stereo} y_frac={y_frac:.2}\n  리드타임 {lead:.0}ms, 평면오차 {err:.2}cm (기준 {base_plane_median:.2}cm, 지금 stereo={base_stereo} y_frac={base_y_frac:.2})"
        ),
        None => println!(
            "\n정확도를 안 깎으면서 기준보다 이르게 거는 조합이 격자 안에 없다 — 지금 트리거가 이미 그 방향의 끝이다"
        ),
    }
    return Ok(());
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
    if args.trigger_sweep {
        return run_trigger_sweep();
    }
    if args.calibrate {
        return run_calibrate();
    }
    if args.all {
        return run_all(&args.out);
    }
    if args.sim_child {
        return sim_child::run();
    }
    if args.grid {
        return run_grid(&args);
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
        "frames={} observed={} ({:.0}% 삼각측량) fps={fps:.1} 첫 공={}  재생 0~{}",
        reviewed.len(),
        reviewed.observed.len(),
        100.0 * reviewed.observed.len() as f64 / reviewed.len() as f64,
        first_ball.map_or("없음".to_owned(), |f| f.to_string()),
        reviewed.last_frame()
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
    // kiss3d OrbitCamera3d 기본 바인딩. 어디에도 안 적혀 있어 매번 다시 찾게 된다.
    println!("sim: 왼쪽 드래그 회전 | 오른쪽 드래그 이동 | 휠 확대 | Enter 초점 리셋");
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
    // 샷이 끝나면 멈춘다. 그 뒤는 되돌아가는 공이라 볼 게 없다.
    let last_frame = reviewed.last_frame();
    let mut index = args.start.min(last_frame);
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
            match Step::of(highgui::wait_key_ex(PAUSED_POLL_MS)?, &mut paused) {
                Step::Quit => break,
                Step::Delta(delta) => index = shift(last, delta, last_frame + 1),
                Step::Home => index = 0,
                _ => index = last,
            }
            continue;
        };
        let state = &reviewed.frames[index];
        // 그리는 건 로봇 마운트까지만 — 그 뒤는 로봇이 이미 지나친 자리다. 예측 자체는
        // 여전히 끝까지 적분한다 (평면 통과·MISS 계산에 필요하다).
        let actual_past = track::clip_to_mount(&reviewed.observed_to(index));
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

        // 이번 프레임에 뭘 봤을 때만 찍는다. 검출이 없으면 적합 상태가 그대로라 같은 줄이
        // 수백 개 쌓이고, 놓친 구간은 프레임 번호가 건너뛰는 것으로 이미 보인다.
        if state.pixels.iter().any(Option::is_some) && printed != Some(index) {
            printed = Some(index);
            println!("{}", console_line(&reviewed, state, index));
        }

        let mut panels = Vec::with_capacity(2);
        for (slot, frame) in [(0usize, &left), (1usize, &right)] {
            let hud = overlay_hud(&reviewed, &score, state, index, slot, speed, paused, fast);
            let panel = overlay::draw(
                &frame.image,
                &params[slot],
                &overlay::Tracks {
                    actual_past: &actual_past,
                    filtered: &filtered,
                    committed: &committed,
                },
                state.pixels[slot],
                state.filtered.map(|s| s.position),
                &format!("cam{slot}"),
                &hud,
            )?;
            panels.push(panel);
        }
        // 좌우로 붙여 한 창에 띄운다.
        let mosaic = Preview::hstack_bgr(&panels)?;
        let shown = Preview::fit_bgr_downscale(
            &mosaic,
            (f64::from(mosaic.cols()) * args.scale).round() as i32,
            (f64::from(mosaic.rows()) * args.scale).round() as i32,
        )?;
        highgui::imshow(WINDOW, &shown.image)?;

        if let Some((_, stdin)) = &mut sim {
            let message = SceneMsg {
                ekf: state.filtered.map(|s| s.position.into()),
                raw: state.observed.map(|o| o.point.into()),
                observed: actual_past.iter().copied().map(Into::into).collect(),
                filtered: filtered.iter().copied().map(Into::into).collect(),
                committed: committed.iter().copied().map(Into::into).collect(),
            };
            if writeln!(stdin, "{}", message.to_line()).is_err() || stdin.flush().is_err() {
                // 사용자가 sim 창을 닫았다 — 조용히 그만 보낸다.
                sim = None;
            }
        }

        // 일시정지여도 0 을 넘기지 않는다. macOS highgui 는 `waitKey(0)` 에서 창 이벤트를
        // 안 돌려서 `q` 가 안 먹는다 — 저장소의 다른 툴이 전부 `.max(1)` 을 쓰는 이유다.
        // 짧게 폴링하면 같은 "멈춰 있음"이면서 키는 받는다.
        let wait = match (paused, fast) {
            (true, _) => PAUSED_POLL_MS,
            (false, true) => live_wait_ms,
            (false, false) => slow_wait_ms,
        };
        let key = highgui::wait_key_ex(wait)?;
        match Step::of(key, &mut paused) {
            Step::Quit => break,
            Step::Delta(delta) => index = shift(index, delta, last_frame + 1),
            Step::Home => index = 0,
            Step::Stay => {}
            Step::Advance => {
                // 끝에 닿으면 **멈춰 선다**. 자동 종료는 `q`/ESC로만.
                if index + 1 > last_frame {
                    paused = true;
                } else {
                    index += 1;
                }
            }
        }
    }

    Preview::destroy_window(WINDOW);
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
        // `wait_key_ex` 는 수정자·플랫폼 비트를 위쪽에 얹어 준다. 화살표는 그 비트를 봐야
        // 하지만 글자 키는 하위 8비트만 보면 된다 — 안 그러면 `q` 가 안 먹는 순간이 있다.
        let ascii = key & 0xFF;
        if ascii == 27 || ascii == i32::from(b'q') || ascii == i32::from(b'Q') {
            return Self::Quit;
        }
        if ascii == i32::from(b' ') {
            *paused = !*paused;
            return Self::Stay;
        }
        if let Some((dx, _)) = Preview::arrow_delta(key)
            && dx != 0
        {
            *paused = true;
            return Self::Delta(i64::from(dx));
        }
        let delta = match ascii {
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

#[cfg(test)]
mod tests {
    use super::*;

    /// `wait_key_ex` 는 수정자·플랫폼 비트를 위쪽에 얹는다. 그걸 안 벗기면 `q` 가 안 먹는다.
    #[test]
    fn quit_survives_the_modifier_bits() {
        let mut paused = false;
        for key in [
            i32::from(b'q'),
            i32::from(b'Q'),
            27,
            // macOS 가 얹는 상위 비트가 붙어도 같아야 한다.
            0x10_0000 | i32::from(b'q'),
            0xF000 | 27,
        ] {
            assert!(
                matches!(Step::of(key, &mut paused), Step::Quit),
                "key={key:#x}"
            );
        }
    }

    /// 화살표는 상위 비트가 곧 정보다 — 하위 8비트만 보면 못 알아본다.
    #[test]
    fn arrows_still_step_frames() {
        let mut paused = false;
        assert!(matches!(Step::of(0x25 << 16, &mut paused), Step::Delta(-1)));
        assert!(paused, "프레임 이동은 일시정지로 들어간다");
        assert!(matches!(Step::of(0x27 << 16, &mut paused), Step::Delta(1)));
    }

    #[test]
    fn space_toggles_pause() {
        let mut paused = false;
        assert!(matches!(Step::of(i32::from(b' '), &mut paused), Step::Stay));
        assert!(paused);
        assert!(matches!(Step::of(i32::from(b' '), &mut paused), Step::Stay));
        assert!(!paused);
    }
}
