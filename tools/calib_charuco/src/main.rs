//! ChArUco 보드 인터랙티브 보정.
//!
//! 기본: 라이브 캡처에서 코너를 확인·선별 저장한 뒤, 종료 시 Calibration JSON을 쓴다.
//! 보조: `--from-images` / `--emit-sim` / `--validate`.

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use clap::Parser;
use opencv::core::{Mat, Scalar};
use opencv::imgcodecs;
use opencv::prelude::*;
use pingpong_bot::{
    Calibration, CameraId, CharucoBoardSpec, DEFAULT_CONFIG_PATH, FrameSource,
    MIN_CHARUCO_CORNERS, OpenCvCapture, PreviewAction, calibrate_charuco,
    calibration_path_from_config, destroy_window, detect_and_draw_charuco, draw_debug_lines,
    show_bgr,
};

#[derive(Parser, Debug)]
#[command(
    name = "calib_charuco",
    about = "ChArUco 인터랙티브 보정 — Space 스냅·코너 확인·s 저장·종료 시 JSON"
)]
struct Args {
    /// 웹캠 인덱스 (미지정 시 0으로 인터랙티브)
    #[arg(long)]
    device: Option<i32>,

    /// 동영상 파일로 같은 UX
    #[arg(long)]
    path: Option<PathBuf>,

    /// 선별 프레임 저장 디렉터리 (기본 calib_frames/cam{id})
    #[arg(long, value_name = "DIR")]
    images_dir: Option<PathBuf>,

    /// 출력 Calibration JSON. 생략 시 --config 의 calibration_path, 없으면 calibration.json
    #[arg(short = 'o', long)]
    output: Option<PathBuf>,

    /// 런타임 TOML (기본 출력 경로용 calibration_path)
    #[arg(long, value_name = "PATH", default_value = DEFAULT_CONFIG_PATH)]
    config: PathBuf,

    /// 종료 시 보정에 필요한 최소 저장 장수
    #[arg(long, default_value_t = 10)]
    min_frames: usize,

    #[arg(long, default_value_t = 0)]
    camera_id: u8,

    #[arg(long)]
    emit_sim: Option<u8>,

    #[arg(long)]
    validate: Option<PathBuf>,

    /// UI 없이 이미지 폴더만으로 보정
    #[arg(long)]
    from_images: Option<PathBuf>,

    #[arg(long, default_value_t = 5)]
    squares_x: i32,
    #[arg(long, default_value_t = 7)]
    squares_y: i32,
    #[arg(long, default_value_t = 0.04)]
    square_length: f32,
    #[arg(long, default_value_t = 0.02)]
    marker_length: f32,
}

fn board_spec(args: &Args) -> CharucoBoardSpec {
    return CharucoBoardSpec {
        squares_x: args.squares_x,
        squares_y: args.squares_y,
        square_length_m: args.square_length,
        marker_length_m: args.marker_length,
    };
}

fn resolve_output(args: &Args) -> PathBuf {
    if let Some(ref out) = args.output {
        return out.clone();
    }
    if let Ok(Some(path)) = calibration_path_from_config(&args.config) {
        return path;
    }
    return PathBuf::from("calibration.json");
}

fn main() -> Result<()> {
    let args = Args::parse();

    if let Some(path) = args.validate {
        let text =
            fs::read_to_string(&path).with_context(|| format!("읽기 실패: {}", path.display()))?;
        let calib: Calibration = serde_json::from_str(&text)?;
        for cam in &calib.cameras {
            println!(
                "  cam {}: {}x{} fx={:.1} dist_len={}",
                cam.camera_id.index(),
                cam.width,
                cam.height,
                cam.fx,
                cam.dist.len()
            );
        }
        println!(
            "ok: {} cameras, min_triangulation={}",
            calib.camera_count(),
            calib.min_cameras_for_triangulation()
        );
        return Ok(());
    }

    if let Some(n) = args.emit_sim {
        let output = resolve_output(&args);
        let calib = Calibration::sim(n);
        let json = serde_json::to_string_pretty(&calib)?;
        fs::write(&output, json).with_context(|| format!("쓰기 실패: {}", output.display()))?;
        println!(
            "wrote sim Calibration ({} cams, dist=[]) → {}",
            n,
            output.display()
        );
        return Ok(());
    }

    if let Some(dir) = &args.from_images {
        return write_calib_from_dir(dir, &args);
    }

    return run_interactive(&args);
}

fn write_calib_from_dir(dir: &PathBuf, args: &Args) -> Result<()> {
    let output = resolve_output(args);
    let (calib, report) = calibrate_charuco(dir, board_spec(args), CameraId(args.camera_id))
        .map_err(anyhow::Error::msg)?;
    let json = serde_json::to_string_pretty(&calib)?;
    fs::write(&output, json).with_context(|| format!("쓰기 실패: {}", output.display()))?;
    println!(
        "wrote ChArUco Calibration → {} (rms={:.4}, frames={}/{})",
        output.display(),
        report.rms,
        report.frames_used,
        report.frames_total
    );
    return Ok(());
}

fn default_images_dir(camera_id: u8) -> PathBuf {
    return PathBuf::from(format!("calib_frames/cam{camera_id}"));
}

fn count_images(dir: &PathBuf) -> usize {
    let Ok(rd) = fs::read_dir(dir) else {
        return 0;
    };
    return rd
        .filter_map(|e| e.ok())
        .filter(|e| {
            matches!(
                e.path().extension().and_then(|x| x.to_str()),
                Some("png" | "jpg" | "jpeg")
            )
        })
        .count();
}

fn next_save_path(dir: &PathBuf, index: usize) -> PathBuf {
    return dir.join(format!("{index:04}.png"));
}

struct ReviewFrame {
    raw: Mat,
    overlay: Mat,
    ok: bool,
    corners: usize,
    markers: usize,
}

fn run_interactive(args: &Args) -> Result<()> {
    if args.device.is_some() && args.path.is_some() {
        bail!("--device 와 --path 를 같이 쓰지 마세요");
    }

    let images_dir = args
        .images_dir
        .clone()
        .unwrap_or_else(|| default_images_dir(args.camera_id));
    fs::create_dir_all(&images_dir).with_context(|| format!("mkdir {}", images_dir.display()))?;

    let cam_id = CameraId(args.camera_id);
    let mut source: Box<dyn FrameSource> = if let Some(path) = &args.path {
        Box::new(
            OpenCvCapture::from_path(cam_id, path)
                .map_err(anyhow::Error::msg)
                .context("path")?,
        )
    } else {
        let device = args.device.unwrap_or(0);
        Box::new(
            OpenCvCapture::from_device(cam_id, device)
                .map_err(anyhow::Error::msg)
                .with_context(|| format!("device {device}"))?,
        )
    };

    let window = "calib:charuco";
    let spec = board_spec(args);
    let output = resolve_output(args);
    let mut review: Option<ReviewFrame> = None;
    let mut save_index = count_images(&images_dir);

    println!(
        "인터랙티브 보정 — dir={}  min_frames={}  -o {}",
        images_dir.display(),
        args.min_frames,
        output.display()
    );
    println!("Space=스냅+코너  s=저장  n=생략  q=종료(+calib)");

    loop {
        let action = if let Some(ref rev) = review {
            let mut panel = rev
                .overlay
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            let status = if rev.ok {
                format!(
                    "REVIEW OK  corners={} markers={} (≥{MIN_CHARUCO_CORNERS})",
                    rev.corners, rev.markers
                )
            } else {
                format!(
                    "REVIEW FAIL  corners={} markers={} — 저장 불가, n으로 생략",
                    rev.corners, rev.markers
                )
            };
            let color = if rev.ok {
                Scalar::new(0.0, 255.0, 0.0, 0.0)
            } else {
                Scalar::new(0.0, 0.0, 255.0, 0.0)
            };
            let lines = [
                status,
                format!("saved={save_index}  s=save  n=skip  q=quit"),
            ];
            draw_debug_lines(&mut panel, &lines, color)?;
            show_bgr(window, &panel, 30)?
        } else {
            let Some(frame) = source.next_frame() else {
                println!("입력 스트림 종료");
                break;
            };
            let mut panel = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            let lines = [
                format!("LIVE  saved={save_index} (need ≥{})", args.min_frames),
                "Space=snap+detect  q=quit(+calib)".into(),
            ];
            draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
            // space를 받기 위해 라이브 프레임도 잠시 들고 있음
            let action = show_bgr(window, &panel, 1)?;
            if matches!(action, PreviewAction::Key(k) if k == i32::from(b' ')) {
                let raw = frame
                    .image
                    .try_clone()
                    .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
                let (overlay, det) =
                    detect_and_draw_charuco(&raw, spec).map_err(anyhow::Error::msg)?;
                println!(
                    "snap: corners={} markers={} ok={}",
                    det.corners, det.markers, det.ok
                );
                review = Some(ReviewFrame {
                    raw,
                    overlay,
                    ok: det.ok,
                    corners: det.corners,
                    markers: det.markers,
                });
                continue;
            }
            action
        };

        match action {
            PreviewAction::Quit => break,
            PreviewAction::Continue => {}
            PreviewAction::Key(k) if k == i32::from(b' ') => {
                // review 중 Space는 무시 (라이브에서만)
            }
            PreviewAction::Key(k) if k == i32::from(b's') || k == i32::from(b'S') => {
                let Some(rev) = review.take() else {
                    continue;
                };
                if !rev.ok {
                    println!("저장 거부 — 코너 부족 (n으로 생략)");
                    continue;
                }
                let out = next_save_path(&images_dir, save_index);
                let path_str = out.to_str().context("non-utf8 save path")?;
                imgcodecs::imwrite(path_str, &rev.raw, &opencv::core::Vector::new())
                    .with_context(|| format!("imwrite {}", out.display()))?;
                println!("saved {} (corners={})", out.display(), rev.corners);
                save_index += 1;
            }
            PreviewAction::Key(k) if k == i32::from(b'n') || k == i32::from(b'N') => {
                if review.take().is_some() {
                    println!("skip");
                }
            }
            PreviewAction::Key(_) => {}
        }
    }

    destroy_window(window);

    let n = count_images(&images_dir);
    if n < args.min_frames {
        println!(
            "저장 {n}장 < min_frames={} — calibrate 생략. 이후:\n  \
             cargo run -p calib-charuco -- --from-images {} -o {}",
            args.min_frames,
            images_dir.display(),
            output.display()
        );
        return Ok(());
    }

    println!("calibrate from {n} images in {} …", images_dir.display());
    return write_calib_from_dir(&images_dir, args);
}
