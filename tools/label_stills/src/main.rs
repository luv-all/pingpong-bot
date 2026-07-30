//! 클립 타임라인 등분 덤프 + 클릭 라벨 → `data/detect_stills/manifest.json`.
//!
//! 비디오 전 프레임 GT는 비범위. 캠·클립당 ~10장, 그중 2~3장은 **무공**(`n`)으로 남긴다.
//! 키: LMB/Enter 공 중심 · 화살표 1px · Shift 확대 · `n` 무공 · `z` 이전 장 · `q`/ESC 저장·종료

mod cli;

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, bail};
use clap::Parser;
use opencv::core::Scalar;
use opencv::prelude::*;
use opencv::{highgui, imgcodecs};
use pingpong_bot::camera;
use pingpong_bot::camera::{PixelPickMouse, Preview, PreviewAction};
use pingpong_bot::defaults::{detect_stills_dir, detect_stills_manifest_path, ensure_parent_dir};
use pingpong_bot::detector::{StillItem, StillsManifest};

use cli::Args;

/// 클립 이름 (`data/clips/fly_01` → `fly_01`).
fn clip_name(args: &Args) -> Result<String> {
    let Some(clip) = &args.offline.clip else {
        bail!("--clip 필수 — 라이브에서는 스틸 GT를 뽑지 않는다");
    };
    let name = clip
        .file_name()
        .and_then(|s| s.to_str())
        .context("clip 이름")?;
    return Ok(name.to_string());
}

/// 총 `total`장에서 `count`장을 고를 때의 프레임 간격. 한 bounce에 몰리지 않게 등분한다.
fn stride_for(total: usize, count: usize) -> usize {
    let count = count.max(1).min(total.max(1));
    return (total / count).max(1);
}

/// 타임라인을 `count` 등분해 프레임을 고른다. 두 번 여는 이유는 총 프레임 수를 먼저 세기 위함.
fn sample_frames(args: &Args, count: usize) -> Result<Vec<(usize, Mat)>> {
    let mut counter = args
        .cam
        .open_mono_input(&args.offline)
        .map_err(anyhow::Error::msg)?;
    let mut total = 0usize;
    while counter.next_frame().is_some() {
        total += 1;
    }
    if total == 0 {
        bail!("클립에 프레임이 없다");
    }
    let count = count.max(1).min(total);
    let stride = stride_for(total, count);
    println!("clip frames={total} → {count}장 (stride={stride})");

    let mut source = args
        .cam
        .open_mono_input(&args.offline)
        .map_err(anyhow::Error::msg)?;
    let mut picked = Vec::with_capacity(count);
    let mut index = 0usize;
    while let Some(frame) = source.next_frame() {
        if index % stride == 0 && picked.len() < count {
            let image = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            picked.push((index, image));
        }
        index += 1;
    }
    return Ok(picked);
}

fn still_path(clip: &str, role: camera::Role, frame: usize) -> String {
    return format!("{clip}_{role}_t{frame:04}.png");
}

/// PNG 저장 + manifest upsert + 즉시 저장.
fn record(
    manifest: &mut StillsManifest,
    dir: &std::path::Path,
    image: &Mat,
    item: StillItem,
) -> Result<()> {
    let png = dir.join(&item.path);
    ensure_parent_dir(&png)?;
    imgcodecs::imwrite(
        png.to_str().context("png 경로")?,
        image,
        &opencv::core::Vector::new(),
    )?;
    let label = match item.pixel {
        Some([u, v]) => format!("ball ({u:.1},{v:.1})"),
        None => "empty".to_string(),
    };
    println!("saved {} — {label}", item.path);
    manifest.upsert(item);
    manifest.save(&detect_stills_manifest_path())?;
    return Ok(());
}

fn main() -> Result<()> {
    let args = Args::parse();
    let cam_id = args.cam.camera_id().map_err(anyhow::Error::msg)?;
    let role = *args.cam.cam.first().context("--cam 필수")?;
    let clip = clip_name(&args)?;

    let stills = sample_frames(&args, args.count)?;
    let dir = detect_stills_dir();
    let manifest_path = detect_stills_manifest_path();
    let mut manifest = StillsManifest::load_or_default(&manifest_path)?;
    manifest.hit_radius_px = args.hit_radius;

    let window = "label:stills";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    let mouse: Arc<Mutex<PixelPickMouse>> = Arc::new(Mutex::new(PixelPickMouse::default()));
    {
        let mouse = Arc::clone(&mouse);
        highgui::set_mouse_callback(
            window,
            Some(Box::new(move |event, x, y, flags| {
                if let Ok(mut m) = mouse.lock() {
                    m.on_event(event, x, y, flags);
                }
            })),
        )?;
    }

    println!(
        "label-stills cam={} clip={clip} → {}",
        cam_id.0,
        manifest_path.display()
    );
    println!("LMB/Enter=공 중심  arrows=1px  Shift=확대  n=무공  z=이전 장  q/ESC=종료");

    let mut cursor = 0usize;
    let mut display_scale = 1.0;

    while cursor < stills.len() {
        let (frame_index, image) = &stills[cursor];
        let panel_w = image.cols();
        let panel_h = image.rows();
        let path = still_path(&clip, role, *frame_index);
        let done = manifest.items.iter().find(|i| i.path == path).cloned();

        let (clicks, hover) = {
            let mut m = mouse.lock().expect("mouse lock");
            m.sync(display_scale, panel_w, panel_h);
            (m.drain_clicks(), m.hover)
        };

        if let Some((mx, my)) = clicks
            .into_iter()
            .find(|(x, y)| *x >= 0 && *y >= 0 && *x < panel_w && *y < panel_h)
        {
            record(
                &mut manifest,
                &dir,
                image,
                StillItem {
                    path,
                    camera_id: cam_id,
                    clip: clip.clone(),
                    frame: *frame_index,
                    pixel: Some([f64::from(mx), f64::from(my)]),
                },
            )?;
            cursor += 1;
            continue;
        }

        let mut view = image
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
        let status = match &done {
            Some(i) => match i.pixel {
                Some([u, v]) => format!("labeled ball ({u:.0},{v:.0})"),
                None => "labeled empty".to_string(),
            },
            None => "unlabeled".to_string(),
        };
        Preview::draw_debug_lines(
            &mut view,
            &[
                format!("{}/{}  frame={frame_index}", cursor + 1, stills.len()),
                format!("{clip} {role}  cam{}", cam_id.0),
                status,
                format!(
                    "ball={} empty={} radius={:.0}px",
                    manifest.ball_count(),
                    manifest.empty_count(),
                    manifest.hit_radius_px
                ),
            ],
            Scalar::new(0.0, 255.0, 255.0, 0.0),
        )?;
        Preview::draw_help_lines(
            &mut view,
            &[
                "LMB/Enter 공 중심",
                "arrows 1px  Shift 확대",
                "n 무공",
                "z 이전 장",
                "q/ESC 종료",
            ],
            Scalar::new(0.0, 255.0, 80.0, 0.0),
        )?;
        if let Some(prev) = &done {
            if let Some([u, v]) = prev.pixel {
                Preview::draw_circle_px(
                    &mut view,
                    camera::Pixel::new(u, v),
                    12,
                    Scalar::new(0.0, 255.0, 0.0, 0.0),
                    2,
                )?;
            }
        }
        if let Some((hx, hy)) = hover {
            if hx >= 0 && hy >= 0 && hx < panel_w && hy < panel_h {
                let _ = Preview::draw_pixel_loupe(&mut view, image, hx, hy);
            }
        }

        let shown = Preview::show_bgr(window, &view, 30)?;
        display_scale = shown.scale;
        match shown.action {
            PreviewAction::Quit => break,
            PreviewAction::Continue => {}
            PreviewAction::Key(k) => {
                if let Some((dx, dy)) = Preview::arrow_delta(k) {
                    let mut m = mouse.lock().expect("mouse lock");
                    m.sync(display_scale, panel_w, panel_h);
                    m.nudge(dx, dy, panel_w, panel_h);
                    continue;
                }
                if k == 13 || k == 10 {
                    mouse.lock().expect("mouse lock").confirm();
                    continue;
                }
                let key = k & 0xff;
                if key == i32::from(b'n') || key == i32::from(b'N') {
                    record(
                        &mut manifest,
                        &dir,
                        image,
                        StillItem {
                            path,
                            camera_id: cam_id,
                            clip: clip.clone(),
                            frame: *frame_index,
                            pixel: None,
                        },
                    )?;
                    cursor += 1;
                } else if key == i32::from(b'z') || key == i32::from(b'Z') || key == 8 {
                    cursor = cursor.saturating_sub(1);
                    println!("← {}/{}", cursor + 1, stills.len());
                }
            }
        }
    }

    manifest.save(&manifest_path)?;
    Preview::destroy_window(window);
    println!(
        "done  ball={} empty={} → {}",
        manifest.ball_count(),
        manifest.empty_count(),
        manifest_path.display()
    );
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stride_spreads_over_whole_timeline() {
        assert_eq!(stride_for(478, 10), 47);
        assert_eq!(stride_for(100, 10), 10);
        // 요청이 프레임 수보다 많으면 매 프레임
        assert_eq!(stride_for(5, 10), 1);
        assert_eq!(stride_for(1, 10), 1);
        // count=0 방어
        assert_eq!(stride_for(50, 0), 50);
    }

    #[test]
    fn still_path_encodes_clip_role_frame() {
        assert_eq!(
            still_path("fly_01", camera::Role::Left, 48),
            "fly_01_left_t0048.png"
        );
        assert_eq!(
            still_path("drop_02", camera::Role::Right, 0),
            "drop_02_right_t0000.png"
        );
    }

    /// 실제 클립을 두 번 열어 등분 샘플이 나오는지 (GUI 없이 확인 가능한 절반).
    /// 툴 테스트의 CWD는 크레이트 디렉터리이므로 클립은 절대 경로로 준다.
    #[test]
    fn samples_evenly_from_real_clip() {
        let clip = concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/clips/fly_01");
        let args = Args::parse_from(["label-stills", "--cam", "left", "--clip", clip]);
        let stills = sample_frames(&args, 5).expect("clip sampling");
        assert_eq!(stills.len(), 5);
        assert_eq!(stills[0].0, 0);
        let step = stills[1].0 - stills[0].0;
        assert!(step > 1, "frames should be spread out, got step={step}");
        for (_, image) in &stills {
            assert_eq!(image.cols(), 1280);
            assert_eq!(image.rows(), 800);
        }
    }
}
