//! 스테레오 프리롤 녹화 — 공 던진 뒤 Space로 최근 N초+포스트롤을 저장.
//!
//! 캡처는 전용 스레드(L→R 순차 grab). 링 버퍼는 JPEG로 보관해 RAM을 줄인다.

use std::collections::VecDeque;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use clap::{Parser, ValueEnum};
use opencv::core::{Mat, Scalar, Size, Vector};
use opencv::imgcodecs;
use opencv::prelude::*;
use opencv::videoio::{VideoWriter, VideoWriterTrait};
use pingpong_bot::{
    FrameSource, OpenCvCapture, PreviewAction, StereoPairCliArgs, destroy_window, draw_cam_label,
    draw_debug_lines, draw_help_lines, hstack_bgr, show_bgr,
};
use serde::Serialize;

const WINDOW: &str = "record_stereo";
const JPEG_QUALITY: i32 = 85;
const RING_MARGIN_SECS: f64 = 1.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum Scene {
    Fly,
    Roll,
    Drop,
}

impl Scene {
    fn as_str(self) -> &'static str {
        return match self {
            Self::Fly => "fly",
            Self::Roll => "roll",
            Self::Drop => "drop",
        };
    }
}

#[derive(Parser, Debug)]
#[command(name = "record-stereo")]
struct Args {
    #[command(flatten)]
    cam: StereoPairCliArgs,

    /// 장면 태그 (클립 디렉터리 prefix). 실행 중 변경 없음.
    #[arg(long, value_enum, default_value_t = Scene::Fly)]
    scene: Scene,

    /// 출력 루트
    /// 클립 루트 (`data/clips/{scene}_{nn}/`)
    #[arg(long, default_value = "data/clips")]
    out: PathBuf,

    /// Space 기준 과거 보관 초
    #[arg(long, default_value_t = 10.0)]
    preroll: f64,

    /// Space 이후 추가 녹화 초
    #[arg(long, default_value_t = 2.0)]
    postroll: f64,
}

#[derive(Clone)]
struct PairSample {
    t: Instant,
    left_jpeg: Vec<u8>,
    right_jpeg: Vec<u8>,
    width: i32,
    height: i32,
}

struct PreviewSlot {
    left: Mat,
    right: Mat,
    grab_fps: f64,
    ring_secs: f64,
    ring_pairs: usize,
}

impl PreviewSlot {
    fn try_clone(&self) -> Result<Self> {
        return Ok(Self {
            left: self
                .left
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone left: {e}"))?,
            right: self
                .right
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone right: {e}"))?,
            grab_fps: self.grab_fps,
            ring_secs: self.ring_secs,
            ring_pairs: self.ring_pairs,
        });
    }
}

enum CaptureCmd {
    /// trigger_at = Space 시각. postroll 끝난 뒤 클립 flush.
    Save {
        trigger_at: Instant,
        preroll: Duration,
        postroll: Duration,
        dir: PathBuf,
        scene: String,
        request_fps: f64,
        backend: String,
        fourcc: String,
        width: i32,
        height: i32,
    },
    Stop,
}

#[derive(Default)]
struct CaptureShared {
    preview: Option<PreviewSlot>,
    /// 마지막 저장 결과 메시지 (HUD/콘솔)
    last_status: Option<String>,
    error: Option<String>,
}

#[derive(Serialize)]
struct ClipMeta {
    scene: String,
    preroll_secs: f64,
    postroll_secs: f64,
    width: i32,
    height: i32,
    request_fps: f64,
    meas_fps: f64,
    writer_fps: f64,
    fourcc: String,
    backend: String,
    frames: usize,
    created_unix_secs: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.preroll <= 0.0 || args.postroll < 0.0 {
        bail!("--preroll > 0, --postroll >= 0 필요");
    }

    let cam = args.cam.as_cam_cli();
    let backend = cam.stream.backend().map_err(anyhow::Error::msg)?;
    let resolved = cam.resolve().map_err(anyhow::Error::msg)?;
    if resolved.len() != 2 {
        bail!(
            "record-stereo는 left+right 두 대 필요 (got {})",
            resolved.len()
        );
    }
    let left_r = resolved[0];
    let right_r = resolved[1];

    fs::create_dir_all(&args.out).with_context(|| format!("create {}", args.out.display()))?;

    let mut left_cap =
        OpenCvCapture::from_device_with_backend(left_r.camera_id, left_r.device, backend)
            .map_err(anyhow::Error::msg)?;
    let mut right_cap =
        OpenCvCapture::from_device_with_backend(right_r.camera_id, right_r.device, backend)
            .map_err(anyhow::Error::msg)?;
    cam.stream
        .apply(&mut left_cap)
        .map_err(anyhow::Error::msg)?;
    cam.stream
        .apply(&mut right_cap)
        .map_err(anyhow::Error::msg)?;

    let (req_w, req_h) = cam.stream.resolved_size();
    let request_fps = cam.stream.fps;
    let backend_s = backend.as_str().to_string();
    let fourcc_s = cam.stream.fourcc.clone();

    let shared = Arc::new(Mutex::new(CaptureShared::default()));
    let cmd_slot: Arc<Mutex<Option<CaptureCmd>>> = Arc::new(Mutex::new(None));
    let stop = Arc::new(AtomicBool::new(false));

    let shared_t = Arc::clone(&shared);
    let cmd_t = Arc::clone(&cmd_slot);
    let stop_t = Arc::clone(&stop);
    let ring_keep = Duration::from_secs_f64(args.preroll + args.postroll + RING_MARGIN_SECS);

    let grab: JoinHandle<()> = thread::spawn(move || {
        grab_loop(left_cap, right_cap, shared_t, cmd_t, stop_t, ring_keep);
    });

    let (sw, sh) = cam.stream.resolved_size();
    println!(
        "record-stereo scene={} out={} preroll={:.0}s postroll={:.0}s  {}x{}@{:.0} {} backend={}",
        args.scene.as_str(),
        args.out.display(),
        args.preroll,
        args.postroll,
        sw,
        sh,
        request_fps,
        fourcc_s,
        backend_s
    );
    println!("Space=save clip  q/ESC=quit  (던진 뒤 데탑에서 Space)");

    let mut saving = false;
    loop {
        if let Ok(g) = shared.lock() {
            if let Some(err) = &g.error {
                let msg = err.clone();
                drop(g);
                stop.store(true, Ordering::Release);
                let _ = grab.join();
                bail!("{msg}");
            }
        }

        let (preview, status) = {
            let Ok(g) = shared.lock() else {
                thread::sleep(Duration::from_millis(20));
                continue;
            };
            let preview = g.preview.as_ref().map(|p| p.try_clone()).transpose()?;
            (preview, g.last_status.clone())
        };

        let Some(prev) = preview else {
            thread::sleep(Duration::from_millis(20));
            continue;
        };

        let mut left = prev
            .left
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
        let mut right = prev
            .right
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;

        let left_lines = [
            format!("{}#{}", left_r.role, left_r.camera_id.0),
            format!("grab {:.1}", prev.grab_fps),
            format!("ring {:.1}s ({} pairs)", prev.ring_secs, prev.ring_pairs),
        ];
        let right_lines = [
            format!("{}#{}", right_r.role, right_r.camera_id.0),
            format!("scene {}", args.scene.as_str()),
        ];
        draw_debug_lines(&mut left, &left_lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
        draw_debug_lines(
            &mut right,
            &right_lines,
            Scalar::new(0.0, 255.0, 255.0, 0.0),
        )?;
        draw_cam_label(
            &mut left,
            left_r.role.as_str(),
            Scalar::new(0.0, 255.0, 255.0, 0.0),
        )?;
        draw_cam_label(
            &mut right,
            right_r.role.as_str(),
            Scalar::new(0.0, 255.0, 255.0, 0.0),
        )?;

        let mut mosaic = hstack_bgr(&[left, right])?;
        let mut help = vec!["Space save", "q quit"];
        if saving {
            help.insert(0, "SAVING…");
        }
        if let Some(s) = &status {
            help.push(s.as_str());
        }
        draw_help_lines(&mut mosaic, &help, Scalar::new(0.0, 255.0, 80.0, 0.0))?;

        match show_bgr(WINDOW, &mosaic, 1)?.action {
            PreviewAction::Quit => break,
            PreviewAction::Continue => {}
            PreviewAction::Key(key) if key == i32::from(b' ') => {
                if saving {
                    println!("save already in progress");
                    continue;
                }
                let dir = next_clip_dir(&args.out, args.scene.as_str())?;
                fs::create_dir_all(&dir).with_context(|| format!("create {}", dir.display()))?;
                if let Ok(mut g) = shared.lock() {
                    g.last_status = None;
                }
                let trigger_at = Instant::now();
                *cmd_slot.lock().expect("cmd lock") = Some(CaptureCmd::Save {
                    trigger_at,
                    preroll: Duration::from_secs_f64(args.preroll),
                    postroll: Duration::from_secs_f64(args.postroll),
                    dir: dir.clone(),
                    scene: args.scene.as_str().to_string(),
                    request_fps,
                    backend: backend_s.clone(),
                    fourcc: fourcc_s.clone(),
                    width: req_w,
                    height: req_h,
                });
                saving = true;
                println!(
                    "save armed → {} (postroll {:.0}s…)",
                    dir.display(),
                    args.postroll
                );
            }
            PreviewAction::Key(_) => {}
        }

        // 저장 완료 감지 (Space 때 last_status 를 비움)
        if saving {
            if let Ok(g) = shared.lock() {
                if g.last_status.is_some() {
                    saving = false;
                }
            }
        }
    }

    *cmd_slot.lock().expect("cmd lock") = Some(CaptureCmd::Stop);
    stop.store(true, Ordering::Release);
    let _ = grab.join();
    destroy_window(WINDOW);
    return Ok(());
}

fn grab_loop(
    mut left: OpenCvCapture,
    mut right: OpenCvCapture,
    shared: Arc<Mutex<CaptureShared>>,
    cmd_slot: Arc<Mutex<Option<CaptureCmd>>>,
    stop: Arc<AtomicBool>,
    ring_keep: Duration,
) {
    let mut ring: VecDeque<PairSample> = VecDeque::new();
    let mut pending: Option<CaptureCmd> = None;
    let mut fps_window_start = Instant::now();
    let mut fps_window_count = 0u64;
    let mut grab_fps = 0.0f64;

    while !stop.load(Ordering::Relaxed) {
        if let Ok(mut slot) = cmd_slot.lock() {
            if let Some(cmd) = slot.take() {
                match cmd {
                    CaptureCmd::Stop => break,
                    other => pending = Some(other),
                }
            }
        }

        let Some(lf) = left.next_frame() else {
            set_error(&shared, "left: 프레임 없음");
            break;
        };
        let Some(rf) = right.next_frame() else {
            set_error(&shared, "right: 프레임 없음");
            break;
        };

        let t = Instant::now();
        let width = lf.image.cols();
        let height = lf.image.rows();
        let Ok(left_jpeg) = encode_jpeg(&lf.image) else {
            set_error(&shared, "left: JPEG encode 실패");
            break;
        };
        let Ok(right_jpeg) = encode_jpeg(&rf.image) else {
            set_error(&shared, "right: JPEG encode 실패");
            break;
        };

        ring.push_back(PairSample {
            t,
            left_jpeg,
            right_jpeg,
            width,
            height,
        });
        trim_ring(&mut ring, t, ring_keep);

        fps_window_count += 1;
        let elapsed = fps_window_start.elapsed().as_secs_f64();
        if elapsed >= 1.0 {
            grab_fps = fps_window_count as f64 / elapsed;
            fps_window_count = 0;
            fps_window_start = Instant::now();
        }

        let ring_secs = ring
            .front()
            .map(|f| t.duration_since(f.t).as_secs_f64())
            .unwrap_or(0.0);
        if let Ok(mut g) = shared.lock() {
            g.preview = Some(PreviewSlot {
                left: lf.image,
                right: rf.image,
                grab_fps,
                ring_secs,
                ring_pairs: ring.len(),
            });
        }

        if let Some(CaptureCmd::Save {
            trigger_at,
            preroll,
            postroll,
            dir,
            scene,
            request_fps,
            backend,
            fourcc,
            width: req_w,
            height: req_h,
        }) = pending.take()
        {
            let until = trigger_at + postroll;
            if t < until {
                // 포스트롤 아직 — 다시 대기
                pending = Some(CaptureCmd::Save {
                    trigger_at,
                    preroll,
                    postroll,
                    dir,
                    scene,
                    request_fps,
                    backend,
                    fourcc,
                    width: req_w,
                    height: req_h,
                });
                continue;
            }

            let from = trigger_at.checked_sub(preroll).unwrap_or(trigger_at);
            let clip: Vec<PairSample> = ring
                .iter()
                .filter(|p| p.t >= from && p.t <= until)
                .cloned()
                .collect();

            let status = match write_clip(
                &dir,
                &scene,
                &clip,
                request_fps,
                grab_fps,
                &backend,
                &fourcc,
                preroll.as_secs_f64(),
                postroll.as_secs_f64(),
                req_w,
                req_h,
            ) {
                Ok(meta) => {
                    println!(
                        "saved {} frames={} meas_fps={:.1}",
                        dir.display(),
                        meta.frames,
                        meta.meas_fps
                    );
                    format!(
                        "saved {}",
                        dir.file_name().and_then(|s| s.to_str()).unwrap_or("?")
                    )
                }
                Err(e) => {
                    eprintln!("save failed {}: {e:#}", dir.display());
                    format!("save failed: {e}")
                }
            };
            if let Ok(mut g) = shared.lock() {
                g.last_status = Some(status);
            }
        }
    }
}

fn set_error(shared: &Mutex<CaptureShared>, msg: &str) {
    if let Ok(mut g) = shared.lock() {
        g.error = Some(msg.to_string());
    }
}

fn trim_ring(ring: &mut VecDeque<PairSample>, now: Instant, keep: Duration) {
    let cutoff = now.checked_sub(keep).unwrap_or(now);
    while let Some(front) = ring.front() {
        if front.t < cutoff {
            ring.pop_front();
        } else {
            break;
        }
    }
}

fn encode_jpeg(image: &Mat) -> Result<Vec<u8>> {
    let mut buf = Vector::<u8>::new();
    let mut params = Vector::<i32>::new();
    params.push(imgcodecs::IMWRITE_JPEG_QUALITY);
    params.push(JPEG_QUALITY);
    let ok = imgcodecs::imencode(".jpg", image, &mut buf, &params)
        .map_err(|e| anyhow::anyhow!("imencode: {e}"))?;
    if !ok {
        bail!("imencode returned false");
    }
    return Ok(buf.to_vec());
}

fn decode_jpeg(bytes: &[u8]) -> Result<Mat> {
    let buf = Vector::<u8>::from_iter(bytes.iter().copied());
    let mat = imgcodecs::imdecode(&buf, imgcodecs::IMREAD_COLOR)
        .map_err(|e| anyhow::anyhow!("imdecode: {e}"))?;
    if mat.empty() {
        bail!("imdecode empty");
    }
    return Ok(mat);
}

fn next_clip_dir(out: &Path, scene: &str) -> Result<PathBuf> {
    let mut max_n = 0u32;
    if out.is_dir() {
        for entry in fs::read_dir(out).with_context(|| format!("read_dir {}", out.display()))? {
            let entry = entry?;
            let name = entry.file_name();
            let Some(name) = name.to_str() else {
                continue;
            };
            let Some(rest) = name.strip_prefix(&format!("{scene}_")) else {
                continue;
            };
            if let Ok(n) = rest.parse::<u32>() {
                max_n = max_n.max(n);
            }
        }
    }
    let dir = out.join(format!("{scene}_{:02}", max_n + 1));
    if dir.exists() {
        bail!("clip dir already exists: {}", dir.display());
    }
    return Ok(dir);
}

fn write_clip(
    dir: &Path,
    scene: &str,
    clip: &[PairSample],
    request_fps: f64,
    grab_fps: f64,
    backend: &str,
    stream_fourcc: &str,
    preroll_secs: f64,
    postroll_secs: f64,
    req_w: i32,
    req_h: i32,
) -> Result<ClipMeta> {
    if clip.is_empty() {
        bail!("클립이 비어 있음 — 프리롤이 아직 안 찼거나 Space가 너무 이름");
    }

    let width = clip[0].width;
    let height = clip[0].height;
    let meas_fps = if clip.len() >= 2 {
        let dt = clip
            .last()
            .unwrap()
            .t
            .duration_since(clip[0].t)
            .as_secs_f64();
        if dt > 1e-6 {
            (clip.len() - 1) as f64 / dt
        } else {
            grab_fps.max(1.0)
        }
    } else {
        grab_fps.max(1.0)
    };
    let writer_fps = if meas_fps.is_finite() && meas_fps > 1.0 {
        meas_fps
    } else if request_fps > 1.0 {
        request_fps
    } else {
        30.0
    };

    let left_path = dir.join("left.avi");
    let right_path = dir.join("right.avi");
    let fourcc =
        VideoWriter::fourcc('M', 'J', 'P', 'G').map_err(|e| anyhow::anyhow!("fourcc: {e}"))?;
    let size = Size::new(width, height);

    let mut left_w = VideoWriter::new(
        left_path
            .to_str()
            .ok_or_else(|| anyhow::anyhow!("non-utf8 path"))?,
        fourcc,
        writer_fps,
        size,
        true,
    )
    .map_err(|e| anyhow::anyhow!("VideoWriter left: {e}"))?;
    let mut right_w = VideoWriter::new(
        right_path
            .to_str()
            .ok_or_else(|| anyhow::anyhow!("non-utf8 path"))?,
        fourcc,
        writer_fps,
        size,
        true,
    )
    .map_err(|e| anyhow::anyhow!("VideoWriter right: {e}"))?;

    if !left_w
        .is_opened()
        .map_err(|e| anyhow::anyhow!("left is_opened: {e}"))?
        || !right_w
            .is_opened()
            .map_err(|e| anyhow::anyhow!("right is_opened: {e}"))?
    {
        bail!("VideoWriter failed to open (MJPG/.avi)");
    }

    for sample in clip {
        let left = decode_jpeg(&sample.left_jpeg)?;
        let right = decode_jpeg(&sample.right_jpeg)?;
        left_w
            .write(&left)
            .map_err(|e| anyhow::anyhow!("write left: {e}"))?;
        right_w
            .write(&right)
            .map_err(|e| anyhow::anyhow!("write right: {e}"))?;
    }
    drop(left_w);
    drop(right_w);

    let created = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);

    let meta = ClipMeta {
        scene: scene.to_string(),
        preroll_secs,
        postroll_secs,
        width,
        height,
        request_fps,
        meas_fps,
        writer_fps,
        fourcc: format!("MJPG (stream was {stream_fourcc})"),
        backend: backend.to_string(),
        frames: clip.len(),
        created_unix_secs: created,
    };
    // req size는 참고용으로만 — 실제 프레임 size가 SSOT
    let _ = (req_w, req_h);

    let meta_path = dir.join("meta.json");
    let json = serde_json::to_string_pretty(&meta)?;
    fs::write(&meta_path, json).with_context(|| format!("write {}", meta_path.display()))?;
    return Ok(meta);
}
