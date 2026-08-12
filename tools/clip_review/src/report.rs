//! 클립 전부를 훑어 성적표와 화면을 파일로 남긴다.
//!
//! 창을 안 띄운다. 3D 도 kiss3d 가 아니라 OpenCV 로 직교 투영 두 장을 그린다 —
//! kiss3d 0.45 에 스냅샷 API 가 없고, 배치에는 결정적인 그림이 낫다. 같은 클립을 두 번
//! 돌리면 같은 파일이 나온다.

use std::path::Path;

use anyhow::{Context, Result};
use opencv::core::{Mat, Point, Scalar};
use opencv::{imgcodecs, imgproc};

use pingpong_bot::Point3;
use pingpong_bot::camera::{self, Calibration, Preview};
use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::robot::motion::InterceptWindow;

use crate::overlay;
use crate::score::Score;
use crate::track::{self, Reviewed};

/// 직교 투영 캔버스 한 장의 크기 [px] 와 여백.
const VIEW_W: i32 = 900;
const VIEW_H: i32 = 620;
const MARGIN: i32 = 40;

const BACKGROUND: Scalar = Scalar::new(28.0, 24.0, 20.0, 0.0);
const TABLE_LINE: Scalar = Scalar::new(200.0, 160.0, 90.0, 0.0);
const TEXT: Scalar = Scalar::new(230.0, 230.0, 230.0, 0.0);
/// overlay 와 같은 색을 쓴다 — 두 화면을 나란히 놓고 봐야 한다.
const RAW: Scalar = Scalar::new(60.0, 255.0, 60.0, 0.0);
const FITTED: Scalar = Scalar::new(255.0, 200.0, 0.0, 0.0);
const PREDICTED: Scalar = Scalar::new(255.0, 0.0, 255.0, 0.0);

/// 월드 [m] → 캔버스 [px]. 축 두 개만 골라 쓴다.
struct Plan {
    /// 가로축으로 쓸 월드 좌표 (0=x, 1=y, 2=z) 와 범위.
    across: (usize, f64, f64),
    down: (usize, f64, f64),
    label: &'static str,
}

impl Plan {
    fn top() -> Self {
        return Self {
            across: (0, -0.3, table::WIDTH_X + 0.3),
            down: (1, table::LENGTH_Y + 0.3, -0.4),
            label: "top  x-y",
        };
    }

    fn side() -> Self {
        return Self {
            across: (1, table::LENGTH_Y + 0.3, -0.4),
            down: (2, table::SURFACE_Z + 0.9, table::SURFACE_Z - 0.2),
            label: "side  y-z",
        };
    }

    /// 두 축을 **같은 축척**으로 그린다.
    ///
    /// 축마다 캔버스에 맞춰 늘리면 가로 오차와 세로 오차가 다른 배율로 보인다. 이 그림은
    /// 오차를 눈으로 재는 용도라 그러면 안 된다.
    fn at(&self, point: Point3) -> Point {
        let scale = self.scale();
        let place = |(axis, from, to): (usize, f64, f64), span: i32| -> i32 {
            let offset = (point.coords[axis] - from) * scale * (to - from).signum();
            let used = (to - from).abs() * scale;
            let centre = f64::from(span) * 0.5 - used * 0.5;
            return (centre + offset) as i32;
        };
        return Point::new(place(self.across, VIEW_W), place(self.down, VIEW_H));
    }

    /// px per m — 두 축 중 더 빠듯한 쪽에 맞춘다.
    fn scale(&self) -> f64 {
        let fit = |(_, from, to): (usize, f64, f64), span: i32| -> f64 {
            return f64::from(span - 2 * MARGIN) / (to - from).abs();
        };
        return fit(self.across, VIEW_W).min(fit(self.down, VIEW_H));
    }
}

fn line(
    img: &mut Mat,
    plan: &Plan,
    from: Point3,
    to: Point3,
    color: Scalar,
    width: i32,
) -> Result<()> {
    imgproc::line(
        img,
        plan.at(from),
        plan.at(to),
        color,
        width,
        imgproc::LINE_AA,
        0,
    )?;
    return Ok(());
}

fn polyline(
    img: &mut Mat,
    plan: &Plan,
    points: &[Point3],
    color: Scalar,
    width: i32,
) -> Result<()> {
    for pair in points.windows(2) {
        line(img, plan, pair[0], pair[1], color, width)?;
    }
    return Ok(());
}

/// 테이블 윤곽과 네트. 궤적이 어디를 지나는지 눈으로 잡을 기준이다.
fn draw_table(img: &mut Mat, plan: &Plan) -> Result<()> {
    let z = table::SURFACE_Z;
    let (w, l) = (table::WIDTH_X, table::LENGTH_Y);
    let corners = [
        Point3::new(0.0, 0.0, z),
        Point3::new(w, 0.0, z),
        Point3::new(w, l, z),
        Point3::new(0.0, l, z),
    ];
    for i in 0..4 {
        line(img, plan, corners[i], corners[(i + 1) % 4], TABLE_LINE, 1)?;
    }
    let net = l * 0.5;
    line(
        img,
        plan,
        Point3::new(0.0, net, z),
        Point3::new(w, net, z),
        TABLE_LINE,
        2,
    )?;
    // 옆에서 볼 때 네트 높이가 있어야 공이 넘었는지 보인다.
    line(
        img,
        plan,
        Point3::new(w * 0.5, net, z),
        Point3::new(w * 0.5, net, z + table::NET_HEIGHT),
        TABLE_LINE,
        2,
    )?;
    return Ok(());
}

fn view(plan: &Plan, reviewed: &Reviewed) -> Result<Mat> {
    let mut img =
        Mat::new_rows_cols_with_default(VIEW_H, VIEW_W, opencv::core::CV_8UC3, BACKGROUND)?;
    draw_table(&mut img, plan)?;
    let raw: Vec<Point3> = reviewed.observed.iter().map(|o| o.point).collect();
    polyline(&mut img, plan, &raw, RAW, 2)?;
    if let Some(contract) = &reviewed.contract {
        let fitted: Vec<Point3> = contract
            .latest
            .measured
            .iter()
            .map(|state| state.position)
            .collect();
        let predicted: Vec<Point3> = contract
            .at_trigger
            .predicted
            .iter()
            .map(|state| state.position)
            .collect();
        polyline(&mut img, plan, &fitted, FITTED, 2)?;
        polyline(&mut img, plan, &predicted, PREDICTED, 2)?;
    }
    Preview::draw_cam_label(&mut img, plan.label, TEXT)?;
    return Ok(img);
}

/// 커밋 순간의 카메라 두 장 + 그 위 오버레이.
fn cameras(reviewed: &Reviewed, clip: &camera::ResolvedStereoOffline) -> Result<Option<Mat>> {
    let Some(contract) = &reviewed.contract else {
        return Ok(None);
    };
    let calibration = Calibration::load_json(&defaults::calibration_path())
        .map_err(anyhow::Error::msg)
        .context("calibration")?;
    let mut left =
        camera::OpenCvCapture::from_path(camera::Id(0), &clip.left).map_err(anyhow::Error::msg)?;
    let mut right =
        camera::OpenCvCapture::from_path(camera::Id(1), &clip.right).map_err(anyhow::Error::msg)?;
    left.seek_frame(contract.frame as u64)
        .map_err(anyhow::Error::msg)?;
    right
        .seek_frame(contract.frame as u64)
        .map_err(anyhow::Error::msg)?;

    let actual_past = track::clip_to_mount(&reviewed.observed_to(contract.frame));
    let filtered = track::clip_to_mount(&reviewed.measured_to(contract.frame));
    let committed = track::clip_to_mount(
        &reviewed
            .predicted()
            .iter()
            .map(|state| state.position)
            .collect::<Vec<_>>(),
    );
    let state = &reviewed.frames[contract.frame];

    let mut panels = Vec::with_capacity(2);
    for (slot, source) in [(0usize, &mut left), (1usize, &mut right)].into_iter() {
        use pingpong_bot::camera::FrameSource;
        let Some(frame) = source.next_frame() else {
            return Ok(None);
        };
        let params = calibration
            .params(camera::Id(slot as u8))
            .with_context(|| format!("cam{slot} params"))?;
        panels.push(overlay::draw(
            &frame.image,
            params,
            &overlay::Tracks {
                actual_past: &actual_past,
                filtered: &filtered,
                committed: &committed,
            },
            state.pixels[slot],
            state.filtered.map(|s| s.position),
            &format!("cam{slot}  f{}", contract.frame),
            &[],
        )?);
    }
    return Ok(Some(Preview::hstack_bgr(&panels)?));
}

/// 클립 하나를 훑어 성적표와 그림 두 장을 남긴다.
pub fn one(clip: &camera::ResolvedStereoOffline, name: &str, out: &Path) -> Result<Score> {
    // meta 에 실측 fps 가 없으면 클립을 못 믿는다 — 시각이 곧 궤적이라 추정으로 때우면 안 된다.
    let fps = clip.meas_fps.context("meta.json 에 meas_fps 가 없다")?;
    let reviewed = track::review(&clip.left, &clip.right, fps).map_err(anyhow::Error::msg)?;
    let score = Score::of(&reviewed);

    let write = |suffix: &str, image: &Mat| -> Result<()> {
        let path = out.join(format!("{name}_{suffix}.png"));
        let text = path.to_str().context("경로에 UTF-8 아닌 글자")?;
        imgcodecs::imwrite(text, image, &opencv::core::Vector::new())
            .with_context(|| format!("imwrite {}", path.display()))?;
        return Ok(());
    };

    let card = out.join(format!("{name}_score.txt"));
    std::fs::write(&card, score.render(name) + "\n")
        .with_context(|| format!("write {}", card.display()))?;

    let top = view(&Plan::top(), &reviewed)?;
    let side = view(&Plan::side(), &reviewed)?;
    write("sim", &Preview::hstack_bgr(&[top, side])?)?;
    if let Some(panels) = cameras(&reviewed, clip)? {
        write("cam", &panels)?;
    }
    return Ok(score);
}

/// 성적표 한 줄 — 클립을 가로질러 비교할 표.
pub fn summary_row(name: &str, score: &Score) -> String {
    let cell = |value: Option<f64>| value.map_or("--".to_owned(), |v| format!("{:.1}", v * 100.0));
    let resid =
        |slot: usize| score.residual[slot].map_or("--".to_owned(), |(p50, _)| format!("{p50:.1}"));
    // 접수 창의 앞·가운데·뒤 — 창을 가로질러 오차가 어떻게 변하는지 보인다.
    let plane = |ratio: f64| {
        cell(
            score
                .plane_error
                .get(plane_index(score.plane_error.len(), ratio))
                .and_then(|(_, gap)| gap.map(|(all, _, _)| all)),
        )
    };
    // 평면 오차는 위치가 우연히 맞아도 좋게 나온다 — 튐 횟수가 다르면 그 옆에 바로 보이게
    // 붙인다. 위치 칸만 보고 "괜찮네" 하고 넘어가는 걸 막는 게 요점이다.
    let bounce = match score.bounce_counts {
        Some((p, o)) if p != o => format!("{p}≠{o}"),
        Some((p, o)) => format!("{p}={o}"),
        None => "--".to_owned(),
    };
    return format!(
        "{name:<8} {:>5} {:>5} {:>6} {:>6} {:>6} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}",
        score.both,
        score.track_switches + 1,
        resid(0),
        resid(1),
        score
            .trigger
            .map_or("--".to_owned(), |(_, _, n)| n.to_string()),
        cell(score.predicted_rmse.map(|(all, _, _)| all)),
        cell(score.lead_error.first().copied().flatten()),
        cell(score.lead_error.get(1).copied().flatten()),
        cell(score.lead_error.get(2).copied().flatten()),
        cell(score.lead_error.get(3).copied().flatten()),
        plane(0.0),
        plane(0.5),
        plane(1.0),
        bounce,
    );
}

/// 평면 목록에서 앞(0.0)·가운데(0.5)·뒤(1.0) 를 고른다.
fn plane_index(count: usize, ratio: f64) -> usize {
    return (count.saturating_sub(1) as f64 * ratio).round() as usize;
}

pub fn summary_header() -> String {
    let planes = InterceptWindow::default().hit_planes();
    let y = |ratio: f64| {
        planes
            .get(plane_index(planes.len(), ratio))
            .map_or("y--".to_owned(), |plane| format!("y{:.2}", plane.y))
    };
    return format!(
        "{:<8} {:>5} {:>5} {:>6} {:>6} {:>6} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}",
        "clip",
        "동시",
        "트랙",
        "잔차0",
        "잔차1",
        "관측",
        "RMSE",
        "0.4s",
        "0.3s",
        "0.2s",
        "0.1s",
        y(0.0),
        y(0.5),
        y(1.0),
        "튐예=실",
    );
}
