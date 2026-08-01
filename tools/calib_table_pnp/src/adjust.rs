//! 찍은 8점을 지우지 않고 개별 미세 조정하는 상태·순수 로직.
//!
//! `z`(pop)/`c`(clear)만 있던 흐름에 "3번 점만 1px 옮기기"를 더한다.
//! 창 없이 테스트되도록 그리기·highgui는 [`crate::overlay`]/[`crate::interactive`]에 둔다.

use pingpong_bot::camera;
use pingpong_bot::camera::TablePnp;
use pingpong_bot::constants::TABLE_LANDMARK_COUNT;

/// `r` 자동 미세탐색 기본 반경 [px] — [`crate::args::Args::refine_radius`] 기본값.
pub const DEFAULT_REFINE_RADIUS_PX: f64 = 3.0;
/// 마우스로 기존 점을 잡을 스냅 반경 [px] (이미지 좌표).
pub const SNAP_PX: f64 = 12.0;
/// 대문자 `HJKL` 굵은 이동 [px]. 소문자·방향키는 1px.
pub const COARSE_STEP: f64 = 5.0;
/// undo 스택 상한.
const HISTORY_CAP: usize = 64;
/// `fov_y` 조절 한 칸 [deg].
pub const FOV_STEP_DEG: f64 = 0.5;
/// `fov_y` 허용 범위 — `TablePnp::calibrate`가 `1 < fov < 179`를 요구한다.
const FOV_MIN_DEG: f64 = 5.0;
const FOV_MAX_DEG: f64 = 150.0;

/// 클릭이 놓일 수 있는 범위. `--pad N`이면 프레임 밖 `N`px까지 (음수 좌표 허용).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PixelBounds {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

/// 패딩 캔버스에서 클릭으로 도달 가능한 이미지 좌표 범위와 동일하게 잡는다.
pub fn pixel_bounds(img_w: i32, img_h: i32, pad: i32) -> PixelBounds {
    let pad = f64::from(pad.max(0));
    return PixelBounds {
        min_x: -pad,
        min_y: -pad,
        max_x: f64::from(img_w.max(1) - 1) + pad,
        max_y: f64::from(img_h.max(1) - 1) + pad,
    };
}

/// `u`로 되돌릴 단위. `fov_y`도 같이 되돌리므로 별도 리셋 키가 필요 없다.
#[derive(Debug, Clone, PartialEq)]
pub struct Snapshot {
    pub clicks: Vec<camera::Pixel>,
    pub fov_y: f64,
}

/// 선택 인덱스 · anchor · undo 스택 · 현재 `fov_y`.
#[derive(Debug, Clone)]
pub struct Adjust {
    /// 선택된 랜드마크. `None`이면 방향키는 기존처럼 aim만 움직인다.
    pub sel: Option<usize>,
    /// 8번째 클릭이 들어온 순간의 원본 클릭 — [`refine_clicks`] 반경의 기준.
    pub anchor: Vec<camera::Pixel>,
    pub fov_y: f64,
    history: Vec<Snapshot>,
}

impl Adjust {
    pub fn new(fov_y: f64) -> Self {
        return Self {
            sel: None,
            anchor: Vec::new(),
            fov_y,
            history: Vec::new(),
        };
    }

    /// 완전 초기화 (`c`/`n`/`Space`) — `fov_y`까지 CLI 값으로 되돌린다.
    pub fn reset(&mut self, fov_y: f64) {
        self.exit_adjust();
        self.fov_y = fov_y;
    }

    /// 클릭이 8점 아래로 떨어질 때 (`z`) — 조정 상태만 버리고 `fov_y`는 유지.
    pub fn exit_adjust(&mut self) {
        self.sel = None;
        self.anchor.clear();
        self.history.clear();
    }

    /// 8점이 다 모인 순간 호출. anchor를 그때의 클릭으로 고정한다.
    pub fn set_anchor(&mut self, clicks: &[camera::Pixel]) {
        self.anchor = clicks.to_vec();
    }

    pub fn select(&mut self, index: usize) {
        if index < TABLE_LANDMARK_COUNT {
            self.sel = Some(index);
        }
    }

    pub fn cycle(&mut self) {
        self.sel = next_sel(self.sel, TABLE_LANDMARK_COUNT);
    }

    pub fn clear_sel(&mut self) {
        self.sel = None;
    }

    /// 되돌릴 지점을 쌓는다 (상한 [`HISTORY_CAP`], 넘으면 가장 오래된 것부터 버림).
    pub fn push_history(&mut self, clicks: &[camera::Pixel]) {
        self.history.push(Snapshot {
            clicks: clicks.to_vec(),
            fov_y: self.fov_y,
        });
        if self.history.len() > HISTORY_CAP {
            self.history.remove(0);
        }
    }

    /// 마지막 스냅샷을 꺼낸다. `fov_y`는 여기서 바로 복원한다.
    pub fn undo(&mut self) -> Option<Snapshot> {
        let snap = self.history.pop()?;
        self.fov_y = snap.fov_y;
        return Some(snap);
    }

    pub fn history_len(&self) -> usize {
        return self.history.len();
    }

    /// `fov_y`를 `delta` 만큼 옮긴다. 실제로 변했으면 true.
    pub fn nudge_fov(&mut self, delta: f64) -> bool {
        let next = (self.fov_y + delta).clamp(FOV_MIN_DEG, FOV_MAX_DEG);
        if (next - self.fov_y).abs() < 1e-9 {
            return false;
        }
        self.fov_y = next;
        return true;
    }

    /// 선택된 점의 anchor 기준 이동량 (HUD `d=(+2,-1)`).
    pub fn offset_from_anchor(&self, clicks: &[camera::Pixel]) -> Option<(f64, f64)> {
        let i = self.sel?;
        let click = clicks.get(i)?;
        let anchor = self.anchor.get(i)?;
        return Some((click.x - anchor.x, click.y - anchor.y));
    }
}

/// 스냅 반경 안에서 `target`에 가장 가까운 점. 동거리면 낮은 인덱스.
pub fn nearest_click(
    clicks: &[camera::Pixel],
    target: camera::Pixel,
    snap_px: f64,
) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (i, p) in clicks.iter().enumerate() {
        let d = (*p - target).norm();
        if d > snap_px {
            continue;
        }
        match best {
            Some((_, bd)) if bd <= d => {}
            _ => best = Some((i, d)),
        }
    }
    return best.map(|(i, _)| i);
}

/// `Tab` 순회. `None` -> 0, 마지막 -> 0.
pub fn next_sel(sel: Option<usize>, n: usize) -> Option<usize> {
    if n == 0 {
        return None;
    }
    return match sel {
        None => Some(0),
        Some(i) => Some((i + 1) % n),
    };
}

/// 이미지 좌표로 옮기고 [`PixelBounds`]로 clamp.
pub fn moved_point(p: camera::Pixel, dx: f64, dy: f64, bounds: PixelBounds) -> camera::Pixel {
    return camera::Pixel::new(
        (p.x + dx).clamp(bounds.min_x, bounds.max_x),
        (p.y + dy).clamp(bounds.min_y, bounds.max_y),
    );
}

/// [`refine_clicks`] 결과.
#[derive(Debug, Clone)]
pub struct RefineOutcome {
    pub clicks: Vec<camera::Pixel>,
    pub rmse_before: f64,
    pub rmse_after: f64,
    /// 소비한 PnP 호출 수 (상한 확인용).
    pub solves: usize,
}

/// 좌표하강 상한 — 폭주 방지용 backstop.
const MAX_SOLVES: usize = 3000;
const MIN_STEP_PX: f64 = 0.25;
const START_STEP_PX: f64 = 1.0;

/// 각 점을 `anchor`에서 `radius` 이내로만 움직여 재투영 RMSE를 낮춘다.
///
/// **`radius`가 유일한 안전장치다.** pose는 클릭에서 매번 재적합되므로 경계가 없으면
/// 클릭을 실제 영상 특징에서 떼어내 서로 완벽히 일관된 배치로 옮겨버린다 —
/// RMSE는 0으로 가지만 캘리브는 무의미해진다.
///
/// RMSE는 `TablePnp::calibrate(..).reproj_rmse`를 그대로 쓴다 (솔버와 같은 메트릭).
/// 해를 못 구하는 후보는 버린다.
pub fn refine_clicks(
    cam_id: camera::Id,
    width: u32,
    height: u32,
    fov_y: f64,
    anchor: &[camera::Pixel],
    start: &[camera::Pixel],
    radius: f64,
    bounds: PixelBounds,
) -> Option<RefineOutcome> {
    if start.len() != TABLE_LANDMARK_COUNT || anchor.len() != TABLE_LANDMARK_COUNT {
        return None;
    }
    let rmse = |pts: &[camera::Pixel]| -> Option<f64> {
        return TablePnp::calibrate(cam_id, None, width, height, fov_y, pts)
            .ok()
            .map(|r| r.reproj_rmse)
            .filter(|r| r.is_finite());
    };

    let mut cur: Vec<camera::Pixel> = start
        .iter()
        .zip(anchor)
        .map(|(p, a)| clamp_to_ball(*p, *a, radius))
        .map(|p| moved_point(p, 0.0, 0.0, bounds))
        .collect();
    let rmse_before = rmse(&cur)?;
    let mut best = rmse_before;
    let mut solves = 0usize;

    if radius <= 0.0 {
        return Some(RefineOutcome {
            clicks: cur,
            rmse_before,
            rmse_after: best,
            solves,
        });
    }

    let mut step = START_STEP_PX;
    while step >= MIN_STEP_PX && solves < MAX_SOLVES {
        let mut improved = false;
        for i in 0..TABLE_LANDMARK_COUNT {
            for (dx, dy) in [(step, 0.0), (-step, 0.0), (0.0, step), (0.0, -step)] {
                if solves >= MAX_SOLVES {
                    break;
                }
                let moved = moved_point(cur[i], dx, dy, bounds);
                if (moved - anchor[i]).norm() > radius || moved == cur[i] {
                    continue;
                }
                let mut cand = cur.clone();
                cand[i] = moved;
                solves += 1;
                let Some(r) = rmse(&cand) else {
                    continue;
                };
                if r < best - 1e-6 {
                    cur = cand;
                    best = r;
                    improved = true;
                }
            }
        }
        if !improved {
            step *= 0.5;
        }
    }

    return Some(RefineOutcome {
        clicks: cur,
        rmse_before,
        rmse_after: best,
        solves,
    });
}

/// `p`를 `center` 중심 반경 `radius` 원 안으로 당긴다.
fn clamp_to_ball(p: camera::Pixel, center: camera::Pixel, radius: f64) -> camera::Pixel {
    let d = (p - center).norm();
    if d <= radius || d <= f64::EPSILON {
        return p;
    }
    return center + (p - center) * (radius / d);
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use pingpong_bot::constants::table;

    fn px(x: f64, y: f64) -> camera::Pixel {
        return camera::Pixel::new(x, y);
    }

    #[test]
    fn nearest_click_respects_snap_radius() {
        let clicks = vec![px(10.0, 10.0), px(30.0, 10.0), px(100.0, 100.0)];
        assert_eq!(nearest_click(&clicks, px(12.0, 11.0), 12.0), Some(0));
        assert_eq!(nearest_click(&clicks, px(28.0, 12.0), 12.0), Some(1));
        // 어느 점에서도 12px 밖
        assert_eq!(nearest_click(&clicks, px(60.0, 60.0), 12.0), None);
        // 동거리면 낮은 인덱스 (20,10은 0과 1에서 각각 10px)
        assert_eq!(nearest_click(&clicks, px(20.0, 10.0), 12.0), Some(0));
    }

    #[test]
    fn next_sel_wraps() {
        assert_eq!(next_sel(None, 8), Some(0));
        assert_eq!(next_sel(Some(0), 8), Some(1));
        assert_eq!(next_sel(Some(7), 8), Some(0));
        assert_eq!(next_sel(Some(3), 0), None);
    }

    #[test]
    fn moved_point_allows_pad_region_and_clamps() {
        let b = pixel_bounds(640, 480, 16);
        assert_eq!(b.min_x, -16.0);
        assert_eq!(b.max_x, 655.0);

        // 패딩 안쪽 음수 좌표는 허용
        assert_eq!(moved_point(px(-10.0, 0.0), -5.0, 0.0, b), px(-15.0, 0.0));
        // 패딩 밖으로는 clamp
        assert_eq!(moved_point(px(-15.0, 0.0), -5.0, 0.0, b), px(-16.0, 0.0));
        // x는 655에서 막히고, y는 484 < max_y(495)라 그대로 간다
        assert_eq!(b.max_y, 495.0);
        assert_eq!(moved_point(px(654.0, 479.0), 5.0, 5.0, b), px(655.0, 484.0));
        assert_eq!(moved_point(px(654.0, 493.0), 5.0, 5.0, b), px(655.0, 495.0));

        // pad 0이면 프레임 안으로만
        let b0 = pixel_bounds(640, 480, 0);
        assert_eq!(moved_point(px(0.0, 0.0), -5.0, -5.0, b0), px(0.0, 0.0));
        assert_eq!(
            moved_point(px(639.0, 479.0), 5.0, 5.0, b0),
            px(639.0, 479.0)
        );
    }

    #[test]
    fn history_undo_restores_clicks_and_fov() {
        let mut adj = Adjust::new(47.3);
        let base = vec![px(1.0, 1.0)];
        adj.push_history(&base);
        assert!(adj.nudge_fov(FOV_STEP_DEG));
        assert!((adj.fov_y - 47.8).abs() < 1e-9);

        let snap = adj.undo().expect("snapshot");
        assert_eq!(snap.clicks, base);
        assert!((adj.fov_y - 47.3).abs() < 1e-9);
        assert!(adj.undo().is_none());
    }

    #[test]
    fn nudge_fov_clamps_at_limits() {
        let mut adj = Adjust::new(FOV_MAX_DEG);
        assert!(!adj.nudge_fov(FOV_STEP_DEG));
        adj.fov_y = FOV_MIN_DEG;
        assert!(!adj.nudge_fov(-FOV_STEP_DEG));
    }

    /// [`pnp.rs`]의 `overhead_cam` 패턴 — 알려진 카메라로 8점을 투영해 기준을 만든다.
    fn overhead_cam() -> camera::Params {
        let target = Vector3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.5,
            table::SURFACE_Z,
        );
        let eye = target + Vector3::new(0.0, -0.4, 2.4);
        return camera::Params::look_at(
            camera::Id::new(0),
            None,
            eye,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            640,
            480,
            70.0_f64.to_radians(),
        );
    }

    fn truth_pixels_with_noise(cam: &camera::Params) -> Vec<camera::Pixel> {
        // 결정적 ±2px 지그재그 노이즈 (테스트 재현성)
        let offsets = [
            (2.0, -1.0),
            (-2.0, 1.0),
            (1.0, 2.0),
            (-1.0, -2.0),
            (2.0, 2.0),
            (-2.0, -2.0),
            (0.0, 2.0),
            (2.0, 0.0),
        ];
        return TablePnp::landmarks()
            .iter()
            .enumerate()
            .map(|(i, m)| {
                let p = cam.project_world(m.world).expect("landmark in FOV");
                camera::Pixel::new(p.x + offsets[i].0, p.y + offsets[i].1)
            })
            .collect();
    }

    fn fov_y_of(cam: &camera::Params) -> f64 {
        return 2.0 * ((f64::from(cam.height) * 0.5) / cam.fy).atan().to_degrees();
    }

    #[test]
    fn refine_lowers_rmse_and_stays_inside_radius() {
        let cam = overhead_cam();
        let anchor = truth_pixels_with_noise(&cam);
        let bounds = pixel_bounds(cam.width as i32, cam.height as i32, 16);
        let radius = 3.0;

        let out = refine_clicks(
            camera::Id::new(0),
            cam.width,
            cam.height,
            fov_y_of(&cam),
            &anchor,
            &anchor,
            radius,
            bounds,
        )
        .expect("refine");

        // 노이즈만큼(≈1.9px) 있던 잔차를 실질적으로 걷어낸다 (관측 0.06px).
        assert!(
            out.rmse_before > 1.0,
            "noisy input should start above 1px, got {}",
            out.rmse_before
        );
        assert!(
            out.rmse_after < out.rmse_before * 0.5,
            "expected a real improvement: {} -> {}",
            out.rmse_before,
            out.rmse_after
        );
        for (i, p) in out.clicks.iter().enumerate() {
            let d = (*p - anchor[i]).norm();
            assert!(
                d <= radius + 1e-6,
                "point {i} escaped radius: {d} > {radius}"
            );
        }
        assert!(out.solves <= MAX_SOLVES, "solves {} over cap", out.solves);
    }

    #[test]
    fn refine_with_zero_radius_is_noop() {
        let cam = overhead_cam();
        let anchor = truth_pixels_with_noise(&cam);
        let bounds = pixel_bounds(cam.width as i32, cam.height as i32, 0);

        let out = refine_clicks(
            camera::Id::new(0),
            cam.width,
            cam.height,
            fov_y_of(&cam),
            &anchor,
            &anchor,
            0.0,
            bounds,
        )
        .expect("refine");

        assert_eq!(out.clicks, anchor);
        assert_eq!(out.solves, 0);
        assert!((out.rmse_after - out.rmse_before).abs() < 1e-12);
    }

    #[test]
    fn refine_rejects_wrong_length() {
        let bounds = pixel_bounds(640, 480, 0);
        let short = vec![px(0.0, 0.0)];
        assert!(
            refine_clicks(
                camera::Id::new(0),
                640,
                480,
                47.3,
                &short,
                &short,
                3.0,
                bounds
            )
            .is_none()
        );
    }
}
