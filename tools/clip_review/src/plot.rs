//! 월드 2면도 — **위에서**(y–x)와 **옆에서**(y–z). 두 면이 가로축(월드 y)을 공유해서
//! 세로로 붙여 놓으면 같은 순간이 같은 가로 위치에 온다.
//!
//! kiss3d 관전 창 대신 이걸 쓴다. sim 씬에는 선을 그리는 수단이 없어서 궤적을 띄우려면
//! 렌더러부터 손대야 하는데, 궤적 두 개를 겹쳐 보는 목적에는 2면도가 더 정확히 읽힌다
//! (깊이 감각 대신 **거리**가 보인다).
//!
//! 가로축은 뒤집어 둔다 — 왼쪽이 파 엔드(슈터), 오른쪽이 로봇이라 공이 왼→오른쪽으로 간다.

use anyhow::Result;
use opencv::core::{Mat, Point, Scalar};
use opencv::imgproc;
use pingpong_bot::Point3;
use pingpong_bot::constants::table;

/// 테이블 바깥 여유 [m].
const MARGIN_M: f64 = 0.4;
/// 옆면도에서 테이블 위로 보여줄 높이 [m].
const HEADROOM_M: f64 = 0.9;
/// 옆면도에서 테이블 아래로 보여줄 깊이 [m] — 바닥에 떨어진 공이 보이도록.
const UNDERROOM_M: f64 = 0.25;
/// 좌표 폭주 방지 — OpenCV에 넘기기 전 자른다.
const DRAW_CLAMP_PX: f64 = 20_000.0;

const BLACK: Scalar = Scalar::new(0.0, 0.0, 0.0, 0.0);
const GRAY: Scalar = Scalar::new(90.0, 90.0, 90.0, 0.0);
const WHITE: Scalar = Scalar::new(255.0, 255.0, 255.0, 0.0);
const CYAN: Scalar = Scalar::new(255.0, 255.0, 0.0, 0.0);
const GREEN: Scalar = Scalar::new(0.0, 255.0, 0.0, 0.0);
const YELLOW: Scalar = Scalar::new(0.0, 255.0, 255.0, 0.0);
const ORANGE: Scalar = Scalar::new(0.0, 140.0, 255.0, 0.0);

/// 월드 → 픽셀 변환과 두 면의 세로 배치.
pub struct WorldPlot {
    width: i32,
    top_height: i32,
    side_height: i32,
}

impl WorldPlot {
    /// 가로 폭에서 두 면의 높이를 **비율로** 정한다 — 축척이 x·y·z 모두 같아진다.
    pub fn new(width: i32) -> Self {
        let span_y = table::LENGTH_Y + 2.0 * MARGIN_M;
        let span_x = table::WIDTH_X + 2.0 * MARGIN_M;
        let span_z = HEADROOM_M + UNDERROOM_M;
        let per_meter = f64::from(width) / span_y;
        return Self {
            width,
            top_height: (span_x * per_meter).round() as i32,
            side_height: (span_z * per_meter).round() as i32,
        };
    }

    fn u(&self, y: f64) -> f64 {
        let lo = -MARGIN_M;
        let hi = table::LENGTH_Y + MARGIN_M;
        // 뒤집어서 파 엔드가 왼쪽.
        return f64::from(self.width) * (hi - y) / (hi - lo);
    }

    fn v_top(&self, x: f64) -> f64 {
        let lo = -MARGIN_M;
        let hi = table::WIDTH_X + MARGIN_M;
        return f64::from(self.top_height) * (x - lo) / (hi - lo);
    }

    fn v_side(&self, z: f64) -> f64 {
        let lo = table::SURFACE_Z - UNDERROOM_M;
        let hi = table::SURFACE_Z + HEADROOM_M;
        let local = f64::from(self.side_height) * (hi - z) / (hi - lo);
        return f64::from(self.top_height) + local;
    }

    fn top(&self, p: Point3) -> Point {
        return pt(self.u(p.y), self.v_top(p.x));
    }

    fn side(&self, p: Point3) -> Point {
        return pt(self.u(p.y), self.v_side(p.z));
    }

    /// 실제 궤적·예측 궤적·현재 상태를 한 장으로.
    pub fn render(
        &self,
        observed: &[Point3],
        predicted: &[Point3],
        ekf: Option<Point3>,
        now: Option<Point3>,
        hud: &[String],
    ) -> Result<Mat> {
        let mut img = Mat::new_rows_cols_with_default(
            self.top_height + self.side_height,
            self.width,
            opencv::core::CV_8UC3,
            BLACK,
        )?;

        self.draw_table(&mut img)?;

        // 실제(흰색)를 먼저, 예측(하늘색)을 위에 — 벌어진 만큼이 그대로 보인다.
        self.draw_track(&mut img, observed, WHITE, 2)?;
        self.draw_track(&mut img, predicted, CYAN, 1)?;

        if let Some(p) = ekf {
            imgproc::circle(&mut img, self.top(p), 4, GREEN, -1, imgproc::LINE_AA, 0)?;
            imgproc::circle(&mut img, self.side(p), 4, GREEN, -1, imgproc::LINE_AA, 0)?;
        }
        if let Some(p) = now {
            imgproc::circle(&mut img, self.top(p), 6, YELLOW, 1, imgproc::LINE_AA, 0)?;
            imgproc::circle(&mut img, self.side(p), 6, YELLOW, 1, imgproc::LINE_AA, 0)?;
        }

        pingpong_bot::camera::Preview::draw_debug_lines(&mut img, hud, WHITE)?;
        return Ok(img);
    }

    fn draw_table(&self, img: &mut Mat) -> Result<()> {
        let z = table::SURFACE_Z;
        let corner = |x: f64, y: f64| Point3::new(x, y, z);

        // 위에서 — 상판 사각형.
        let outline = [
            corner(0.0, 0.0),
            corner(table::WIDTH_X, 0.0),
            corner(table::WIDTH_X, table::LENGTH_Y),
            corner(0.0, table::LENGTH_Y),
            corner(0.0, 0.0),
        ];
        for pair in outline.windows(2) {
            imgproc::line(
                img,
                self.top(pair[0]),
                self.top(pair[1]),
                GRAY,
                1,
                imgproc::LINE_8,
                0,
            )?;
        }

        // 네트 — 예측 선언 기준이 될 평면이라 두 면에 다 긋는다.
        let net_y = table::LENGTH_Y * 0.5;
        imgproc::line(
            img,
            self.top(corner(0.0, net_y)),
            self.top(corner(table::WIDTH_X, net_y)),
            ORANGE,
            1,
            imgproc::LINE_8,
            0,
        )?;
        imgproc::line(
            img,
            self.side(corner(0.0, net_y)),
            self.side(Point3::new(0.0, net_y, z + table::NET_HEIGHT)),
            ORANGE,
            1,
            imgproc::LINE_8,
            0,
        )?;

        // 옆에서 — 상판 선.
        imgproc::line(
            img,
            self.side(corner(0.0, 0.0)),
            self.side(corner(0.0, table::LENGTH_Y)),
            GRAY,
            1,
            imgproc::LINE_8,
            0,
        )?;

        // 두 면 경계.
        imgproc::line(
            img,
            Point::new(0, self.top_height),
            Point::new(self.width, self.top_height),
            GRAY,
            1,
            imgproc::LINE_8,
            0,
        )?;
        pingpong_bot::camera::Preview::draw_cam_label(img, "top (y-x) / side (y-z)", GRAY)?;
        return Ok(());
    }

    fn draw_track(
        &self,
        img: &mut Mat,
        points: &[Point3],
        color: Scalar,
        thickness: i32,
    ) -> Result<()> {
        for pair in points.windows(2) {
            imgproc::line(
                img,
                self.top(pair[0]),
                self.top(pair[1]),
                color,
                thickness,
                imgproc::LINE_AA,
                0,
            )?;
            imgproc::line(
                img,
                self.side(pair[0]),
                self.side(pair[1]),
                color,
                thickness,
                imgproc::LINE_AA,
                0,
            )?;
        }
        return Ok(());
    }
}

fn pt(x: f64, y: f64) -> Point {
    return Point::new(
        x.clamp(-DRAW_CLAMP_PX, DRAW_CLAMP_PX) as i32,
        y.clamp(-DRAW_CLAMP_PX, DRAW_CLAMP_PX) as i32,
    );
}
