# Spatial → Color 검출 개선 구현 플랜

> **For agentic workers:** REQUIRED SUB-SKILL: `superpowers:subagent-driven-development` (권장) 또는
> `superpowers:executing-plans`로 태스크 단위 실행. 단계는 체크박스(`- [ ]`)로 추적한다.

**Goal:** 테이블 복도(corridor) 공간 keep으로 배경을 먼저 자르고, 스틸 GT ~10장 위에서
전처리·색공간·게이트·morph 조합을 정량 채점해 이긴 조합만 본선 검출기에 심는다.

**Architecture:** 현행 `mask → color → contour → scorer/ROI` 파이프라인에서 (1) `mask` 자리를
`FloorEdgeMask`(하단 컷) **AND** `TableCorridorMask`(테이블 XY 프리즘)를 합친 `SpatialMask`로
일반화하고, (2) colormask 단계의 축(전처리·색공간·게이트·morph)을 전부 `src/` 타입으로 구현한 뒤
(3) `tools/eval_colormask`가 stills GT로 조합을 랭킹하고, (4) 승자만 `detector_for`에 고정한다.

**Tech Stack:** Rust · OpenCV(`opencv` crate) · nalgebra · clap · serde_json

## Global Constraints

- 함수 반환은 **항상 `return expr;`** — 저장소 전역 스타일 ([`floor_edge.rs`](../../../src/detector/spatial/floor_edge.rs) 참고).
- **one-type-per-file** — 파일 하나에 공개 타입 하나. `mod.rs`는 선언 + `pub use`만.
- doc comment는 **한국어**, 첫 줄 `//!` 모듈 요약.
- 경로 SSOT는 [`src/defaults/calib.rs`](../../../src/defaults/calib.rs) — `data/calibration.json` · `data/colormask.json` · `data/clips`. 새 산출물도 `DEFAULT_DATA_DIR` 아래 상수로 추가한다.
- **호환 alias·침묵 폴백 금지.** 캘리브/colormask 없으면 `bail!`.
- TDD — 실패 테스트 먼저, `cargo test --workspace` 통과 후 커밋. 태스크당 커밋 1개 이상.
- 캘리브 여유 상수는 [`MAX_REPROJ_RMSE_PX`](../../../src/defaults/calib.rs) (= 7.0 px) 하나만 쓴다. 새 매직넘버 금지.

---

## 0. 현재 코드 상태 (2026-07-30 확인)

이 플랜은 Cursor 플랜 `spatial_then_color_a5553d0d`를 저장소로 옮기면서 **실제 코드에 맞춰 갱신**한 것이다.
원본 플랜이 쓰여진 7/29 이후 `refactor/one-type-per-file` 머지(`833291b`)로 모듈 경로가 바뀌었다.

### 있는 것

| 항목 | 위치 | 상태 |
|---|---|---|
| 바닥 컷 마스크 | [`src/detector/spatial/floor_edge.rs`](../../../src/detector/spatial/floor_edge.rs) | `FloorEdgeMask` — RMSE 마진 `δ = RMSE·Z_cam/fx`만큼 바깥으로 민 변, 사다리꼴 fill, `apply_bgr`/`draw_edge_line`, 단위 테스트 2개 |
| 캠별 면적 밴드 | [`src/detector/spatial/ball_area.rs`](../../../src/detector/spatial/ball_area.rs) | `ScorerParams::from_calib` |
| 조립 DSL | [`src/detector/builder.rs`](../../../src/detector/builder.rs) | `.mask().then().then().scorer().roi().build()` |
| 조립 SSOT | [`src/defaults/vision.rs`](../../../src/defaults/vision.rs) | `detector_for(camera::Id)` — `assemble`에서 `FloorEdgeMask` + colormask + contour |
| appearance 체인 | [`src/detector/appearance/chain.rs`](../../../src/detector/appearance/chain.rs) · [`layer.rs`](../../../src/detector/appearance/layer.rs) | `AppearanceLayer::apply(frame, prior) -> Option<Mat>`, 단계별 누적 마스크 `stage_masks` |
| 색 게이트 | [`colormask/detector.rs`](../../../src/detector/appearance/colormask/detector.rs) | **`inRange` 축별 AABB뿐** |
| 색 파라미터 | [`colormask/params.rs`](../../../src/detector/appearance/colormask/params.rs) · [`cam.rs`](../../../src/detector/appearance/colormask/cam.rs) | `ColormaskParams{space, c0..c2 min/max}` + `ColormaskCam.samples: Vec<[u8;3]>` (BGR, 좌표 없음) |
| 색공간 | [`colormask/color_space.rs`](../../../src/detector/appearance/colormask/color_space.rs) | **`Ycrcb` · `Hsv` 둘뿐** |
| 튜너 | [`tools/tune_colormask/`](../../../tools/tune_colormask/) | 퍼센타일 AABB(`--trim` 기본 10, `--margin` 3), 산점도 3장 + iso 큐브, `data/colormask.json` upsert |
| 디버그 뷰어 | [`tools/detect_full/src/main.rs`](../../../tools/detect_full/src/main.rs) | 5패널 `0 raw → 1 floor-mask → 2 colormask → 3 +contour → 4 roi`, 패널별 HUD |
| 오프라인 클립 | [`data/clips/`](../../../data/clips/) | `fly_01`(478f) · `roll_01` · `drop_02`, 1280×800, meas_fps ≈ 39.9. `--clip`은 `MonoOfflineArgs` |
| 색 샘플 | `data/colormask.json` | cam0 1129개 / cam1 882개 BGR 샘플, 현재 둘 다 `hsv` |

### 없는 것 (이 플랜이 만드는 것)

- 테이블 복도 keep · `FLIGHT_BAND_M` 상수 — `rg 'corridor|flight_band'` 0 hit
- 스틸 GT (`data/detect_stills/`) · 라벨 툴 · eval 하네스 — 0 hit
- 전처리(WB/CLAHE/bilateral/색맹축) · morph 레이어 · Lab/커스텀 색공간 · 타원체 게이트 — 전부 미구현

### 진단 (사진에서 확인된 것)

- floor-edge는 **하단만** 제거 → 윗 배경(책상·모니터·벽)이 그대로 colormask로 들어가 `nonzero ≈ 23k`.
- 카메라가 warm 캐스트 → 흰 테이블·목재 바닥이 주황 상자를 공유.
- 공이 저화질로 적은 픽셀 → 대표색이 주황/노랑/하양(하이라이트)으로 흩어지고, 그림자·혼색으로 초록끼 픽셀 발생.
- 축정렬 AABB는 극단값 1~2개로 상자가 커지고, 퍼센타일 trim으로도 **모서리의 빈 색**(갈·초록)은 못 막는다.

### 원본 플랜 대비 변경점

| 원본 | 이 문서 | 이유 |
|---|---|---|
| `cs.custom_wb_lab`를 색공간 축에 둠 | **삭제** — `(pre.gray_world, cs.lab)` 조합으로 표현 | 같은 연산의 중복 열거. 그리드에서 자동으로 커버됨 |
| 게이트 JSON 확장 방식 미정 | `ColormaskCam.gate: Option<EllipsoidGate>` 옵션 필드 | 기존 `data/colormask.json` 하위호환 유지 |
| 타원체 평가 방식 미정 | **256³ u8 LUT** | 1280×800 × 40 fps 본선에서 픽셀당 Mahalanobis는 못 씀 |
| `mask: FloorEdgeMask` 일반화 방법 미정 | 새 타입 `SpatialMask { keep, floor, corridor }` | `detect_full`이 `mask.cut_x` 등을 직접 읽고 있어 필드 보존 필요 |
| manifest 타입을 툴에 두고 Task 9에서 라이브러리로 이동 | **처음부터** `detector::stills` SSOT (구현 시 반영) | 툴 둘이 같은 스키마를 읽는다 — 나중 이동은 불필요한 왕복 |
| `tune-colormask`는 Task 10에서 손봄 | Task 6에서 **4공간 순환**으로 일반화 (구현 시 반영) | `ColorSpace` 확장으로 `match`가 non-exhaustive가 되어 컴파일이 깨짐 |

---

## 1. 파일 구조

```
src/detector/spatial/
  floor_edge.rs        (수정) project_unbounded 재사용 — 변경 없음
  table_corridor.rs    (신규) TableCorridorMask
  spatial_mask.rs      (신규) SpatialMask — floor AND corridor
  mod.rs               (수정) 선언·재export
src/detector/appearance/
  preprocess.rs        (신규) Preprocess — BGR→BGR 보정
  morph.rs             (신규) MorphOp — AppearanceLayer
  colormask/
    color_space.rs     (수정) Lab · CustomHab 추가
    gate.rs            (신규) ColorGate / EllipsoidGate + LUT
    cam.rs             (수정) gate 옵션 필드
    detector.rs        (수정) gate 분기
src/detector/{builder,detector}.rs   (수정) mask: SpatialMask, pre: Preprocess
src/defaults/vision.rs               (수정) FLIGHT_BAND_M, assemble
tools/label_stills/                  (신규) 스틸 덤프 + 클릭 라벨
tools/eval_colormask/                (신규) 메소드 그리드 채점
data/detect_stills/manifest.json     (신규 산출물)
```

각 파일 책임 하나. `TableCorridorMask`는 기하만, `SpatialMask`는 합성만, `ColorGate`는 판정만 한다.

---

## 2. 메소드 그리드 (후보 풀 — 전부 구현한다)

**축을 곱하되 채점은 2단.** 후보에서 빠지는 메소드는 없고, 풀 카테시안 곱을 한 번에 돌리지도 않는다.

### A. 전처리 (`Preprocess`, BGR → BGR)

| ID | 메소드 | 의도 |
|---|---|---|
| `none` | 없음 | 베이스라인 |
| `gray_world` | gray-world WB | warm 캐스트 완화 |
| `warm_pushback` | 채널 게인으로 황·적 억제 | 나무·흰 벽이 주황으로 보이는 문제 |
| `clahe_v` | Lab L 채널 CLAHE | 작은 공·하이라이트 대비 |
| `bilateral` | bilateral 필터 | 노이즈↓, 엣지 유지 |
| `gauss` | 작은 Gaussian | 센서 노이즈 |
| `cb_sim` | 색맹(deuteranope) 시뮬 축 | 사람 눈에 덜 헷갈리는 대비 강제 |

### B. 커널 (`MorphOp`, 게이트 비트맵 뒤)

`none` · `open3` · `open5` · `close3` · `open_close`

### C. 색공간 (`ColorSpace`)

`hsv`(현행) · `ycrcb`(현행) · `lab` · `custom_h_ab`(HSV H + Lab a\*b\*)

### D. 게이트 (`ColorGate`, 샘플로 피팅)

`aabb`(현행) · `aabb_pct`(퍼센타일 trim 스윕) · `ellipsoid`(Mahalanobis 전공분산) · `ellipsoid_diag`(대각 근사)

피팅 소스: `data/colormask.json`의 cam별 BGR `samples` (cam0 1129 / cam1 882).

### E. 공간 (그리드 밖 필수 트랙)

`spatial.floor_only`(현행) → `spatial.corridor`(Task 1~2). **색 그리드는 corridor on 위에서 돈다.**

### 스윕 순서

1. **메인:** `pre{none, gray_world, warm_pushback}` × `cs{hsv, ycrcb, lab, custom_h_ab}` × `gate{aabb, ellipsoid}` × `morph{none, open3}` = **48 조합**
2. **확장:** 메인 상위 5개에만 `clahe_v` / `bilateral` / `gauss` / `cb_sim` / `aabb_pct` / `ellipsoid_diag` / `open5` / `close3` / `open_close` 추가

---

## 3. GT 스키마 (스틸 ~10장)

**비디오 전 프레임 GT는 비범위.** 클립 타임라인을 등분해 캠·클립당 ~10장, 그중 2~3장은 무공.

```
data/detect_stills/
  manifest.json
  fly_01_left_t000.png
  fly_01_left_t048.png
  ...
```

```json
{
  "hit_radius_px": 20.0,
  "items": [
    { "path": "fly_01_left_t048.png", "camera_id": 0, "clip": "fly_01", "frame": 48, "pixel": [812.0, 340.5] },
    { "path": "fly_01_left_t400.png", "camera_id": 0, "clip": "fly_01", "frame": 400, "pixel": null }
  ]
}
```

`pixel: null` = 무공. 검출되면 **FP**, 없으면 **TN**. 채점은 hit / miss / FP / TN.

---

## Task 1: TableCorridorMask

테이블 XY 사각형을 RMSE 마진만큼 팽창한 뒤 `z ∈ [SURFACE_Z, SURFACE_Z + FLIGHT_BAND_M]` 프리즘의
8꼭짓점을 투영 → convex hull → fill. floor-edge와 달리 **keep(255) 영역을 그린다.**

> **한계 (구현 중 실측, 2026-07-30):** hull은 프리즘의 **실루엣**이므로 keep은 부피가 아니라
> **시선 원뿔**이다. 프리즘과 같은 방향의 **더 먼 배경**은 잘리지 않는다.
> 실제 캘리브(cam0 eye ≈ `(-1.45, 1.35, 2.07)` — 상판보다 1.3 m 위)에서는 밴드 상단 꼭짓점이
> 프레임 밖(v ≈ −150)으로 나가, 윗 배경까지 keep에 들어온다. `fly_01` frame 40 실측:
>
> | `FLIGHT_BAND_M` | keep | colormask nonzero |
> |---|---|---|
> | 1.0 | 61% | 23,346 (개선 전과 동일) |
> | 0.3 | 45% | 12,963 |
> | ≥0.75 | 포화 (cam0 61% / cam1 69%) | — |
>
> **결정:** 밴드는 **1.0 m 유지** — "공이 상판 1 m 위로 가지 않는다"는 물리 판단이 기준이고,
> 검출 가능 공간을 배경 컷 편의로 좁히지 않는다. 따라서 corridor의 실효는 **바닥·측면 컷(≈39%)**이고,
> **윗 배경 오탐 제거는 색 그리드(Task 5~9)가 담당한다.** 깊이 기반 컷은 스테레오 필요 — 비범위.
> 단위 테스트도 이 성질에 맞춰 "밴드 위 점이 hull 밖"을 검증한다.

**Files:**
- Create: `src/detector/spatial/table_corridor.rs`
- Modify: `src/detector/spatial/mod.rs`
- Modify: `src/defaults/vision.rs` (상수 `FLIGHT_BAND_M`)

**Interfaces:**
- Consumes: `crate::detector::spatial::floor_edge::project_unbounded(&camera::Params, Point3) -> Option<(f64,f64,f64)>` (이미 `pub(crate)`, 형제 모듈에서 그대로 호출)
- Produces: `TableCorridorMask { keep: Mat, hull: Vector<Point>, band_m: f64, margin_m: f64, width: i32, height: i32 }`,
  `TableCorridorMask::from_params(params: &camera::Params, band_m: f64) -> Result<Self>`,
  `draw_hull(&self, img: &mut Mat, color: Scalar, thickness: i32) -> Result<()>`

- [ ] **Step 1: `FLIGHT_BAND_M` 상수 추가**

`src/defaults/vision.rs`의 `MOTION_WEIGHT` 위에:

```rust
/// 테이블 상판 위 공 비행 높이 keep [m].
pub const FLIGHT_BAND_M: f64 = 1.0;
```

`src/defaults/mod.rs`가 `pub use vision::*` 계열이면 자동, 아니면 재export를 추가한다.

- [ ] **Step 2: 실패 테스트 작성**

`src/detector/spatial/table_corridor.rs` 하단:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    fn overhead_looking_down() -> camera::Params {
        let eye = Vector3::new(-0.4, table::LENGTH_Y * 0.5, table::SURFACE_Z + 1.6);
        let target = Vector3::new(table::WIDTH_X * 0.35, table::LENGTH_Y * 0.5, table::SURFACE_Z);
        return camera::Params::look_at(
            camera::Id(0),
            None,
            eye,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            640,
            480,
            55.0_f64.to_radians(),
        );
    }

    fn keep_at(mask: &TableCorridorMask, params: &camera::Params, p: Point3) -> Option<u8> {
        let (u, v, _) = project_unbounded(params, p)?;
        let (u, v) = (u.round() as i32, v.round() as i32);
        if !(0..mask.width).contains(&u) || !(0..mask.height).contains(&v) {
            return None;
        }
        return mask.keep.at_2d::<u8>(v, u).ok().copied();
    }

    #[test]
    fn corridor_keeps_table_and_band_drops_far_exterior() {
        let params = overhead_looking_down();
        let mask = TableCorridorMask::from_params(&params, 1.0).expect("corridor");
        assert_eq!(mask.keep.cols(), 640);
        assert!(mask.margin_m > 0.0);

        let center = Point3::new(table::WIDTH_X * 0.5, table::LENGTH_Y * 0.5, table::SURFACE_Z);
        let above = Point3::new(table::WIDTH_X * 0.5, table::LENGTH_Y * 0.5, table::SURFACE_Z + 0.5);
        let far = Point3::new(table::WIDTH_X + 2.0, table::LENGTH_Y * 0.5, table::SURFACE_Z);

        if let Some(k) = keep_at(&mask, &params, center) {
            assert_eq!(k, 255, "table surface must be kept");
        }
        if let Some(k) = keep_at(&mask, &params, above) {
            assert_eq!(k, 255, "in-band point must be kept");
        }
        if let Some(k) = keep_at(&mask, &params, far) {
            assert_eq!(k, 0, "far exterior must be dropped");
        }
    }

    #[test]
    fn band_height_widens_keep_area() {
        let params = overhead_looking_down();
        let low = TableCorridorMask::from_params(&params, 0.2).unwrap();
        let high = TableCorridorMask::from_params(&params, 1.5).unwrap();
        let low_n = opencv::core::count_non_zero(&low.keep).unwrap();
        let high_n = opencv::core::count_non_zero(&high.keep).unwrap();
        assert!(high_n > low_n, "taller band should keep more: {high_n} vs {low_n}");
    }
}
```

- [ ] **Step 3: 테스트 실패 확인**

Run: `cargo test -p pingpong-bot table_corridor -- --nocapture`
Expected: FAIL — `TableCorridorMask` 미정의 (컴파일 에러)

- [ ] **Step 4: 구현**

`src/detector/spatial/table_corridor.rs` 상단:

```rust
//! 테이블 XY 프리즘(상판 + 비행 밴드) 투영 → convex hull keep 마스크.
//!
//! floor-edge가 "바닥을 지우는" 마스크라면, 이쪽은 "대 위만 남기는" 마스크다.
//! XY는 [`MAX_REPROJ_RMSE_PX`] → 미터 환산 마진만큼 바깥으로 팽창한다.

use crate::camera;
use anyhow::{Result, bail, ensure};
use opencv::core::{Point, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::floor_edge::project_unbounded;
use crate::Point3;
use crate::constants::table;
use crate::defaults::MAX_REPROJ_RMSE_PX;

/// 테이블 복도 keep 마스크 (255=검출 허용).
#[derive(Clone)]
pub struct TableCorridorMask {
    pub keep: Mat,
    /// 그리기용 convex hull (이미지 좌표).
    pub hull: Vector<Point>,
    /// 상판 위 비행 밴드 높이 [m].
    pub band_m: f64,
    /// XY 팽창 마진 [m].
    pub margin_m: f64,
    pub width: i32,
    pub height: i32,
}

impl TableCorridorMask {
    pub fn from_params(params: &camera::Params, band_m: f64) -> Result<Self> {
        let w = params.width as i32;
        let h = params.height as i32;
        ensure!(w > 1 && h > 1, "bad image size {}x{}", w, h);
        ensure!(params.fx > 0.0, "corridor: fx must be > 0");
        ensure!(band_m > 0.0, "corridor: band_m must be > 0");

        let z0 = table::SURFACE_Z;
        let center = Point3::new(table::WIDTH_X * 0.5, table::LENGTH_Y * 0.5, z0);
        let Some((_, _, z_cam)) = project_unbounded(params, center) else {
            bail!("corridor: table center behind camera");
        };
        let margin_m = MAX_REPROJ_RMSE_PX * z_cam / params.fx;
        ensure!(margin_m.is_finite() && margin_m >= 0.0, "corridor: bad margin");

        let x0 = -margin_m;
        let x1 = table::WIDTH_X + margin_m;
        let y0 = -margin_m;
        let y1 = table::LENGTH_Y + margin_m;
        let z1 = z0 + band_m;

        let mut pts = Vector::<Point>::new();
        for &z in &[z0, z1] {
            for &(x, y) in &[(x0, y0), (x1, y0), (x1, y1), (x0, y1)] {
                let Some((u, v, _)) = project_unbounded(params, Point3::new(x, y, z)) else {
                    continue;
                };
                // 프레임 밖으로 나간 꼭짓점도 hull 계산에는 넣되, 좌표는 넉넉히 clamp
                let u = u.clamp(f64::from(-w), f64::from(2 * w));
                let v = v.clamp(f64::from(-h), f64::from(2 * h));
                pts.push(Point::new(u.round() as i32, v.round() as i32));
            }
        }
        ensure!(pts.len() >= 3, "corridor: too few projectable corners ({})", pts.len());

        let mut hull = Vector::<Point>::new();
        imgproc::convex_hull(&pts, &mut hull, true, true)?;

        let mut keep = Mat::new_rows_cols_with_default(h, w, opencv::core::CV_8UC1, Scalar::all(0.0))?;
        imgproc::fill_convex_poly(&mut keep, &hull, Scalar::all(255.0), imgproc::LINE_8, 0)?;

        return Ok(Self { keep, hull, band_m, margin_m, width: w, height: h });
    }

    /// keep hull 외곽선을 `img`에 그린다.
    pub fn draw_hull(&self, img: &mut Mat, color: Scalar, thickness: i32) -> Result<()> {
        let polys = Vector::<Vector<Point>>::from_iter([self.hull.clone()]);
        imgproc::polylines(img, &polys, true, color, thickness, imgproc::LINE_8, 0)?;
        return Ok(());
    }
}
```

`src/detector/spatial/mod.rs`:

```rust
//! 고정 캠 테이블 투영 → 공간 keep 마스크 · 공 면적 밴드.

mod ball_area;
mod floor_edge;
mod table_corridor;

pub(crate) use ball_area::scorer_params_from_calib;
pub use floor_edge::FloorEdgeMask;
pub use table_corridor::TableCorridorMask;
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot table_corridor`
Expected: PASS (2 tests)

- [ ] **Step 6: 커밋**

```bash
git add src/detector/spatial/table_corridor.rs src/detector/spatial/mod.rs src/defaults/vision.rs
git commit -m "feat(vision): table corridor keep mask from calib prism"
```

---

## Task 2: SpatialMask 합성 + 조립 배선

`Detector.mask`를 합성 keep 타입으로 일반화한다. `detect_full`이 `mask.cut_x` / `mask.margin_m` /
`mask.line_y_at_*` / `mask.keep` / `mask.width|height`를 직접 읽으므로 **floor 필드를 보존**한다.

**Files:**
- Create: `src/detector/spatial/spatial_mask.rs`
- Modify: `src/detector/spatial/mod.rs`, `src/detector/detector.rs`, `src/detector/builder.rs`, `src/detector/mod.rs`, `src/defaults/vision.rs`

**Interfaces:**
- Consumes: `FloorEdgeMask`, `TableCorridorMask` (Task 1)
- Produces: `SpatialMask { keep, floor: FloorEdgeMask, corridor: Option<TableCorridorMask>, width, height }`,
  `SpatialMask::floor_only(FloorEdgeMask) -> Self`,
  `SpatialMask::with_corridor(FloorEdgeMask, TableCorridorMask) -> Result<Self>`,
  `apply_bgr(&Mat) -> Result<Mat>`, `keep_percent(&self) -> f64`
- `DetectorBuilder::mask(impl Into<SpatialMask>)` — `From<FloorEdgeMask> for SpatialMask` 제공으로 기존 호출부 무변경

- [ ] **Step 1: 실패 테스트 작성**

`src/detector/spatial/spatial_mask.rs` 하단:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::table;
    use nalgebra::Vector3;

    fn cam() -> camera::Params {
        let eye = Vector3::new(-0.4, table::LENGTH_Y * 0.5, table::SURFACE_Z + 1.6);
        let target = Vector3::new(table::WIDTH_X * 0.35, table::LENGTH_Y * 0.5, table::SURFACE_Z);
        return camera::Params::look_at(
            camera::Id(0),
            None,
            eye,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            640,
            480,
            55.0_f64.to_radians(),
        );
    }

    #[test]
    fn corridor_and_floor_keeps_no_more_than_floor_alone() {
        let params = cam();
        let floor = FloorEdgeMask::from_params(camera::Id(0), &params).unwrap();
        let floor_only = SpatialMask::floor_only(floor.clone());
        let corridor = TableCorridorMask::from_params(&params, 1.0).unwrap();
        let combined = SpatialMask::with_corridor(floor, corridor).unwrap();

        let a = opencv::core::count_non_zero(&floor_only.keep).unwrap();
        let b = opencv::core::count_non_zero(&combined.keep).unwrap();
        assert!(b <= a, "AND must not grow keep: {b} > {a}");
        assert!(b > 0, "corridor AND floor should keep something");
        assert!(combined.keep_percent() <= floor_only.keep_percent());
    }

    #[test]
    fn apply_bgr_blacks_dropped_pixels() {
        let params = cam();
        let floor = FloorEdgeMask::from_params(camera::Id(0), &params).unwrap();
        let corridor = TableCorridorMask::from_params(&params, 1.0).unwrap();
        let mask = SpatialMask::with_corridor(floor, corridor).unwrap();
        let bgr = Mat::new_size_with_default(
            opencv::core::Size::new(640, 480),
            opencv::core::CV_8UC3,
            Scalar::all(200.0),
        )
        .unwrap();
        let out = mask.apply_bgr(&bgr).unwrap();
        for y in 0..480 {
            for x in 0..640 {
                let k: u8 = *mask.keep.at_2d(y, x).unwrap();
                if k == 0 {
                    let px: opencv::core::Vec3b = *out.at_2d(y, x).unwrap();
                    assert_eq!(px, opencv::core::Vec3b::from([0, 0, 0]));
                    return;
                }
            }
        }
        panic!("mask should drop at least one pixel");
    }
}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cargo test -p pingpong-bot spatial_mask`
Expected: FAIL — `SpatialMask` 미정의

- [ ] **Step 3: `SpatialMask` 구현**

```rust
//! floor-edge 컷 AND 테이블 복도 keep — 본선 검출기의 공간 게이트.

use crate::camera;
use anyhow::{Result, ensure};
use opencv::core::Scalar;
use opencv::prelude::*;

use super::floor_edge::FloorEdgeMask;
use super::table_corridor::TableCorridorMask;

/// 공간 keep 합성 (255=검출 허용).
#[derive(Clone)]
pub struct SpatialMask {
    pub keep: Mat,
    pub floor: FloorEdgeMask,
    pub corridor: Option<TableCorridorMask>,
    pub width: i32,
    pub height: i32,
}

impl SpatialMask {
    pub fn floor_only(floor: FloorEdgeMask) -> Self {
        let keep = floor.keep.clone();
        let (width, height) = (floor.width, floor.height);
        return Self { keep, floor, corridor: None, width, height };
    }

    pub fn with_corridor(floor: FloorEdgeMask, corridor: TableCorridorMask) -> Result<Self> {
        ensure!(
            floor.width == corridor.width && floor.height == corridor.height,
            "mask size mismatch: floor {}x{} vs corridor {}x{}",
            floor.width, floor.height, corridor.width, corridor.height
        );
        let mut keep = Mat::default();
        opencv::core::bitwise_and(&floor.keep, &corridor.keep, &mut keep, &Mat::default())?;
        let (width, height) = (floor.width, floor.height);
        return Ok(Self { keep, floor, corridor: Some(corridor), width, height });
    }

    /// keep=0 화소를 검게.
    pub fn apply_bgr(&self, bgr: &Mat) -> Result<Mat> {
        ensure!(
            bgr.cols() == self.width && bgr.rows() == self.height,
            "frame size {}x{} != mask {}x{}",
            bgr.cols(), bgr.rows(), self.width, self.height
        );
        let mut out = Mat::zeros(self.height, self.width, bgr.typ())?.to_mat()?;
        bgr.copy_to_masked(&mut out, &self.keep)?;
        return Ok(out);
    }

    /// 남긴 화소 비율 [%] — HUD용.
    pub fn keep_percent(&self) -> f64 {
        let total = self.width.saturating_mul(self.height).max(1);
        let kept = opencv::core::count_non_zero(&self.keep).unwrap_or(0);
        return 100.0 * f64::from(kept) / f64::from(total);
    }

    /// 컷 선 + 복도 hull 오버레이.
    pub fn draw_overlay(&self, img: &mut Mat, floor_color: Scalar, corridor_color: Scalar) -> Result<()> {
        self.floor.draw_edge_line(img, floor_color, 2)?;
        if let Some(c) = &self.corridor {
            c.draw_hull(img, corridor_color, 2)?;
        }
        return Ok(());
    }
}

impl From<FloorEdgeMask> for SpatialMask {
    fn from(floor: FloorEdgeMask) -> Self {
        return Self::floor_only(floor);
    }
}
```

`mod.rs`에 `mod spatial_mask;` + `pub use spatial_mask::SpatialMask;` 추가. `use crate::camera;`가 테스트에서만
쓰이면 `#[cfg(test)]` 쪽으로 옮긴다.

- [ ] **Step 4: `Detector` / `DetectorBuilder` 배선**

`src/detector/detector.rs`: `pub mask: FloorEdgeMask` → `pub mask: SpatialMask`, import 교체.
`detect()` 본문은 그대로 (`self.mask.apply_bgr`).

`src/detector/builder.rs`:

```rust
mask: Option<SpatialMask>,
...
pub fn mask(mut self, mask: impl Into<SpatialMask>) -> Self {
    self.mask = Some(mask.into());
    return self;
}
```

`src/detector/mod.rs`에 `SpatialMask`, `TableCorridorMask` 재export 추가.

- [ ] **Step 5: `detector_for`에서 corridor 조립**

`src/defaults/vision.rs`:

```rust
fn assemble(camera_id: camera::Id, color: ColormaskParams, cam: &camera::Params) -> Result<Detector> {
    let circ = ScorerParams::default().min_circularity;
    let scorer = ScorerParams::from_calib(cam, circ)?;
    let floor = FloorEdgeMask::from_params(camera_id, cam)?;
    let corridor = TableCorridorMask::from_params(cam, FLIGHT_BAND_M)?;
    let mask = SpatialMask::with_corridor(floor, corridor)?;

    return Detector::builder()
        .mask(mask)
        .then(ColormaskDetector::new(color))
        .then(ContourDetector::from(&scorer))
        .scorer(Scorer::from(&scorer).with_motion_weight(MOTION_WEIGHT))
        .roi(RoiParams::default())
        .build();
}
```

- [ ] **Step 6: 전체 테스트**

Run: `cargo test --workspace`
Expected: PASS. 컴파일 에러가 나면 `detector.mask.*` 접근부를 `detector.mask.floor.*`로 고친다 (Task 3에서 다룰 `detect_full` 포함).

- [ ] **Step 7: 커밋**

```bash
git add src/detector src/defaults/vision.rs
git commit -m "feat(vision): compose floor-edge and corridor into SpatialMask"
```

---

## Task 3: detect_full corridor 패널·HUD

**Files:**
- Modify: `tools/detect_full/src/main.rs`, `tools/detect_full/README.md`

**Interfaces:**
- Consumes: `SpatialMask::{draw_overlay, keep_percent, floor, corridor}` (Task 2)
- Produces: 없음 (툴)

- [ ] **Step 1: 패널 라벨·오버레이 교체**

`main.rs`에서 `detector.mask.draw_edge_line(...)` 호출을 다음으로:

```rust
detector.mask.draw_overlay(
    &mut mask_panel,
    Scalar::new(255.0, 255.0, 0.0, 0.0), // cyan: floor cut line
    Scalar::new(255.0, 0.0, 255.0, 0.0), // magenta: corridor hull
)?;
```

`Preview::draw_cam_label(&mut mask_panel, "1 floor-mask", cyan)?` → `"1 spatial-keep"`.

- [ ] **Step 2: HUD에 corridor 정보 추가**

기존 mask 패널 HUD 블록을 다음으로:

```rust
let corridor_hud = match &detector.mask.corridor {
    Some(c) => format!("corridor band={:.2}m margin={:.3}m", c.band_m, c.margin_m),
    None => "corridor off".to_string(),
};
draw_panel_hud(
    &mut mask_panel,
    &[
        "spatial keep".to_string(),
        format!("cut_x={:.3}m  margin={:.3}m", detector.mask.floor.cut_x, detector.mask.floor.margin_m),
        corridor_hud,
        format!("keep={:.0}%  (cut={:.0}%)", detector.mask.keep_percent(), 100.0 - detector.mask.keep_percent()),
    ],
    cyan,
)?;
```

기존 `keep_nonzero` / `total_pixels` / `cut_percent` 지역변수는 `keep_percent()`로 대체하고 지운다.
`detector.mask.width|height` 참조는 그대로 동작한다 (`SpatialMask`도 같은 필드를 가진다).

- [ ] **Step 3: 빌드·수동 확인**

Run: `cargo run -p detect-full -- --cam left --clip fly_01 --no-roi`
Expected: 패널 1이 `1 spatial-keep`으로 뜨고, 마젠타 hull 안쪽만 남으며 HUD에 `corridor band=1.00m`과 `keep=…%`가 보인다.

- [ ] **Step 4: README 스텝 문구 갱신**

`tools/detect_full/README.md`와 `main.rs` 상단 doc comment의
`0 raw → 1 floor-mask → 2 colormask → 3 +contour → 4 roi`를
`0 raw → 1 spatial-keep → 2 colormask → 3 +contour → 4 roi`로.

- [ ] **Step 5: 커밋**

```bash
git add tools/detect_full
git commit -m "feat(tools): show corridor keep and band in detect-full"
```

---

## Task 4: 스틸 GT 덤프 + 클릭 라벨러

**Files:**
- Create: `tools/label_stills/Cargo.toml`, `tools/label_stills/src/main.rs`, `tools/label_stills/src/cli.rs`, `tools/label_stills/src/manifest.rs`, `tools/label_stills/README.md`
- Modify: `Cargo.toml`(workspace members는 `tools/*` glob이라 수정 불필요), `src/defaults/calib.rs`(경로 상수)

**Interfaces:**
- Consumes: `camera::{MonoOfflineArgs, CamCliArgs, FrameSource, Preview, PixelPickMouse}`,
  `defaults::{DEFAULT_DATA_DIR, ensure_parent_dir}`
- Produces: `data/detect_stills/manifest.json` (§3 스키마), `manifest.rs`의
  `StillsManifest { hit_radius_px: f64, items: Vec<StillItem> }`,
  `StillItem { path: String, camera_id: camera::Id, clip: String, frame: usize, pixel: Option<[f64; 2]> }`,
  `StillsManifest::load_or_default(&Path)`, `upsert(&mut self, StillItem)`, `save(&self, &Path)`

- [ ] **Step 1: 경로 상수 추가**

`src/defaults/calib.rs`:

```rust
/// 정량 eval용 스틸 GT 루트 (`manifest.json` + PNG).
pub const DEFAULT_DETECT_STILLS_DIR: &str = "data/detect_stills";
/// 스틸 GT manifest SSOT.
pub const DEFAULT_DETECT_STILLS_MANIFEST: &str = "data/detect_stills/manifest.json";

/// [`DEFAULT_DETECT_STILLS_MANIFEST`]의 `PathBuf`.
pub fn detect_stills_manifest_path() -> PathBuf {
    return PathBuf::from(DEFAULT_DETECT_STILLS_MANIFEST);
}
```

- [ ] **Step 2: manifest 실패 테스트**

`tools/label_stills/src/manifest.rs` 하단:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    fn item(path: &str, pixel: Option<[f64; 2]>) -> StillItem {
        return StillItem {
            path: path.to_string(),
            camera_id: camera::Id(0),
            clip: "fly_01".to_string(),
            frame: 48,
            pixel,
        };
    }

    #[test]
    fn upsert_replaces_same_path() {
        let mut m = StillsManifest::default();
        m.upsert(item("a.png", Some([1.0, 2.0])));
        m.upsert(item("a.png", None));
        assert_eq!(m.items.len(), 1);
        assert!(m.items[0].pixel.is_none());
    }

    #[test]
    fn roundtrip_keeps_null_pixel() {
        let mut m = StillsManifest::default();
        m.upsert(item("a.png", None));
        m.upsert(item("b.png", Some([3.5, 4.5])));
        let text = serde_json::to_string(&m).unwrap();
        assert!(text.contains("\"pixel\":null"));
        let back: StillsManifest = serde_json::from_str(&text).unwrap();
        assert_eq!(back.items.len(), 2);
        assert_eq!(back.hit_radius_px, m.hit_radius_px);
    }
}
```

- [ ] **Step 3: 테스트 실패 확인**

Run: `cargo test -p label-stills`
Expected: FAIL — 크레이트/타입 없음

- [ ] **Step 4: 크레이트 + manifest 구현**

`tools/label_stills/Cargo.toml` (다른 툴 것을 복사하고 이름만 교체):

```toml
[package]
name = "label-stills"
version.workspace = true
edition.workspace = true

[dependencies]
pingpong-bot = { path = "../.." }
anyhow.workspace = true
clap.workspace = true
opencv.workspace = true
serde = { workspace = true, features = ["derive"] }
serde_json.workspace = true
```

> 워크스페이스에 해당 key가 없으면 `tools/tune_colormask/Cargo.toml`의 표기를 그대로 따른다.

`src/manifest.rs`:

```rust
//! 스틸 GT manifest — `data/detect_stills/manifest.json`.

use std::path::Path;

use anyhow::{Context, Result};
use pingpong_bot::camera;

/// 기본 hit 판정 반경 [px].
pub const DEFAULT_HIT_RADIUS_PX: f64 = 20.0;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct StillItem {
    pub path: String,
    pub camera_id: camera::Id,
    pub clip: String,
    pub frame: usize,
    /// `[u, v]` = 유공, `null` = 무공.
    pub pixel: Option<[f64; 2]>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct StillsManifest {
    pub hit_radius_px: f64,
    pub items: Vec<StillItem>,
}

impl Default for StillsManifest {
    fn default() -> Self {
        return Self { hit_radius_px: DEFAULT_HIT_RADIUS_PX, items: Vec::new() };
    }
}

impl StillsManifest {
    pub fn load_or_default(path: &Path) -> Result<Self> {
        if !path.exists() {
            return Ok(Self::default());
        }
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("manifest 읽기: {}", path.display()))?;
        return serde_json::from_str(&text)
            .with_context(|| format!("manifest JSON: {}", path.display()));
    }

    /// 같은 `path`면 교체, 없으면 추가.
    pub fn upsert(&mut self, item: StillItem) {
        if let Some(slot) = self.items.iter_mut().find(|i| i.path == item.path) {
            *slot = item;
            return;
        }
        self.items.push(item);
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        pingpong_bot::defaults::ensure_parent_dir(path)?;
        let text = serde_json::to_string_pretty(self)?;
        std::fs::write(path, text)
            .with_context(|| format!("manifest 쓰기: {}", path.display()))?;
        return Ok(());
    }
}
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `cargo test -p label-stills`
Expected: PASS (2 tests)

- [ ] **Step 6: 덤프 + 라벨 UI 구현**

`src/cli.rs`:

```rust
use clap::Parser;
use pingpong_bot::camera::{CamCliArgs, MonoOfflineArgs};

#[derive(Parser, Debug)]
#[command(about = "클립 등분 덤프 → 클릭 라벨 → data/detect_stills/manifest.json")]
pub struct Args {
    #[command(flatten)]
    pub cam: CamCliArgs,
    #[command(flatten)]
    pub offline: MonoOfflineArgs,
    /// 뽑을 스틸 수 (타임라인 등분)
    #[arg(long, default_value_t = 10)]
    pub count: usize,
    /// hit 판정 반경 [px] — manifest에 저장
    #[arg(long, default_value_t = 20.0)]
    pub hit_radius: f64,
}
```

`src/main.rs` 동작:

1. `args.cam.open_mono_input(&args.offline)`로 클립 열기 — `--clip` 필수(`bail!` if `!args.offline.has_offline()`).
2. 전 프레임을 순회하며 인덱스를 세고, `stride = total / count`로 `count`개만 `Vec<(usize, Mat)>`에 보관.
   `total`은 첫 순회로 알 수 없으므로 **두 번 연다**: 1회차는 프레임 수만 세고, 2회차에서 `stride` 간격으로 보관.
3. 각 스틸마다 창에 띄우고:
   - 좌클릭 = 공 중심 (`PixelPickMouse::drain_clicks` + `sync(scale, w, h)`, 화살표 키는 `Preview::arrow_delta`로 nudge, `Preview::draw_pixel_loupe`로 확대)
   - `n` = 무공(`pixel: null`) 후 다음
   - `u` = 직전 라벨 취소
   - `q`/ESC = 저장 후 종료
4. 라벨 확정 시 `data/detect_stills/{clip}_{role}_t{frame:04}.png`로 `imgcodecs::imwrite`, manifest `upsert`, 즉시 `save`.
5. 종료 시 요약 출력: `labeled=8 empty=2 → data/detect_stills/manifest.json`.

- [ ] **Step 7: 실제 라벨 수집 (사람 작업)**

Run:
```bash
cargo run -p label-stills -- --cam left  --clip fly_01 --count 10
cargo run -p label-stills -- --cam right --clip fly_01 --count 10
```
Expected: `data/detect_stills/`에 PNG 20장 + manifest. **캠당 최소 2장은 `n`(무공)** 으로 남긴다.

- [ ] **Step 8: 커밋**

```bash
git add tools/label_stills src/defaults/calib.rs data/detect_stills
git commit -m "feat(tools): still GT dumper and click labeler"
```

> PNG가 커서 저장소가 부담되면 `data/detect_stills/*.png`를 `.gitignore`에 넣고 manifest만 커밋한다.
> 그 경우 README에 재생성 명령을 적는다.

---

## Task 5: 전처리 레이어 (`Preprocess`)

**Files:**
- Create: `src/detector/appearance/preprocess.rs`
- Modify: `src/detector/appearance/mod.rs`, `src/detector/mod.rs`, `src/detector/detector.rs`, `src/detector/builder.rs`

**Interfaces:**
- Produces: `Preprocess` enum `{ None, GrayWorld, WarmPushback, ClaheV, Bilateral, Gauss, CbSim }`,
  `Preprocess::apply(&self, bgr: &Mat) -> Result<Mat>`,
  `Preprocess::all() -> [Preprocess; 7]`, `impl Display`(ID 문자열: `none`/`gray_world`/…),
  `impl std::str::FromStr`
- `Detector { pub pre: Preprocess, … }` — `mask.apply_bgr` **뒤**, `roi.detect` **앞**에 적용
- `DetectorBuilder::pre(Preprocess)` — 미지정 시 `Preprocess::None`

- [ ] **Step 1: 실패 테스트 작성**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use opencv::core::{CV_8UC3, Size};

    fn warm_patch() -> Mat {
        // B=60, G=120, R=220 — 붉은 캐스트
        return Mat::new_size_with_default(Size::new(32, 32), CV_8UC3, Scalar::new(60.0, 120.0, 220.0, 0.0)).unwrap();
    }

    fn mean_bgr(m: &Mat) -> [f64; 3] {
        let s = opencv::core::mean(m, &Mat::default()).unwrap();
        return [s[0], s[1], s[2]];
    }

    #[test]
    fn none_is_identity() {
        let src = warm_patch();
        let out = Preprocess::None.apply(&src).unwrap();
        assert_eq!(mean_bgr(&out), mean_bgr(&src));
    }

    #[test]
    fn gray_world_flattens_channel_means() {
        let src = warm_patch();
        let before = mean_bgr(&src);
        let after = mean_bgr(&Preprocess::GrayWorld.apply(&src).unwrap());
        let spread = |m: [f64; 3]| m.iter().cloned().fold(f64::MIN, f64::max) - m.iter().cloned().fold(f64::MAX, f64::min);
        assert!(spread(after) < spread(before), "{after:?} vs {before:?}");
    }

    #[test]
    fn warm_pushback_reduces_red_bias() {
        let src = warm_patch();
        let before = mean_bgr(&src);
        let after = mean_bgr(&Preprocess::WarmPushback.apply(&src).unwrap());
        assert!(after[2] < before[2], "red should drop: {} -> {}", before[2], after[2]);
    }

    #[test]
    fn all_variants_preserve_size_and_type() {
        let src = warm_patch();
        for p in Preprocess::all() {
            let out = p.apply(&src).unwrap_or_else(|e| panic!("{p}: {e}"));
            assert_eq!(out.size().unwrap(), src.size().unwrap(), "{p}");
            assert_eq!(out.typ(), src.typ(), "{p}");
        }
    }

    #[test]
    fn id_roundtrips_through_str() {
        for p in Preprocess::all() {
            let s = p.to_string();
            assert_eq!(s.parse::<Preprocess>().unwrap(), p);
        }
    }
}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cargo test -p pingpong-bot preprocess`
Expected: FAIL — `Preprocess` 미정의

- [ ] **Step 3: 구현**

각 variant 구현 요지 (전부 `CV_8UC3` in/out 유지):

| variant | 구현 |
|---|---|
| `None` | `bgr.try_clone()` |
| `GrayWorld` | `core::mean`으로 채널 평균 → `gain[c] = mean_all / mean[c]` → `Mat::mul`/`convert_to`로 스케일 |
| `WarmPushback` | 고정 게인 `[B,G,R] = [1.10, 1.00, 0.85]` (상수로 파일 상단 선언) |
| `ClaheV` | `COLOR_BGR2Lab` → `split` → L에 `imgproc::create_clahe(2.0, Size::new(8,8))` → `merge` → `COLOR_Lab2BGR` |
| `Bilateral` | `imgproc::bilateral_filter(src, dst, 5, 50.0, 50.0, BORDER_DEFAULT)` |
| `Gauss` | `imgproc::gaussian_blur(src, dst, Size::new(3,3), 0.0, 0.0, BORDER_DEFAULT, ALGO_HINT_DEFAULT)` |
| `CbSim` | LMS deuteranope 시뮬 3×3 행렬을 RGB에 적용 — 행렬은 파일 상단 `const DEUTERANOPE_RGB: [[f64;3];3]`로 두고 `core::transform` 사용 |

`Display`/`FromStr`은 `ColorSpace`([`color_space.rs`](../../../src/detector/appearance/colormask/color_space.rs))와 같은 형태로 쓴다.
clap 인자로 쓰려면 `#[derive(ValueEnum)]`도 붙인다.

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot preprocess`
Expected: PASS (5 tests)

- [ ] **Step 5: Detector에 배선**

`src/detector/detector.rs`:

```rust
pub struct Detector {
    pub mask: SpatialMask,
    pub pre: Preprocess,
    pub roi: RoiTrack,
    pub scorer: ScorerParams,
}
```

`detect()`:

```rust
pub fn detect(&mut self, frame: &Frame) -> Option<camera::Pixel> {
    let Ok(masked) = self.mask.apply_bgr(&frame.image) else {
        return None;
    };
    let Ok(image) = self.pre.apply(&masked) else {
        return None;
    };
    let gated = Frame { camera_id: frame.camera_id, image, timestamp: frame.timestamp };
    return self.roi.detect(&gated);
}
```

`builder.rs`에 `pre: Option<Preprocess>` 필드 + `pub fn pre(mut self, pre: Preprocess) -> Self` + `build()`에서 `unwrap_or(Preprocess::None)`.

- [ ] **Step 6: 커밋**

```bash
cargo test --workspace
git add src/detector
git commit -m "feat(vision): preprocess layer for white balance and contrast"
```

---

## Task 6: 색공간 확장 (Lab · custom H+a*b*)

**Files:**
- Modify: `src/detector/appearance/colormask/color_space.rs`, `src/detector/appearance/colormask/detector.rs`

**Interfaces:**
- Produces: `ColorSpace::{Ycrcb, Hsv, Lab, CustomHab}`,
  `ColorSpace::convert(&self, bgr: &Mat) -> Result<Mat>` (3채널 u8),
  `ColorSpace::all() -> [ColorSpace; 4]`
- `ColormaskDetector::color_mask`는 `params.space.convert(&frame.image)`를 쓰도록 교체

- [ ] **Step 1: 실패 테스트 작성**

`color_space.rs` 하단:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use opencv::core::{CV_8UC3, Scalar, Size};
    use opencv::prelude::*;

    fn orange() -> Mat {
        // BGR 주황
        return Mat::new_size_with_default(Size::new(8, 8), CV_8UC3, Scalar::new(30.0, 120.0, 235.0, 0.0)).unwrap();
    }

    #[test]
    fn all_spaces_convert_to_three_channel_u8() {
        let src = orange();
        for cs in ColorSpace::all() {
            let out = cs.convert(&src).unwrap_or_else(|e| panic!("{cs}: {e}"));
            assert_eq!(out.channels(), 3, "{cs}");
            assert_eq!(out.depth(), opencv::core::CV_8U, "{cs}");
            assert_eq!(out.size().unwrap(), src.size().unwrap(), "{cs}");
        }
    }

    #[test]
    fn custom_hab_takes_hue_from_hsv_and_ab_from_lab() {
        let src = orange();
        let hsv = ColorSpace::Hsv.convert(&src).unwrap();
        let lab = ColorSpace::Lab.convert(&src).unwrap();
        let custom = ColorSpace::CustomHab.convert(&src).unwrap();
        let h: opencv::core::Vec3b = *hsv.at_2d(0, 0).unwrap();
        let l: opencv::core::Vec3b = *lab.at_2d(0, 0).unwrap();
        let c: opencv::core::Vec3b = *custom.at_2d(0, 0).unwrap();
        assert_eq!(c[0], h[0], "c0 = HSV H");
        assert_eq!(c[1], l[1], "c1 = Lab a*");
        assert_eq!(c[2], l[2], "c2 = Lab b*");
    }

    #[test]
    fn str_roundtrip_covers_new_spaces() {
        for cs in ColorSpace::all() {
            assert_eq!(cs.to_string().parse::<ColorSpace>().unwrap(), cs);
        }
    }
}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cargo test -p pingpong-bot color_space`
Expected: FAIL — `Lab`/`CustomHab`/`convert`/`all` 없음

- [ ] **Step 3: 구현**

- enum에 `Lab`, `CustomHab` 추가. `#[serde(rename_all="lowercase")]`이므로 JSON은 `"lab"`, `"customhab"`이 된다 —
  가독성을 위해 variant에 `#[serde(rename = "custom_h_ab")]`, `#[value(name = "custom_h_ab")]`을 붙인다.
- `FromStr`/`Display`에 `"lab"`, `"custom_h_ab"` 추가.
- `convert`:

```rust
pub fn convert(&self, bgr: &Mat) -> Result<Mat> {
    let cvt = |code: i32| -> Result<Mat> {
        let mut out = Mat::default();
        imgproc::cvt_color(bgr, &mut out, code, 0, opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT)?;
        return Ok(out);
    };
    return match self {
        Self::Ycrcb => cvt(imgproc::COLOR_BGR2YCrCb),
        Self::Hsv => cvt(imgproc::COLOR_BGR2HSV),
        Self::Lab => cvt(imgproc::COLOR_BGR2Lab),
        Self::CustomHab => {
            let hsv = cvt(imgproc::COLOR_BGR2HSV)?;
            let lab = cvt(imgproc::COLOR_BGR2Lab)?;
            let mut hsv_ch = Vector::<Mat>::new();
            let mut lab_ch = Vector::<Mat>::new();
            opencv::core::split(&hsv, &mut hsv_ch)?;
            opencv::core::split(&lab, &mut lab_ch)?;
            let merged = Vector::<Mat>::from_iter([hsv_ch.get(0)?, lab_ch.get(1)?, lab_ch.get(2)?]);
            let mut out = Mat::default();
            opencv::core::merge(&merged, &mut out)?;
            Ok(out)
        }
    };
}
```

- `ColormaskDetector::color_mask`의 `match` 블록을 `let converted = self.params.space.convert(&frame.image).ok()?;`로 교체.

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot color_space colormask`
Expected: PASS

- [ ] **Step 5: 커밋**

```bash
git add src/detector/appearance/colormask
git commit -m "feat(vision): Lab and custom H+a*b* color spaces"
```

---

## Task 7: 게이트 모델 (`ColorGate` — AABB / 타원체 + LUT)

**Files:**
- Create: `src/detector/appearance/colormask/gate.rs`, `src/detector/appearance/colormask/ellipsoid_gate.rs`
- Modify: `src/detector/appearance/colormask/{mod.rs, cam.rs, detector.rs}`

**Interfaces:**
- Produces:
  - `EllipsoidGate { space: ColorSpace, mean: [f64;3], inv_cov: [[f64;3];3], threshold: f64, diagonal: bool }` (serde)
  - `EllipsoidGate::fit(space, samples_bgr: &[[u8;3]], percentile: f64, diagonal: bool) -> Result<Self>`
  - `EllipsoidGate::distance_sq(&self, v: [u8;3]) -> f64`
  - `EllipsoidGate::lut(&self) -> Vec<u8>` — 256³ 통과표 (1 = 통과)
  - `ColorGate::{Aabb(ColormaskParams), Ellipsoid(EllipsoidGate)}`, `ColorGate::space()`, `ColorGate::mask(&self, bgr: &Mat) -> Result<Mat>`
- `ColormaskCam`에 `#[serde(default, skip_serializing_if = "Option::is_none")] pub gate: Option<EllipsoidGate>` 추가 (기존 JSON 하위호환)

- [ ] **Step 1: 실패 테스트 작성**

`ellipsoid_gate.rs` 하단:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    /// 주황 근처 등방 클러스터 + 초록끼 아웃라이어 2개.
    fn samples() -> Vec<[u8; 3]> {
        let mut v = Vec::new();
        for i in 0..40u8 {
            v.push([30 + i % 5, 118 + i % 7, 232 - i % 6]);
        }
        v.push([90, 200, 90]);
        v.push([95, 205, 95]);
        return v;
    }

    #[test]
    fn fit_accepts_cluster_and_rejects_outlier() {
        let g = EllipsoidGate::fit(ColorSpace::Hsv, &samples(), 90.0, false).unwrap();
        assert!(g.distance_sq([31, 120, 231]) <= g.threshold, "cluster must pass");
        assert!(g.distance_sq([90, 200, 90]) > g.threshold, "green outlier must fail");
    }

    #[test]
    fn diagonal_fit_ignores_cross_terms() {
        let g = EllipsoidGate::fit(ColorSpace::Hsv, &samples(), 90.0, true).unwrap();
        assert!(g.diagonal);
        for r in 0..3 {
            for c in 0..3 {
                if r != c {
                    assert_eq!(g.inv_cov[r][c], 0.0);
                }
            }
        }
    }

    #[test]
    fn lut_matches_distance_check() {
        let g = EllipsoidGate::fit(ColorSpace::Hsv, &samples(), 90.0, false).unwrap();
        let lut = g.lut();
        assert_eq!(lut.len(), 256 * 256 * 256);
        for v in [[31u8, 120, 231], [90, 200, 90], [0, 0, 0], [255, 255, 255]] {
            let idx = (v[0] as usize) << 16 | (v[1] as usize) << 8 | v[2] as usize;
            let expected = u8::from(g.distance_sq(v) <= g.threshold);
            assert_eq!(lut[idx], expected, "{v:?}");
        }
    }

    #[test]
    fn fit_needs_enough_samples() {
        assert!(EllipsoidGate::fit(ColorSpace::Hsv, &[[1, 2, 3]], 90.0, false).is_err());
    }
}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cargo test -p pingpong-bot ellipsoid`
Expected: FAIL — 타입 없음

- [ ] **Step 3: `EllipsoidGate` 구현**

```rust
//! 샘플 공분산 Mahalanobis 타원체 색 게이트.
//!
//! 픽셀당 거리 계산은 1280×800·40 fps에 못 쓰므로, 판정은 256³ u8 LUT로 미리 굽는다.

use anyhow::{Result, ensure};
use nalgebra::{Matrix3, Vector3};

use super::ColorSpace;

/// 피팅에 필요한 최소 샘플 수.
const MIN_SAMPLES: usize = 8;
/// 특이 공분산 방지용 대각 보정.
const COV_EPSILON: f64 = 1e-6;

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EllipsoidGate {
    pub space: ColorSpace,
    pub mean: [f64; 3],
    pub inv_cov: [[f64; 3]; 3],
    /// 통과 임계 (거리 **제곱**).
    pub threshold: f64,
    pub diagonal: bool,
}
```

`fit` 절차:
1. `samples`(BGR)를 1×N `CV_8UC3` Mat으로 만들고 `space.convert`로 변환 → N개 `[u8;3]`
2. 평균 `mean`, 공분산 `cov` (`diagonal`이면 비대각 0)
3. `cov += COV_EPSILON * I` 후 `Matrix3::try_inverse()` — 실패 시 `bail!`
4. 각 샘플의 `distance_sq` 정렬 → `percentile` 위치 값을 `threshold`

`distance_sq`: `d = v - mean`, `d^T · inv_cov · d`. **입력은 이미 변환된 색공간 값**(LUT를 굽는 쪽에서 변환).

`lut()`: 256³ 루프를 돌며 `distance_sq([c0,c1,c2]) <= threshold`를 `u8`로 채운 `Vec<u8>` (16.7 MB).
인덱스는 `c0<<16 | c1<<8 | c2`.

`gate.rs`:

```rust
//! 색 게이트 모델 — 축정렬 AABB 또는 Mahalanobis 타원체.

pub enum ColorGate {
    Aabb(ColormaskParams),
    Ellipsoid(EllipsoidGate),
}
```

`ColorGate::mask(&self, bgr: &Mat) -> Result<Mat>`:
- `Aabb` → 기존 `in_range` 경로
- `Ellipsoid` → `space.convert` 후 LUT 조회로 `CV_8UC1` 채우기 (LUT는 `ColorGate` 생성 시 1회 굽고 내부 캐시)

> LUT 캐시는 `ColorGate::Ellipsoid(gate)` 안에 `lut: Vec<u8>`를 같이 들고 있으면 serde가 지저분해진다.
> 직렬화 대상은 `EllipsoidGate`만 두고, 런타임 캐시는 `ColormaskDetector`가 생성자에서 굽는다.

- [ ] **Step 4: `ColormaskDetector` 분기 + `ColormaskCam` 필드**

`cam.rs`에 `pub gate: Option<EllipsoidGate>` 추가 (`#[serde(default, skip_serializing_if = "Option::is_none")]`).
기존 `data/colormask.json`은 필드가 없으므로 `None`으로 로드된다 — **회귀 없음**.

`ColormaskDetector::new(params)`는 그대로 두고, `ColormaskDetector::with_gate(ColorGate) -> Self`를 추가한다.
`color_mask`는 내부 게이트에 위임한다.

- [ ] **Step 5: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot ellipsoid gate colormask`
Expected: PASS. LUT 테스트가 느리면 `#[ignore]` 대신 `--release`로 돌린다:
`cargo test -p pingpong-bot --release ellipsoid`

- [ ] **Step 6: 하위호환 확인**

Run: `cargo run -p detect-full -- --cam left --clip fly_01 --max-frames 30`
Expected: 기존 `data/colormask.json`(gate 필드 없음)으로 정상 동작.

- [ ] **Step 7: 커밋**

```bash
git add src/detector/appearance/colormask
git commit -m "feat(vision): Mahalanobis ellipsoid color gate with LUT"
```

---

## Task 8: morph 후처리 레이어

**Files:**
- Create: `src/detector/appearance/morph.rs`
- Modify: `src/detector/appearance/mod.rs`, `src/detector/mod.rs`

**Interfaces:**
- Produces: `MorphOp { None, Open3, Open5, Close3, OpenClose }`,
  `impl AppearanceLayer for MorphOp` (prior 마스크에 morphology 적용 후 반환),
  `MorphOp::all() -> [MorphOp; 5]`, `Display`/`FromStr`/`ValueEnum`

- [ ] **Step 1: 실패 테스트 작성**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use opencv::core::{CV_8UC1, Point, Scalar, Size};

    /// 큰 원 + 1px 점 하나.
    fn speckled_mask() -> Mat {
        let mut m = Mat::new_size_with_default(Size::new(64, 64), CV_8UC1, Scalar::all(0.0)).unwrap();
        imgproc::circle(&mut m, Point::new(20, 20), 8, Scalar::all(255.0), -1, imgproc::LINE_8, 0).unwrap();
        *m.at_2d_mut::<u8>(60, 60).unwrap() = 255;
        return m;
    }

    #[test]
    fn open3_removes_single_pixel_speckle() {
        let src = speckled_mask();
        let out = MorphOp::Open3.apply_mask(&src).unwrap();
        assert_eq!(*out.at_2d::<u8>(60, 60).unwrap(), 0, "speckle must go");
        assert_eq!(*out.at_2d::<u8>(20, 20).unwrap(), 255, "blob must stay");
    }

    #[test]
    fn none_is_identity() {
        let src = speckled_mask();
        let out = MorphOp::None.apply_mask(&src).unwrap();
        assert_eq!(opencv::core::count_non_zero(&out).unwrap(), opencv::core::count_non_zero(&src).unwrap());
    }

    #[test]
    fn close3_fills_small_hole() {
        let mut src = speckled_mask();
        *src.at_2d_mut::<u8>(20, 20).unwrap() = 0; // 구멍
        let out = MorphOp::Close3.apply_mask(&src).unwrap();
        assert_eq!(*out.at_2d::<u8>(20, 20).unwrap(), 255, "hole must be filled");
    }
}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cargo test -p pingpong-bot morph`
Expected: FAIL

- [ ] **Step 3: 구현**

`apply_mask(&self, mask: &Mat) -> Result<Mat>`를 공개 API로 두고,
`AppearanceLayer::apply`는 `prior`가 있으면 그것에, 없으면 `None`을 반환한다 (morph는 앞선 마스크를 전제로 한다).

커널은 `imgproc::get_structuring_element(MORPH_ELLIPSE, Size::new(k,k), Point::new(-1,-1))`,
연산은 `imgproc::morphology_ex(src, dst, MORPH_OPEN|MORPH_CLOSE, &kernel, Point::new(-1,-1), 1, BORDER_CONSTANT, morphology_default_border_value()?)`.
`OpenClose`는 open 후 close.

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot morph`
Expected: PASS (3 tests)

- [ ] **Step 5: 커밋**

```bash
git add src/detector/appearance
git commit -m "feat(vision): morphology layer for mask post-processing"
```

---

## Task 9: eval 하네스 (`tools/eval_colormask`)

**Files:**
- Create: `tools/eval_colormask/Cargo.toml`, `src/main.rs`, `src/cli.rs`, `src/combo.rs`, `src/score.rs`, `README.md`

**Interfaces:**
- Consumes: Task 1~8 전부 — `SpatialMask`, `Preprocess`, `ColorSpace`, `ColorGate`/`EllipsoidGate`, `MorphOp`,
  `label_stills`의 manifest 스키마(**타입은 복사하지 않고** `StillsManifest`를 `pingpong_bot`으로 옮겨 공유한다 — Step 1)
- Produces:
  - `Combo { pre: Preprocess, space: ColorSpace, gate: GateKind, morph: MorphOp }`, `Display` = `pre.none+cs.hsv+gate.aabb+morph.none`
  - `Score { hit: usize, miss: usize, fp: usize, tn: usize }`, `Score::rank_key(&self) -> (i64, i64)` — hit 내림차순, fp 오름차순

- [ ] **Step 1: manifest 타입을 라이브러리로 이동**

`tools/label_stills/src/manifest.rs`를 `src/detector/stills/` (신규, `mod.rs` + `still_item.rs` + `stills_manifest.rs`)로 옮기고
`label_stills`는 `pingpong_bot::detector::{StillsManifest, StillItem}`을 쓰게 고친다.
**이유:** 툴 둘이 같은 스키마를 읽으므로 SSOT가 하나여야 한다 (호환 alias 금지 원칙과 동일).

Run: `cargo test --workspace` → PASS 확인 후 커밋:
```bash
git commit -am "refactor(vision): move stills manifest into library SSOT"
```

- [ ] **Step 2: 채점 실패 테스트 작성**

`tools/eval_colormask/src/score.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hit_when_within_radius() {
        let mut s = Score::default();
        s.record(Some([100.0, 100.0]), Some(camera::Pixel::new(110.0, 100.0)), 20.0);
        assert_eq!((s.hit, s.miss, s.fp, s.tn), (1, 0, 0, 0));
    }

    #[test]
    fn miss_when_outside_radius_or_absent() {
        let mut s = Score::default();
        s.record(Some([100.0, 100.0]), Some(camera::Pixel::new(200.0, 100.0)), 20.0);
        s.record(Some([100.0, 100.0]), None, 20.0);
        assert_eq!((s.hit, s.miss, s.fp, s.tn), (0, 2, 0, 0));
    }

    #[test]
    fn empty_gt_splits_fp_and_tn() {
        let mut s = Score::default();
        s.record(None, Some(camera::Pixel::new(10.0, 10.0)), 20.0);
        s.record(None, None, 20.0);
        assert_eq!((s.hit, s.miss, s.fp, s.tn), (0, 0, 1, 1));
    }

    #[test]
    fn rank_prefers_more_hits_then_fewer_fp() {
        let a = Score { hit: 8, miss: 0, fp: 3, tn: 2 };
        let b = Score { hit: 8, miss: 0, fp: 1, tn: 2 };
        let c = Score { hit: 9, miss: 0, fp: 9, tn: 2 };
        let mut v = vec![a, b, c];
        v.sort_by_key(|s| s.rank_key());
        assert_eq!(v[0].hit, 9);
        assert_eq!(v[1].fp, 1);
    }
}
```

> 정답 픽셀이 **범위 밖 검출**과 **미검출**을 모두 `miss`로 세는 것에 주의 — FP는 무공 스틸에서만 센다.

- [ ] **Step 3: 테스트 실패 확인**

Run: `cargo test -p eval-colormask`
Expected: FAIL

- [ ] **Step 4: `Score` / `Combo` 구현**

`rank_key`는 정렬용이므로 `(-(hit as i64), fp as i64)`를 반환한다 (오름차순 정렬 시 hit 많고 fp 적은 순).

- [ ] **Step 5: 파이프라인 실행부 구현**

`main.rs` 흐름:

1. `StillsManifest::load_or_default(&detect_stills_manifest_path())` — 비어 있으면 `bail!("stills GT 없음 — label-stills 먼저")`.
2. 스틸의 `camera_id`별로 `camera_params_for` + `colormask_for` + BGR `samples` 로드.
3. `SpatialMask` 준비 — 기본 corridor on. `--no-corridor`면 floor only (A/B용).
4. 조합 생성: `--sweep main`(기본) = 48조합, `--sweep extended` = 메인 상위 `--top`(기본 5)에 확장 축 추가, `--sweep all` = 전체 곱.
5. 각 조합·각 스틸에 대해: PNG 로드 → `mask.apply_bgr` → `pre.apply` → gate mask → morph → `ContourDetector` → `Scorer::pick_best` → `Score::record`.
   - `gate.aabb`는 해당 캠의 저장된 `ColormaskParams`를 `space`만 바꿔 재피팅 (샘플 퍼센타일 `--trim`, 기본 10)
   - `gate.aabb_pct`는 `--trim` 스윕 `{5, 15, 25}`
   - `gate.ellipsoid*`는 `EllipsoidGate::fit(space, samples, --gate-pct(기본 90), diagonal)`
6. 표 출력 (rank / combo / hit / miss / fp / tn / keep%), `--top N`만.
7. `-o DIR`이면 상위 조합의 스틸별 마스크 오버레이 PNG 저장.

출력 예:

```
stills=20 (ball=14 empty=6) corridor=on hit_radius=20px
rank  combo                                      hit  miss  fp  tn
 1    pre.gray_world+cs.lab+gate.ellipsoid+open3  13    1    1   5
 2    pre.none+cs.custom_h_ab+gate.ellipsoid+none 12    2    2   4
 …
 17   pre.none+cs.hsv+gate.aabb+morph.none         9    5    6   0   (현행)
```

**현행 조합(`pre.none+cs.hsv+gate.aabb+morph.none`)은 항상 표에 포함**시켜 베이스라인을 보이게 한다.

- [ ] **Step 6: 메인 스윕 실행**

Run: `cargo run --release -p eval-colormask -- --sweep main --top 15`
Expected: 48조합 표. 현행 대비 FP가 낮고 hit이 비열등한 조합이 상위에 온다.

- [ ] **Step 7: 확장 스윕 실행**

Run: `cargo run --release -p eval-colormask -- --sweep extended --top 10 -o /tmp/eval_overlay`
Expected: 상위 5개에 확장 축을 붙인 표 + 오버레이 PNG.

- [ ] **Step 8: 커밋**

```bash
git add tools/eval_colormask
git commit -m "feat(tools): method-grid eval harness over still GT"
```

---

## Task 10: 승자 본선 반영

**Files:**
- Modify: `src/defaults/vision.rs`, `src/detector/appearance/colormask/cam.rs`(필요 시), `tools/tune_colormask/src/main.rs`, `data/colormask.json`
- Modify: `TODO.md`, `tools/tune_colormask/README.md`

**Interfaces:**
- Consumes: Task 9의 랭킹 1위 `Combo`
- Produces: `assemble`이 승자 `(pre, space, gate, morph)`로 조립

- [ ] **Step 1: 승자 조합을 `assemble`에 고정**

`src/defaults/vision.rs`:

```rust
/// eval_colormask 메인+확장 스윕 승자 (2026-07-30). 근거: docs/superpowers/plans/2026-07-30-spatial-then-color.md
pub const WINNER_PRE: Preprocess = Preprocess::GrayWorld;   // ← eval 결과로 교체
pub const WINNER_MORPH: MorphOp = MorphOp::Open3;           // ← eval 결과로 교체
```

`assemble`에 `.pre(WINNER_PRE)`와 `.then(WINNER_MORPH)`(colormask **뒤**, contour **앞**)를 추가한다.

- [ ] **Step 2: 게이트·색공간 SSOT 갱신**

- 승자가 `gate.aabb*`면 → `tune-colormask --space <승자> --trim <승자>`로 다시 피팅해 `data/colormask.json` upsert.
- 승자가 `gate.ellipsoid*`면 → eval이 `--save-gate` 플래그로 `ColormaskCam.gate`를 upsert하게 하고(Task 9 구현에 포함),
  `ColormaskDetector`가 `gate`가 있으면 그것을 쓰도록 `colormask_for` 경로를 `ColorGate`로 반환하게 바꾼다.

- [ ] **Step 3: tune-colormask를 승자 모델에 맞춤**

- `--space`에 `lab` / `custom_h_ab` 노출 (Task 6에서 `ValueEnum` 붙였으면 자동)
- 승자가 타원체면 미리보기 마스크도 타원체로 계산하고, 산점도에 **타원 단면**(2축 투영)을 그린다.
- 저장 시 `gate` 필드까지 upsert.

- [ ] **Step 4: 정성 확인**

Run:
```bash
cargo run -p detect-full -- --cam left  --clip fly_01
cargo run -p detect-full -- --cam right --clip fly_01
```
Expected: 대 위 bounce 궤적이 육안으로 따라가지고, 패널 2의 `nonzero`가 이전(≈23k) 대비 크게 감소.

- [ ] **Step 5: 문서 갱신**

`TODO.md` §3에 결과 한 줄 + 이 플랜 링크. `tools/tune_colormask/README.md`에 새 색공간·게이트 설명.

- [ ] **Step 6: 커밋**

```bash
cargo test --workspace
git add -A
git commit -m "feat(vision): adopt eval-winning colormask pipeline"
```

---

## 성공 기준

- **stills(~10/캠):** corridor 후 무공 FP↓, 유공 hit 유지/상승. 그리드 상위 조합이 현행
  (`pre.none + cs.hsv + gate.aabb + morph.none`) 대비 **FP↓ + hit 비열등**.
- **clips:** `detect-full --clip fly_01`에서 대 위 bounce 궤적이 육안으로 이어짐.
- **회귀 없음:** `cargo test --workspace` 통과, 기존 `data/colormask.json`으로도 실행 가능.

## 비범위

- 비디오 전 프레임 GT, 타임라인 밀집 라벨러
- 신경망 검출기, 물리 칸막이, 스테레오 3D GT
- 카메라 **하드웨어** WB 재설정 (소프트웨어 `Preprocess`로만 실험)
- Scorer/ROI 캠별화, 런타임 핫 리로드

## 원본 컨텍스트

- Cursor 플랜: `~/.cursor/plans/spatial_then_color_a5553d0d.plan.md`
- Cursor 세션: `composerData:6561d12d-31d2-4e4f-9aa1-4486e6427fbe` ("Color mask detection issue", 2026-07-23~29)
- 선행 완료 플랜: `floor_edge_spatial_mask_f775453f` (바닥 컷 + 면적 밴드), `per-cam_colormask_data_b73f8d9a` (`data/` SSOT)
