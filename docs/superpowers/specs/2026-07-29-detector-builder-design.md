# Detector builder (SimScene-style) — Design

## Goal

본선 검출 조립을 `SimScene::builder()`처럼 읽히게 하고, appearance는 **선언한 레이어를 `.then` 호출 순서대로** 게이트 체인한다. mask·ROI는 “순서 단계”가 아니다.

## Non-goals

- mask / ROI 재배치 API
- `fuse(generators![…])` 병렬 FirstSurviving를 빌더 기본으로 승격 (내부 fuse는 유지)
- Arc 핸들 분리

## Pipeline model

| 부품 | 역할 | 순서 |
|------|------|------|
| **mask** | 프레임 전처리 (floor-edge 등) | 항상 최전방, 고정 |
| **appearance `.then`** | 이진 마스크 게이트 체인 | **호출 순서 = 실행 순서** |
| **scorer** | fuse 점수 (+ motion soft) | 정책, 슬라이드 아님 |
| **roi** | track/acquire 탐색 정책 | 정책 래퍼, 슬라이드 아님 |

실행: `mask.apply_bgr` → (ROI 정책이 정한 영역에서) appearance 체인 → fuse scorer.

## API

```rust
let color = ColormaskDetector::new(colormask_for(cam_id)?);
let edges = ContourDetector::new(scorer_params.clone());

let det = Detector::builder()
    .mask(FloorEdgeMask::from_params(cam_id, &cam)?)
    .then(color)
    .then(edges)
    .scorer(Scorer::from(&scorer_params).with_motion_weight(MOTION_WEIGHT))
    .roi(RoiParams::default())
    .build()?;
```

- 레이어는 **객체로 선언** 후 `.then`에 넣는다 (`color`/`contour` 매직 메서드 없음).
- `.then` 순서 변경 → 게이트 방향 변경 (예: contour 먼저면 풀/선행 게이트 위 Canny 후 color).

## Types

### `AppearanceLayer`

```rust
fn apply(&mut self, frame: &Frame, prior: Option<&Mat>) -> Option<Mat>;
```

- `ColormaskDetector`: `color_mask`; `prior` 있으면 AND.
- `ContourDetector`: `prior` 있으면 gated Canny → dilate → `prior ∩ thick` (기존 cascade와 동일); 없으면 풀프레임 edges.

### `AppearanceChain`

`.then`으로 쌓인 `Vec<Box<dyn AppearanceLayer>>`. `CandidateGenerator`: 최종 마스크에서 contour candidates.

### `Detector` (번들, 구 `SpatialGate`)

```rust
pub struct Detector {
    pub mask: FloorEdgeMask,
    pub roi: RoiTrack,
    pub scorer: ScorerParams, // HUD/스냅샷
}
```

`BallDetector`: mask 적용 후 `roi.detect`.

### `DetectorBuilder`

필수: mask, ≥1 then, scorer, roi. 누락 시 `build` 에러.

## defaults SSOT

`detector_for(cam_id)` → 캘리브/colormask/area 밴드 로드 후 위 체인만 호출.

## Call sites

- `detect_full`: `detector.mask` / `detector.roi` / `detector.scorer`.
- `Box<dyn BallDetector>` 소비처는 `Detector` 구현으로 유지.

## detect_full panels

패널은 계속 color→contour 누적 시각화. 본선 defaults가 그 순서일 때와 일치. 순서를 바꾼 실험 조립은 패널이 어긋날 수 있음 (후속: chain `stage_masks` 노출).
