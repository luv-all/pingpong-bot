# detect-full

런타임과 같은 **fuse DSL** 본선 (`defaults::detector_for(cam_id)`) + adaptive ROI 튜닝.

파이프라인 스텝(읽는 순서):

| 0 raw | 1 floor-mask | 2 colormask |
| 3 +contour | 4 roi | |

- **0**: 원본 BGR
- **1**: 캘리브 테이블 옆변(`x=0` / `x=W`) 투영 사다리꼴로 바닥 제거 + 변 선
- **2→3**: 마스크된 프레임에서 색 통과 영역만 Canny (`ColorContourCascade`)
- **track 중**: 2·3도 ROI 크롭에서 계산 (본선과 동일 영역)
- Scorer `min/max_area`는 캘리브+`BALL_RADIUS`로 캠별 추정

키:

- **`r`**: ROI track on/off
- **`[` `]`**: `k` (±0.25)
- **`,` `.`**: `m` (±0.25)
- **`-` `=`**: `pad` (±4)
- **`p`**: `RoiParams::default()` paste 스니펫
- **`q` / ESC**: 종료

SSOT: `src/defaults/vision.rs` → `detector_for` / `colormask_for` / `camera_params_for` · `data/colormask.json` · `data/calibration.json`

appearance 단독 비교(병렬): [detect-appearance](../detect_appearance/README.md).

```bash
cargo run -p detect-full
cargo run -p detect-full -- --no-roi
cargo run -p detect-full -- --path clip.mp4
cargo run -p detect-full -- --images ./frames -o out/
```
