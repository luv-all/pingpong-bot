# detect-full

런타임과 같은 **fuse DSL** 본선 (`defaults::detector()`) + adaptive ROI 튜닝.

파이프라인 스텝(읽는 순서):

| 0 original | 1 colormask |
| 2 +contour | 3 roi      |

- **1→2**: 색 통과 영역에서만 Canny (`ColorContourCascade`)
- **track 중**: 1·2도 ROI 크롭에서 계산 (본선과 동일 영역)

키:

- **`r`**: ROI track on/off
- **`[` `]`**: `k` (±0.25)
- **`,` `.`**: `m` (±0.25)
- **`-` `=`**: `pad` (±4)
- **`p`**: `defaults::roi()` paste 스니펫
- **`q` / ESC**: 종료

SSOT: `src/defaults/vision.rs` → `detector()` / `roi()` / `colormask()` / `scorer()`

appearance 단독 비교(병렬): [detect-appearance](../detect_appearance/README.md).

```bash
cargo run -p detect-full
cargo run -p detect-full -- --no-roi
cargo run -p detect-full -- --path clip.mp4
cargo run -p detect-full -- --images ./frames -o out/
```
