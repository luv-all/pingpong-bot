# tune-colormask

탁구공 위 픽셀을 클릭해 **YCrCb / HSV** `inRange` 범위를 뽑는다.  
파일은 건드리지 않는다 — 콘솔에 `defaults::colormask()`에 붙여넣을 Rust 조각만 출력.

## 화면

위에서 아래:

1. **original | mask** — 클릭 샘플 · 현재 space 마스크
2. **색상 띠** — 샘플 swatch + min→max 보간 띠

## 사용

```bash
cargo run -p tune-colormask                 # --device 0
cargo run -p tune-colormask -- --space hsv
cargo run -p tune-colormask -- --device 0 --margin 5
cargo run -p tune-colormask -- --path clip.mp4
```

| 키 | 동작 |
|----|------|
| LMB | 공 픽셀 샘플 추가 (좌측 original만) |
| `z` / Backspace | 마지막 샘플 취소 |
| `c` | 샘플 전체 삭제 |
| `Space` | freeze / live |
| `s` | ycrcb ↔ hsv (미리보기) |
| `p` | 양쪽 space `ColormaskParams` Rust 출력 |
| `q` / ESC | 종료 |

`p` 출력을 `src/defaults/vision.rs`의 `colormask()`에 수동으로 넣는다.
