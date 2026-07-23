# detect-appearance

appearance 레이어만 **좌우 비교** — `colormask` | `contour`.  
파라미터는 `defaults::colormask()` · `defaults::scorer()`.

fuse·ROI·motion은 [detect-full](../detect_full/README.md).

```bash
cargo run -p detect-appearance
cargo run -p detect-appearance -- --path clip.mp4 -o out/
cargo run -p detect-appearance -- --images ./frames
```

`q` / ESC 종료.
