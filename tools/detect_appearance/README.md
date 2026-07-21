# detect-appearance

appearance 레이어만 **좌우 비교** — `colormask` | `contour`.

fuse·ROI·motion은 [detect-full](../detect_full/README.md).

```bash
cargo run -p detect-appearance
cargo run -p detect-appearance -- --config config/default.toml
cargo run -p detect-appearance -- --path clip.mp4 -o out/
```

파라미터: `[vision.appearance.colormask]` · `[vision.scorer]` (contour 단독 필터).
