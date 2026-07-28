# detect-appearance

appearance 레이어만 **좌우 비교** — `colormask` | `contour`.  
colormask는 `colormask_for(cam_id)` (`data/colormask.json` 필수). Scorer는 `ScorerParams::default()`.

fuse·ROI·motion은 [detect-full](../detect_full/README.md).

```bash
cargo run -p detect-appearance
cargo run -p detect-appearance -- --path clip.mp4 -o out/
cargo run -p detect-appearance -- --images ./frames
```

`q` / ESC 종료.
