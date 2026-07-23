# detect-full

런타임과 같은 **fuse DSL** 본선 (`defaults::detector()`) + ROI 토글.

- SSOT: `src/defaults/vision.rs` → `detector()` / `colormask()` / `scorer()`
- **`r`**: ROI track on/off · **`q` / ESC**: 종료

appearance만 좌우 비교: [detect-appearance](../detect_appearance/README.md).

```bash
cargo run -p detect-full
cargo run -p detect-full -- --no-roi
cargo run -p detect-full -- --path clip.mp4
cargo run -p detect-full -- --images ./frames -o out/
```
