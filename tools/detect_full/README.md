# detect-full

런타임과 같은 **fuse DSL** 본선 (`fuse` / `generators!` / `track`) + ROI 토글.

- 조립 인라인: `tools/detect_full/src/main.rs` → `build_detector`
- 라이브러리 SSOT: `src/detector/dsl.rs` → `fuse_vision` / `track_vision`
- **`r`**: ROI track on/off

appearance만 좌우 비교: [detect-appearance](../detect_appearance/README.md).

```bash
cargo run -p detect-full
cargo run -p detect-full -- --no-roi
cargo run -p detect-full -- --config config/default.toml --path clip.mp4
```

파라미터: `[vision]` 전체 (`generators` · `appearance.*` · `scorer` · `motion` · `roi_half_px`).
