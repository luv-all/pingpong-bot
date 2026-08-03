# Plan: colormask AABB viz

## Files

- `tools/tune_colormask/src/main.rs` — `build_strip` → swatch + 3 scatters + iso cube
- `tools/tune_colormask/README.md` — 화면 설명 갱신

## Tasks

1. 순수 함수: `iso_project`, scatter 좌표 매핑 — 단위 테스트
2. `build_swatch` / `build_scatter` / `build_iso_cube` / `build_range_panel`
3. `main` 레이아웃 연결, 대각선 띠 제거
4. README + `cargo test -p tune-colormask`
