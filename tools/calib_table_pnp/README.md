# calib-table-pnp

탁구대 **규격 랜드마크 8점**을 클릭해 OpenCV `solvePnP`(IPPE)로 카메라 외참 `R|t`를 잡고, 같은 창에서 **월드 XY×Z 무지개 격자**로 투영을 확인한 뒤 `Calibration` JSON을 쓴다. Charuco 없이 FOV로 `K`만 근사 (`dist=[]`).

라이브 스트림·FOV 기본은 **Arducam B0332** datasheet SSOT (`arducam_b0332`: 1280×800@120 MJPG, HFOV70°→VFOV≈47.3°).

| 파일 | 역할 |
|------|------|
| `interactive.rs` | Space 스냅 · LMB 클릭 · 자동 PnP · 격자 · s 저장 |
| `world_grid.rs` | XY×Z 무지개 격자 오버레이 (툴 전용) |
| `cli.rs` | `--from-pixels` / `--validate` / merge |
| `args.rs` | clap |

## 흐름

1. `Space` — 스냅 (Review)
2. LMB — 랜드마크 8점 순서대로 클릭 → **8번째에서 자동 PnP**
3. RMSE OK → 무지개 격자(Solved). 어긋나면 `z`/`c`로 다시 찍기
4. `s` — JSON 저장

멀티캠: `--cam left` / `--cam right` 각각 실행 후 merge.

## 사용

```bash
cargo run -p calib-table-pnp -- --cam left -o calibration.json
cargo run -p calib-table-pnp -- --cam right --merge calibration.json -o calibration.json
cargo run -p calib-table-pnp -- --cam left --backend dshow -o calibration.json
cargo run -p calib-table-pnp -- --path capture.mp4 --cam left -o cam0.json
```

| 키 | 동작 |
|----|------|
| `Space` | 스냅 |
| `LMB` | 랜드마크 클릭 |
| `Shift`+이동 | 8× loupe |
| `z` / `c` | undo / clear |
| `s` | 저장 |
| `n` | live |
| `q` | 종료 |
