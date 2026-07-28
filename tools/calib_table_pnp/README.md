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
3. RMSE OK → pending 사이드카 자동 저장 + 무지개 격자. FAIL여도 **초록(클릭) vs 마젠타(이상 재투영)** + 노란 잔차선 → `z`/`c`로 다시 찍기
4. `s` — 본파일 upsert 후 pending 삭제. `q`해도 pending은 남음 (재실행 후 `s`만으로도 promote 가능)

멀티캠: 카메라별로 `cam0.json` / `cam1.json` 만든 뒤 필요 시 합침. 또는 `-o calibration.json --merge`.

## 사용

```bash
# -o 생략 → cam{id}.json (left=cam0, right=cam1)
cargo run -p calib-table-pnp -- --cam left
cargo run -p calib-table-pnp -- --cam right

cargo run -p calib-table-pnp -- --cam left --backend dshow
cargo run -p calib-table-pnp -- --path capture.mp4 --cam left

# 공유 번들에 upsert
cargo run -p calib-table-pnp -- --cam left -o calibration.json
cargo run -p calib-table-pnp -- --cam right --merge calibration.json -o calibration.json
```

| 키 | 동작 |
|----|------|
| `Space` | 스냅 |
| `LMB` | 랜드마크 클릭 |
| `Shift`+이동 | 8× loupe |
| `z` / `c` | undo / clear |
| `s` | 본파일 promote (세션 accepted 또는 디스크 pending) |
| `n` | live |
| `q` | 종료 (pending 유지) |

오버레이: **초록○** = 클릭, **마젠타×** = PnP 이상 재투영, **노란선** = 잔차(px). FAIL여도 표시되므로 잔차 큰 점부터 `z`로 다시 찍으면 된다.

pending: 공유 `calibration.pending.json`에 `cameras[]` upsert. `s`는 현재 cam만 본파일로 promote하고 pending에서 해당 항목만 제거.
