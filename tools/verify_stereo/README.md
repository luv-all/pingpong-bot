# verify-stereo

캘리브 JSON으로 **스테레오 월드 격자·공 삼각측량**을 눈으로 검증한다.

창 3개:

1. `verify:left` — 왼쪽 캠 + 격자(점·선) + 검출(녹) / 재투영(마젠타)
2. `verify:right` — 오른쪽 동일
3. `verify-stereo sim` — `SimScene` 탁구대 + 공 (`--sim`, 기본 on; 부모→자식 stdin JSON)

캘리브 기본 경로: `calibration.json` (`defaults::calib::DEFAULT_CALIBRATION_PATH`).

```bash
cargo run -p verify-stereo
cargo run -p verify-stereo -- --calibration calibration.json
cargo run -p verify-stereo -- --sim false   # OpenCV만
cargo run -p verify-stereo -- --video left.mp4 --video right.mp4
```

항상 left+right (`--cam` 없음).

sim 브리지: 부모가 자식 stdin에 `{"x":..,"y":..,"z":..}` 또는 `hide` 한 줄씩 씀.

| 키 | 동작 |
|----|------|
| `g` | 격자 토글 |
| `d` | 검출 토글 |
| `Space` | 동결 |
| `+/-` `[]` `.,` | 격자 간격·층·Z |
| `q` / ESC | 종료 (sim 자식도 종료) |

오버레이: **초록○** 검출, **마젠타×** 삼각 재투영, **노란선** 잔차. HUD에 `xyz` [m] · reproj RMSE [px].
