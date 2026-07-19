# calib-charuco

ChArUco **인터랙티브** 보정. 라이브(또는 영상)에서 코너를 눈으로 확인한 뒤만 저장하고, 종료 시 `Calibration` JSON을 쓴다.

[hinguri-pingpong vision/pre](https://github.com/studio-void/hinguri-pingpong/tree/main/src/vision/pre)와 같은 확인 UX.

## 기본 (캡처 → 확인 → 보정)

```bash
cargo run -p calib-charuco -- --device 0
# -o 생략 시 --config(기본 config/default.toml)의 calibration_path
# 없으면 calibration.json
```

| 키 | 동작 |
|----|------|
| `Space` | 스냅 + 마커/ChArUco 코너 오버레이 (심사 모드) |
| `s` | 코너 충분하면 PNG 저장 |
| `n` | 잘못된 검출 생략 → 다시 라이브 |
| `q` | 종료. 저장 ≥ `--min-frames`(기본 10)이면 calibrate → `-o` |

초록 마커 + 마젠타 ChArUco 코너가 보드를 따라가면 저장, 아니면 `n`.

```bash
cargo run -p calib-charuco -- --device 0 \
  --images-dir ./boards/cam0 --min-frames 12 -o config/cam0.json

# 녹화본으로 같은 UX
cargo run -p calib-charuco -- --path capture.mp4 --camera-id 1 -o config/cam1.json
```

## 보조

```bash
# 이미 선별한 폴더만 보정
cargo run -p calib-charuco -- --from-images ./boards/cam0 -o config/cam0.json

cargo run -p calib-charuco -- --emit-sim 3 -o config/calibration.json
cargo run -p calib-charuco -- --validate config/cam0.json
```

멀티캠: 캠마다 인터랙티브(또는 `--from-images`) 후 `cameras[]`를 한 JSON으로 합친다. 외부 R\|t 자동은 후속.
