# calib-charuco

ChArUco 보드 사진 → `Calibration` JSON (인트린식 + `dist`).
런타임은 이 JSON만 `calibration_path`로 로드한다.

```bash
# 실보정 (캠 1대분 이미지 폴더)
cargo run -p calib-charuco -- --from-images ./boards/cam0 -o config/cam0.json

# sim 더미 (N캠 JSON 한 파일)
cargo run -p calib-charuco -- --emit-sim 3 -o config/calibration.json

cargo run -p calib-charuco -- --validate config/cam0.json
```

멀티캠: 캠마다 `--from-images` 후 `cameras[]`를 한 JSON으로 합친다.
외부 pose(R|t) 자동은 후속.
