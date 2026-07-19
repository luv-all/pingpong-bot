# calib-charuco 인터랙티브 캡처 디자인

날짜: 2026-07-19  
상태: 승인·구현

## 목표

폴더 일괄 보정만으로는 불량 보드 촬영이 JSON에 섞인다. hinguri `vision/pre`처럼 **라이브에서 코너 오버레이를 확인한 뒤만** 저장하고, 종료 시 그 장들로 calibrate한다.

## UX

1. `VideoCapture` 창 `calib:charuco`
2. `Space` → freeze + `detect_and_draw_charuco` (markers + charuco corners)
3. `s` → 코너 ≥4 이면 PNG 저장 / `n` → 버림
4. `q` → 저장 ≥ `min_frames` 이면 `calibrate_charuco` → `-o` JSON

보조: `--from-images`, `--emit-sim`, `--validate`, `--path`(녹화 리뷰).

## API

- `detect_and_draw_charuco` / `CharucoFrameDetect` / `MIN_CHARUCO_CORNERS` in `camera/charuco.rs`
- `PreviewAction::Key` for Space/s/n
