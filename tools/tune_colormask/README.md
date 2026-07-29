# tune-colormask

탁구공 픽셀을 찍어 **YCrCb / HSV** `inRange` 범위를 만든다.  
양꼬리 퍼센타일(`--trim`, 기본 10% → p10..p90) 후 `--margin`을 더한다.  
`p`(또는 샘플 있는 채 종료) 시 `--cam`의 범위+BGR 샘플을 `data/colormask.json`에 upsert.  
`--cam left|right` **필수**.

## 화면

1. **original | mask** — 샘플 · 현재 space 마스크  
2. **색상 띠** — BGR swatch  
3. **산점도 3 + iso** — 채널 쌍 AABB

## 사용

```bash
# 라이브
cargo run -p tune-colormask -- --cam left
cargo run -p tune-colormask -- --cam right --space hsv

# 오프라인 클립
cargo run -p tune-colormask -- --cam left --clip fly_01
cargo run -p tune-colormask -- --cam right --clip fly_01 --trim 15
```

## 옵션

| 옵션 | 기본 | 설명 |
|------|------|------|
| `--cam left\|right` | **필수** | upsert 대상 카메라 |
| `--clip NAME\|DIR` | — | `data/clips/<name>`의 해당 캠 영상 |
| `--images DIR` | — | 이미지 시퀀스 |
| `--space ycrcb\|hsv` | `ycrcb` | 시작 색공간 (`s`로 토글) |
| `--margin N` | 3 | 퍼센타일 구간에 더할 여유 (0..=32) |
| `--trim PCT` | 10 | 양꼬리 절단 % (`0` = min/max) |
| `--max-frames N` | 0 | 0이면 제한 없음 |
| `--wait-ms MS` | (자동) | `waitKey` 대기 |
| `--backend` | `recommended` | 라이브 OpenCV 백엔드 |
| `--width` / `--height` | 1280 / 800 | 라이브 해상도 |
| `--fps` | 120 | 라이브 요청 FPS |
| `--fourcc` | `MJPG` | 라이브 FOURCC |
| `--threaded true\|false` | `true` | 라이브 grab 스레드 |
| `--preset full\|mid\|low` | — | 해상도 프리셋 |

## 키

| 키 | 동작 |
|----|------|
| LMB / Enter | 공 픽셀 샘플 추가 (좌측 original) |
| `←↑→↓` / `hjkl` | aim 1px |
| `Shift`+이동 | 8× loupe |
| `z` / Backspace | 마지막 샘플 취소 |
| `c` | 샘플 전체 삭제 |
| `Space` | freeze / live |
| `s` | ycrcb ↔ hsv |
| `p` | 저장 |
| `q` / ESC | 종료 (샘플 있으면 저장) |

SSOT: `data/colormask.json` → `detector_for(CameraId)`.
