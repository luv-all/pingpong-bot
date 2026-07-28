# tune-colormask

탁구공 위 픽셀을 클릭해 **YCrCb / HSV** `inRange` 범위를 뽑는다.  
채널별 **양꼬리 퍼센타일**(`--trim`, 기본 10% → p10..p90)로 하이라이트·그림자·혼색 아웃라이어를 잘라낸 뒤 `--margin`을 더한다.  
`p` (및 샘플 있는 채 종료) 시 현재 `--cam`을 [`data/colormask.json`](../../data/colormask.json)에 upsert. Rust 스니펫도 콘솔에 출력.

## 화면

위에서 아래:

1. **original | mask** — 클릭 샘플 · 현재 space 마스크
2. **색상 띠** — 샘플 swatch (실제 BGR)
3. **산점도 3 + iso** — 채널 쌍(c0-c1 / c0-c2 / c1-c2)에 샘플 점·AABB 사각형, 오른쪽에 아이소메트릭 AABB 와이어

## 사용

```bash
cargo run -p tune-colormask                 # --cam left → data/colormask.json cam0
cargo run -p tune-colormask -- --cam right  # cam1 upsert
cargo run -p tune-colormask -- --cam left --space hsv
cargo run -p tune-colormask -- --cam left --margin 5
cargo run -p tune-colormask -- --cam left --trim 15   # 더 공격적으로 꼬리 절단
cargo run -p tune-colormask -- --cam left --trim 0    # 예전 min/max
cargo run -p tune-colormask -- --path clip.mp4
```

| 키 | 동작 |
|----|------|
| LMB / Enter | 공 픽셀 샘플 추가 (좌측 original만, aim 위치) |
| `←↑→↓` / `hjkl` | aim 1px (마우스 이동 시 재동기화; nudge 중 loupe 유지) |
| `Shift`+이동 | 8× 원형 loupe (좌측 original, 중심 픽셀 정밀 정렬) |
| `z` / Backspace | 마지막 샘플 취소 |
| `c` | 샘플 전체 삭제 |
| `Space` | freeze / live |
| `s` | ycrcb ↔ hsv (미리보기) |
| `p` | 저장(현재 space) + 양쪽 space Rust 출력 |
| `q` / ESC | 종료 (샘플 있으면 저장) |

SSOT: `defaults::DEFAULT_COLORMASK_PATH` → `detector_for(CameraId)`.
