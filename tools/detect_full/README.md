# detect-full

런타임과 같은 **Detector** (`defaults::detector_for`) + adaptive ROI 튜닝.

조립 (`vision.rs`): `.mask(…) .then(Colormask) .then(Contour) .scorer(…) .roi(…)`

## 파이프라인 패널

| 0 raw | 1 floor-mask | 2 colormask | 3 +contour | 4 roi |
|-------|--------------|-------------|------------|-------|

- **0**: 원본 — hit rate / mode HUD
- **1**: 테이블 옆변을 `MAX_REPROJ_RMSE_PX`만큼 바깥(`x=-δ` / `x=W+δ`)으로 민 투영으로 바닥 제거 — cut_x / margin / keep
- **2→3**: appearance (color→contour) — nonzero / area / circularity
- **4**: ROI 박스 · radius_scale / motion_scale / padding
- track 중이면 2·3도 ROI 크롭에서 계산

SSOT: `data/colormask.json` · `data/calibration.json`

## 사용

```bash
# 라이브 (캠 역할 필수)
cargo run -p detect-full -- --cam left
cargo run -p detect-full -- --cam right --no-roi

# 오프라인 클립
cargo run -p detect-full -- --cam left --clip fly_01
cargo run -p detect-full -- --cam right --clip drop_02
```

## 옵션

| 옵션 | 기본 | 설명 |
|------|------|------|
| `--cam left\|right` | **필수** | 어느 쪽 카메라인지 |
| `--clip NAME\|DIR` | — | `data/clips/<name>`의 해당 캠 영상 |
| `--images DIR` | — | 이미지 시퀀스 디렉터리 (png/jpg…) |
| `--no-roi` | off | 시작 시 ROI track off |
| `-o` / `--output DIR` | — | 프레임 덤프 출력 |
| `--max-frames N` | 300 | 처리 상한 |
| `--no-preview` | off | 창 없이 돌림 |
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
| `r` | ROI track on/off |
| `[` `]` | `radius_scale` ±0.25 |
| `,` `.` | `motion_scale` ±0.25 |
| `-` `=` | `padding` ±4 |
| `p` | `RoiParams::default()` paste 스니펫 |
| `q` / ESC | 종료 |
