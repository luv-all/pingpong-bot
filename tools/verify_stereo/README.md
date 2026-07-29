# verify-stereo

캘리브(`data/calibration.json`)로 **스테레오 월드 격자·공 삼각측량**을 눈으로 검증한다.  
항상 left+right (`--cam` 없음).

창:

1. `verify:left` — 격자 + 검출(녹) / 재투영(마젠타)
2. `verify:right` — 동일
3. `verify-stereo sim` — `SimScene` 탁구대+공 (기본 on)

## 사용

```bash
# 라이브
cargo run -p verify-stereo

# 오프라인 클립 (data/clips/fly_01/{left,right}.avi)
cargo run -p verify-stereo -- --clip fly_01

# sim 창 끄기
cargo run -p verify-stereo -- --clip fly_01 --sim false
```

## 옵션

| 옵션 | 기본 | 설명 |
|------|------|------|
| `--clip NAME\|DIR` | — | `data/clips/<name>` 또는 디렉터리. left/right 자동 |
| `--sim true\|false` | `true` | SimScene 자식 창 |
| `--backend` | `recommended` | OpenCV 백엔드. 라이브만 |
| `--width` / `--height` | 1280 / 800 | 라이브 스트림 해상도 |
| `--fps` | 120 | 라이브 요청 FPS |
| `--fourcc` | `MJPG` | 라이브 FOURCC |
| `--threaded true\|false` | `true` | 라이브 grab 스레드 |
| `--preset full\|mid\|low` | — | 해상도 프리셋 (주면 width/height보다 우선) |

## 키

| 키 | 동작 |
|----|------|
| `g` | 격자 토글 |
| `d` | 검출 토글 |
| `Space` | 동결 |
| `+/-` `[]` `.,` | 격자 간격·층·Z |
| `q` / ESC | 종료 (sim 자식도 종료) |

오버레이: **초록○** 검출, **마젠타×** 재투영, **노란선** 잔차. HUD: `xyz` [m] · reproj RMSE [px].

sim 브리지: 부모→자식 stdin에 `{"x":..,"y":..,"z":..}` 또는 `hide` 한 줄.
