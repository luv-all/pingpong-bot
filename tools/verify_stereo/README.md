# verify-stereo

캘리브(`data/calibration.json`)로 **스테레오 월드 격자·공 삼각측량**을 눈으로 검증한다.  
항상 left+right (`--cam` 없음).

창:

1. `verify:left` — 격자 + 검출(녹) / 재투영(마젠타) / EKF(시안)
2. `verify:right` — 동일
3. `verify-stereo sim` — `SimScene` 탁구대 + 주황 공(EKF) + 반투명 공(생 삼각측량) (기본 on)

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
| `e` | EKF 토글 (끄면 생값만, 켤 때 필터 리셋) |
| `Space` | 동결 |
| `+/-` `[]` `.,` | 격자 간격·층·Z |
| `q` / ESC | 종료 (sim 자식도 종료) |

오버레이: **초록○** 검출, **마젠타×** 생 재투영, **빨강× + REJECT** 게이트가 막은 생값,
**시안○** EKF 재투영, **노란선** 잔차.

## EKF 게이팅

검출기가 한 프레임 엉뚱한 곳을 잡으면 삼각측량 결과가 튄다. `estimator::Ekf`가
마할라노비스 게이트로 그런 측정을 **무시**하고 예측으로 그 프레임을 넘기므로
위치·속도가 끊기지 않는다. 거부가 `gate_reject_limit`회 연속되면 그때 트랙을
버리고 재시드한다 (`defaults::EstimatorParams` SSOT).

HUD 2행이 필터 상태다:

```text
ekf=(x,y,z) |v|=..m/s d2=<잔차>/<임계> streak=<연속거부>/<한도> rej=<누적> reset=<누적>
```

- `d2`가 임계를 넘은 프레임은 빨간 ×로 표시되고 `rej`가 오른다 — 필터가 무엇을 걸렀는지 보는 창구
- `reset`이 자주 오르면 게이트가 너무 좁거나(`gate_chi2`↑) 검출이 실제로 불안정한 것
- 동결(`Space`) 중에는 같은 프레임이 반복되므로 필터를 돌리지 않는다

sim 브리지: 부모→자식 stdin에 `{"raw":{"x":..,"y":..,"z":..},"ekf":{…}}` 한 줄
(각 필드 `null` 가능, `hide`는 둘 다 숨김). sim 창의 주황 공이 EKF, 반투명 공이 생값이라
튄 프레임에서 반투명 공만 날아가는 걸로 확인한다.
