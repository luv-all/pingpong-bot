# record-stereo

연구실 스테레오 리그 **프리롤 녹화**. Git LFS로 `data/clips/`에 올려 집에서 재생한다.

프리뷰만 켜 두고 공을 던진 뒤, 데탑에서 `Space` → 최근 `--preroll`(기본 10초) + `--postroll`(기본 2초) 저장.  
장면은 CLI `--scene`만 (실행 중 변경 없음). 항상 left+right.

## 사용

```bash
cargo run -p record-stereo -- --scene fly
cargo run -p record-stereo -- --scene roll
cargo run -p record-stereo -- --scene drop

# USB 대역이 빡세면
cargo run -p record-stereo -- --scene fly --preset mid
```

워크플로: 연출 → 돌아와 **10초 안에** `Space` → 다음 テイ크 반복 → `q`.

## 옵션

| 옵션 | 기본 | 설명 |
|------|------|------|
| `--scene fly\|roll\|drop` | `fly` | 클립 디렉터리 prefix (`fly_01` …) |
| `--out DIR` | `data/clips` | 출력 루트 |
| `--preroll SECS` | 10 | Space 기준 과거 보관 |
| `--postroll SECS` | 2 | Space 이후 추가 녹화 |
| `--backend` | `recommended` | OpenCV 백엔드 (Windows→MSMF) |
| `--width` / `--height` | 1280 / 800 | 스트림 해상도 |
| `--fps` | 120 | 요청 FPS (실제는 grab+JPEG 병목으로 더 낮을 수 있음 → `meta.meas_fps`) |
| `--fourcc` | `MJPG` | 캡처 FOURCC |
| `--threaded true\|false` | `true` | (이 툴은 자체 grab 스레드; 스트림 플래그는 공용 SSOT) |
| `--preset full\|mid\|low` | — | 해상도 프리셋 |

## 키

| 키 | 동작 |
|----|------|
| `Space` | 프리롤+포스트롤 flush → `{scene}_{nn}/` |
| `q` / ESC | 종료 |

## 출력

```
data/clips/fly_01/
  left.avi
  right.avi
  meta.json
```

- 코덱: MJPG `.avi`
- `meta.json`: scene, preroll/postroll, meas_fps, writer_fps, frames, backend…

재생:

```bash
cargo run -p verify-stereo -- --clip fly_01
cargo run -p measure-restitution -- --clip drop_02
cargo run -p detect-full -- --cam left --clip fly_01
```

## Git LFS

`.gitattributes`: `data/clips/**/*.avi`. 연구실 commit/push → 집 `git lfs pull`.

## 동기

하드웨어 genlock 없음. left→right 순차 grab. 비행은 최대 ~1프레임 어긋날 수 있음.
