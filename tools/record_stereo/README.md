# record-stereo

연구실 스테레오 리그에서 **프리롤 녹화** 후 Git LFS로 집에 가져갈 클립을 만든다.

프리뷰만 켜 두고 공을 던진 뒤, 데탑으로 와서 `Space` → 최근 `--preroll`(기본 10초) + `--postroll`(기본 2초)를
`data/clips/{scene}_{nn}/` 에 저장한다.

```bash
# 비행
cargo run -p record-stereo -- --scene fly

# 굴림 / 드롭
cargo run -p record-stereo -- --scene roll
cargo run -p record-stereo -- --scene drop

# USB 대역이 빡세면
cargo run -p record-stereo -- --scene fly --preset mid
```

카메라/스트림 인자는 `StereoPairCliArgs` (항상 left+right, Windows recommended=MSMF).

| 키 | 동작 |
|----|------|
| `Space` | 프리롤+포스트롤 flush → 다음 `{scene}_{nn}` |
| `q` / ESC | 종료 |

장면(`--scene`)은 CLI만 — 실행 중 변경 없음. 장면 바꿀 때 프로세스를 다시 켠다.

### 출력

```
data/clips/fly_01/
  left.avi
  right.avi
  meta.json
```

- 코덱: MJPG in `.avi` (OpenCV VideoWriter가 제일 덜 까다로움)
- `meta.json`: scene, preroll/postroll, meas_fps, writer_fps, frames, backend…
- 재생 예:

```bash
cargo run -p verify-stereo -- \
  --video data/clips/fly_01/left.avi \
  --video data/clips/fly_01/right.avi
```

### Git LFS

`.gitattributes`에 `data/clips/**/*.avi` 가 있다. 연구실에서 클립 commit/push → 집에서 `git lfs pull`.

### 동기

하드웨어 genlock 없음. 캡처 스레드에서 left→right 순차 grab + 공통 `Instant`. 비행은 최대 ~1프레임 어긋날 수 있음.
