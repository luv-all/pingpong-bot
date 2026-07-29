# measure-friction

테이블 **롤**에서 마찰 $\mu$를 측정한다.  
파일은 안 고친다 — stdout에 `PhysicsParams::default()` 붙여넣기 스니펫만.

캘리브는 항상 `data/calibration.json`. 항상 left+right.

## 사용

```bash
# 오프라인 클립 (권장)
cargo run -p measure-friction -- --clip roll_01

# 라이브
cargo run -p measure-friction

# 수동 / sim
cargo run -p measure-friction -- --vt-pairs 2.0:1.4
cargo run -p measure-friction -- --sim
```

창 `measure:friction`. `q` 종료.

`--clip`이면 `meta.json`의 `meas_fps`를 timeline으로 쓴다.

## 옵션

| 옵션 | 기본 | 설명 |
|------|------|------|
| `--clip NAME\|DIR` | — | `data/clips/<name>` left+right |
| `--timeline-fps F` | clip meta | 파일 재생 타임라인 FPS |
| `--no-preview` | off | 창 없이 |
| `--wait-ms MS` | 33 | 프리뷰 `waitKey` |
| `--max-frames N` | 10000 | 프레임 상한 |
| `--vt-pairs VIN:VOUT,…` | — | 접선 속도쌍으로 $\mu$ |
| `--sim` | off | Rapier 롤로 $\mu$ |
| `--horiz-speed MPS` | 2.0 | sim 수평 속도 |
| `--drop-height M` | 0.25 | sim 낙하 높이 [m] |
| `--backend` | `recommended` | 라이브 백엔드 |
| `--width` / `--height` | 1280 / 800 | 라이브 해상도 |
| `--fps` | 120 | 라이브 요청 FPS |
| `--fourcc` | `MJPG` | 라이브 FOURCC |
| `--threaded true\|false` | `true` | 라이브 grab 스레드 |
| `--preset full\|mid\|low` | — | 해상도 프리셋 |
