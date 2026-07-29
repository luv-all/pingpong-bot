# measure-restitution

반발계수 $e$ (옵션 항력 $k$)를 측정한다.  
파일은 안 고친다 — stdout에 `PhysicsParams::default()` 붙여넣기 스니펫만.

영상 모드: 스테레오 검출 → 삼각 → 바운스에서 $e = |v_z'|/|v_z|$.  
캘리브는 항상 `data/calibration.json`. 항상 left+right.

## 사용

```bash
# 오프라인 클립 (권장)
cargo run -p measure-restitution -- --clip drop_02

# 라이브
cargo run -p measure-restitution

# 수동 / sim
cargo run -p measure-restitution -- --heights 0.40,0.29,0.21
cargo run -p measure-restitution -- --vz-pairs 2.0:1.7,1.8:1.53
cargo run -p measure-restitution -- --sim
cargo run -p measure-restitution -- --sim-ballistics
cargo run -p measure-restitution -- --drag-csv traj.csv
```

창 `measure:restitution`. `q` 종료.

`--clip`이면 `meta.json`의 `meas_fps`를 timeline으로 쓴다 (`--timeline-fps`로 덮어쓰기 가능).

## 옵션

| 옵션 | 기본 | 설명 |
|------|------|------|
| `--clip NAME\|DIR` | — | `data/clips/<name>` left+right |
| `--timeline-fps F` | clip meta | 파일 재생 타임라인 FPS |
| `--no-preview` | off | 창 없이 |
| `--wait-ms MS` | 33 | 프리뷰 `waitKey` |
| `--max-frames N` | 10000 | 프레임 상한 |
| `--heights H0,H1,…` | — | 높이로 $e$ (캡처 대신) |
| `--vz-pairs VIN:VOUT,…` | — | 법선 속도쌍으로 $e$ |
| `--sim` | off | Rapier 드롭으로 $e$ |
| `--sim-ballistics` | off | 탄도식 드롭으로 $e$ |
| `--drag-csv PATH` | — | 궤적 CSV로 항력 $k$ |
| `--drop-height M` | 0.40 | sim/ballistics 낙하 높이 [m] |
| `--backend` | `recommended` | 라이브 백엔드 |
| `--width` / `--height` | 1280 / 800 | 라이브 해상도 |
| `--fps` | 120 | 라이브 요청 FPS |
| `--fourcc` | `MJPG` | 라이브 FOURCC |
| `--threaded true\|false` | `true` | 라이브 grab 스레드 |
| `--preset full\|mid\|low` | — | 해상도 프리셋 |
