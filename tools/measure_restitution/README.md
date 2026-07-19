# measure-restitution

반발계수 $e$와 (옵션) 항력 $k$를 구해 `config`의 `[physics]`에 넣는다.

## 영상 (권장)

멀티캠 + 캘리브 → 검출 → 삼각측량 → 바운스에서 $e = |v_z'|/|v_z|$.

창 이름 `measure:restitution` — 카메라 패널을 **가로로 붙인 한 창**.

| 표시 | 의미 |
|------|------|
| 초록 원 | 검출 픽셀 |
| 마젠타 원 | 바운스 접촉 |
| 시안 / 주황 원 | 직전 / 직후 프레임 3D 투영 |
| 빨강 / 라임 화살표 | $v_\mathrm{in}$ / $v_\mathrm{out}$ |
| 텍스트 | $e$, 벡터, contact·prev→next $z$ |

```bash
cargo run -p measure-restitution -- \
  --calibration calib.json \
  --video cam0.mp4 --video cam1.mp4

cargo run -p measure-restitution -- \
  --calibration calib.json \
  --device 0 --device 1 --dry-run
```

카메라 나열 순 = `camera_id` 0,1,…  
`q`/`ESC` 조기 종료. `--no-preview` · `--fps` · `--detector` · `--wait-ms` · `--max-frames`

## 수동 / sim

```bash
cargo run -p measure-restitution -- --heights 0.40,0.29,0.21
cargo run -p measure-restitution -- --vz-pairs 2.0:1.7 --dry-run
cargo run -p measure-restitution -- --sim
cargo run -p measure-restitution -- --drag-csv traj.csv
```

기본 `--config`는 `config/default.toml`.
