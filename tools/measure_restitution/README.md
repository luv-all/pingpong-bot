# measure-restitution

반발계수 $e$와 (옵션) 항력 $k$를 측정한다.  
파일은 건드리지 않는다 — stdout에 `defaults::physics()` 붙여넣기용 Rust 스니펫만 출력.

## 영상 (권장)

멀티캠 + **Calibration JSON** → 검출 → 삼각측량 → 바운스에서 $e = |v_z'|/|v_z|$.

```bash
# 캡처 모드: --calibration 필수 (미지정 device면 0,1)
cargo run -p measure-restitution -- --calibration calibration.json

cargo run -p measure-restitution -- \
  --calibration calibration.json \
  --device 0 --device 1

cargo run -p measure-restitution -- \
  --calibration calibration.json \
  --video cam0.mp4 --video cam1.mp4
```

창 `measure:restitution`. 매 프레임 오버레이, `q` 종료.

## 수동 / sim / 항력

```bash
cargo run -p measure-restitution -- --heights 0.40,0.29,0.21
cargo run -p measure-restitution -- --vz-pairs 2.0:1.7,1.8:1.53
cargo run -p measure-restitution -- --sim
cargo run -p measure-restitution -- --sim-ballistics
cargo run -p measure-restitution -- --drag-csv traj.csv
```
