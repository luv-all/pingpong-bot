# measure-restitution

반발계수 $e$와 (옵션) 항력 $k$를 구해 `config`의 `[physics]`에 넣는다.

## 영상 (권장)

멀티캠 + **TOML `calibration_path`** → 검출 → 삼각측량 → 바운스에서 $e = |v_z'|/|v_z|$.

```bash
# 인자 없음 → device 0,1 + TOML calibration_path
cargo run -p measure-restitution

# config/default.toml 에 calibration_path = "calibration.json" 있으면
cargo run -p measure-restitution -- --device 0 --device 1

cargo run -p measure-restitution -- \
  --config config/experiment.toml \
  --video cam0.mp4 --video cam1.mp4

# 덮어쓰기
cargo run -p measure-restitution -- --calibration other.json --device 0 --device 1
```

창 `measure:restitution`. Space 아님 — 매 프레임 오버레이, `q` 종료.

## 수동 / sim

```bash
cargo run -p measure-restitution -- --heights 0.40,0.29,0.21
cargo run -p measure-restitution -- --sim --dry-run
```
