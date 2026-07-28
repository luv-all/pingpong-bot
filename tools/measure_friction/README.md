# measure-friction

테이블 위 **롤**에서 $\mu$를 측정한다.  
파일은 건드리지 않는다 — stdout에 `PhysicsParams::default()` 붙여넣기용 Rust 스니펫만 출력.

## 영상 (권장)

```bash
# 캡처 모드: --calibration 필수 (기본 --cam left,right)
cargo run -p measure-friction -- --calibration calibration.json

cargo run -p measure-friction -- \
  --calibration calibration.json \
  --cam left,right

cargo run -p measure-friction -- \
  --calibration calibration.json \
  --video a.mp4 --video b.mp4
```

## 수동 / sim

```bash
cargo run -p measure-friction -- --vt-pairs 2.0:1.4
cargo run -p measure-friction -- --sim
```
