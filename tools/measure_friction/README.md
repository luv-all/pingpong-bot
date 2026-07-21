# measure-friction

테이블 위 **롤**에서 $\mu$를 구하고 `[physics].friction`에 넣는다.

## 영상 (권장)

```bash
# 인자 없음 → device 0,1 + TOML calibration_path
cargo run -p measure-friction

cargo run -p measure-friction -- --device 0 --device 1

cargo run -p measure-friction -- --config config/experiment.toml --video a.mp4 --video b.mp4
```

`--calibration`으로 덮어쓰기 가능. 기본 `--config`는 `config/default.toml`.
