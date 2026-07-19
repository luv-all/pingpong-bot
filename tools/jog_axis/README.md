# jog-axis

Dynamixel 4축 수동 조그. URDF 관절각(도). 실기는 Windows + `--features real`.

```bash
cargo run -p jog-axis -- --config config/real-hardware.toml --dry-run --joint 0 --angle-deg 5
cargo run -p jog-axis --features real -- --config config/real-hardware.toml --joint 0 --angle-deg 5
```

설정: [`config/real-hardware.toml`](../../config/real-hardware.toml)
