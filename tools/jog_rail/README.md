# jog-rail

AXL 리니어 레일 수동 조그. 위치 단위는 미터. 실기는 Windows + `--features real`.

```bash
cargo run -p jog-rail -- --config config/real-hardware.toml --position-m 0.05 --dry-run
cargo run -p jog-rail -- --config config/real-hardware.toml --delta-m -0.01 --dry-run
cargo run -p jog-rail --features real -- --config config/real-hardware.toml --position-m 0.05
```

설정: [`config/real-hardware.toml`](../../config/real-hardware.toml) — live 모드는 `[hardware.rail] enabled = true` 필요.
