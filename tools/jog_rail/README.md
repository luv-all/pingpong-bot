# jog-rail

AXL 리니어 레일 수동 조그. 위치 단위는 미터. 실기는 Windows + `pingpong-bot` `real` feature(크레이트 의존성으로 이미 활성).

```bash
cargo run -p jog-rail -- --config config/real-hardware.toml --position-m 0.05 --dry-run
cargo run -p jog-rail -- --config config/real-hardware.toml --delta-m=-0.01 --dry-run
cargo run -p jog-rail -- --config config/real-hardware.toml --position-m 0.05
```

설정: [`config/real-hardware.toml`](../../config/real-hardware.toml) — live 모드는 `[hardware.rail] enabled = true` 필요.

성공 종료 시 Windows에서는 AXL/OpenCV DLL detach 지연을 피하려고 서보 OFF 후 `TerminateProcess`로 바로 끝낸다.
