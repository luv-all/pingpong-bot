# Design: macOS jog — Dynamixel live, rail soft-skip

**Date:** 2026-07-31  
**Status:** approved (user: approach A — non-Windows auto-skip rail)  
**Out of scope:** AXL on macOS, rail dry-run mixed with live Dynamixel, `--no-rail` CLI flag, Windows live AXL error softening

---

## Goal

macOS에서 `jog`(및 동일 `RealHardware::new` 경로)가 **AXL 레일 없이** Dynamixel 관절 실기를 쓸 수 있게 한다.

**Done when:**

- Mac에서 `cargo run -p jog -- --port /dev/tty…`가 하드웨어 초기화에 성공한다.
- Dynamixel Sync / Preview / Apply (`j`, `angles`, `ik`, `pose`, `swing`의 관절부)가 동작한다.
- 레일은 `rail_x = 0` 고정(기존 `enabled=false` / `rail=None`과 동일).
- 스킵 시 warn 로그가 남는다.
- Windows live AXL은 현행 유지 — 개방 실패는 hard error.

---

## Context

| 항목 | 현재 | 이 슬라이스 |
|------|------|-------------|
| `RailConfig::default().enabled` | `true` | 변경 없음 |
| Mac `AxlRail::open` | 항상 `InvalidConfig` | 호출 전에 soft-skip |
| Mac `RealHardware::new` | 레일 때문에 전체 실패 | Dynamixel만 열고 계속 |
| Windows live rail | `AxlRail::open` hard fail | 변경 없음 |

**원인:** `tools/jog`이 `RailConfig::default()`(`enabled=true`)를 넘기고, `RealHardware::from_bus` live 분기가 `AxlRail::open`을 호출한다. non-Windows에서 `open`은 항상 에러라 Dynamixel도 열리지 않는다.

**선택 (user A):** non-Windows에서는 레일만 끄고 Dynamixel live 진행.  
거절: jog만 `enabled=false`(다른 진입점 미해결), live Dynamixel + rail dry-run(실기 아님).

---

## Behavior

`RealHardware::from_bus` live 레일 분기:

1. `rail`이 `None`이거나 `enabled=false` → 기존과 같이 `rail=None`, `rail_x=0`.
2. `is_dry_run` → 기존과 같이 `AxlRail::dry_run` (OS 무관).
3. live + **Windows** → `AxlRail::open`; 실패 시 hard error.
4. live + **non-Windows** → `AxlRail::open`을 호출하지 않고 warn 후 `rail=None`.

Warn 메시지 요지: AXL은 Windows+real만 지원하므로 레일을 비활성화하고 Dynamixel만 사용한다.

`read_pose().rail_x`는 `0.0`. 궤적 executor의 레일 커맨드는 기존 `rail=None` 경로와 동일(관절만 재생).

---

## Call sites

변경은 `src/hardware/real.rs`의 `from_bus`에 집중한다. 호출부는 그대로:

- `tools/jog/src/main.rs` — `RealHardware::new(dxl, Some(rail_cfg), …)`
- `src/main.rs` — `run_real`의 `RealHardware::new`

jog UI의 `r` / `rd` 슬라이더는 남겨 둔다. Apply 시 레일 HW가 없으면 `rail_x=0`이므로 실기 레일은 움직이지 않는다(의도된 제한).

---

## Docs

`tools/jog/README.md` 실행 절에 한 줄:

- macOS 실기: Dynamixel만. 레일은 자동 스킵(`rail_x=0`). Windows에서만 AXL live.

---

## Testing

| 케이스 | 기대 |
|--------|------|
| non-Windows unit: enabled rail + `is_dry_run=false` 경로(또는 `from_bus` 동등) | `Ok`, `read_pose().rail_x == 0.0` |
| dry-run + enabled rail | 기존 dry-run 레일 테스트 유지 |
| Windows live open 실패 | hard error (회귀 없음; CI가 Windows가 아니면 cfg로 문서화) |

수동: Mac에서 `cargo run -p jog -- --port <usbserial>` → Sync 후 작은 `j` Preview/Apply.

---

## Non-goals

- macOS에서 AXL.dll / 보드 지원
- 레일 soft-skip을 CLI로 끄는 옵션
- Windows에서 DLL 없음일 때 자동 스킵 (보드 미연결을 조용히 숨기지 않음)
