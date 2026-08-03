# RealHardware Dynamixel 4-DOF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Windows에서 검증된 Python 설정과 같은 Dynamixel 4축을 Rust `RealHardware`와 `jog-axis`에서 구동한다.

**Architecture:** 모터 좌표 변환은 순수 Rust `MotorMapping`, 통신은 `rustypot` 기반 `DynamixelBus`, 궤적 재생은 단일 executor가 담당한다. AXL은 `rail_x=0`인 명시적 스텁으로 남긴다.

**Tech Stack:** Rust 2024, rustypot, serialport, serde/TOML, clap, crossbeam-channel

## Global Constraints

- Python 기준값: COM8, 57,600 baud, Protocol 2.0, IDs `[1,3,4,5]`, signs `[-1,1,1,1]`.
- tick 변환·각도 리밋은 `test-manipulator/src/manipulator/config.py`와 같아야 한다.
- AXL은 구현하지 않고 `rail_x=0`; 이동 요청은 경고한다.
- 하드웨어가 없는 macOS에서도 단위 테스트와 `--features real` 빌드가 가능해야 한다.
- 실제 포트 쓰기는 `--dry-run` 없이만 허용한다.

---

### Task 1: Dynamixel 설정과 모터 좌표 변환

**Files:**
- Create: `src/hardware/dynamixel.rs`
- Modify: `src/hardware/mod.rs`
- Modify: `src/config.rs`
- Modify: `config/example.toml`

**Interfaces:**
- Produces: `DynamixelConfig`, `MotorMapping::radians_to_ticks`, `MotorMapping::ticks_to_radians`.

- [ ] Python 기본값으로 rad→tick, 역변환, 리밋 clamp 실패 테스트를 작성한다.
- [ ] `cargo test motor_mapping --features real`로 RED를 확인한다.
- [ ] 설정 타입과 변환 구현을 추가한다.
- [ ] 설정 길이·범위 검증 테스트를 추가하고 RED→GREEN을 확인한다.
- [ ] `cargo test motor_mapping --features real`을 통과시킨다.

### Task 2: rustypot 버스와 dry-run

**Files:**
- Modify: `Cargo.toml`
- Modify: `src/hardware/dynamixel.rs`
- Modify: `src/error.rs`

**Interfaces:**
- Produces: `DynamixelBus::open`, `enable_torque`, `write_joints`, `read_joints`.

- [ ] fake/dry-run 버스가 마지막 목표각을 읽기로 돌려주는 실패 테스트를 작성한다.
- [ ] `rustypot` API로 Protocol 2.0 SyncWrite/SyncRead와 profile register 쓰기를 구현한다.
- [ ] 통신 실패를 `HwError`로 변환하고 종료 시 토크를 best-effort로 끈다.
- [ ] dry-run 테스트와 `cargo check --features real`을 통과시킨다.

### Task 3: RealHardware와 궤적 executor

**Files:**
- Modify: `src/hardware/real.rs`
- Create: `src/hardware/rail_stub.rs`
- Modify: `src/hardware/mod.rs`

**Interfaces:**
- Produces: `RealHardware::new(config)`, `RealHardware::dry_run(config)`, `Hardware` 구현.

- [ ] dry-run에서 `command` 직후 busy, 종료 후 not-busy, 최종 포즈 도달 테스트를 작성한다.
- [ ] 전용 worker/channel로 `SwingTrajectory::sample_at`을 200Hz 재생한다.
- [ ] `read_pose`는 모터 각도와 `rail_x=0`을 반환하게 한다.
- [ ] 중복 command는 sim과 같이 무시하고, 비영 레일 이동은 한 번 경고한다.
- [ ] executor 테스트를 RED→GREEN으로 통과시킨다.

### Task 4: jog-axis 실기 스모크 도구

**Files:**
- Modify: `tools/jog_axis/Cargo.toml`
- Modify: `tools/jog_axis/src/main.rs`
- Create: `config/real-hardware.toml`

**Interfaces:**
- CLI: `jog-axis --config ... --joint 0 --angle-deg 5 [--dry-run]`
- CLI: `jog-axis --config ... --angles-deg 0,0,0,0 [--dry-run]`

- [ ] CLI 파싱·단축 목표 조립 테스트를 작성하고 RED를 확인한다.
- [ ] 같은 `DynamixelConfig`와 `RealHardware` 코드 경로로 단일 목표 궤적을 전송한다.
- [ ] 기본은 hardware, 명시적 `--dry-run`만 쓰기를 막는다.
- [ ] CLI 테스트와 dry-run 명령을 통과시킨다.

### Task 5: run_real 최소 연결과 문서

**Files:**
- Modify: `src/main.rs`
- Modify: `README.md`
- Modify: `TODO.md`
- Modify: `assets/robots/4-dof/README.md`

**Interfaces:**
- `mode="real"`은 설정으로 `RealHardware`를 열고 현재 포즈를 읽는 최소 스모크를 수행한다.

- [ ] real 설정 누락/잘못된 모터 수 validation 테스트를 작성한다.
- [ ] `run_real(runtime)`에 연결·pose read를 구현한다.
- [ ] 카메라 pipeline은 아직 연결하지 않았음을 오류가 아닌 명시적 로그로 남긴다.
- [ ] ID↔URDF 관절 순서와 AXL 스텁 상태를 문서화한다.
- [ ] 전체 테스트, real feature check, rustfmt, clippy를 실행한다.

## Verification

```bash
source .envrc
cargo fmt --all -- --check
cargo test --workspace --features real
cargo clippy --workspace --all-targets --features real -- -D warnings
cargo run -p jog-axis -- --config config/real-hardware.toml --angles-deg 0,0,0,0 --dry-run
```

Windows 벤치에서는 마지막 명령에서 `--dry-run`을 제거하기 전에 반드시 단축·작은 각도부터 검증한다.
