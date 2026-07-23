# Dynamixel 미러 + 다물체 토크 + 진입점 SSOT/DX

구현 체크리스트 (플랜 원본: Cursor plan `mirror_torque_dynamics`).

## Phase 3a — 진입점 SSOT

- [x] `src/defaults.rs` — arm · physics · tunables · detector · dynamixel · intercept
- [x] `Arm::competition` 제거 → `arm()`
- [x] 도메인 `Default`/`from_embedded` competition 프리셋 제거
- [x] Vision 조립은 defaults `fuse(generators![…])`; `fuse_vision`은 툴 어댑터
- [x] `main` = entry + local/CLI

## Phase 1 — 미러 + 축별 토크

- [x] `mirror_slaves` + SyncWrite 미러 (`2*zero - master`)
- [x] entry `max_joint_torques = [12,6,6,6]`
- [x] 플래너 관절별 `I*|α_i| ≤ τ_max[i]`
- [x] dry-run / 단위 테스트

## Phase 3b — TOML 런타임 SSOT 삭제

- [x] bin `RuntimeConfig` 삭제
- [x] `config/default.toml` · `example.toml` 삭제
- [x] `src/local.rs` + `config/local.example.toml`
- [x] `config/real-hardware.toml` = jog_rail 보드 오버레이만
- [x] `jog_axis` → `dynamixel()` + `--port`

## Phase 2 — Rapier 다물체

- [x] `ArmMultibody` spawn + `motor_max_force = τ_max`
- [x] `ArmMultibody` 기본 ON (EE 충돌·τ_max 모터). 가벼운 링크·EE만 CCD. stall 허들 4ms/12회.
- [x] 듀얼 vs 단일 yaw motor force 회귀

## 리트머스

1. 진입점 파일만 읽고 파이프라인 설명 가능
2. `Arm`/`DynamixelConfig`에 `competition` 프리셋 메서드 없음
3. 테스트는 defaults / fixtures로 조립
