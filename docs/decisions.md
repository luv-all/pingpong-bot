# 결정 사항 묶음 — 자체 판단·애매 코드 정리

스윙·타격 경로에 **공식 없이 임의로 넣은 값·폴백·이중 경로**를 모아 둔다.  
각 항목은 **결정 후** 코드에서 매직넘버/폴백을 제거하거나, 측정·스펙으로 고정한다.

관련 공식(유지): `required_racket_velocity` — \(v_{out}, n, e \rightarrow v_r\) (`domain/impact.rs`).

---

## A. 타격 목표 \(v_{out}\) (최우선)

**결정 (2026-07):** 옵션 2+로프트 — `loft_return_velocity`  
임팩트 → 네트 `y=LENGTH_Y/2`, `z ≥ SURFACE_Z+NET_HEIGHT+NET_CLEARANCE` 를  
`LOFT_TIME_TO_NET` 탄도로 역산. `plan_swing` 본선 사용.  
`cooperative_return_velocity`는 레거시/테스트용.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| A1 | ~~`v_out = -0.35 × v_in`~~ → loft 탄도 | `impact.rs` `loft_return_velocity` | ✅ |
| A2 | loft 최소 `v_y≥1`, `v_z≥0.5` | `loft_return_velocity` | ✅ (시뮬 상수, 추후 측정) |
| A3 | `‖v_out‖ ≤ 6` | `MAX_RETURN_SPEED` | 잠정 (구 4→6) |
| A4 | `e = 0.85` | `DEFAULT_RESTITUTION` | 미측정 |

---

## B. 스윙 실행 — 속도 유지 vs 폴백

**결정 (2026-07):** 타격 모드에서 끝속도 0 폴백·contact 폴백 금지.  
한계 초과 시 스케일만 하고 스케일된 \(v_r\) 유지. sim은 첫 `plan_swing`만 commit.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| B1 | `fit_end_velocity` → 스케일 유지 (0 금지) | `physics.rs` | ✅ |
| B2 | `build_feasible_trajectory` 끝속도 유지 | `physics.rs` | ✅ |
| B3 | sim contact 폴백 **제거** | `world.rs` `try_auto_swing` | ✅ |
| B4 | `MAX_JOINT_ACCEL = 120`, `max_joint_speed = 8` | `physics.rs` / `Arm::competition` | 시뮬 전용 상한 |
| B5 | `MIN_SWING_SECS = 0.08` | `physics.rs` | 상수 유지 |

---

## C. 재계획·이중 경로

**결정 (2026-07):** 비행 중 첫 유효 계획만 commit, 스윙 중 재계획 없음.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| C1 | ~~20ms refine~~ → commit-once | `world.rs` | ✅ |
| C2 | sim `try_auto_swing` vs app control | `world.rs` / `app/lib.rs` | sim이 타격 권한 (busy 시 app skip) |
| C3 | `is_busy`면 타겟 discard | `app/lib.rs` | 유지 |

---

## D. 라켓 면·기구학

**문제:** 3DOF인데 면 방향을 팔과 분리해 임의 open pitch로 고정했다.

| ID | 현재 | 위치 | 결정할 것 |
|----|------|------|-----------|
| D1 | `RACKET_OPEN_PITCH = 0.45` | `robot.rs` | 면 법선 = f(관절)? 고정 open? \(v_{out}\)에서 역산한 \(n\)? |
| D2 | `racket_face_toward_opponent` | `robot.rs` | 손목 DOF 추가 전 임시인지, 경진 최종 모델인지 |
| D3 | `Arm::competition` 링크 0.16/0.14/0.08 | `robot.rs` | URDF 실측 체인으로 고정 vs primitive 근사 유지 |
| D4 | URDF mesh ≠ competition Arm 제어 | `bin` / `robot_builder` | 제어도 URDF FK? mesh는 뷰어만? (현재: 후자) |

---

## E. 접수·예측

| ID | 현재 | 위치 | 결정할 것 |
|----|------|------|-----------|
| E1 | `DEFAULT_HIT_PLANE_Y = 0.30` | `table.rs` | 팔 도달·슈터 탄도에 맞춘 **스펙 값**으로 확정 |
| E2 | 평면 지난 공 → `short_lead_prediction` | `estimator.rs` | 허용할지, 예측 실패로 둘지 |
| E3 | `BALL_RESTITUTION = 0.88` (sim 추정) vs `ball::RESTITUTION = 0.85` | `estimator.rs` / `ball.rs` | **단일 상수**로 통일 |
| E4 | `MIN_LEAD=0.05`, `MAX_LEAD=1.2` | `estimator.rs` | 근거 있는 구간으로 고정 |

---

## F. 아직 스텁 (2단계에서 교체 예정 — 결정만 명시)

| ID | 현재 | 위치 | 결정할 것 |
|----|------|------|-----------|
| F1 | `PassThroughEstimator` | `domain/estimator.rs` | EKF 착수 조건 |
| F2 | 삼각측량 스텁 | `triangulation.rs` | DLT 일정 |
| F3 | 검출 패스스루 | `infra/detector.rs` | OpenCV 어댑터 |
| F4 | §7.4 토크 검증 스텁 | `physics.rs` | 동역학 모델 도입 시점 |

---

## 권장 결정 순서

1. **A** — \(v_{out}\) (무엇을 칠지)  
2. **B** — 속도 불가 시 실패 vs 폴백 (어떻게 실행할지)  
3. **C** — 스윙 권한·재계획 동결 규칙  
4. **D / E** — 면·팔·hit plane·상수 통일  
5. **F** — 로드맵과 맞춤

---

## 체크리스트 (회의용)

- [x] A1 \(v_{out}\) 정의 확정 — loft 탄도
- [ ] A4 \(e\) 측정 계획
- [x] B1–B3 타격 모드에서 속도 0 폴백 금지
- [x] C1 임팩트 전 스윙 동결 — commit-once
- [x] C2 sim vs control 단일 권한 — sim 타격, app busy skip
- [ ] D1 면 법선 정책
- [ ] D3/D4 URDF↔제어 정합
- [ ] E1 hit plane y (현재 0.30, 스펙 확정 여부)
- [ ] E3 restitution 단일화

작성 기준: 대화 중 식별된 자체 판단·애매 코드 (2026-07-11).
