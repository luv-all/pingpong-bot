# 결정 사항 묶음 — 자체 판단·애매 코드 정리

스윙·타격 경로에 **공식 없이 임의로 넣은 값·폴백·이중 경로**를 모아 둔다.  
각 항목은 **결정 후** 코드에서 매직넘버/폴백을 제거하거나, 측정·스펙으로 고정한다.

관련 공식(유지): `required_racket_velocity` — \(v_{out}, n, e \rightarrow v_r\) (`planner/impact.rs`).

---

## A. 타격 목표 \(v_{out}\) (최우선)

**결정 (2026-07):** 랠리 중앙 리턴 — `rally_return_velocity`
임팩트 → 상대 코트 중앙 `(WIDTH/2, 3·LENGTH/4)` 바운드를
`RALLY_TIME_TO_BOUNCE` 중력 탄도로 역산하며 네트 여유 높이도 검증한다.
`cooperative_return_velocity`는 레거시/테스트용.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| A1 | ~~`v_out = -0.35 × v_in`~~ → 상대 코트 중앙 바운드 탄도 | `impact.rs` `rally_return_velocity` | ✅ |
| A2 | 바운드 시간 `0.55 s`, 네트 clearance 검증 | `rally_return_velocity` | ✅ (Rapier 폐루프 보정, 추후 실측) |
| A3 | `‖v_out‖ ≤ 6` | `MAX_RETURN_SPEED` | 잠정 (구 4→6) |
| A4 | collider `e = 0.85`, 유연 접촉 명령 역산 `e_eff = 0.42` | `DEFAULT_RESTITUTION` / `RACKET_EFFECTIVE_RESTITUTION` | Rapier 회귀 ✅ / 실측 후 갱신 |

---

## B. 스윙 실행 — 속도 유지 vs 폴백

**결정 (2026-07):** 타격 모드에서 끝속도 0 폴백·contact 폴백 금지.  
한계 초과 시 스케일만 하고 스케일된 \(v_r\) 유지.  
**갱신:** commit은 `[MIN_SWING, SWING_COMMIT_MAX]` 창 + (ground truth 경로) 네트 통과 후에만.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| B1 | `fit_end_velocity` → 스케일 유지 (0 금지) | `physics.rs` | ✅ |
| B2 | `build_feasible_trajectory` 끝속도 유지 | `physics.rs` | ✅ |
| B3 | sim contact 폴백 **제거** | `world.rs` `try_auto_swing` | ✅ |
| B4 | `MAX_JOINT_ACCEL = 400`, `max_joint_speed = 16` | `physics.rs` / `Arm::competition` | 시뮬 경연 상한 |
| B5 | `MIN_SWING=0.08`, `COMMIT_MAX=0.35`, 팔로스루 `0.06 s` | `constants/control` | ✅ |
| B6 | 임팩트 내부 knot + 사후 clamp 없는 두 구간 quintic | `trajectory.rs` / `state.rs` | ✅ |

---

## C. 재계획·이중 경로

**결정 (2026-07):** 비행 중 commit 창에 들어온 첫 계획만 실행, 스윙 중 재계획 없음.  
발사 직후(긴 lead) commit 금지 — 조기 스윙 완료 방지.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| C1 | commit-once + **commit 창 대기** | `world.rs` / `app` | ✅ |
| C2 | sim 기본 = ground truth / `sim.use_ground_truth=false` = control | `world` / `bin` | ✅ (승격 조건 ↓) |
| C3 | `is_busy`면 타겟 discard | `app/lib.rs` | 유지 |
| C4 | ground truth **및 EKF control**: `ball_y ≤ 0.55·LENGTH` 후 commit | `world` / `SimHardware` | ✅ |

### C2 — 기본 모드를 ground truth → EKF로 올리는 조건

`sim.use_ground_truth=false`가 **sim 기본**이 되려면 아래를 만족한다. 미달이면 ground truth 유지.

1. **예측**: commit 창 + 미드코트 게이트에서 EKF impact vs Rapier/탄도 진실 RMSE **&lt; 8 cm** (단위 테스트 `tracked_ballistic_impact_near_truth_in_commit_window`로 회귀).
2. **타격**: headless TOML에서 `sim.use_ground_truth=false`로 기본 슈터 N발 중 리턴/접촉 성공률이 ground truth 경로의 **≥ 80%** (TODO §6 스모크와 연동, 수치 확정 전 수동 확인).
3. **재발사**: 주차→발사 텔레포트 후 EKF가 점프 리셋되어 속도 시드가 다시 된다.
4. **물리 정합**: sim 파이프라인 EKF drag는 Rapier와 같이 **0** (`BallEkf::new(0.0)`). 실측 \(k\)는 §0.3 이후 `with_defaults`/설정으로.

현재(2026-07-13): (1)(3)(4) 코드 반영. (2)는 수동/`sim.use_ground_truth=false` 확인 후 승격.

---

## D. 라켓 면·기구학

**결정 (2026-07):** `competition`은 기존 4DOF primitive, URDF 프리셋은
URDF `origin`·축·한계·EE 변환을 보존한 일반 revolute 직렬 체인을 제어 SSOT로 쓴다.
FK는 행렬 체인, 타격은 레일+관절을 함께 푸는 위치 3축·면법선 2축 pose IK와
generalized Jacobian 속도 역산을 사용한다. roll은 구속하지 않는다.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| D1 | `q3` 손목 open | `Arm::competition` | ✅ |
| D2 | `racket_face_toward_opponent(yaw, open)` | `robot.rs` | ✅ |
| D3 | 링크 0.18/0.18 + stub×2 | `constants/arm` | ✅ |
| D4 | URDF 제어·FK·IK·뷰어 SSOT, 변환 실패 시 시작 오류 | `robot::serial` / `robot::urdf` | ✅ |
| D5 | 레일 포함 라켓 중심·면법선 pose IK와 finite-difference Jacobian 회귀 | `robot` | ✅ |

---

## E. 접수·예측

**결정 (2026-07):** 단일 hit plane 대신 TOML 인터셉트 구간의 모든 교차를
예측하고, 정확한 접촉점·시간·한계·충돌을 검사해 하나를 선택한다.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| E1 | `[intercept] y=0.20..0.55, step=0.05` | runtime TOML / `InterceptWindow` | ✅ |
| E2 | 평면 지난 공 → 예측 안 함 | `ballistics.rs` | ✅ (short-lead 스팸 제거) |
| E3 | `TABLE_BOUNCE_RESTITUTION = RESTITUTION` | `constants/ball.rs` | ✅ 단일화 (0.85) |
| E4 | `MIN_LEAD=0.05`, `MAX_LEAD=1.2` | `constants/estimator.rs` | ✅ 시뮬 기본 |
| E5 | Rapier 공·테이블 restitution = E3 | `sim/world.rs` | ✅ |
| E6 | 활성 라켓 접촉 → `vy>0` → 네트 통과 → 중앙 ±20 cm 바운스 | `sim/world.rs` 통합 테스트 | ✅ |

---

## F. 관측·토크 스텁 교체

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| F1 | `BallEkf` | `estimator/ekf.rs` | ✅ |
| F2 | DLT / OpenCV triangulate | `camera` | ✅ OpenCV 필수; 2뷰 `triangulatePoints`, 다중 뷰 DLT |
| F3 | 검출 패스스루 (sim은 투영 픽셀) | `detector` | ✅ Phase2 — 다음: OpenCV 원/공 검출 |
| F4 | 대각 관성 토크 검증 `Iα` | `physics::verify_torque_limits` | ✅ 스텁(풀 동역학은 후속) |
| F5 | OpenCV 경계 | `camera` / `detector` | ✅ 필수 의존성, 단일 크레이트 내부 모듈 격리 |

---

## G. 테이블 관통 방지 (OBB)

**결정 (2026-07):** 키네마틱 팔은 Rapier가 못 막음 → planner OBB 검사.
전완·라켓 OBB(상완은 마운트 접촉 제외), 플레이 영역(`y≥0.08`) 최저점 ≥ `SURFACE_Z + CLEARANCE`.
`clamp_above_table`으로 EE 리프트 후 재IK (sim·real 공통).

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| G1 | 전완·라켓 OBB | `collision.rs` | ✅ |
| G2 | 대기 추종 시 클램프, 스윙 재생은 계획값 유지 | `RobotState` | ✅ |
| G3 | `plan_swing` 전 구간 충돌 시 후보 거부 | `physics.rs` | ✅ |

---

## H. 모듈 역할명

**결정 (2026-07):** 배포 단위는 단일 `pingpong-bot` 크레이트이며 기능별
모듈로 구분한다. OpenCV는 필수이며, 나머지 외부 구현은 feature와 모듈
의존으로 경계를 유지한다.

| 모듈 | 역할 |
|------|------|
| `camera` | Calibration · FrameSource · triangulate · SimCamera · ChArUco |
| `detector` | 공 검출 |
| `estimator` | 상태·hit-plane 예측 |
| `planner` | plan_swing · impact · collision |
| `robot` | Arm · URDF · 프리셋 |
| `sim` | Rapier · 가상 장치 어댑터 |
| `hardware` / `telemetry` | 실물·로깅 어댑터 |

파이프라인: camera + detector → Estimator → Planner.

---

## 권장 결정 순서

1. **A** — \(v_{out}\) ✅  
2. **B** — 속도 폴백 ✅  
3. **C** — 스윙 권한 ✅  
4. **D / E** — 면·팔·hit plane ✅  
5. **F** — 관측(infra vision) ✅ · Rerun·Dynamixel은 실물 마일스톤

---

## 체크리스트 (회의용)

- [x] A1 \(v_{out}\) 정의 확정 — loft 탄도
- [x] A4 \(e\) 측정 계획 — `tools/measure_restitution` (`--heights` / `--vz-pairs` / `--sim`)
- [x] B1–B3 타격 모드에서 속도 0 폴백 금지
- [x] C1 임팩트 전 스윙 동결 — commit 창 + once
- [x] C2 본선 = app control (ground truth off) / sim 기본 ground truth
- [x] B5/B6 짧은 스윙 창·y 보존 clamp
- [x] E5 Rapier↔탄도 restitution
- [x] D1 면 법선 = 손목 open 관절
- [x] D3/D4 competition primitive / URDF 제어 SSOT 분리
- [x] E1 hit plane y = 0.30
- [x] G1–G3 테이블 OBB 클램프 (전완·라켓)

작성 기준: 대화 중 식별된 자체 판단·애매 코드 (2026-07-11). F5·thin types 갱신 2026-07-15.
