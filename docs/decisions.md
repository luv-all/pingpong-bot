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
| A4 | `e = 0.85` | `DEFAULT_RESTITUTION` | 툴 준비 ✅ / 실측 후 갱신 |


---

## B. 스윙 실행 — 속도 유지 vs 폴백

**결정 (2026-07):** 타격 모드에서 끝속도 0 폴백·contact 폴백 금지.  
한계 초과 시 스케일만 하고 스케일된 \(v_r\) 유지.  
**갱신:** commit은 `[MIN_SWING, SWING_COMMIT_MAX]` 창 + (oracle) 네트 통과 후에만.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| B1 | `fit_end_velocity` → 스케일 유지 (0 금지) | `physics.rs` | ✅ |
| B2 | `build_feasible_trajectory` 끝속도 유지 | `physics.rs` | ✅ |
| B3 | sim contact 폴백 **제거** | `world.rs` `try_auto_swing` | ✅ |
| B4 | `MAX_JOINT_ACCEL = 120`, `max_joint_speed = 8` | `physics.rs` / `Arm::competition` | 시뮬 전용 상한 |
| B5 | `MIN_SWING=0.08`, `COMMIT_MAX=0.20`, `DURATION≈0.15` | `constants/control` | ✅ |
| B6 | 임팩트 clamp 시 hit-plane **y 보존** | `robot.rs` | ✅ |

---

## C. 재계획·이중 경로

**결정 (2026-07):** 비행 중 commit 창에 들어온 첫 계획만 실행, 스윙 중 재계획 없음.  
발사 직후(긴 lead) commit 금지 — 조기 스윙 완료 방지.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| C1 | commit-once + **commit 창 대기** | `world.rs` / `app` | ✅ |
| C2 | sim 기본 = 오라클 타격 / `--ekf-swing` = control | `world` / `bin` | ✅ (승격 조건 ↓) |
| C3 | `is_busy`면 타겟 discard | `app/lib.rs` | 유지 |
| C4 | oracle **및 EKF control**: `ball_y ≤ 0.55·LENGTH` 후 commit | `world` / `SimHardware` | ✅ |

### C2 — 기본 모드를 oracle → EKF로 올리는 조건

`--ekf-swing`이 실험 플래그가 아니라 **sim 기본**이 되려면 아래를 만족한다. 미달이면 오라클 유지.

1. **예측**: commit 창 + 미드코트 게이트에서 EKF impact vs Rapier/탄도 진실 RMSE **&lt; 8 cm** (단위 테스트 `tracked_ballistic_impact_near_truth_in_commit_window`로 회귀).
2. **타격**: headless `--ekf-swing`으로 기본 슈터 N발 중 리턴/접촉 성공률이 오라클의 **≥ 80%** (TODO §6 스모크와 연동, 수치 확정 전 수동 확인).
3. **재발사**: 주차→발사 텔레포트 후 EKF가 점프 리셋되어 속도 시드가 다시 된다.
4. **물리 정합**: sim 파이프라인 EKF drag는 Rapier와 같이 **0** (`BallEkf::new(0.0)`). 실측 \(k\)는 §0.3 이후 `with_defaults`/설정으로.

현재(2026-07-13): (1)(3)(4) 코드 반영. (2)는 수동/`--ekf-swing` 확인 후 승격.

---

## D. 라켓 면·기구학

**결정 (2026-07):** **4DOF** — yaw + 어깨 + 팔꿈치(2R 접힘) + 손목 open. Dynamixel 접힘과 동일.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| D1 | `q3` 손목 open | `Arm::competition` | ✅ |
| D2 | `racket_face_toward_opponent(yaw, open)` | `robot.rs` | ✅ |
| D3 | 링크 0.18/0.18 + stub×2 | `constants/arm` | ✅ |
| D4 | urdf-test 3축 mesh / 제어 4DOF primitive | `bin` | ✅ |

---

## E. 접수·예측

**결정 (2026-07):** hit plane·restitution·lead 구간을 스펙으로 고정.

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| E1 | `DEFAULT_HIT_PLANE_Y = 0.30` | `constants/table.rs` | ✅ 스펙 확정 |
| E2 | 평면 지난 공 → 예측 안 함 | `ballistics.rs` | ✅ (short-lead 스팸 제거) |
| E3 | `TABLE_BOUNCE_RESTITUTION = RESTITUTION` | `constants/ball.rs` | ✅ 단일화 (0.85) |
| E4 | `MIN_LEAD=0.05`, `MAX_LEAD=1.2` | `constants/estimator.rs` | ✅ 시뮬 기본 |
| E5 | Rapier 공·테이블 restitution = E3 | `sim/world.rs` | ✅ |

---

## F. 관측·토크 스텁 교체

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| F1 | `BallEkf` | `domain/ekf.rs` | ✅ |
| F2 | DLT / OpenCV triangulate | `infra::vision` | ✅ DLT 폴백; `opencv` feature 시 `triangulatePoints`(2뷰) |
| F3 | 검출 패스스루 (sim은 투영 픽셀) | `infra::vision` | ✅ Phase2 — 다음: OpenCV 원/공 검출 |
| F4 | 대각 관성 토크 검증 `Iα` | `physics::verify_torque_limits` | ✅ 스텁(풀 동역학은 후속) |
| F5 | **비전 SSOT = OpenCV(infra)** | `infra::vision` | ✅ Calibration·삼각측량·`FrameSource` 이전. domain `CameraSource`/`Detector` 제거 |

---

## G. 테이블 관통 방지 (OBB)

**결정 (2026-07):** 키네마틱 팔은 Rapier가 못 막음 → domain OBB 클램프.
전완·라켓 OBB(상완은 마운트 접촉 제외), 플레이 영역(`y≥0.08`) 최저점 ≥ `SURFACE_Z + CLEARANCE`.
`clamp_above_table`으로 EE 리프트 후 재IK (sim·real 공통).

| ID | 현재 | 위치 | 상태 |
|----|------|------|------|
| G1 | 전완·라켓 OBB | `collision.rs` | ✅ |
| G2 | 스윙 샘플·추종 시 클램프 | `RobotState` | ✅ |
| G3 | `plan_swing` 임팩트 자세 클램프 | `physics.rs` | ✅ |

---

## H. 모듈 역할명

**결정 (2026-07):** 비전은 `pingpong_infra::vision`. domain은 추정·제어.

| 모듈 | 역할 |
|------|------|
| `infra::vision` | Calibration · triangulate · FrameSource · (OpenCV) ChArUco |
| `infra` camera | SimCamera / SyntheticCamera |
| `estimator` | 상태·hit-plane 예측 (domain) |
| `planner` | plan_swing · impact · collision |
| `robot` | Arm · URDF |
| `hardware` / `telemetry` | infra 어댑터 |

파이프라인: vision(detect→triangulate) → Estimator → Planner.

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
- [x] C2 본선 = app control (oracle off) / sim 기본 oracle
- [x] B5/B6 짧은 스윙 창·y 보존 clamp
- [x] E5 Rapier↔탄도 restitution
- [x] D1 면 법선 = 손목 open 관절
- [x] D3/D4 URDF mesh / competition 제어 분리
- [x] E1 hit plane y = 0.30
- [x] G1–G3 테이블 OBB 클램프 (전완·라켓)

작성 기준: 대화 중 식별된 자체 판단·애매 코드 (2026-07-11). F5·thin types 갱신 2026-07-15.
