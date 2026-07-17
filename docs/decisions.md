# 결정 사항 (decisions)

코드에 “왜 이렇게 했지?”가 남을 만한 값·경로·폴백을 모아 둔다.  
숫자는 가능하면 `src/constants/` 또는 TOML이 SSOT이고, 여기는 **의도**를 적는다.

관련 공식(유지): `required_racket_velocity` — \(v_{out}, n, e \rightarrow v_r\) (`planner/impact.rs`).

마지막 정리: **2026-07-17** (단일 크레이트·동적 인터셉트·competition=4-dof 체인·OpenCV 필수).

---

## 한 줄로 보기

| | 지금 하는 일 |
|--|-------------|
| **A** | 어디로 공을 보낼지 — 상대 코트 중앙 바운드 |
| **B** | 스윙 속도 0으로 포기하지 않음 · commit 창 |
| **C** | 한 공당 스윙 한 번 · sim 기본은 ground truth |
| **D** | 팔 모델 — competition은 4-dof 체인(메시 없음) |
| **E** | 어디서 칠지 — `[intercept]` y 구간을 여러 장 샘플 |
| **F** | 관측 — OpenCV 필수, 검출은 아직 패스스루 |
| **G** | 테이블 뚫지 않기 — OBB |
| **H** | 구조 — `domain/infra/...` 없음, `src/` 모듈 |
| **I** | 안 치는 경우(암묵 포기) — 문서화·검증 과제 |

---

## A. 어디로 보낼까 (\(v_{out}\))

**결정:** 맞은 뒤 공이 상대 코트 중앙 `(WIDTH/2, 3·LENGTH/4)` 근처에
바운드하도록 `rally_return_velocity`로 목표 속도를 잡는다.
바운드까지 시간 `RALLY_TIME_TO_BOUNCE = 0.55 s`, 네트 위 여유도 본다.

| ID | 내용 | 어디 | 상태 |
|----|------|------|------|
| A1 | 예전 `-0.35×v_in` 폐기 → 중앙 바운드 탄도 | `impact.rs` | ✅ |
| A2 | 바운드 시간 0.55 s + 네트 clearance | 상수 `impact` | ✅ (실측 보정 여지) |
| A3 | \(\|v_{out}\| ≤ 6\) m/s | `MAX_RETURN_SPEED` | 잠정 |
| A4 | Rapier \(e=0.85\), 명령 역산용 \(e_{eff}=0.42\) | ball / impact | ✅ / 실측 후 갱신 |

---

## B. 스윙을 어떻게 칠까

**결정:** 타격 모드에서 “끝속도 0”이나 “그냥 접촉만” 폴백은 없다.  
관절 한계에 걸리면 **스케일만** 하고 방향은 유지한다.

스윙을 **시작**하는 시점은 commit 창 안이어야 한다:

- 너무 이름(`> COMMIT_MAX`) → 기다림 (발사 직후 긴 궤적으로 조기 끝남 방지)
- 너무 늦음(`< MIN_SWING`) → 불가

| ID | 내용 | 상태 |
|----|------|------|
| B1–B2 | 끝속도 유지, 0 금지 | ✅ |
| B3 | sim contact 폴백 제거 | ✅ |
| B4 | `MAX_JOINT_ACCEL=400`, `max_joint_speed=16` (시뮬 상한) | 시뮬용 |
| B5 | `MIN_SWING=0.08`, `COMMIT_MAX=0.35`, 팔로스루 `0.06 s` | ✅ |
| B6 | 임팩트 knot + 팔로스루, 두 구간 quintic | ✅ |

---

## C. 언제 / 누가 스윙을 결정하나

**결정:**

1. **한 공에 스윙 한 번** (`swing_committed`). 스윙 중 재계획 없음.
2. **sim 기본**은 Rapier 진실로 치는 ground truth.  
   EKF로 치려면 TOML `sim.use_ground_truth = false`.
3. **미드코트 게이트:** `ball_y ≤ 0.55 · LENGTH_Y` 이후에만 commit  
   (상대 코트에서는 탄도가 아직 흔들림). ground truth·EKF control 공통.

| ID | 내용 | 코드 | 상태 |
|----|------|------|------|
| C1 | commit-once + 창 대기 | `sim/world`, `hardware/sim` | ✅ |
| C2 | 기본 ground truth / 실험은 EKF | TOML `[sim]` | ✅ (EKF 기본 승격은 아래) |
| C3 | 바쁠 때 타겟 버림 | `pipeline` | 유지 |
| C4 | 미드코트 게이트 `0.55·LENGTH` | `ball_past_midcourt_for_commit` | ✅ |

### C2 — EKF를 기본으로 올리려면

아직 기본은 ground truth. 아래로 바꾸려면:

1. commit 창 + 미드코트에서 EKF impact RMSE **&lt; 8 cm** (테스트 있음)
2. headless에서 EKF 타격 성공률 ≥ ground truth의 **80%** (TODO §6, 미확정)
3. 주차→발사 후 EKF 점프 리셋
4. sim EKF drag = 0 (Rapier와 동일)

(1)(3)(4)는 코드에 있음. (2) 스모크는 남음.

---

## D. 팔·라켓 모델

**결정:**

- **`competition` (기본):** 메시 없는 stick. 관절 origin·축·한계·EE는  
  `all-4-export.urdf`(4-dof)와 **같은 직렬 체인**.  
  (`LINK_UPPER/FOREARM = 0.18`은 **옛 planar 빌더**용 — competition FK에 안 씀.)
- **`4-dof` / `competition-urdf` / `urdf-test`:** URDF가 제어·FK·IK·뷰어 SSOT.  
  변환 실패 시 시작 오류 (competition으로 조용히 대체하지 않음).
- 타격 IK: 레일 + 관절로 **위치 3 + 면법선 2**. roll은 안 맞춤.
- 면 법선: 손목 open (`q3`). 제어/Rapier 라켓은 **local +Z = 면 법선**.
- **시각화:** primitive 라켓은 **원판**(지름 ~15 cm).  
  **충돌·Rapier**는 여전히 블레이드 OBB 박스 (`RACKET_HALF_*`).

| ID | 내용 | 상태 |
|----|------|------|
| D1 | 손목 open → 면 | ✅ |
| D2 | `racket_face_toward_opponent` | ✅ |
| D3 | competition = 4-dof 체인 길이 (legacy 0.18 링크는 별개) | ✅ (문구 정정 2026-07-17) |
| D4 | URDF SSOT, 실패 시 에러 | ✅ |
| D5 | pose IK + Jacobian 회귀 | ✅ |
| D6 | 블레이드 치수 ~15×16×1 cm (손잡이 제외). 뷰어 원판 | ✅ |

카탈로그: `src/robot/catalog.rs` (`ROBOTS`).

---

## E. 어디서 칠까 (동적 인터셉트)

**예전:** hit plane 하나 (`y = 0.30`).  
**지금:** TOML `[intercept]`의 `y_min..=y_max`를 `sample_step`마다 잘라  
여러 평면에 대해 탄도 교차를 예측한 뒤, **하나**를 고른다.

기본값 (`config/default.toml`): `y = 0.20..0.55`, `step = 0.05`.

### 고르는 순서 (`plan_best_swing`)

1. 각 y에 대해 예측 (평면 **이미 지난** 공은 예측 없음 → E2)
2. commit 창 `[MIN_SWING, COMMIT_MAX]` 안인 것만
3. **지금 라켓 위치에서 가까운** impact부터 시도
4. `plan_swing` 성공 + 접촉점 오차 ≤ 5 mm + (계획 시) 테이블 OBB 통과
5. 전부 실패 → 이번 틱은 스킵 (`InfeasibleSwing` 로그).  
   `swing_committed`는 안 올려서, 조건이 되면 다시 시도할 수 있음.

| ID | 내용 | 상태 |
|----|------|------|
| E1 | 다중 y 샘플 `InterceptWindow` | ✅ |
| E2 | 평면 지난 공 → 예측 안 함 | ✅ |
| E3 | 테이블 바운스 \(e\) = 공 \(e\) (0.85) | ✅ |
| E4 | lead `0.05..1.2` s | ✅ |
| E5 | Rapier restitution = E3 | ✅ |
| E6 | 접촉→리턴→네트→중앙 바운스 통합 테스트 | ✅ |
| E7 | 선택 기준 = 거리 정렬 + 접촉 5 mm (점수식 없음) | ✅ |

코드: `planner/mod.rs` (`InterceptWindow`), `planner/physics.rs` (`plan_best_swing`).

---

## F. 관측·토크

| ID | 내용 | 상태 |
|----|------|------|
| F1 | `BallEkf` | ✅ |
| F2 | OpenCV 2뷰 `triangulatePoints` + 3뷰↑ nalgebra DLT | ✅ |
| F3 | 검출 = 패스스루 (sim은 투영 픽셀). 실검출은 TODO | 스텁 |
| F4 | 토크 = 대각 \(I\alpha\) 스텁 | 스텁 |
| F5 | OpenCV **필수**. 시스템 **4.x** (`opencv@4`). crate `0.98.2+`. 5.x 금지 | ✅ |
| F6 | ChArUco는 초안(휴리스틱 K). 완전 `calibrateCameraCharuco`는 TODO | 초안 |

---

## G. 테이블에 팔 안 박기

키네마틱 팔은 Rapier가 안 막아 줌 → planner가 OBB로 검사.  
전완·라켓만 (상완은 마운트와 겹칠 수 있어 제외).  
플레이 `y ≥ 0.08`에서 최저점 ≥ `SURFACE_Z + CLEARANCE`.  
대기 추종 시 `clamp_above_table`, 스윙 재생 중에는 계획값 유지.

| ID | 내용 | 상태 |
|----|------|------|
| G1–G3 | OBB · 클램프 · plan 시 거부 | ✅ |

---

## H. 코드 구조

**결정:** `domain` / `infra` / `app` / `bin` 워크스페이스는 접고  
단일 `pingpong-bot` + `src/` 기능 모듈.  
트레잇·타입은 `ports.rs`/`types.rs` 대신 각 모듈에 둔다.

| 모듈 | 하는 일 |
|------|---------|
| `camera` | 캡처·캘리브·삼각측량·sim 카메라 |
| `detector` | 공 검출 |
| `estimator` | EKF·탄도 |
| `planner` | 인터셉트·스윙·충돌·임팩트 |
| `robot` | Arm·URDF·카탈로그 |
| `sim` | Rapier·뷰어·슈터 |
| `hardware` / `telemetry` / `pipeline` | 실물·로그·루프 |
| `config` (bin) | TOML만 SSOT — CLI는 경로 하나 |

런타임 설정: `config/default.toml`. CLI 플래그로 robot/mode override 없음.

네트 높이: ITTF **0.1525 m** (`constants/table::NET_HEIGHT`).

---

## I. 안 치는 경우 (암묵 “포기”)

이름 붙인 “포기 모드” API는 없다. 아래면 **스윙이 안 나가거나** 공이 회수된다.

| 상황 | 무슨 일 | decisions / 코드 |
|------|---------|------------------|
| 인터셉트 y를 이미 지남 | 예측 없음 | E2 |
| 아직 상대 코트 | commit 대기 | C4 |
| lead가 창 밖 | 대기 또는 불가 | B5 / C1 |
| IK·한계·테이블 충돌 | 그 후보 스킵, 다음 y | E7, G3 |
| 후보 전부 실패 | 이번 틱 스킵 (재시도 가능) | `try_auto_swing` |
| 테이블 밖 / `z < 0.35` | 슈터로 park | `park_if_out_of_play` |
| **테이블 위를 굴러감** | 공중 교차가 없으면 후보 0 → 안 침 | **의도인지 미확정** → TODO §6 |

확인 과제: 구름 공을 “포기”로 명시할지, 낮은 y·바운스 후 인터셉트를 넣을지.

---

## 열린 과제 (TODO와 맞출 것)

- 시뮬 GUI 렉 원인
- 구름 공 / 포기 조건 명문화 (위 I)
- EKF 타격 스모크 → C2 승격
- OpenCV 실검출 · 완전 ChArUco
- A4 \(e\)·마찰·drag 실측값으로 constants 갱신

자세한 체크리스트: [`TODO.md`](../TODO.md).

---

## 체크리스트 (짧은 확인용)

- [x] A — 중앙 바운드 \(v_{out}\)
- [x] B — 속도 0 폴백 금지, commit 창
- [x] C — once + 미드코트 + sim GT 기본
- [x] D — competition = 4-dof 체인, URDF는 실패 시 에러
- [x] E — 동적 인터셉트 (단일 y=0.30 아님)
- [x] F — OpenCV 필수 4.x
- [x] G — 테이블 OBB
- [x] H — 단일 크레이트 `src/`
- [ ] I — 구름 공 포기 정책 확정
- [ ] C2 — EKF 기본 승격 (스모크)

작성: 2026-07-11. 갱신: 2026-07-15 (thin types) · **2026-07-17** (플랫 구조·인터셉트·geometry·포기 경로).
