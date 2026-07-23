# TODO — pingpong-bot

실행 체크리스트. 상세 스펙은 `[plan.md](plan.md)`·`[docs/phase2.md](docs/phase2.md)`·`[docs/decisions.md](docs/decisions.md)`를 본다.  
로봇 프리셋 추가는 `[src/robot/catalog.rs](src/robot/catalog.rs)` `ROBOTS`만.

**우선순위 감각:** 시뮬 URDF·EKF(대체로 완료) → **OpenCV 실물 캡처·검출** → (스핀/Magnus) → 모터 → Rerun/리플레이.

---

## 0. 지금 당장 (시뮬·제어 갭)

### 0.1 URDF를 제어 SSOT로 사용

- [x] `origin xyz/rpy`, revolute 축·한계, EE 고정 변환을 domain 일반 직렬 체인으로 보존
- [x] 일반 행렬 FK + Jacobian DLS 수치 IK로 임의 revolute 관절 수 지원
- [x] 제어 `Joints`와 URDF 링크 pose가 같은 관절 순서를 사용
- [x] `build_with_arm_fallback`·관절 매핑 제거 — URDF 변환 실패 시 시작 오류
- [x] `all-4-export.urdf`의 URDF FK와 domain `Arm` FK 일치 회귀 테스트
- [x] 리니어 레일 X도 mesh/마운트에 반영되는지 확인

### 0.2 동적 인터셉트·실제 랠리

- [x] TOML `y_min..=y_max` 후보를 app/sim 공용 선택기로 평가
- [x] 레일+관절 위치·면법선 pose IK와 generalized Jacobian 속도 역산
- [x] 임팩트 내부 knot와 팔로스루를 잇는 두 구간 quintic
- [x] Rapier 활성 접촉→네트 통과→상대 코트 중앙 바운스 통합 회귀

### 0.3 EKF control 타격

- [x] EKF control이 ground truth 경로 수준으로 안정될 때까지 예측·commit 튜닝
  - drag=0(sim), 속도 FD 시드, 텔레포트 점프 리셋, C4 미드코트 게이트를 control에도 적용
- [x] 기본 모드를 ground truth → EKF로 올릴 조건 문서화 (decisions C2)

### 0.4 측정으로 잠글 상수 (decisions)

- [x] A4 반발계수 e — `tools/measure-restitution` (공식·CLI·`--sim`; 실측값으로 constants 갱신은 보드 준비 후)
- [x] 마찰 μ — `tools/measure-friction` + 탄도 바운스 v_t'=(1-\mu)v_t
- [x] drag k — `measure_restitution --drag-csv` (마일스톤 2.5 비행 로그 적합)

---

## 1. plan.md §7 제어 API (남은 것)

이미 있는 것(이름만 다를 수 있음):

| plan 시그니처              | 현재                                                   |
| ---------------------- | ---------------------------------------------------- |
| `racket_for_return`    | `required_racket_velocity` + `rally_return_velocity` |
| `ik`                   | `Arm::inverse_pose_with_rail` (위치 3 + 면법선 2)         |
| `joint_vel` (J^+ v_r)  | `Arm::velocities_for_racket_velocity`                |
| `plan_swing` / quintic | 동적 후보 선택 + 임팩트/팔로스루 2구간                              |
| `verify_torque_limits` | **대각 I\alpha 스텁**                                    |

아직/약함:

- [ ] **풀 매니퓰레이터 동역학** \tau = M\ddot q + C\dot q + g (`required_torque`)
- [ ] `is_feasible(tau, MotorLimits)` — 모터별 \tau_{\max} 테이블 + plan_swing 연동
- [ ] 토크 피드포워드를 `Hardware::command` 경로에 실을지 결정 (sim PD만 vs real FF)

---

## 2. 공 추적·스핀 / Magnus

출처: 공유 MD *「공 위치 추적」* (Tebbe trajectory-based ω, Model A/B/C fallback).  
plan §6은 협력 랠리 기본에서 Magnus를 뺐지만, 추적 정확도용으로 단계 도입.

- [ ] **스펙 문서화** — `docs/`에 공 추적·ω 추정 초안 안착 (카메라 2GS+1, Model A/B/C, bounce 구간 분리)
- [ ] Model A/B 안정화 확인 (중력 / 중력+drag) — 이미 ballistics·EKF 기반 있음, 인터페이스 정리
- [ ] 궤적 fitting → p,v,a → 잔여 가속도 → **ω 추정**
- [ ] Model C: a = g - k|v|v + k_m(\omega\times v), EKF 상태 확장 또는 별도 스핀 추정기
- [ ] prediction_error · spin_confidence · Model A/B fallback
- [ ] 바운드 전/후 fitting 구간 분리
- [ ] 출력: p(t), v(t), \omega, spin_type — Planner/로프트와 연결 여부 결정
- [ ] sim 슈터 spin(top/side/drill)과 추정 ω 교차검증

---

## 3. 관측 파이프라인 (OpenCV)

설계: [`docs/superpowers/specs/2026-07-18-vision-pipeline-design.md`](docs/superpowers/specs/2026-07-18-vision-pipeline-design.md)  
플랜: (작성 예정) `docs/superpowers/plans/2026-07-18-vision-pipeline.md`

보정은 **오프라인 툴 → JSON → 런타임 로드**. 캡처와 검출은 `Frame` + `BallDetector`로 분리.

- [x] `camera`에 OpenCV 삼각측량과 다중 뷰 DLT 통합
- [x] OpenCV **공 검출** — fuse + `detect-appearance` / `detect-full` (ROI `r`)
- [x] ChArUco (`calib_charuco --emit-sim` / `--from-images` 인트린식+`dist`)
- [x] 탁구대 랜드마크 solvePnP (`calib_table_pnp` — 6점 + FOV `K` + `R|t`)
- [x] UVC / 파일 (`OpenCvCapture` / `VideoCapture`)
- [x] `[vision]` TOML + 파이프라인 연결 (undistort → detect → observation)
- [ ] 멀티캠 동기·타임스탬프 (120 Hz 가정) — **비범위**
- [x] 외부 pose (탁구대 PnP 수동 클릭) — Charuco 자동 피팅은 비범위
---

## 4. 하드웨어 (마일스톤 5)

설계: [`docs/superpowers/specs/2026-07-17-real-hardware-dynamixel-design.md`](docs/superpowers/specs/2026-07-17-real-hardware-dynamixel-design.md)  
SSOT 매핑: `test-manipulator` `DynamixelConfig` (ID 1/3/4/5, signs, tick limits).

- [x] `rustypot` + `[hardware.dynamixel]` TOML → `RealHardware` (`read_pose` / `command`)
- [x] `SwingExecutor` — quintic `sample_at` → SyncWrite goal (200 Hz, `is_busy`)
- [x] `jog` REPL — 관절·레일·IK/pose·임팩트 속도 스윙 (`jog-axis`/`jog-rail` 대체)
- [x] `config/real-hardware.toml` + `run_real` 최소 연결·현재각 스모크 코드
- [x] AXL 리니어 레일 — `read_pose` 실측 `rail_x` + `RealHardware` 궤적 동기 (`command_abs_m`)
- [x] AXL 레일 스윙 동기 — `Hardware::command`가 관절·레일을 같은 `stream_hz`로 샘플링
- [ ] `run_real` + 카메라·`pipeline` (하드웨어 검증 후)
- [ ] Windows 벤치: `jog --dry-run` → 작은 `j`/`rd` → `swing` 순서로 재검증
- [ ] 실물 안전: E-stop 경로 (tick clamp·profile 속도/가속도는 적용됨)
- [x] URDF 관절 순서 ↔ `motor_ids` 문서·설정 길이 검증

---

## 5. 텔레메트리·디버깅 (마일스톤 4)

- [ ] Rerun `Telemetry` 어댑터 (영상·3D점·예측·지연 타임라인)
- [ ] span 지연 계측 완성
- [ ] 실패 세션 리플레이 (로그 → 파이프라인 재주입)

---

## 6. 시뮬레이터 품질

- [ ] **시뮬 GUI 렉 원인 조사** — kiss3d/Rapier/`physics_hz`/`frame_hz`/URDF mesh·패널 중 병목 특정
- [ ] **테이블 위 굴러다니는 공 포기 조건 정리** — 왜 안 치는지 코드 경로·게이트를 문서화하고, decisions에 “명시적 포기”가 있는지 맞추기
  - 관련: E2(평면 지난 공 예측 안 함), C1/C4(commit 창·미드코트), `plan_swing` `InfeasibleSwing` 스킵, `park_if_out_of_play`
  - 관측: 테이블 면을 굴러가면 인터셉트 평면 교차가 없어 스윙이 안 나감 — 의도인지 구멍인지 판정
- [ ] 라켓–공 충돌이 Rapier에서 신뢰 가능한지 (CCD·라켓 collider vs 키네마틱)
- [ ] ground truth/EKF 타격 성공률 스모크 기준 (예: N발 중 M리턴)
- [ ] 네트 넘김 / 바운스 후 / 사이드 샷 시나리오 테스트 세트
- [ ] GUI: 프리셋/`--robot` 표시, hit-plane·예측 마커 유지보수

---

## 7. ML (나중 — plan §10)

같은 vision API 뒤 교체 전제. 지금은 고전 파이프라인 우선.

- [ ] sim 라벨로 검출기 학습 데이터
- [ ] 충돌/스핀 잔차 보정 소모델
- [ ] 랠리 길이 목표 정책 탐색

---

## 8. 문서·정리

- [ ] `docs/phase2.md` 진도를 이 TODO와 주기적으로 동기
- [ ] 공 추적 MD → `docs/spin-tracking.md` (또는 동등)로 저장
- [ ] README 상태 표와 어긋나면 갱신
- [ ] A4 \(e\)·마찰·drag **실측값**으로 constants 갱신 (보드 준비 후)

---

## 빠른 검증 루틴

```bash
cargo test --workspace
cargo run -p pingpong-bot
cargo run -p pingpong-bot -- config/example.toml
```

작성: 2026-07-12. 갱신: 2026-07-21 — fuse 검출·정크 purge·단일 crate 구조.