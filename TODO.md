# TODO — pingpong-bot

실행 체크리스트. 상세 스펙은 [`plan.md`](plan.md)·[`docs/phase2.md`](docs/phase2.md)·[`docs/decisions.md`](docs/decisions.md)를 본다.  
로봇 프리셋 추가는 [`crates/app/src/arm.rs`](crates/app/src/arm.rs) `ROBOTS`만.

**우선순위 감각:** 시뮬 URDF 실동화 → EKF 타격 안정화 → (스핀/Magnus) → 실물 OpenCV·모터 → Rerun/리플레이.

---

## 0. 지금 당장 (시뮬·제어 갭)

### 0.1 URDF를 “껍데기”에서 움직이게

지금: URDF = kiss3d mesh만, 제어·IK는 competition 빌더 `Arm` (`build_with_arm_fallback`).  
뷰어 FK와 제어 관절이 어긋나면 팔이 겉으로만 보이거나 관절이 안 맞는다.

- [x] sim에서 **제어 `Joints` → URDF 링크 pose**로 매 프레임 동기 (뷰어가 제어 각을 따라 그리기)
- [x] 관절 수/이름이 다를 때(예: urdf-test 3축 vs 제어 4DOF) 매핑 테이블 명시 (`ROBOTS` 또는 infra)
- [ ] 장기: URDF↔`Arm` 기구학 일치 시 `try_into_arm`으로 fallback 제거 (competition_arm.urdf 4DOF화 또는 URDF를 제어 SSOT로)
- [x] 리니어 레일 X도 mesh/마운트에 반영되는지 확인

### 0.2 EKF control 타격

- [ ] `--ekf-swing`이 오라클 수준으로 안정될 때까지 예측·commit 튜닝
- [ ] 기본 모드를 oracle → EKF로 올릴 조건 문서화 (decisions C2)

### 0.3 측정으로 잠글 상수 (decisions)

- [ ] A4 반발계수 \(e\) — `tools/measure-restitution`
- [ ] 마찰 μ — `tools/measure-friction`
- [ ] drag \(k\) — 비행 데이터로 추정 (마일스톤 2.5)

---

## 1. plan.md §7 제어 API (남은 것)

이미 있는 것(이름만 다를 수 있음):

| plan 시그니처 | 현재 |
|---------------|------|
| `racket_for_return` | `required_racket_velocity` + `loft_return_velocity` |
| `ik` | `Arm::inverse_kinematics*` |
| `joint_vel` (\(J^+ v_r\)) | `Arm::joint_velocities_for_ee_velocity` |
| `plan_swing` / quintic | `planner::plan_swing` |
| `verify_torque_limits` | **대각 \(I\alpha\) 스텁** |

아직/약함:

- [ ] **풀 매니퓰레이터 동역학** \(\tau = M\ddot q + C\dot q + g\) (`required_torque`)
- [ ] `is_feasible(tau, MotorLimits)` — 모터별 \(\tau_{\max}\) 테이블 + plan_swing 연동
- [ ] 토크 피드포워드를 `Hardware::command` 경로에 실을지 결정 (sim PD만 vs real FF)
- [ ] (선택) plan 이름에 맞춘 thin alias로 문서·코드 정렬

---

## 2. 공 추적·스핀 / Magnus

출처: 공유 MD *「공 위치 추적」* (Tebbe trajectory-based ω, Model A/B/C fallback).  
plan §6은 협력 랠리 기본에서 Magnus를 뺐지만, 추적 정확도용으로 단계 도입.

- [ ] **스펙 문서화** — `docs/`에 공 추적·ω 추정 초안 안착 (카메라 2GS+1, Model A/B/C, bounce 구간 분리)
- [ ] Model A/B 안정화 확인 (중력 / 중력+drag) — 이미 ballistics·EKF 기반 있음, 인터페이스 정리
- [ ] 궤적 fitting → \(p,v,a\) → 잔여 가속도 → **ω 추정**
- [ ] Model C: \(a = g - k|v|v + k_m(\omega\times v)\), EKF 상태 확장 또는 별도 스핀 추정기
- [ ] prediction_error · spin_confidence · Model A/B fallback
- [ ] 바운드 전/후 fitting 구간 분리
- [ ] 출력: \(p(t), v(t), \omega\), spin_type — Planner/로프트와 연결 여부 결정
- [ ] sim 슈터 spin(top/side/drill)과 추정 ω 교차검증

---

## 3. 관측 파이프라인 (실물)

- [ ] OpenCV **원/공 검출** Detector (`detect_*` 툴 → infra 어댑터)
- [ ] ChArUco **실보정** (`calib_charuco` emit-sim 넘어 실제 보드)
- [ ] UVC / 글로벌 셔터 카메라 `CameraSource` (Windows)
- [ ] 멀티캠 동기·타임스탬프 (120 Hz 가정)
- [ ] ROI 추적 안정화

---

## 4. 하드웨어 (마일스톤 5)

- [ ] Dynamixel SDK → `RealHardware` (`command` / `read_joints`)
- [ ] AXL 리니어 레일
- [ ] `--mode real` DI (bin)
- [ ] `jog-axis` 툴로 단축 조그·리밋 확인
- [ ] 실물 안전: 토크/속도 리밋, E-stop 경로
- [ ] URDF/빌더 관절 순서 ↔ 모터 ID 매핑 SSOT

---

## 5. 텔레메트리·디버깅 (마일스톤 4)

- [ ] Rerun `Telemetry` 어댑터 (영상·3D점·예측·지연 타임라인)
- [ ] span 지연 계측 완성
- [ ] 실패 세션 리플레이 (로그 → 파이프라인 재주입)

---

## 6. 시뮬레이터 품질

- [ ] 라켓–공 충돌이 Rapier에서 신뢰 가능한지 (CCD·라켓 collider vs 키네마틱)
- [ ] 오라클/EKF 타격 성공률 스모크 기준 (예: N발 중 M리턴)
- [ ] BallScript 시나리오 세트 (네트 넘김, 바운스 후, 사이드)
- [ ] GUI: 프리셋/`--robot` 표시, hit-plane·예측 마커 유지보수

---

## 7. ML (나중 — plan §10)

포트 뒤 교체 전제. 지금은 고전 파이프라인 우선.

- [ ] sim 라벨로 검출기 학습 데이터
- [ ] 충돌/스핀 잔차 보정 소모델
- [ ] 랠리 길이 목표 정책 탐색

---

## 8. 문서·정리

- [ ] `docs/phase2.md` 진도를 이 TODO와 주기적으로 동기
- [ ] 공 추적 MD → `docs/spin-tracking.md` (또는 동등)로 저장
- [ ] README 상태 표와 어긋나면 갱신
- [ ] decisions 미체크: A4 \(e\) 측정 계획

---

## 빠른 검증 루틴

```bash
cargo test --workspace
cargo run -p pingpong-bin -- --robot urdf-test
cargo run -p pingpong-bin -- --ekf-swing   # EKF 타격 실험
cargo run -p pingpong-bin -- --config config/example.toml
```

작성: 2026-07-12. Phase 2 소프트웨어 본선 이후 잔여·실물·스핀·URDF 실동화 기준.
