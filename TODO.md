# TODO — pingpong-bot

실행 체크리스트. 상세 스펙은 [`plan.md`](plan.md)·[`docs/phase2.md`](docs/phase2.md)·[`docs/decisions.md`](docs/decisions.md).  
앱 숫자는 [`src/defaults/`](src/defaults/) SSOT. 로봇 활성 프리셋은 `defaults::robot()` 본문만.

공개 API는 도메인 모듈 경로 (`estimator::Impact` / `robot::motion::Planner` /
`estimator::Kinematics` / `camera::Preview` / `sim::eval::Protocol` …).
자유함수 root dump·호환 alias는 쓰지 않는다.

파이프라인은 역할 기준: `detector` (검출) → `estimator` (삼각측량·EKF·예측·반발 역산)
→ `robot::motion` (관절공간 계획) → `hardware`.
`ball`/`shooter`/`swing`/`planner`/`eval` 도메인은 해체됨 — 공 상태·피더·채점은
`sim::physics`/`sim::launch`/`sim::eval`, 팔–테이블 관통은 `robot::collision`.

**우선순위:** **리턴 파워(eval)** → 실기 단발 1발 계측 / Windows 벤치 → 시뮬 품질·포기 정책 → ω 추정 → 풀 동역학 후속.

> **🔴 지금 최우선 — 리턴 파워:** eval 30/90 (통과선 45). 30발 전부 1점.
> 진단·반증: [`docs/superpowers/plans/2026-07-27-return-power.md`](docs/superpowers/plans/2026-07-27-return-power.md).
> **새 세션은 이 문서부터** — 배제·반증한 레버를 다시 밟지 말 것.

---

## 0. 지금 당장

### 0.1 시뮬·제어 (대체로 완료)

- [x] URDF → domain 직렬 체인 / FK·IK / 레일
- [x] 동적 인터셉트·quintic·Rapier 다물체 EE 폐루프
- [x] EKF control 튜닝 (drag=0 sim, C4 미드코트 게이트) — decisions C2 문서화
- [ ] C2 승격 — EKF 타격 성공률 스모크 후 기본값 GT→EKF 전환

### 0.2 측정으로 잠글 상수

- [x] e / μ / drag 측정 툴 + [`docs/measure-physics.md`](docs/measure-physics.md)
- [x] 테이블 바운스 커널 `estimator::Kinematics::bounce_on_table`
- [ ] Rapier 테이블–공 μ ↔ 커널 정렬 (랜덤샷·랠리 재튜닝 동반)
- [ ] A4 e·마찰·drag **실측값**으로 `PhysicsParams` / `impact()` 갱신 (보드 준비 후)

---

## 1. 제어 API (현황)

| 역할 | facade |
|------|--------|
| 리턴·라켓 속도 | `estimator::Impact::rally_return` / `required_racket_velocity` |
| 스윙 | `robot::motion::Planner::plan` / `plan_best` / `plan_bang_bang` |
| IK·속도 | `Arm::inverse_pose_with_rail` / `velocities_for_racket_velocity` |
| 토크 | `Arm::required_torque` / `Arm::torque_feasible` |

- [x] RNEA·토크 게이트·FF (`torque_feedforward` default true)
- [x] Dynamixel Goal Current + sim RNEA→다물체 effort + τ HUD

---

## 2. 공 추적·스핀 / Magnus

순방향 Model C는 sim·탄도·플래너에 들어감. **역방향 ω 추정**만 남음.

- [x] Model C 순방향 (`estimator::Kinematics` / Rapier 외력 / `sim::launch` spin)
- [ ] 스펙 — `docs/`에 카메라·Model A/B/C·bounce 구간
- [ ] 궤적 fitting → ω 추정 · EKF 확장 또는 별도 추정기
- [ ] prediction_error · spin_confidence · A/B fallback · 바운드 전후 분리
- [ ] sim 슈터 spin ↔ 추정 ω 교차검증

---

## 3. 관측 파이프라인

설계: [`docs/superpowers/specs/2026-07-18-vision-pipeline-design.md`](docs/superpowers/specs/2026-07-18-vision-pipeline-design.md)  
조립 SSOT: `defaults::detector_for` · `data/colormask.json`.  
진입: `camera::Preview` / `camera::Charuco` / `camera::TablePnp` / `estimator::Triangulate` / `detector::Detector`.

- [x] 삼각·검출·ROI·colormask·ChArUco·탁구대 PnP·UVC/파일·undistort 파이프
- [ ] 멀티캠 동기·타임스탬프 — **비범위**

---

## 4. 하드웨어

- [x] `RealHardware` · SwingExecutor · jog · AXL 레일 · URDF↔motor_ids
- [x] `run_real` **단발 타격** — `src/real/` (채널 단일소유 파이프라인).
      설계 [`src/real/README.md`](src/real/README.md) ·
      계획 [`docs/superpowers/plans/2026-07-31-run-real-single-shot.md`](docs/superpowers/plans/2026-07-31-run-real-single-shot.md)
- [ ] Windows 벤치: `jog --dry-run` → 작은 `j`/`rd` → `swing` → `--mode real --dry-run` → 실기 1발
- [ ] 실기 1발 계측 후 랠리 확장 (커밋 래치 해제·샷 간 EKF 리셋·연속 포기 정책)
- [ ] 실물 E-stop 경로 (tick clamp·profile은 적용됨)

---

## 5. 시뮬레이터 품질

- [x] GUI 렉 (dev opt-level) · 라켓–공 physical SSOT · GUI 기본 디버그
- [ ] 테이블 위 구름 공 포기 조건 — decisions I (인터셉트 평면 교차 없음 → 스윙 안 나감)
- [ ] GT/EKF 타격 성공률 스모크 → C2
- [ ] 네트 넘김 / 바운스 후 / 사이드 샷 시나리오 세트
- [ ] GUI: 활성 로봇 프리셋 표시, hit-plane·예측 마커 유지보수

---

## 6. 문서·후속

- [ ] `docs/phase2.md` ↔ 이 TODO·defaults 동기
- [ ] 공 추적 MD → `docs/spin-tracking.md`
- [ ] ML (plan §10) — vision API 뒤 교체 전제, 고전 파이프라인 우선

---

## 빠른 검증

```bash
cargo test --workspace
cargo run -p pingpong-bot -- --mode sim
# 실캠 단발 리허설 (모터 정지, macOS에서도 됨)
cargo run -p pingpong-bot -- --mode real --dry-run
# Windows real:
# cargo run -p pingpong-bot -- --mode real --dxl-port COM8
```

갱신: 2026-07-30 — 도메인 재편: ball/shooter/swing/planner/eval 해체, 계획은 `robot::motion`,
공 반발 역산은 `estimator::Impact`, robot↔hardware 레이어 분리, 호환 alias 제거.
