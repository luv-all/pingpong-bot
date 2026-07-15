# 2단계 구현 로드맵

1단계 sim E2E(Z-up·URDF·슈터 GUI) 위에, **스윙·임팩트·관측·추정·설정**까지 Phase 2 소프트웨어 본선을 닫았다.  
실물(OpenCV 검출·Dynamixel·Rerun 리플레이)은 마일스톤 4–5로 남긴다.

애매 상수·폴백 결정은 [`decisions.md`](decisions.md)를 본다.

---

## 현재 진도 (2026-07-11)

| 영역 | 상태 | 비고 |
|------|------|------|
| 접수 평면 `HitPlane { y }` | ✅ | `DEFAULT_HIT_PLANE_Y = 0.30` |
| 4DOF IK + 리니어 레일 | ✅ | yaw·어깨·팔꿈치(2R)·손목 open |
| quintic 스윙 + 관절 한계 | ✅ | `domain::trajectory`, `plan_swing` |
| 임팩트 역산 + 로프트 \(v_{out}\) | ✅ | `domain::impact` |
| 본선 타격 = sim ground truth (기본) | ✅ | `sim.use_ground_truth=false`로 EKF control 실험 |
| ground truth 자동 스윙 | ✅ | commit 창(0.08–0.20s) + 네트 통과 후 |
| 탄도 예측 (적분+바운스) | ✅ | `domain::ballistics` |
| DLT + sim 핀홀 Calibration | ✅ | `CameraParams::sim_layout` |
| EKF hit-plane | ✅ | `BallEkf` |
| 토크 한계 (대각 \(I\alpha\)) | ✅ | `verify_torque_limits` |
| TOML 단일 설정 | ✅ | `config/default.toml` + `bin/config.rs` + Calibration JSON |
| 상수 SSOT | ✅ | `domain::constants` |
| 역할 모듈 | ✅ | camera·detector·triangulator·estimator·planner·robot |
| OpenCV 검출 / ChArUco 실보정 | ⏳ | 실물 — `calib_charuco --emit-sim`만 |
| Rerun / Dynamixel | ⏳ | 마일스톤 4–5 |

---

## 마일스톤 0 — 기준 고정

| 항목 | 상태 |
|------|------|
| Z-up 월드 좌표 | ✅ |
| `HitPlane { y }` | ✅ |
| 스윙·\(v_{out}\) 정책 (A–C) | ✅ |
| D/E 면·hit plane·restitution | ✅ |

---

## 마일스톤 1 — 관측 파이프라인

| # | 작업 | 상태 |
|---|------|------|
| 1.1 | ChArUco CLI (`--emit-sim` / `--validate`) | ✅ 스텁+emit |
| 1.2 | DLT `triangulate_synced` | ✅ |
| 1.3 | OpenCV 원 검출 | ⏳ 실물 (sim은 투영 픽셀) |
| 1.4 | sim 카메라 = domain Calibration | ✅ |
| 1.5 | TOML 단일 설정 | ✅ |

**완료 기준 (sim):** 카메라→DLT→3D — 달성. RMSE 튜닝은 측정 후.

---

## 마일스톤 2 — 궤적 추정

| # | 작업 | 상태 |
|---|------|------|
| 2.1 | `BallEkf` | ✅ |
| 2.2–2.3 | `predict_hit_plane` + 바운스 | ✅ |
| 2.4 | app 삼각측량→EKF | ✅ |
| 2.5 | k (drag) 측정 | ⏳ `tools/measure_*` |

---

## 마일스톤 3 — 스윙 계획

| # | 작업 | 상태 |
|---|------|------|
| 3.1–3.4 | IK·plan_swing·로프트·sim 타격 | ✅ |
| 3.5 | 토크 검증 (대각 관성) | ✅ (풀 동역학 후속) |
| 3.6 | 면 법선·URDF↔제어 | ✅ decisions D |

---

## 마일스톤 4 — 관측·디버깅

| # | 작업 | 상태 |
|---|------|------|
| 4.1 | Rerun `Telemetry` | ⏳ (`TracingTelemetry`로 대체) |
| 4.2 | span 지연 | 🔶 부분 |
| 4.3 | 실패 리플레이 | ⏳ |

---

## 마일스톤 5 — 실물 (Windows)

| # | 작업 | 상태 |
|---|------|------|
| 5.1 | UVC 카메라 | ⏳ |
| 5.2 | Dynamixel `RealHardware` | ⏳ `NotImplemented` |
| 5.3 | TOML `mode = "real"` | ⏳ |

---

## Phase 2 종료 기준 (소프트웨어)

- [x] 관측: DLT + sim Calibration 공유
- [x] 추정: BallEkf + hit-plane
- [x] 타격: loft + commit-once + control 본선
- [x] 설정: 선택적 TOML 경로 하나 (`config/default.toml`)
- [x] 결정: A–F 문서화
- [ ] 실물 OpenCV·모터·Rerun — **Phase 2.5 / 마일스톤 4–5**

## 다음 (스핀/Magnus 문서 후)

1. 스핀·Magnus 모델 스펙  
2. OpenCV Detector + ChArUco 실보정  
3. Rerun Telemetry  
4. Dynamixel / TOML `mode = "real"`

각 마일스톤마다 `cargo test --workspace` + sim GUI 스모크.
