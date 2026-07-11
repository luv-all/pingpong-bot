# pingpong-bot

사람과 오래 협력 랠리를 이어가는 핑퐁 로봇 런타임.  
Rust 헥사고날 아키텍처(ports & adapters)로 domain / app / infra / bin을 분리했다.

상세 설계는 [`plan.md`](plan.md)를 본다.

---

## 요구 사항

- [Rust](https://rustup.rs/) (edition 2024 — 최신 stable 권장)
- Cargo (workspace)

macOS·Linux에서 **sim 모드**로 end-to-end 파이프라인을 돌릴 수 있다.  
**real 모드**(실 카메라·모터)는 Windows + `pingpong-infra/real` feature — 2단계 예정.

---

## 빠른 시작

```bash
# 전체 workspace 빌드·검증
cargo check --workspace
cargo test --workspace

# sim 파이프라인 실행 (기본: 카메라 3대, 300프레임)
cargo run -p pingpong-bin

# 짧은 스모크 테스트 (5배속)
cargo run -p pingpong-bin -- --frames 60 --sim-speed 5
```

실행하면 **Rapier3d 디지털 트윈**(탁구대·공·로봇 팔) 위에서 가상 카메라가 공을 촬영하고, 제어 루프가 라켓을 구동한다.  
로그는 `tracing`으로 stdout에 출력된다.

---

## 런타임 CLI (`pingpong-bot`)

```bash
cargo run -p pingpong-bin -- [OPTIONS]
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--mode sim\|real` | `sim` | 실행 모드. `real`은 아직 미구현 |
| `--frames N` | `300` | 가상 카메라 프레임 수 |
| `--camera-count N` | `3` | sim 카메라 대수 (삼각측량 최소 2대) |
| `--hit-plane-y M` | `0.30` | 접수 평면 y 좌표 [m] (로봇 앞 깊이) |
| `--sim-speed X` | `1.0` | sim 시간 배율 (10 = 10배속) |
| `--physics-hz H` | `1000` | Rapier 물리 적분 주파수 [Hz] |
| `--frame-hz H` | `120` | 가상 카메라 프레임률 [Hz] |
| `--no-gui` | — | headless sim (기본은 GUI 켜짐) |
| `--shoot-on-start` | `false` | headless sim 시작 시 1회 발사 |

### 예시

```bash
# GUI sim (기본) — 슈터에서 「발사」로 공 쏘기
cargo run -p pingpong-bin

# headless + 시작 시 1회 발사
cargo run -p pingpong-bin -- --no-gui --frames 120 --shoot-on-start --sim-speed 5

# 카메라 2대, 120프레임, 타격 높이 0.80m
cargo run -p pingpong-bin -- --camera-count 2 --frames 120 --hit-plane-y 0.30

# 로그 레벨 조정 (debug까지)
RUST_LOG=debug cargo run -p pingpong-bin -- --frames 30

# 특정 crate 로그만
RUST_LOG=pingpong_app=debug,info cargo run -p pingpong-bin -- --frames 30
```

---

## 프로젝트 구조

```
crates/
  domain/   순수 도메인 — 타입, `Arm`/FK, `constants/`(ITTF 규격), 포트 trait
  app/      파이프라인 오케스트레이션 — 카메라·추정·제어 스레드, 채널
  infra/    어댑터 — Rapier sim, TracingTelemetry, SyntheticCamera(레거시) 등
  bin/      CLI 진입점 — sim/real 모드 DI

tools/      실험·캘리브·검증용 독립 바이너리 (9개, 1단계는 스텁)
plan.md     기술 마스터 플랜
```

의존 방향: `bin` → `app` / `infra` → `domain`

**로봇 모델(`Arm`)** 은 `domain/robot.rs`에만 있다. 부팅 시 `Arc<Arm>`으로 sim·real·제어가 **같은 불변 객체**를 공유한다. Rapier·Dynamixel은 `RacketPose`를 각자 SDK 형식으로 변환할 뿐, FK/관절 상태는 domain에만 있다.

### sim 모드 — Rapier3d 디지털 트윈 (plan §9)

```
SimSession (물리 스레드 @ physics-hz, CCD)
  ├─ SimWorld: ITTF 탁구대 + 슈터(+x) + 로봇 라켓(-x) + 공
  ├─ SimCamera × N: 공 3D 위치 → 핀홀 픽셀 투영
  └─ SimHardware: plan_swing → 관절 목표 → 라켓 이동

슈터(+x) ──발사──► 테이블 ──► 로봇(-x) 라켓
         ▲ GUI 「발사」 버튼으로 트리거
```

```bash
# GUI로 슈터 설정·발사 (권장)
cargo run -p pingpong-bin

# URDF + STL mesh (Fusion 360 export → assets/robots/custom/meshes/)
cargo run -p pingpong-bin -- --urdf assets/robots/custom/robot.urdf --ee-link racket_link

# 실물 로봇 URDF (3축 + STL mesh, `assets/robots/urdf-test/`)
cargo run -p pingpong-bin -- \
  --urdf assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf \
  --ee-link pingpong_paddle_v5_1

# URDF primitive 예시 (3축 → plan_swing `Arm` 변환 가능)
cargo run -p pingpong-bin -- --urdf assets/robots/competition_arm.urdf --ee-link racket_link

# headless: 시작 시 1회 발사
cargo run -p pingpong-bot -- --frames 60 --shoot-on-start --sim-speed 5
```

- 좌표계: **Z-up**, 원점 = 탁구대 로봇 쪽 꼭짓점 (바닥)
  - **+X** = 너비 1.525 m, **+Y** = 길이 2.74 m, **+Z** = 고도 (테이블 면 `z = 0.76 m`)
- **로봇** `y ≈ 0` 쪽, **슈터** `+y` 끝 (상대편)
- 공: 슈터에 **주차** → GUI 「발사」 시에만 비행, 이탈 시 자동 회수
- GUI: yaw/pitch/roll 조준·속도·top/side/drill spin·시간배율 + 발사/회수 버튼
- **kiss3d 3D + egui 패널** (단일 창 — macOS EventLoop 제약)

제어 루프는 100 Hz. `Target` 슬롯은 1칸(최신 예측만 유지).

---

## 실험 도구 (`tools/`)

각 도구는 `domain`/`infra`와 같은 타입·포트를 공유한다.  
**Phase 2:** `calib_charuco --emit-sim` / `--validate` 사용 가능. OpenCV·측정 도구 본문은 실물 마일스톤.

| crate | 바이너리 | 용도 |
|-------|----------|------|
| `calib-charuco` | `calib_charuco` | ChArUco 보정 (`--emit-sim`으로 sim Calibration JSON) |
| `measure-restitution` | `measure_restitution` | 반발계수 e 측정 |
| `measure-friction` | `measure_friction` | 마찰계수 μ 측정 |
| `jog-axis` | `jog_axis` | 축 수동 조그 |
| `capture-flying-ball` | `capture_flying_ball` | 비행 공 캡처 데이터셋 |
| `detect-bgsub` | `detect_bgsub` | 배경 차분 검출 실험 |
| `detect-colormask` | `detect_colormask` | 색상 마스크 검출 실험 |
| `detect-contour` | `detect_contour` | contour·원형도 검출 실험 |
| `detect-roi` | `detect_roi` | ROI 추적 검출 실험 |

```bash
cargo run -p calib-charuco -- --emit-sim 3 -o calibration.json
cargo run -p pingpong-bin -- --config config/example.toml --gui
```

---

## 개발

```bash
# 특정 crate만
cargo check -p pingpong-domain
cargo test -p pingpong-domain

# 릴리스 빌드
cargo build -p pingpong-bin --release
# → target/release/pingpong-bot
```

---

## 현재 구현 상태

| 영역 | 상태 |
|------|------|
| workspace 스캐폴딩, sim E2E 파이프라인 | ✅ |
| **Rapier3d 디지털 트윈** (탁구대·슈터·로봇·공·SimCamera) | ✅ |
| **egui 슈터 GUI** (발사 트리거·파라미터) | ✅ |
| **kiss3d 3D 뷰** (탁구대·로봇·슈터·공) | ✅ |
| **URDF 로봇 로드** (`--urdf`, `--ee-link`) | ✅ (primitive + **STL/OBJ mesh**) |
| Z-up 좌표계 + `HitPlane { y }` 접수 평면 | ✅ |
| **BallScript** (시간·위치·속도·임펄스 스케줄) | ✅ |
| **RobotBuilder** (URDF mesh + 마운트 프리셋) | ✅ |
| `sample_at` 타임스탬프 보간 | ✅ |
| DLT 삼각측량, ChArUco 캘리브레이션 | ⏳ 2단계 |
| EKF / RK4 궤적 추정 | ⏳ 2단계 |
| OpenCV 검출, 실 카메라 | ⏳ 2단계 |
| Rerun 시각화, Dynamixel/AXL real | ⏳ 2단계 |
| `--config` TOML, `--mode real` | ⏳ 2단계 |

삼각측량·EKF·스윙 계획 본체는 아직 스텁이지만, sim에서는 **실제 3D 물리**로 공이 날고 라켓이 움직인다.

**2단계 상세 로드맵:** [`docs/phase2.md`](docs/phase2.md)

---

## 라이선스

(미정)
