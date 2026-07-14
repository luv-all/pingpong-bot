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
| `--robot ID` | `competition` | 로봇 프리셋 (`crates/app/src/arm.rs` `ROBOTS`) |
| `--urdf PATH` | — | URDF 직접 지정 (`--robot` / config `robot` 무시) |
| `--ee-link LINK` | — | URDF 엔드이펙터 link (`--urdf`와 함께) |
| `--config PATH` | — | TOML (`hit_plane_y`, `camera_count`, `robot`, …) |
| `--frames N` | `300` | 가상 카메라 프레임 수 (headless) |
| `--camera-count N` | `3` | sim 카메라 대수 (삼각측량 최소 2대) |
| `--hit-plane-y M` | `0.30` | 접수 평면 y 좌표 [m] (로봇 앞 깊이) |
| `--sim-speed X` | `1.0` | sim 시간 배율 (10 = 10배속) |
| `--physics-hz H` | `1000` | Rapier 물리 적분 주파수 [Hz] |
| `--frame-hz H` | `120` | 가상 카메라 프레임률 [Hz] |
| `--no-gui` | — | headless sim (기본은 GUI 켜짐) |
| `--shoot-on-start` | `false` | headless sim 시작 시 1회 발사 |
| `--ekf-swing` | `false` | EKF 예측으로 타격 (기본은 sim 오라클) |

우선순위: `--urdf` > CLI `--robot` > TOML `robot` > `competition`.

### 예시

```bash
# GUI sim (기본) — 슈터에서 「발사」로 공 쏘기
cargo run -p pingpong-bin

# 프리셋: mesh = urdf-test, 제어 = competition 빌더
cargo run -p pingpong-bin -- --robot urdf-test

# config + CLI가 robot을 덮어씀
cargo run -p pingpong-bin -- --config config/example.toml --robot competition-urdf

# URDF 직접 지정 (프리셋 무시, 제어는 기본 competition 빌더)
cargo run -p pingpong-bin -- --urdf assets/robots/custom/robot.urdf --ee-link racket_link

# headless + 시작 시 1회 발사
cargo run -p pingpong-bin -- --no-gui --frames 120 --shoot-on-start --sim-speed 5

# 카메라 2대, 120프레임
cargo run -p pingpong-bin -- --camera-count 2 --frames 120 --hit-plane-y 0.30

# 로그 레벨 조정
RUST_LOG=debug cargo run -p pingpong-bin -- --frames 30
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

tools/      실험·캘리브·검증용 독립 바이너리
plan.md     기술 마스터 플랜
TODO.md     실행 체크리스트
```

의존 방향: `bin` → `app` / `infra` → `domain`

**로봇**
- 기구학·제어 `Arm` 타입은 `domain/robot/`에만 있다. 부팅 시 `Arc<Arm>`으로 sim·real·제어가 같은 불변 객체를 공유한다.
- **프리셋 SSOT**는 [`crates/app/src/arm.rs`](crates/app/src/arm.rs)의 `ROBOTS`다. id·URDF 경로·링크 길이·관절 한도·`control_to_urdf` 매핑은 여기만 고친다.
- URDF가 있는 프리셋도 **제어·IK는 `build`(빌더 `Arm`)**, URDF는 kiss3d **mesh 시각화**용이다 (예: `urdf-test` mesh 3축 ↔ 제어 4DOF, `control_to_urdf`로 앞 3축 동기).

| id | mesh | 제어 |
|----|------|------|
| `competition` | 없음 (빌더만) | competition 빌더 |
| `urdf-test` | `assets/robots/urdf-test/.../urdf-test.urdf` | 동일 빌더 |
| `competition-urdf` | `assets/robots/competition_arm.urdf` | 동일 빌더 |

```toml
# config/example.toml
robot = "competition"
```

### sim 모드 — Rapier3d 디지털 트윈 (plan §9)

```
SimSession (물리 스레드 @ physics-hz, CCD)
  ├─ SimWorld: ITTF 탁구대 + 슈터(+y) + 로봇 라켓(y≈0) + 공
  ├─ SimCamera × N: 공 3D 위치 → 핀홀 픽셀 투영
  └─ SimHardware: plan_swing → 관절 목표 → 라켓 이동

슈터(+y) ──발사──► 테이블 ──► 로봇(y≈0) 라켓
         ▲ GUI 「발사」 버튼으로 트리거
```

```bash
cargo run -p pingpong-bin
cargo run -p pingpong-bin -- --robot urdf-test
cargo run -p pingpong-bin -- --no-gui --frames 60 --shoot-on-start --sim-speed 5
```

- 좌표계: **Z-up**, 원점 = 탁구대 로봇 쪽 꼭짓점 (바닥)
  - **+X** = 너비 1.525 m, **+Y** = 길이 2.74 m, **+Z** = 고도 (테이블 면 `z = 0.76 m`)
- **로봇** `y ≈ 0` 쪽, **슈터** `+y` 끝 (상대편)
- 공: 슈터에 **주차** → GUI 「발사」 시에만 비행, 이탈 시 자동 회수
- GUI: yaw/pitch/roll 조준·속도·top/side/drill spin·시간배율 + 발사/회수 버튼
- **kiss3d 3D + egui 패널** (단일 창 — macOS EventLoop 제약)

제어 루프는 100 Hz. `Prediction` 슬롯은 1칸(최신 예측만 유지).

---

## 실험 도구 (`tools/`)

각 도구는 `domain`/`infra`와 같은 타입·포트를 공유한다.

| crate | 바이너리 | 상태 | 용도 |
|-------|----------|------|------|
| `calib-charuco` | `calib_charuco` | ✅ | ChArUco 보정 (`--emit-sim` / `--validate`) |
| `measure-restitution` | `measure_restitution` | ✅ | 반발계수 \(e\), 항력 \(k\) |
| `measure-friction` | `measure_friction` | ✅ | 마찰계수 \(\mu\) |
| `jog-axis` | `jog_axis` | ⏳ 스텁 | 축 수동 조그 |
| `capture-flying-ball` | `capture_flying_ball` | ⏳ 스텁 | 비행 공 캡처 데이터셋 |
| `detect-bgsub` | `detect_bgsub` | ⏳ 스텁 | 배경 차분 검출 |
| `detect-colormask` | `detect_colormask` | ⏳ 스텁 | 색상 마스크 검출 |
| `detect-contour` | `detect_contour` | ⏳ 스텁 | contour·원형도 검출 |
| `detect-roi` | `detect_roi` | ⏳ 스텁 | ROI 추적 검출 |

### 캘리브레이션

```bash
cargo run -p calib-charuco -- --emit-sim 3 -o calibration.json
cargo run -p calib-charuco -- --validate calibration.json
cargo run -p pingpong-bin -- --config config/example.toml
```

### 물리 계수 측정 (`measure_*`)

공식은 `domain::estimator::identify`, 기본 상수는 `domain::constants::ball` / `physics`다.  
측정 결과는 TOML 스니펫으로 찍히며 `-o`로 저장한 뒤 constants에 반영한다.

**반발계수 \(e\)** — \(e \approx \sqrt{h_{i+1}/h_i}\) 또는 \(e = |v_n'|/|v_n|\)

측정값은 기본으로 [`config/example.toml`](config/example.toml)의 `[physics]`에 **merge**된다.  
다른 파일을 쓰려면 `--config path`, 쓰기 없이 확인만 하려면 `--dry-run`.

```bash
# 연속 바운스 정점 높이 [m] → config [physics].restitution
cargo run -p measure-restitution -- --heights 0.40,0.29,0.21

# 법선 속력 쌍 |vin|:|vout|
cargo run -p measure-restitution -- --vz-pairs 2.0:1.7,1.9:1.61

# 탄도 적분으로 설정된 TABLE_BOUNCE_RESTITUTION 검증 (모델 SSOT)
cargo run -p measure-restitution -- --sim-ballistics --dry-run

# Rapier 낙하 — 솔버 실효 e
cargo run -p measure-restitution -- --sim

# 다른 config 파일에 쓰기
cargo run -p measure-restitution -- --heights 0.40,0.29 --config config/my.toml
```

**마찰 \(\mu\)** — \(v_t' = (1-\mu)\,v_t\) → \(\mu = 1 - |v_t'|/|v_t|\)

```bash
cargo run -p measure-friction -- --vt-pairs 2.0:1.4,1.5:1.05
cargo run -p measure-friction -- --sim
cargo run -p measure-friction -- --sim --dry-run
```

**항력 \(k\)** — 비행 궤적 CSV `t,x,y,z` 최소제곱 (마일스톤 2.5)

```bash
cargo run -p measure-restitution -- --drag-csv traj.csv
```

런타임 반영:

```bash
cargo run -p pingpong-bin -- --config config/example.toml
# → Rapier 반발 + BallEkf drag/friction/restitution 예측에 [physics] 사용
```

참고: 필드를 비우면 `PhysicsParams::default()` (e=0.85, μ=0.15, drag=0).
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
| **로봇 프리셋** (`--robot` / TOML `robot`, `app/arm.rs` `ROBOTS`) | ✅ |
| **URDF mesh** (`--urdf` 또는 프리셋 `urdf_rel`) + 제어→URDF 관절 매핑 | ✅ |
| Z-up 좌표계 + `HitPlane { y }` 접수 평면 | ✅ |
| **BallScript** (시간·위치·속도·임펄스 스케줄) | ✅ |
| **RobotBuilder** (URDF mesh + sim 마운트) | ✅ |
| `sample_at` 타임스탬프 보간 | ✅ |
| DLT 삼각측량, ChArUco 캘리브레이션 | ✅ (sim emit / validate) |
| EKF / 궤적 추정 | ✅ (sim; `--ekf-swing` 실험, 기본은 오라클) |
| `measure_restitution` / `measure_friction` (e·μ·k) | ✅ |
| `--config` TOML | ✅ |
| OpenCV 검출, 실 카메라 | ⏳ 2단계 |
| Rerun 시각화, Dynamixel/AXL real | ⏳ 2단계 |
| `--mode real` | ⏳ 2단계 |

sim에서는 **실제 3D 물리**로 공이 날고, 오라클(또는 `--ekf-swing`)로 라켓이 움직인다.

**로드맵:** [`docs/phase2.md`](docs/phase2.md) · 잔여 작업 [`TODO.md`](TODO.md) · 결정 [`docs/decisions.md`](docs/decisions.md)

---

## 라이선스

(미정)
