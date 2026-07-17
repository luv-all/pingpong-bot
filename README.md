# pingpong-bot

사람과 오래 협력 랠리를 이어가는 핑퐁 로봇 런타임.  
Rust 경연용 단일 애플리케이션 크레이트다. 카메라·검출·추정·로봇·시뮬레이션·
계획을 `src/` 아래 기능별 모듈로 나눈다. OpenCV는 필수 의존성이며,
Rapier·실물 하드웨어 경계는 feature와 모듈로 격리한다.

상세 설계는 [`plan.md`](plan.md)를 본다.

---

## 요구 사항

- [Rust](https://rustup.rs/) (edition 2024)
- 시스템 **OpenCV 4.x** + `libclang` (`opencv` crate **0.98.2+**)
- sim: macOS/Linux. real(카메라·모터): Windows — 2단계

**주의:** OpenCV **5.x** 금지. Homebrew는 `opencv@4`. crate 0.98.2 미만이면 LLVM 22에서 바인딩이 깨진다.

### OpenCV · libclang

환경 변수는 [`.envrc`](.envrc)에 두고 `direnv allow .` (권장). `~/.zshrc`에 넣지 않는다.

**macOS**

```bash
brew install llvm opencv@4 pkgconf direnv
# OpenCV 5가 있으면: brew uninstall opencv && brew install opencv@4
# ~/.zshrc: eval "$(direnv hook zsh)"  →  cd 프로젝트  →  direnv allow .
pkg-config --modversion opencv4   # 4.x
cargo check --workspace
```

수동 export는 `.envrc`와 동일 (`LIBCLANG_PATH`, `PKG_CONFIG_PATH`, `DYLD_FALLBACK_LIBRARY_PATH`).

**Windows** (로컬 미검증 — [opencv INSTALL](https://docs.rs/crate/opencv/0.98.2/source/INSTALL.md) 기준)

```powershell
# VS C++ Build Tools + LLVM + vcpkg opencv4 (contrib 불필요, Charuco는 메인 objdetect)
vcpkg install opencv4:x64-windows
$env:VCPKG_ROOT = "C:\path\to\vcpkg"          # installed\... 가 아니라 vcpkg 루트
$env:VCPKGRS_DYNAMIC = "1"
$env:LIBCLANG_PATH = "C:\Program Files\LLVM\bin"
$env:PATH = "C:\Program Files\LLVM\bin;$env:VCPKG_ROOT\installed\x64-windows\bin;$env:PATH"
cargo check --workspace
```

공식 OpenCV zip을 쓸 때만 `OPENCV_INCLUDE_PATHS` / `OPENCV_LINK_PATHS` /
`OPENCV_LINK_LIBS=opencv_world4120`(버전에 맞게)를 수동 지정한다.
vcpkg 기본은 `world`가 꺼져 있어 `opencv_world4`만으로는 실패하기 쉽다.
DLL 못 찾으면 `installed\x64-windows\bin`이 PATH에 있는지 본다.

---

## 빠른 시작

```bash
# 전체 workspace 빌드·검증
cargo check --workspace
cargo test --workspace

# sim 파이프라인 실행 (`config/default.toml`)
cargo run -p pingpong-bot

# 다른 실험 설정
cargo run -p pingpong-bot -- config/experiment.toml
```

실행하면 **Rapier3d 디지털 트윈**(탁구대·공·로봇 팔) 위에서 가상 카메라가 공을 촬영하고, 제어 루프가 라켓을 구동한다.  
로그는 `tracing`으로 stdout에 출력된다.

---

## 런타임 설정 (`pingpong-bot`)

```bash
cargo run -p pingpong-bot                       # config/default.toml
cargo run -p pingpong-bot -- config/test.toml  # 지정한 TOML
```

CLI는 선택적인 TOML 경로 하나만 받는다. 모드·로봇·카메라·sim·물리 값은
모두 TOML이 SSOT이며 CLI override는 없다. `calibration_path`와 `urdf_path`의
상대 경로는 해당 TOML 파일의 디렉터리를 기준으로 해석한다. 전체 필드 설명은
[`config/example.toml`](config/example.toml)을 참고한다.
`[intercept]`의 `y_min..=y_max`를 `sample_step` 간격으로 예측해, 현재 로봇
포즈에서 접촉 오차·관절/레일 한계·테이블 충돌을 통과하는 타격점만 선택한다.

### 예시

```bash
# GUI sim (기본) — 슈터에서 「발사」로 공 쏘기
cargo run -p pingpong-bot

# 설명이 포함된 예시를 복사해 robot, [sim], urdf_path 등을 수정
cp config/example.toml config/experiment.toml
cargo run -p pingpong-bot -- config/experiment.toml

# 로그 레벨 조정
RUST_LOG=debug cargo run -p pingpong-bot
RUST_LOG=pingpong_bot=debug,info cargo run -p pingpong-bot -- config/experiment.toml
```

### Dynamixel 4축 (Windows)

`config/real-hardware.toml`은 Python `test-manipulator`에서 확인한 COM8·ID
1/3/4/5·방향·tick 리밋을 그대로 쓴다. 각도는 모터 절대각이 아니라 **URDF 관절각**이다.

```powershell
# 통신 없이 전체 경로 확인
cargo run -p jog-axis -- --config config/real-hardware.toml --angles-deg 0,0,0,0 --dry-run

# 실기는 작은 단축 이동부터 (예: URDF joint 0 → 5°)
cargo run -p jog-axis -- --config config/real-hardware.toml --joint 0 --angle-deg 5

# 포트 연결 + 현재 4축 읽기 스모크
cargo run -p pingpong-bot --features real -- config/real-hardware.toml
```

AXL 레일은 아직 `rail_x=0` 스텁이며, 비영 레일 궤적은 경고 후 팔 4축만 실행한다.
실카메라 pipeline은 다음 단계다.

---

## 아키텍처

도메인 핫패스는 모드 공통. `sim`/`real`은 **프레임·하드웨어만** 갈아 끼우고,
`pipeline`이 스레드·채널로 돌린다.

### 도메인

```mermaid
flowchart TB
  subgraph adapters ["① 모드 어댑터 — FrameSource + Hardware 주입"]
    direction LR
    sim["<b>sim</b><br/>Rapier 트윈 · 가상캠 · 뷰어"]
    real["<b>real</b><br/>Dynamixel 구현 · 실캠 예정"]
  end

  subgraph hot ["② 핫패스 — 관측 → 추정 → 계획 → 명령"]
    direction LR
    camera["<b>camera</b><br/>캘리브 · 삼각측량 · 프레임"]
    detector["<b>detector</b><br/>픽셀에서 공 검출"]
    estimator["<b>estimator</b><br/>궤적 · 타격 예측"]
    planner["<b>planner</b><br/>인터셉트 · 스윙"]
    hardware["<b>hardware</b><br/>관절 명령 · 포즈"]
    camera --> detector --> estimator --> planner --> hardware
  end

  robot["<b>robot</b><br/>Arm · FK/IK · URDF"]
  robot -.->|기구학| planner
  hardware -.->|상태| robot

  sim -->|가상 프레임| camera
  sim -->|SimHardware| hardware
  real -.->|실 프레임| camera
  real -.->|RealHardware| hardware

  subgraph support ["③ 지원"]
    direction LR
    pipeline["<b>pipeline</b><br/>스레드 · 채널"]
    telemetry["<b>telemetry</b><br/>예측 · 스윙 로그"]
    constants["<b>constants</b><br/>ITTF · 제어 상수"]
  end

  pipeline -.->|오케스트레이션| hot
  estimator --> telemetry
  planner --> telemetry
  constants -.-> hot
```

### 파이프라인 스레드

`pipeline`의 공통 워커는 카메라당 1 + 추정 1 + 제어 1이다.

```mermaid
flowchart LR
  frames["<b>FrameSource × N</b><br/>SimCamera 또는 실 카메라"]
  camT["<b>Camera worker × N</b><br/>프레임 취득 · 공 검출"]
  estT["<b>Estimation worker × 1</b><br/>동기화 · 삼각측량 · EKF"]
  ctrlT["<b>Control worker × 1</b><br/>100 Hz · 인터셉트 · 스윙"]
  actuator["<b>Hardware</b><br/>SimHardware 또는 RealHardware"]
  telemetry["<b>Telemetry</b><br/>예측 · 스윙 로그"]

  frames --> camT
  camT -->|"채널 · BallObservation"| estT
  estT -->|"최신값 슬롯 1칸 · Prediction[]"| ctrlT
  ctrlT --> actuator
  estT -.-> telemetry
  ctrlT -.-> telemetry
```

모드에 따라 공통 워커 바깥의 구현과 추가 스레드만 달라진다.

```mermaid
flowchart LR
  subgraph simSide ["sim 모드"]
    direction LR
    viewer["<b>Viewer</b><br/>메인 스레드 · GUI"]
    physics["<b>Physics</b><br/>별도 스레드 · Rapier"]
    simCamera["<b>SimCamera</b><br/>공유 월드에서 프레임 생성"]
    simHardware["<b>SimHardware</b><br/>공유 월드에 관절 명령"]
    viewer -.->|표시 · 슈터| physics
    physics --> simCamera
    simHardware --> physics
  end

  common["<b>공통 pipeline workers</b><br/>Camera × N → Estimation → Control"]
  simCamera --> common --> simHardware

  subgraph realSide ["real 모드 · 예정"]
    direction LR
    realCamera["<b>실 카메라</b><br/>UVC · 멀티캠"]
    realHardware["<b>RealHardware</b><br/>Dynamixel · AXL 스텁"]
  end

  realCamera -.-> common -.-> realHardware
```

GUI sim에서는 `Viewer`가 메인 스레드이고 `pipeline` 전체가 백그라운드에서 돈다.
`use_ground_truth`면 `Physics`가 타격까지 처리하고, 아니면 `Control` 명령을 사용한다.

결정은 [`docs/decisions.md`](docs/decisions.md).

---

## 프로젝트 구조

```
src/
  camera/     캡처·캘리브레이션·삼각측량·가상 카메라
  detector/   공 검출
  estimator/  EKF·탄도·물리계수 식별
  robot/      Arm·FK/IK·URDF·프리셋
  sim/        Rapier 월드·슈터·뷰어·sim 어댑터
  planner/    충돌·임팩트·스윙 궤적
  pipeline/   카메라→추정→제어 오케스트레이션
  hardware/   SimHardware / RealHardware
  constants/  공·탁구대·로봇·제어 상수
  config.rs   TOML 런타임 설정
  main.rs     CLI와 sim/real 조립

tools/      실험·캘리브·검증용 독립 바이너리
plan.md     기술 마스터 플랜
TODO.md     실행 체크리스트
```

**로봇**
- 기구학·제어 `Arm`은 `src/robot/`에만 있다. 부팅 시 `Arc<Arm>`으로 공유한다.
- **프리셋**은 [`src/robot/catalog.rs`](src/robot/catalog.rs) `ROBOTS`.
- URDF 프리셋은 origin·축·한계·EE를 보존한 직렬 체인으로 변환한다. 실패 시 시작 오류.
- `competition`만 메시 없음 — `4-dof`와 같은 단순화 체인.

| id | 모델 | 제어·FK·IK |
|----|------|------------|
| `competition` | 없음 (빌더만) | `4-dof` URDF의 축·offset·한계를 보존한 단순화 체인 |
| `urdf-test` | `assets/robots/urdf-test/.../urdf-test.urdf` | 해당 URDF |
| `competition-urdf` | `assets/robots/competition_arm.urdf` | 해당 URDF |
| `4-dof` | `assets/robots/4-dof/urdf/all-4-export.urdf` | 해당 URDF |

```toml
# config/default.toml
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
cargo run -p pingpong-bot
cargo run -p pingpong-bot -- config/experiment.toml
```

- 좌표계: **Z-up**, 원점 = 탁구대 로봇 쪽 꼭짓점 (바닥)
  - **+X** = 너비 1.525 m, **+Y** = 길이 2.74 m, **+Z** = 고도 (테이블 면 `z = 0.76 m`)
- **로봇** `y ≈ 0` 쪽, **슈터** `+y` 끝 (상대편)
- 공: 슈터에 **주차** → GUI 「발사」 시에만 비행, 이탈 시 자동 회수
- 타격: 동적 인터셉트 → 레일+관절 pose IK(라켓 중심·면법선) → 임팩트 knot
  → 팔로스루 순으로 실행하며, 실제 Rapier 접촉·네트 통과·상대 코트 중앙
  바운스를 회귀 테스트한다.
- GUI: yaw/pitch/roll 조준·속도·top/side/drill spin·시간배율 + 발사/회수 버튼
- **kiss3d 3D + egui 패널** (단일 창 — macOS EventLoop 제약)

제어 루프는 100 Hz. `Prediction` 슬롯은 1칸(최신 예측만 유지).

---

## 실험 도구 (`tools/`)

각 도구는 루트 `pingpong_bot` 라이브러리와 같은 타입을 공유한다. 카메라
캘리브레이션 산출물은 `pingpong_bot::Calibration`이다.

| crate | 바이너리 | 상태 | 용도 |
|-------|----------|------|------|
| `calib-charuco` | `calib_charuco` | ✅ | ChArUco (`--emit-sim` / `--validate` / `--from-images`) |
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
# 시스템 OpenCV 필요:
# cargo run -p calib-charuco -- --from-images ./boards -o calibration.json
cargo run -p pingpong-bot
```

OpenCV는 모든 빌드에서 필수다:

```bash
cargo test -p pingpong-bot --lib
```

### 물리 계수 측정 (`measure_*`)

공식은 `estimator::identify`, 기본 상수는 `constants::ball` / `physics`다.
측정 결과는 TOML 스니펫으로 찍히며 `-o`로 저장한 뒤 constants에 반영한다.

**반발계수 \(e\)** — \(e \approx \sqrt{h_{i+1}/h_i}\) 또는 \(e = |v_n'|/|v_n|\)

측정값은 기본으로 [`config/default.toml`](config/default.toml)의 `[physics]`에 **merge**된다.
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
cargo run -p pingpong-bot
# → Rapier 반발 + BallEkf drag/friction/restitution 예측에 [physics] 사용
```

런타임 필드는 TOML에서 명시한다. 누락되거나 타입이 틀리면 시작 전에 실패한다.
---

## 개발

```bash
# 특정 crate만
cargo check -p pingpong-bot --lib
cargo test -p pingpong-bot --lib

# 릴리스 빌드
cargo build -p pingpong-bot --release
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
| **로봇 프리셋** (TOML `robot`, `robot/catalog.rs` `ROBOTS`) | ✅ |
| **URDF mesh** (TOML `urdf_path` 또는 프리셋 `urdf_rel`) + 제어→URDF 관절 매핑 | ✅ |
| Z-up 좌표계 + `HitPlane { y }` 접수 평면 | ✅ |
| **BallScript** (시간·위치·속도·임펄스 스케줄) | ✅ |
| **RobotBuilder** (URDF mesh + sim 마운트) | ✅ |
| `sample_at` 타임스탬프 보간 | ✅ |
| DLT/OpenCV 삼각측량 (`camera`, 2뷰는 `triangulatePoints`) | ✅ |
| ChArUco (`calib_charuco --emit-sim` / `--from-images`) | ✅ 초안 |
| EKF / 궤적 추정 | ✅ (sim; 기본은 `sim.use_ground_truth=true`) |
| `measure_restitution` / `measure_friction` (e·μ·k) | ✅ |
| TOML 단일 설정 (`config/default.toml`) | ✅ |
| OpenCV 원/공 검출, 실 카메라 `VideoCapture` | ⏳ |
| Dynamixel 4축 `RealHardware` · `jog-axis` | ✅ (Windows 실기 재검증 필요) |
| AXL 레일, Rerun 시각화 | ⏳ (AXL은 x=0 스텁) |
| TOML `mode = "real"` | ✅ 연결·현재각 스모크 (실캠 pipeline 예정) |

sim에서는 **실제 3D 물리**로 공이 날고, ground truth 또는 EKF control로 라켓이 움직인다.

**로드맵:** [`docs/phase2.md`](docs/phase2.md) · 잔여 작업 [`TODO.md`](TODO.md) · 결정 [`docs/decisions.md`](docs/decisions.md)

---

## 라이선스

(미정)
