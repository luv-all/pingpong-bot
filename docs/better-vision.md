# Better Vision

> detector → estimator를 **지우고 다시 짠다.** 이 문서가 SSOT다.
>
> 담당: 비전(detector→estimator). 하드웨어·기구학은 별도 담당.

---

## 1. 계약이 먼저다

비전이 기구학에 넘기는 것은 **7차원 상태의 궤적 두 개**다. 궤적이므로 시퀀스 축이 하나 더
붙어 8차원이다.

```rust
/// 한 시점의 공 상태 — 7차원 `[x y z vx vy vz t]`.
#[derive(Clone, Copy, Debug)]
pub struct State {
    /// [`Trajectory::origin`] 기준 경과 [s].
    pub t: f64,
    pub position: Point3,
    pub velocity: Vector3<f64>,
}

/// 공 하나의 궤적 — 지금을 기준으로 지나온 길과 앞으로 갈 길.
///
/// 비전이 기구학에 넘기는 **유일한** 타입. 다른 건 아무것도 안 넘긴다.
///
/// 이름이 `measured`/`predicted`인 건 **믿을 만한 정도가 다르기 때문**이다. 둘 다 필터
/// 상태지만 앞쪽은 관측에 묶여 있고 뒤쪽은 외삽이다. 소비자가 그 차이를 알아야 한다.
#[derive(Clone, Debug)]
pub struct Trajectory {
    /// 샷 일련번호 — 기구학이 "같은 공인가"를 판단하는 근거.
    pub seq: u64,
    /// `t = 0`의 벽시계. 지연 보상은 소비자가 `origin.elapsed()`로 한다.
    pub origin: Instant,
    /// 지나온 길 — 관측이 있던 시각의 필터 상태.
    pub measured: Vec<State>,
    /// 앞으로 갈 길 — 지금 상태에서 굴린 것. 공이 플레이 부피를 벗어날 때까지.
    pub predicted: Vec<State>,
}
```

**이 계약에서 나머지가 전부 따라 나온다.**

| 계약이 요구하는 것 | 그래서 설계가 이렇게 된다 |
|---|---|
| 모든 샘플이 `v`를 갖는다 | 필터가 위치·속도 전체 상태를 들고 있어야 한다 → §4 EKF |
| `measured`가 촘촘해야 쓸모 있다 | 프레임을 버리면 안 된다 → 후보 K개 + 픽셀 공간 갱신 |
| `predicted`가 끝까지 간다 | 접수 평면 개념이 비전에 없다 → `InterceptWindow` 의존 삭제 |
| 언제 보낼지 정해야 한다 | 선언 기준(네트 통과) → §4 `Phase` |
| `origin`이 정직해야 한다 | 셔터 시각 보정 → §6 Stage 5 |

**비전이 모르는 것:** 로봇 도달 범위, 접수 평면, IK 가능 여부, 라켓. "어디를 칠까"와
"닿는가"는 전부 기구학 쪽이다. 비전이 내리는 판단은 **"믿을 만한 트랙이 있는가"** 하나뿐이다.

> 기구학 담당과 합의 완료. 그쪽이 알아서 구현한다.

---

## 2. 왜 지우고 다시 짜는가

실기에서 셋이 무너졌다 — **검출 튐**, **예측 타점 오차**, **레이턴시 부족**.

코드 문제만이 아니다. AI가 다 쓰면서 길고 장황해졌고, 3명이 컨텍스트를 공유하지 못한 채
각자 다른 그림을 들고 일했다. **성공 기준은 둘이다: 지표가 오르는 것, 그리고 코드가
줄어드는 것.** 기능을 더하는 방향이면 같은 문제가 반복된다.

지금 `src/detector` 2,448줄 + `src/estimator` 2,758줄이다. 새 구조는 **그보다 작아야 한다.**

### 2.1 색이 1차 판별기인 게 근본 오류

실제 적을 늘어놓으면 즉시 보인다.

| 적 | 색으로 갈리나 | 실제로 갈리는 근거 |
|---|---|---|
| **바닥에 떨어진 공** | ❌ 진짜 탁구공. 색·모양·크기 동일 | **정지해 있음** |
| **피부(팔·손)** | ❌ 주황 색상역과 겹침 | 크기가 기대 반지름 대비 과대 + 스테레오 부정합 |
| **벽면·바닥** | ❌ 태양광에 따라 겹침 | **정지해 있음** |
| **태양광 변동** | ❌ 정적 색 상자가 원리상 붕괴 | 배경 모델이 조명과 함께 표류 |

**넷 중 색으로 갈리는 게 하나도 없다.** 측정된 귀결 — 색 하한을 25 풀면 비행 밖 오검출이
**5 → 382**로 폭발한다(`9006677`). 상자를 아무리 정교하게 깎아도(타원체든 커스텀 공간이든)
배경에 같은 색이 있으면 진다.

### 2.2 `clip-review`가 잡아낸 것 (fly_04, 실측)

```
COMMIT  frame 394 t=5.351s  tti 0.42s  sigma 12cm
  at y=0.08  pred x+0.68 z+1.02   real x+0.53 z+1.05   MISS 15.1cm
```

**오차 15 cm 중 z는 3 cm, x가 15 cm다.** 실제 x 궤적:

```
0.73 0.73 0.72 0.71 0.69 0.73 0.73 0.72 0.71 0.66 0.65 0.70 0.69 0.68 0.68 0.67 0.67 0.66
```

0.23 s에 0.73 → 0.66, 즉 **vx ≈ −0.3 m/s로 꾸준히 흐른다.** 그런데 커밋 예측의 x는 0.68에
멈춰 있다 — vx를 0으로 봤다.

못 잡는 이유: 커밋(f394)은 첫 검출(f385) **9프레임 뒤**다. 그 0.12 s 동안 x의 진짜 이동은
3.6 cm인데 **측정 노이즈가 ±2 cm**라 신호가 노이즈에 묻힌다.

**게이트가 이걸 못 거른다.** `impact_sigma`는 스칼라 하나(12 cm < 15 cm 통과)인데 그 값은
잘 관측되는 y축이 지배한다. x축 속도가 아직 쓰레기여도 통과한다. → **축별로 봐야 한다.**

그리고 **프레임 중복**: 연속 프레임에서 두 캠 검출 픽셀이 완전히 같은 경우가 **116/201
(58%)**. 공이 프레임당 ~20 px 움직이니 반올림 탓이 아니다. `meas_fps` 73.6은 부풀려진
값이고 실제 고유 샘플은 그 절반일 수 있다. 원인 미확정 — Stage 1에서 확인한다.

---

## 3. 지우는 것 / 남기는 것

**전부 지운다. 단 물리 커널은 건드리지 않는다.**

| 대상 | 처분 | 이유 |
|---|---|---|
| `src/detector/` 전체 (2,448줄) | 🗑 삭제 | §4로 새로 |
| `src/estimator/ekf.rs` (601) | 🗑 삭제 | 3D 관측 모델 자체가 틀렸다 |
| `src/estimator/decision.rs` (272) | 🗑 삭제 | 커밋 판단이 기구학으로 넘어갔다 |
| `src/estimator/triangulate.rs` + `tri/` (392) | ✂️ 축소 | **N-view DLT 하나만** 남긴다 (시드용). 지금은 2-view일 때 OpenCV `triangulate_points`로 빠지는 특수 분기가 따로 있는데, 실패하면 어차피 같은 DLT로 폴백한다 — 분기를 지우면 코드가 줄고 3대 이상이 공짜로 된다. `Calibration::min_cameras_for_triangulation()`의 하드코딩 `2`도 같이 푼다 |
| `src/estimator/prediction.rs`, `estimator.rs`, `hit_plane.rs`, `snapshot.rs` | 🗑 삭제 | 계약이 `Trajectory`로 대체 |
| `src/real/estimator_worker.rs` | 🗑 삭제 | `fuse`/skew/보간 전부 소멸 |
| `src/pipeline/` | 🗑 삭제 | 동기 로직의 두 번째 사본 |
| **`ballistics.rs`·`bounce.rs`·`kinematics.rs`** (540) | ✅ **유지** | ⚠️ **sim이 6곳에서 쓰고 회귀 테스트가 핀으로 박아뒀다** (`tests/bounce_kernel_matches_sim.rs`). 건드리면 sim이 깨진다 |
| `impact.rs`·`measure/` | ✅ 유지 | 리턴 파워·물리 측정 — 비전 범위 밖 |

> ✅ 완료 (`8c2b1b1`) — `camera::Pixel`을 `nalgebra::Point2<f64>` 별칭으로. 21줄 삭제.

---

## 4. 새 틀

### 4.0 한눈에

**층이 셋, 계약이 하나다.** 나머지는 그 안에서 자기 일만 한다.

```text
              ┌──────────────────────────────────────────────┐
   Frame ────▶│ 검출   픽셀을 줄여 후보를 낸다                │────▶ Vec<Candidate>
              └──────────────────────────────────────────────┘
              ┌──────────────────────────────────────────────┐
              │ 추정   후보를 골라 상태를 갱신한다            │────▶ State
              └──────────────────────────────────────────────┘
              ┌──────────────────────────────────────────────┐
              │ 계약   상태를 궤적으로 묶어 내보낸다          │────▶ Trajectory
              └──────────────────────────────────────────────┘
```

**소유 관계** — 위가 아래를 든다. 곁가지가 없다.

```text
Vision                          단일 진입점. feed(frame) -> Option<Trajectory>
├── cameras: Vec<Camera>        캠 수는 캘리브가 정한다
│   └── Camera                  id + params + detector. 자기 캘리브를 자기가 든다
│       └── Detector            layers를 순서대로 돌리고 후보를 뽑는다
│           ├── layers: Vec<Box<dyn Layer>>
│           │   ├── Background  정지한 것을 끈다
│           │   └── ColorGate   공 색이 아닌 것을 끈다
│           └── extract: ShapeGate   마스크 → Vec<Candidate>  (종단, Layer 아님)
└── tracker: Tracker            정책 — 언제 시드/선언/폐기
    └── filter: Filter          수학 — 픽셀 관측으로 [p,v] 갱신
```

**타입 전부** — 이게 다다.

| 타입 | 층 | 한 줄 |
|---|---|---|
| `State` | 계약 | `[x y z vx vy vz t]` 한 점 |
| `Trajectory` | 계약 | `measured` + `predicted`. **밖으로 나가는 유일한 타입** |
| `Vision` | 조립 | 진입점. 프레임 넣으면 계약이 나온다 |
| `Camera` | 조립 | id + params + detector |
| `Candidate` | 검출 | 한 프레임의 공 후보 (픽셀·반지름·점수) |
| `Layer` | 검출 | **트레잇.** 마스크를 줄인다. 순서 교체·추가가 자유 |
| `Background` | 검출 | `Layer` — 정지한 것 |
| `ColorGate` | 검출 | `Layer` — 색 판별면 |
| `ShapeGate` | 검출 | 종단. 마스크 → 후보 |
| `Detector` | 검출 | 위 넷을 조립 |
| `Filter` | 추정 | 픽셀 공간 EKF. 카메라도 페이즈도 모른다 |
| `Phase` | 추정 | `Idle` → `Tracking` → `Declared` |
| `Gate` | 추정 | 이번 관측을 받았나 (`Accepted`/`Rejected`/`Seeded`) |
| `Tracker` | 추정 | `Filter` + `Phase` + 폐기 정책 |
| `Trace` | 툴 | **본선은 안 만든다.** 단계별 마스크 + 그 프레임 판정 |

읽는 순서: **`Trajectory`(§1) → `Vision`(§4.1) → `Layer`(§4.2) → `Tracker`(§4.3).**
나머지는 그 넷에 딸린 것이라 필요할 때 보면 된다.

```
src/vision/
  mod.rs           Vision · Camera — 단일 진입점
  contract.rs      State · Trajectory           ← §1, 밖으로 나가는 유일한 타입
  detect/
    mod.rs         Layer 트레잇 · Detector — 캐스케이드 조립
    background.rs  Background : Layer — 정지한 것을 지운다
    color.rs       ColorGate  : Layer — 포함·제외로 피팅한 판별면
    shape.rs       ShapeGate         — 종단, 마스크 → Candidate
  track/
    mod.rs         Tracker — 정책: 생애주기 (Idle → Tracking → Declared) · 트랙 폐기
    filter.rs      Filter  — 수학: 픽셀 공간 EKF. 카메라도 페이즈도 모른다
    associate.rs   연관 · 시드
  trace.rs         Trace — 툴 전용 단계별 산출물
```

### 4.1 단일 진입점

```rust
/// 카메라 하나 — **자기 캘리브와 자기 검출기를 자기가 든다.**
///
/// `Calibration`을 상위가 통째로 들고 다니면 쓸 때마다 `params(id)`로 되찾아야 하고,
/// 그 조회가 매번 `Option`이라 "없을 리 없는데" 분기가 곳곳에 생긴다. 로드할 때 한 번
/// 나눠 주면 그 뒤로는 전부 있는 값이다.
pub struct Camera {
    pub id: camera::Id,
    pub params: camera::Params,
    detector: Detector,
}

/// 프레임을 먹이면 계약이 나온다. 이게 비전의 전부다.
pub struct Vision {
    /// **개수는 캘리브레이션 파일이 정한다** — 코드는 2대인지 3대인지 모른다.
    cameras: Vec<Camera>,
    tracker: Tracker,
}

impl Vision {
    /// 캘리브레이션을 카메라들에게 나눠 주고 끝 — `Vision`은 그걸 들고 있지 않는다.
    pub fn load(calibration: Calibration, color: &Path) -> Result<Self>;

    /// 프레임 하나. 선언된 트랙이 있으면 그 순간의 계약을 돌려준다.
    ///
    /// **프레임은 경계를 넘지 않는다** — 이미지는 여기서 끝나고 밖으로는 숫자만 나간다.
    pub fn feed(&mut self, frame: &Frame) -> Option<Trajectory>;

    /// 툴 전용. 본선 경로는 [`Trace`]를 **만들지 않는다** (비용 0).
    pub fn feed_traced(&mut self, frame: &Frame) -> (Option<Trajectory>, Trace);
}
```

### 4.2 검출 — 색을 후보 생성기로 강등

```rust
/// 한 프레임의 공 후보. **하나로 좁히지 않는다** — 최종 판정은 기하가 한다.
#[derive(Clone, Copy, Debug)]
pub struct Candidate {
    pub pixel: Pixel,
    pub radius_px: f64,
    /// 0..1. 높을수록 공 같다. 순위용이지 판정용이 아니다.
    pub score: f64,
}

/// 캐스케이드의 한 단계. **모든 단계가 같은 일을 한다 — 켜진 픽셀을 끈다.**
///
/// 이 프로젝트의 목적이 실험이라 단계를 바꿔 끼우고 순서를 뒤집으며 재는 일이 계속 생긴다.
/// 같은 트레잇을 먹이면 그게 조립 한 줄이고, `detect-full`이 단계 구성을 몰라도 패널을
/// 그릴 수 있다 ([`Layer::name`]으로 제목을 뽑는다).
///
/// # 불변식 — 줄이기만 한다
///
/// 켜진 픽셀을 **끄기만** 한다. 늘리면 뒤 단계의 비용 가정이 깨지고(각 단계는 "앞에서
/// 이미 대부분 껐다"를 전제로 짜인다) 순서를 바꿀 수 있다는 성질도 함께 무너진다.
pub trait Layer {
    /// 패널 제목·스윕 라벨. 짧게.
    fn name(&self) -> &'static str;

    /// `mask`를 줄인다.
    fn refine(&mut self, frame: &Frame, mask: &mut Mask);
}

pub struct Detector {
    /// **순서 = 실행 순서.** 싸고 잘 거르는 것부터 꽂는다.
    layers: Vec<Box<dyn Layer>>,
    /// 종단 — 마스크를 후보로. 이건 하는 일이 달라서 [`Layer`]가 아니다.
    extract: ShapeGate,
    /// 프레임마다 재할당하지 않는다 — 지금 구조가 느린 주범이었다.
    scratch: Scratch,
}

impl Detector {
    /// 점수 상위 K개 (K=4). 빈 벡터 = 못 찾음.
    pub fn detect(&mut self, frame: &Frame) -> Vec<Candidate>;
}
```

기본 조립 — 싸고 잘 거르는 것부터:

```rust
Detector::new(
    vec![
        Box::new(Background::new()),  // 정지한 것 전멸 + 태양광 적응. 다운스케일 2×
        Box::new(ColorGate::fit(..)), // 움직이는 것 중 공 색만. 남은 픽셀에만 돈다
    ],
    ShapeGate::from_calib(&params),   // 크기·원형도 → 상위 K개
)
```

순서를 바꾸거나 단계를 빼는 게 이 `vec!` 한 줄이라, "배경 모델 없이 색만"·"색 없이 배경만"
같은 비교가 코드 변경 없이 스윕으로 돈다.

#### `Background` — 두 문제를 동시에 푼다

```rust
/// 최근 N프레임 동안 안 변한 픽셀은 배경이다.
///
/// 떨어진 공·벽·바닥이 여기서 죽는다. 그리고 모델이 조명과 함께 **표류**하므로
/// 태양광 변동도 같이 흡수한다 — 정적 색 상자로는 원리상 불가능한 일이다.
///
/// 카메라가 흔들리면 무너지지만, 그건 캘리브레이션도 같이 무너지는 상황이라
/// 허용 가능한 실패 양식이다.
pub struct Background { /* 다운스케일 러닝 모델 */ }

impl Background {
    pub fn foreground(&mut self, frame: &Frame) -> &Mask;
}
```

#### `ColorGate` — 상자가 아니라 판별면

```rust
/// 포함·제외 샘플에서 피팅한 1차원 투영. `w · bgr + b > 0`.
///
/// 곱셈 3 + 덧셈 2라 `cvtColor`보다 싸다. "이 방에서 공과 배경을 가장 잘 가르는 축"을
/// 손으로 고르는 게 아니라 **데이터가 정한다** — 이게 "custom color space"의 실체다.
///
/// 배경 모델이 정지 confuser를 이미 죽였으므로 여기서 상대할 적은 **움직이는 것**
/// (피부·옷)뿐이다. 문제가 훨씬 작다.
pub struct ColorGate { w: [f64; 3], b: f64 }

impl ColorGate {
    /// `tune-colormask`가 만든 포함/제외 샘플로 피팅.
    pub fn fit(positive: &[[u8; 3]], negative: &[[u8; 3]]) -> Self;
    pub fn keep(&self, bgr: [u8; 3]) -> bool;
}
```

선형으로 안 갈리면 그때 타원체(Mahalanobis)로 올린다. **숫자가 요구할 때만** — 아니면 코드만 는다.

#### `ShapeGate` — 크기 사전확률

```rust
/// 캘리브에서 나온 기대 반지름과의 **편차**로 점수를 낸다.
///
/// 지금은 `area · circularity`라 가장 큰 블롭이 이긴다 — 그래서 팔이 공을 이겼다.
/// 기대 반지름은 깊이에 따라 변하므로 트랙이 있으면 그 깊이를 쓰고, 없으면 밴드를 쓴다.
pub struct ShapeGate { min_radius_px: f64, max_radius_px: f64 }

impl ShapeGate {
    pub fn candidates(&self, mask: &Mask, expect: Option<f64>) -> Vec<Candidate>;
}
```

### 4.3 추정 — 필터가 픽셀을 직접 먹는다

**삼각측량하고-필터 → 필터가 픽셀을 직접 받는다.** 이게 가장 큰 변경이다.

```rust
/// 상태 `[p, v]` 6차원. 관측은 **픽셀 2차원**이다.
///
/// 삼각측량하고 3D로 갱신하면 두 캠이 같은 순간에 같은 것을 잡아야만 한 번 갱신된다.
/// 픽셀을 직접 먹으면 프레임마다 갱신되고, 한쪽만 봐도 3 자유도 중 2개가 구속된다.
pub struct Filter { x: Vector6<f64>, p: Matrix6<f64> }

pub enum Gate { Accepted, Rejected { d2: f64 }, Seeded }

impl Filter {
    /// 야코비안은 핀홀 투영의 미분(2×6), `R = σ_px² · I₂`.
    ///
    /// `σ_px`는 **실측 가능한 진짜 픽셀 노이즈**(~1 px)다. 지금 `r_meas = 9e-4`는 실측
    /// 노이즈보다 5배 부풀려져 있는데, 주석이 정직하게 인정한다 — "R은 관측 노이즈가
    /// 아니라 필터가 모르는 전부다". 픽셀 공간에서는 R이 진짜 R이 되고 모델 오차는
    /// 있어야 할 곳인 Q로 간다.
    pub fn update_pixel(&mut self, cam: &camera::Params, pixel: Pixel, t: f64) -> Gate;

    pub fn state(&self, t: f64) -> State;

    /// 축별 σ. **스칼라 하나로 뭉치지 않는다** — §2.2에서 x축만 쓰레기인 걸
    /// 스칼라 `impact_sigma`가 가려서 15 cm를 빗나갔다.
    pub fn sigma(&self) -> [f64; 6];

    /// 공이 플레이 부피를 벗어날 때까지 굴린다. 물리는 `estimator::Kinematics` SSOT.
    pub fn rollout(&self, from: f64, step: f64) -> Vec<State>;
}
```

#### 연관 — 여기가 배경 FP를 죽인다

```rust
/// 트랙이 있을 때 — 예측을 그 카메라로 투영해 픽셀 공간 마할라노비스로 하나 고른다.
/// 게이트 밖이면 `None`, 그 프레임은 예측으로 넘어간다.
pub fn associate(f: &Filter, cam: &camera::Params, cs: &[Candidate], t: f64) -> Option<Candidate>;

/// 트랙이 없을 때 — 카메라 조합에서 물리적으로 말 되는 것 하나를 찾는다.
///
/// **카메라 수에 의존하지 않는다.** 모든 쌍을 삼각측량해 물리 게이트(비행 부피 안,
/// 속도 범위 안)를 태우고, 살아남은 것마다 *나머지 카메라 중 재투영이 맞는 개수*를 세서
/// 가장 많이 동의하는 것을 고른다. 2대면 쌍이 전부고, 3대면 셋째 캠이 표를 하나 더 준다.
/// 한 대가 가려도 나머지로 계속 선다.
///
/// **나쁜 시드보다 늦은 시드가 낫다.** 실패하면 조용히 다음 프레임에 재시도한다.
pub fn seed(views: &[(&camera::Params, &[Candidate])]) -> Option<State>;
```

배경에 주황이 있어도 **다른 캠에 에피폴라 정합 상대가 없고, 물리적으로 말 되는 궤적 위에도
없다.** 그래서 색 임계를 오히려 풀 수 있다 — 지금 막힌 그 거래(FP 5→382)가 여기서 열린다.

#### 카메라 대수는 설정이지 구조가 아니다

이 설계는 **N대에 무관하다.** 3대로 늘리는 게 재작성이 아니라 캘리브레이션 파일 한 줄이
되도록 처음부터 그렇게 짠다.

| | 카메라가 늘면 |
|---|---|
| `Vision::feed` | 프레임이 오는 대로 처리한다. 캠 수를 아예 모른다 |
| `Filter::update_pixel` | **바뀔 게 없다.** 관측 하나 = 픽셀 하나. 3대면 갱신이 1.5배 |
| `seed` | 조합이 늘고 표가 늘어 시드가 더 튼튼해진다 |
| N-view DLT | 시선이 늘수록 조건수가 좋아진다 |

지금 구조가 2대에 묶여 있던 건 **삼각측량-후-필터**였기 때문이다. 두 캠이 같은 순간에 같은
것을 잡아야만 한 번 갱신됐고, 그래서 캠이 늘어도 "짝 맞추기"만 더 어려워졌다. 픽셀 공간에서는
카메라가 서로를 기다리지 않으므로 **한 대 추가 = 측정 소스 하나 추가**, 그게 전부다.

#### `Tracker` — 정책. `Filter`는 수학만 한다

```rust
pub enum Phase {
    /// 트랙 없음. 시드를 시도한다.
    Idle,
    /// 추적 중, 아직 안 보낸다.
    Tracking,
    /// 네트를 넘었다 — 이제부터 매 프레임 [`Trajectory`]를 낸다.
    Declared,
}

/// 공 하나의 트랙 — 필터의 **생애주기와 정책**을 든다.
///
/// [`Filter`]는 순수 수학이다. 예측하고, 갱신하고, 야코비안을 만든다 — 카메라도 후보도
/// 페이즈도 모른다. 그래서 합성 상태만으로 테스트된다.
///
/// 언제 시드할지, 언제 선언할지, 언제 버릴지, 무엇을 쌓을지는 전부 여기다.
/// **지금 `ekf.rs`가 601줄인 건 그 둘을 한 파일에 섞었기 때문이다.**
pub struct Tracker {
    filter: Filter,
    phase: Phase,
    seq: u64,
    /// 관측이 있던 시각의 필터 상태 — 계약의 `measured`가 된다.
    measured: Vec<State>,
    /// 연속 거부 수. 한도를 넘으면 트랙을 버린다.
    rejects: u32,
    last_seen: Option<f64>,
}

impl Tracker {
    /// 카메라 하나의 후보를 먹인다 — 프레임마다 그 카메라 것만. **다른 캠을 안 기다린다.**
    ///
    /// 연관에 성공하면 픽셀로 필터를 갱신하고 `measured`에 한 점을 쌓는다.
    /// 실패하면 그 프레임은 예측으로 넘어간다 (트랙은 안 끊는다).
    pub fn observe(&mut self, cam: &Camera, candidates: &[Candidate], t: f64) -> Gate;

    /// [`Phase::Idle`]일 때만. 카메라 전체 후보에서 물리적으로 말 되는 시드를 찾는다.
    pub fn try_seed(&mut self, views: &[(&Camera, &[Candidate])], t: f64) -> bool;

    /// 지금 계약. 선언 전이면 `None`.
    pub fn trajectory(&self, now: f64) -> Option<Trajectory>;

    pub fn phase(&self) -> Phase;
}
```

**트랙을 버리는 조건** — 전부 `Tracker`가 판정한다. 하나라도 걸리면 `Idle`로 돌아가고
`seq`가 하나 오른다.

| 조건 | 뜻 |
|---|---|
| 연속 거부 한도 초과 | 필터가 틀렸다 — 예측이 관측과 계속 안 맞는다 |
| 관측 공백 (stale) | 공을 잃었다 |
| `y`가 다시 증가 | 공이 멀어진다 — 다음 샷이다 |
| 플레이 부피 이탈 | 끝났다 |

#### 시드만 다른 캠을 기다린다

**갱신은 안 기다린다** — 관측 하나가 픽셀 하나라 프레임이 오는 대로 필터에 들어간다.
그런데 **시드는 최소 두 시선이 필요하다.** 프레임은 한 대씩 오므로 [`Vision`]이 카메라별
최근 후보를 짧은 TTL로 들고 있다가 [`Tracker::try_seed`]에 넘긴다.

```rust
impl Vision {
    pub fn feed(&mut self, frame: &Frame) -> Option<Trajectory> {
        let t = self.elapsed(frame.timestamp);
        let cam = self.camera_mut(frame.camera_id)?;
        let candidates = cam.detector.detect(frame);

        match self.tracker.phase() {
            // 시드 전에만 캠별 최근 후보를 모은다. 트랙이 서면 이 버퍼는 안 쓴다.
            Phase::Idle => {
                self.pending.put(cam.id, &candidates, t);
                self.tracker.try_seed(&self.pending.views(t), t);
            }
            _ => {
                self.tracker.observe(cam, &candidates, t);
            }
        }
        return self.tracker.trajectory(t);
    }
}
```

이게 지금 구조의 `MAX_SYNC_LAG`·보간과 다른 점: **한 번만, 그것도 트랙이 없을 때만**
기다린다. 지금은 매 프레임 짝을 맞추느라 `stale_skipped`가 220~610이었다.

| | 언제 | 왜 |
|---|---|---|
| **추적 시작** | 타당한 시드가 서는 즉시 | 측정을 최대한 모은다 |
| **선언 (`Declared`)** | 네트 평면(`y = LENGTH_Y/2`)을 로봇 쪽으로 타당한 속도로 통과 | 계약의 기준점 |

**네트를 추적 시작으로 잡으면 안 된다.** 5 m/s 공이 네트에서 로봇까지 1.37 m를 가는 데
0.27 s뿐이라 필터가 속도를 수렴시킬 시간이 그것밖에 안 남는다. 계속 추적하다가 네트에서
선언하면 넘기는 순간엔 이미 속도가 잡혀 있다.

지금은 첫 삼각측량이 아무 때나 시드해서 오검출로도 트랙이 선다. 선언 기준이 그걸 막는다.

### 4.4 툴이 보는 것

```rust
/// 단계별 산출물 — `detect-full`·`clip-review` 전용.
///
/// 본선 경로(`Vision::feed`)는 이걸 **만들지 않는다.** 진단이 공짜여야 켜 두는데,
/// 공짜가 아니면 아무도 안 켜고 결국 아무도 진단을 안 한다.
pub struct Trace {
    /// 단계마다 `(이름, 그 단계 직후 마스크)`. **단계 구성이 바뀌면 패널도 같이 바뀐다** —
    /// 툴이 단계를 하드코딩하지 않으므로 새 레이어를 꽂아도 뷰어를 안 고친다.
    pub stages: Vec<(&'static str, Mask)>,
    pub candidates: Vec<Candidate>,
    pub chosen: Option<Candidate>,
    pub gate: Gate,
    /// 축별 σ — 스칼라로 뭉치면 §2.2를 또 놓친다.
    pub sigma: [f64; 6],
    pub phase: Phase,
}
```

---

## 5. 검증 — 툴 두 개로만 한다

새 코드는 **[`detect-full`](../tools/detect_full/)과 [`clip-review`](../tools/clip_review/)를
통과해야 다음 단계로 간다.** 별도 하네스를 새로 만들지 않는다.

| 툴 | 무엇을 본다 | 어디를 잡는다 |
|---|---|---|
| [`detect-full`](../tools/detect_full/README.md) | 단계별 패널 (`Trace`), 후보 K개, ms/frame | **검출** — 뭘 놓치고 뭘 잘못 무는가 |
| [`clip-review`](../tools/clip_review/README.md) | 측정 궤적 vs 예측 궤적, 0.1x 스크럽, 콘솔 프레임 로그 | **추정** — 예측이 실제로 수렴하는가 |

```bash
# 검출 — 캠 하나, 단계별 패널
cargo run --release -p detect-full -- --cam left --clip fly_04

# 추정 — 두 캠 + sim, 궤적 두 개 겹쳐 보기
cargo run --release -p clip-review -- --clip fly_04

# 물리 커널 회귀 (지우면 안 되는 것)
cargo test --release --test bounce_kernel_matches_sim

# 코드 줄 수 — 늘면 그 단계는 실패
tokei src/vision src/detector src/estimator
```

`clip-review`는 이미 실기와 **같은 커밋 게이트**를 태워 `MISS`를 cm로 낸다. 새 구조가
그 숫자를 낮추지 못하면 안 고쳐진 것이다.

> ⚠️ 새 구조로 갈아탈 때 두 툴의 어댑터를 같이 고쳐야 한다. 툴이 먼저 죽으면 눈이
> 없는 채로 개발하게 된다 — **툴을 먼저 새 타입에 맞춘 뒤** 본선을 바꾼다.

---

## 6. 단계

**규칙:** 한 단계 = 한 커밋. 커밋 본문에 측정표. 매 단계 줄 수를 §8에 기록한다.
**한 단계 끝나면 멈추고 실기 테스트를 요청한다** — 측정 없이 다음 레버로 넘어가면 무엇이
효과였는지 분리할 수 없다 (`2026-07-27-return-power.md`에서 확립).

| # | 단계 | 내용 | 완료 기준 |
|---|---|---|---|
| 0 | **baseline 박기** | 9개 클립을 `clip-review`로 돌려 `MISS`·검출률을 §8에 기록 | 표가 §8에 박힘 |
| 1 | **캡처 정상화** | 모노/컬러 모순 확인, `request_short_exposure` 호출, **프레임 중복 원인 확정**, 실제 120 fps | `meas_fps`가 고유 프레임 수와 일치 |
| 2 | **`contract.rs` + 툴 어댑터** | `State`/`Trajectory` 먼저 만들고 두 툴을 여기에 맞춘다 | 툴이 새 타입으로 돈다 |
| 3 | **`detect/` 새로** | Background → ColorGate → ShapeGate → 후보 K개 | `detect-full`에서 떨어진 공·팔이 죽는다 |
| 4 | **`track/` 새로** | 픽셀 EKF + 연관 + 생애주기 | `clip-review` `MISS` 감소, 측정 수 증가 |
| 5 | **레이턴시** | 셔터→계약 실측, `origin` 오프셋 보정 | 구간별 예산표 |
| 6 | **슈터 캘리브 + 스핀** | 눈금 → 실제 `v0`·`ω`. **ω를 아는 상태로 예측이 맞는지 먼저** 본 뒤 상태 확장 | 클립마다 진짜 GT |
| 7 | **카메라 재배치·증설** | 2대 유지가 기본. §4.3대로 짜 두면 3대는 캘리브 파일 변경이지 재작성이 아니다 | 격자점 조건수 지도로 배치 비교 |

**후순위:** 반발계수·마찰계수 위치별 정밀 계산 — 스핀보다 명백히 2차다.

### Stage 1 상세 (가장 먼저)

1. **모노/컬러 모순.** 데이터시트 상수는 OV9281을 **모노크롬 글로벌 셔터**라고 하는데
   `data/colormask.json`에는 명백히 유채색인 BGR 샘플 2,078개가 있다. 둘 중 하나가 틀렸다.
   실기에서 5분. **모노면 `ColorGate` 자체가 통째로 바뀐다 — 가장 먼저 확인한다.**
2. **프레임 중복 (58%).** AVI를 바이트 단위로 비교해 확정한다. 원인이 `record_stereo`의
   인덱스 짝짓기라면 거기서 고친다. 이게 안 고쳐지면 유효 샘플 수가 절반이라 뒤 단계가
   전부 헛돈다.
3. `request_short_exposure()`는 **아무도 호출하지 않는다.** 함수만 있다.
4. 실측 fps가 120에 못 미치면 해상도를 낮춘다 — `960×600`/`640×400` 프리셋이 이미 있다.
   **`meas_fps`만 믿는다** (드라이버 `CAP_PROP_FPS`는 거짓말한다).

---

## 7. 비범위

- 기구학·IK·로봇 제어 — 다른 담당
- 신경망 검출기 — GPU 없음
- 하드웨어 genlock — §4.3이 소프트웨어로 무력화한다 (카메라가 서로를 안 기다린다)
- 물리 커널(`ballistics`/`bounce`/`kinematics`) 변경 — sim과 공유 SSOT
- ChArUco 완전 캘리브레이션 — 현재 인트린식은 데이터시트 FOV 근사(`fx=fy=913.39`,
  `cx/cy` 중앙 고정, `dist=[]`)이고 cam0 rmse가 4.15 px로 cam1(1.35)의 3배다.
  Stage 4 후에도 cam0 잔차가 남으면 그때 최우선 후속

---

## 8. 측정 기록

> 단계마다 여기에 붙인다.

| 단계 | 날짜 | 측정 | 줄 수 | 결론 |
|---|---|---|---|---|
| baseline | 2026-08-01 | fly_04: 검출 44/589, `MISS` 15.1 cm (x축 15 / z축 3), 프레임 중복 58% | detector 2448 + estimator 2758 | Stage 0에서 9개 클립 전부 채울 것 |
