# 레일 프레임 실측 반영 + 런타임 마운트 조정

2026-07-30

## 문제

`defaults::rail_frame()`이 가정하는 베이스 높이가 실물과 12.5cm 다르다.

| 항목 | 코드 현재 | 실측 |
| --- | --- | --- |
| 베이스 높이 (바닥 기준) | 0.81 (`SURFACE_Z + 0.05`) | **0.935** |
| 레일 프로파일 두께 | 0.04 (`RAIL_VISUAL_HEIGHT`, 시각화 전용) | **0.055** |
| 바닥 → 프로파일 하단 | (표현 없음) | **0.88** |

`above_table: 0.05`는 "실기 브래킷(~면 위 3~5cm)과 맞춤"이라는 근거로 들어간 값인데, 실측이 그 가정을 뒤집었다.

동시에 마운트 위치 탐색 방식 자체가 불편하다. 현재 `behind_table_end`를 바꾸려면 상수를 고치고 재컴파일해야 한다. 실물에서는 레일을 밀면 되는 쉬운 조정이고, 시뮬에서도 같은 난이도여야 탐색이 돌아간다.

## 제약

- **레일 두께 0.055는 고정.** 프로파일이 이미 제작돼 있다.
- **바닥→하단 높이와 끝면→레일 거리는 조정 가능.** 둘 다 실물에서 바꿀 수 있다.
- **마운트 조정은 공이 주차(`BallState::Parked`)된 동안만.** 비행 중이면 비활성화.
- 마운트가 움직이면 팔은 관절각을 유지한 채 강체로 따라온다 — 실물에서 레일을 민 것과 같은 결과.

## 설계

### 1. 기하 모델

```rust
/// 레일 프로파일 두께 [m] — 실측, 고정.
pub const RAIL_THICKNESS: f64 = 0.055;

pub struct RailFrame {
    /// 레일 마운트 y [m] — 탁구대 끝면(y=0) 기준, 테이블 밖이면 음수. 조정 가능.
    pub mount_y: f64,
    /// 바닥(z=0) → 레일 프로파일 하단 [m]. 조정 가능.
    pub rail_bottom_z: f64,
}

impl RailFrame {
    pub fn mount_y(self) -> f64 { self.mount_y }
    pub fn mount_z(self) -> f64 { self.rail_bottom_z + RAIL_THICKNESS }
}
```

기본값 `{ mount_y: -0.10, rail_bottom_z: 0.88 }` → 베이스 `(y, z) = (−0.10, 0.935)`.

세 가지가 바뀐다.

**높이 기준이 테이블 면에서 바닥으로 옮겨간다.** 줄자로 재는 값이 바닥 기준이고, 월드 z=0이 이미 바닥이라 `table::SURFACE_Z`를 경유할 이유가 없다. 레일은 탁구대 면 높이를 알 필요가 없는 물건이다. 이 변경으로 `RailFrame`의 `table` 모듈 의존이 사라진다.

**두께가 상수로 분리된다.** 조정 가능한 축(`rail_bottom_z`)과 못 바꾸는 축(`RAIL_THICKNESS`)이 타입 수준에서 갈려, 슬라이더가 실물에서 못 하는 조정을 노출하지 않는다.

**y도 월드 좌표로 통일한다.** 처음에는 `behind_table_end`(뒤로 갈수록 양수)로 뒀는데, `rail_bottom_z`는 월드 절대 좌표라 한 구조체 안에서 부호 관례가 어긋났다. 슬라이더에 `y`라고 적으면서 화면에 0.300(월드 −0.300)을 띄우게 되는 것도 문제였다. 두 필드를 같은 좌표계로 두면 파생 좌표를 따로 표시할 필요가 없어진다.

`RAIL_VISUAL_HEIGHT = 0.04`는 삭제하고 `RAIL_THICKNESS`로 대체한다 — 실측이 있으니 장식일 이유가 없다. `RAIL_VISUAL_WIDTH = 0.06`은 단면 폭 실측이 없어 장식으로 유지한다.

### 2. 런타임 조정

**상태 보관.** `SimRuntimeControls`에 `rail_frame: RailFrame`을 추가한다. `shooter: launch::Settings`와 같은 층위 — GUI가 매 프레임 쓰고, 물리 스레드가 읽는 슬라이더 상태.

**전달.** `SimStepInput`에 `rail_frame: RailFrame`을 추가한다. `session.rs:110-127`이 컨트롤 락 안에서 값을 스냅샷하고 락을 놓은 뒤 `step()`에 넘기는 기존 패턴을 그대로 쓴다 — 물리 연산 동안 뮤텍스를 잡지 않는다는 `SimStepInput`의 목적과 일치한다.

**적용.** `step()`이 `input.park`을 처리하는 지점(`world.rs:441` 근처)에서 `input.rail_frame`과 현재 `arm.rail`의 마운트를 비교한다. 다르고 `ball_state == BallState::Parked`이면:

1. `Arc::make_mut(&mut self.arm)`으로 `base.coords.y/z`와 `rail.mount_y/mount_z`를 갱신
2. `sync_robot_bodies_to_state()` 한 번 호출 — rapier 베이스와 자식 링크를 즉시 재배치

`BallState::InFlight`면 비교 결과를 버린다. 관절각은 손대지 않으므로 팔은 마운트와 함께 평행이동한다.

`step()`의 인자는 `Option<SimStepInput>`이고, 입력이 `None`인 경로(테스트·오프라인 하네스)는 마운트를 건드리지 않는다 — 기존 동작 유지.

**Eval이 자동으로 따라온다.** `start_eval_protocol`은 `Arc::clone(&world_guard.arm)`으로 라이브 월드의 팔을 그대로 복제해 평가 스레드에 넘긴다 (`panel.rs:590-593`). 마운트 변경이 `world.arm`에 들어가므로 슬라이더를 조정한 뒤 "Run 30"을 누르면 조정된 마운트가 채점된다. 별도 배선이 필요 없고, 마운트를 Eval 창이 아니라 전역 설정으로 두는 판단과도 맞아떨어진다. `Robot`을 재조립하거나 별도 config를 두는 대안에서는 이 연결이 공짜로 오지 않는다.

**왜 이것만으로 충분한가.** `effective_sim_mount()`가 `arm.rail.mount_y/mount_z`를 매 프레임 읽어 `set_base_xy(x, y, z)`로 넘긴다 (`world.rs:1194-1215`, `1218-1224`). 레일 x 추종용으로 이미 존재하는 경로가 y/z도 그대로 받는다. 물리 쪽 신규 배선이 없다.

**왜 Arc 교체나 URDF 재조립이 아닌가.** 마운트는 `Arm.base`와 `Arm.rail`에만 들어가고 FK는 전부 `mount_at_rail()`을 거친다 (`arm.rs:574-579`). 링크 길이·관성·한계는 마운트와 무관하므로 URDF를 다시 읽을 이유가 없다. `pipeline::Config`의 `Arc<Robot>`을 갈아끼우면 실기 경로까지 영향권에 들어오는데, 이 기능은 sim 탐색 도구다.

### 3. GUI — "Rig" 창

현재 레이아웃은 좌측 `Shooter`→`Eval`, 우측 `Status`→`View` 4창이다. `Rig`를 **좌측 `Shooter` 바로 아래**에 넣어 좌측을 `Shooter`→`Rig`→`Eval`로 만든다.

처음에는 우측 `View` 아래에 뒀다 — "좌측은 공을 쏘는 것, 우측은 장비·보기 설정"으로 읽었기 때문이다. 그 구분이 틀렸다. `Shooter`와 `Rig`는 둘 다 **리그를 어디에 놓았나**(슈터 위치 / 로봇 마운트 위치)이고, 우측은 읽기 전용 상태(`Status`)와 보기 설정(`View`)이다. 조정 손잡이는 한쪽에 모여 있어야 한다.

Eval 창에 넣지 않는 이유는 그대로다 — 마운트는 평가 프로토콜 파라미터가 아니라 리그 전역 설정이다.

내용:

- 슬라이더 `y` [m], 범위 −0.30..0.05
- 슬라이더 `레일 하단 z` [m], 범위 0.70..1.10
- 파생값 한 줄: `면 위 +0.175 m`
- 기본값 복귀 버튼 (Parked이고 기본값이 아닐 때만 활성)
- `ball_state != Parked`면 `add_enabled_ui(false)` + "공 주차 후 조정 가능" 한 줄

두 슬라이더가 월드 좌표를 그대로 보여주므로 `base y/z` 파생 표시는 두지 않는다. 두께도 슬라이더로 못 바꾸는 상수라 패널에 띄우지 않는다. "면 위"만 예외로 남긴다 — 도달 범위를 감각적으로 판단할 때 쓰는 기준은 바닥이 아니라 탁구대 면 대비 높이다.

초안에는 "기본 마운트가 아니면 `READY_JOINTS_4DOF` 미재산출" 배지와 설명 두 줄이 있었는데 뺐다. 슬라이더를 돌리는 중에 필요한 정보가 아니고, 재산출은 별도 담당의 오프라인 작업이라 상수 주석에 적는 것이 맞다.

`PanelUiState`에 `rail_frame: RailFrame` 필드를 추가하고 `from_controls`에서 초기화한다. `draw`가 매 프레임 `ctrl.rail_frame = ui_state.rail_frame`으로 넘긴다 — `shooter`·`time_scale`과 같은 패턴.

`ball_state`는 이미 `StatusSnapshot`으로 패널에 올라온다 (`status_snapshot.rs:12`). 게이트에 새 배선이 필요 없다.

### 4. SSOT 우회 정리

런타임 변경이 들어오면 `defaults::rail_frame()`을 직접 읽는 지점이 stale 버그가 된다. 두 곳.

**`sync_rail_stroke` (`scene_dynamics.rs:550-579`).** 바로 위에서 `world.arm().rail`을 꺼내놓고 y/z만 `rail_frame()`에서 다시 읽는다. `rail.mount_y`/`rail.mount_z`로 바꾼다.

**뷰어 URDF 메시는 손댈 필요가 없다.** `urdf_nodes.rs:42-44`가 `world.effective_sim_mount()`를 `link_poses_with_mount`에 넘겨 그린다 — `urdf.mount`의 position은 배치에 쓰이지 않는다. 구현 중에 `Arc::make_mut(urdf).mount.position`을 갱신하는 코드를 넣었다가 제거했다: 아무것도 바꾸지 않으면서 첫 슬라이더 조작에 URDF 모델(메시 포함) 전체를 깊은 복사한다.

**레일 프로파일 큐브 (`scene/mod.rs:87-97`).** `build_table_scene`에서 한 번 배치되고 끝이다. 핸들을 `SceneDynamics`로 올려 매 프레임 `arm.rail` 기준으로 동기화한다. 큐브 높이는 `RAIL_THICKNESS`, 중심 z는 `rail_bottom_z + RAIL_THICKNESS * 0.5`.

### 5. 마운트 스윕 도구 인터페이스

높이 기준이 바닥으로 옮겨지면서 `SimRobotMount::rep103_z_up_at_table_end_with_mount(base_y, height_offset_m)`의 두 번째 인자 의미가 어긋난다 (현재 `SURFACE_Z + height_offset`). `primitive_4dof_with_mount(mount_y, mount_z)`는 이미 절대 z를 받으므로 영향 없다.

`rep103_z_up_at_table_end_with_mount`를 `(base_y, base_z)` 절대 좌표로 바꿔 `primitive_4dof_with_mount`와 관례를 통일한다. 호출자는 `tools/mount_search`류 스윕 도구뿐이고, 스윕 범위 지정도 절대 z가 더 읽기 쉽다.

## 테스트

- `rail_frame()` 기본값이 `mount_z() == 0.935`, `mount_y() == -0.10`을 낸다 (기존 `rail_frame_mounts_behind_and_above_table` 갱신)
- `RailFrame { rail_bottom_z: h }`의 `mount_z()`가 `h + RAIL_THICKNESS`다 — 두께가 상수로 더해지는지
- Parked 상태에서 `controls.rail_frame`을 바꾸고 스텝하면 `world.arm().rail.mount_z`와 `effective_sim_mount().position[2]`가 따라온다
- InFlight 상태에서 같은 변경이 무시된다 — `arm.rail.mount_z`가 그대로
- 마운트 변경 후 관절각이 불변이고 EE 위치가 마운트 이동량만큼만 움직인다 (강체 평행이동 회귀). `mount.rs:143-191`의 기존 테스트와 같은 성격
- `sync_rail_stroke`가 `defaults::rail_frame()`이 아니라 `arm.rail`을 따른다 — 기본값과 다른 마운트를 넣고 마커 위치 확인
- 마운트를 바꾼 뒤 `start_eval_protocol`이 복제하는 `Robot`의 `arm.rail`이 바뀐 값이다 — Eval이 조정된 마운트를 채점하는지

## 범위 밖

**레일 몸체 충돌체.** 두께를 알게 됐으니 레일이 z∈[0.88, 0.935] 부피를 점유하는 것은 표현 가능해졌지만, `collision.rs`는 "테이블 면 관통"만 보는 모듈이고 팔이 레일에 부딪힌 근거가 아직 없다. 별건.

**`mount_y`·`READY_JOINTS_4DOF` 재산출 — 스윙 튜닝 담당 인계.** 이번 작업은 파라미터와 탐색 도구까지다. 값 확정은 그쪽 영역이라 손대지 않고, 조사 중 측정한 수치만 상수 주석에 남겼다.

`READY_JOINTS_4DOF`를 새 마운트에서 재산출하면 `[0.8612, 0.0, 0.1889, -1.2076]`이고 최악 Δq가 1.282→0.767 rad(필요시간 0.835→0.499s)로 줄어든다. 다만 값을 바꾸면 `JOINT_EFFECTIVE_INERTIA_4DOF` 재측정과 `robot::tests::default_arm_produces_racket_pose` 단정문 갱신(재산출 값에서는 라켓이 베이스보다 아래로 내려온다 — 임팩트 대역에 가까워지므로 정상)이 함께 따라온다. 그래서 이번 작업에는 넣지 않았다.

**`world.rs`의 `#[ignore]` 스윙 테스트 복구.** "rail_frame mount needs shot_tune retune"으로 이미 10개가 대기 중이었고, 베이스가 12.5cm 올라가 **두 개가 추가로 들어갔다**.

| 테스트 | 증상 | 휴지 자세 재산출로 복구되나 |
| --- | --- | --- |
| `auto_swing_plans_with_strike_velocity` | 임팩트 궤적이 관절속도 한계 초과 | **된다** (재산출 값으로 통과 확인) |
| `random_shot_grid_still_swings_when_robot_starts_from_center` | off-center 샷이 포기 후 접수 | 안 된다 — `mount_y` 재스윕 필요 |

둘 다 `rail_bottom_z`를 0.755(= 옛 베이스 z 0.81)로 되돌리면 통과하므로 원인은 마운트 높이 하나이고, 아래 "알려진 위험"이 실현된 것이다. 실측이 실물이므로 값을 되돌리지 않고 `#[ignore]`에 인계 문구를 달았다.

**`bang_bang_swing_planning_does_not_block_physics_step`의 판정 방식.** 조사 중 이 테스트가 두 가지로 공허하다는 것을 발견해 코드에 `NOTE`로 남겼다(고치지 않음 — 스윙 튜닝 영역). ① 루프가 4500스텝을 실시간보다 훨씬 빠르게 돌아서, debug 빌드에서 수백 ms 걸리는 워커 계획이 끝나기 전에 공이 착지한다 — 계획 시간이 측정 구간에 애초에 들어오지 않는다(계획 소비 0회로 직접 확인). ② wall-clock은 측정 스레드 스케줄링에 좌우돼 테스트 병렬 실행 부하에 흔들린다(단독 실행 최악 0.62ms, 전체 병렬 실행 최악 85ms). 구조적 대안은 워커가 `is_busy`인 스텝이 존재하는지 세는 것 — 부하와 무관하게 "동기 호출이 아님"을 직접 단정한다.

## 알려진 위험

베이스가 12.5cm 올라가면 낮은 공에 대한 도달성이 나빠질 수 있다. `mount_search`가 height 0.05에서 최적을 찾았던 것은 그 방향이 유리했다는 뜻이기도 하다. 실측이 0.935이므로 시뮬을 실측에 맞추는 것이 맞고, 슬라이더는 정확히 이 영향을 눈으로 확인하기 위한 도구다.

맞춘 직후 스윙 성공률이 눈에 띄게 떨어져도 그것은 버그가 아니라, 그동안 시뮬이 실물보다 유리한 마운트를 가정하고 있었다는 발견이다.
