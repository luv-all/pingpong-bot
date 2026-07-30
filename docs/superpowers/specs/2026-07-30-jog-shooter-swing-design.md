# jog 스윙을 슈터 기반으로 교체

날짜: 2026-07-30

## 문제

jog의 스윙 커맨드는 도달 위치(`arrival_xyz`)와 도달 시 입사 속도(`ball_vin`)를
사용자가 직접 준다. 두 값은 서로 독립인 슬라이더라서, 이론적으로 가능한 모든
조합이 입력된다 — 실제로는 그런 탄도로 날아올 수 없는 공이 대부분이다. 그래서
IK/임팩트 역산이 거의 항상 실패하고, 스윙을 테스트하기 어렵다.

해결: 입력을 **슈터 파라미터**로 바꾼다. 슈터 위치·조준각·초기 속도를 주면
도달점과 입사 속도는 탄도 예측에서 나온다. 물리적으로 불가능한 조합은 애초에
입력될 수 없다.

## 범위

- jog 모션 종류를 `Joint / Angles / RailAbs / Ik / Pose / Swing` 6개로 정리.
  기존 `Swing`(라켓 상대이동 + 속도 직접) · `AimBall` · `SwingBall` 3개를
  슈터 기반 `Swing` 하나로 대체한다.
- 슈터 패널 위젯과 슈터 3D Visual을 공용 추출해 메인 sim과 jog가 공유한다.

명시적 비범위: 공을 실제로 발사해 임팩트 시각에 맞춰 스윙을 자동 트리거하는
타이밍 동기화. jog는 **예측만** 한다. (Shoot 버튼 자체는 슈터 위젯에 있으므로
시각 확인용으로 쓸 수 있지만, 스윙 재생과 동기화되지 않는다.)

## 슈터 기하 — 변경하지 않음

당초 실측 슈터 좌표(발사구 `(WIDTH_X/2, LENGTH_Y−0.275, 면+0.225)`, pitch +15°,
speed 7.5)를 기본값으로 넣으려 했으나 **채택하지 않았다.**

넣어보니 `cargo test`가 9건 깨졌고, 원인은 하나로 수렴했다 — 발사구를 면+46 cm
에서 면+22.5 cm로 24 cm 낮추면 "로봇 코트에 1회 바운스하고 라켓이 들어갈 높이로
오는" 공을 만들 수 없다. `Kinematics::predict_to` 실측:

| pitch / speed | 테이블 바운스 | 접수 평면 도달 높이 |
|---|---|---|
| 15° / 7.5 | 0회 — 테이블을 그대로 넘어감 | 면+0.33 |
| 15° / 5.0 | 1회 (y=0.65) | 면+0.22 |
| 10° / 5.5 | 1회 (y=0.71) | 면+0.19 |
| 10° / 6.0 | 1회 (y=0.48) | 면+0.15 |

바운스하는 대역은 도달 높이가 전부 면+15~22 cm로, 기존 검증값 면+0.31보다
10 cm 이상 낮다. 이 높이에서는 커밋 파이프라인이 스윙을 걸지 못했다 — eval
30발 전부 미타격(`counts=[30,0,0,0]`). 계측해 보니 예측과 레일 코스 추종은
정상인데(레일 0.705 → 0.32로 공을 정확히 따라감) `swing_committed`가 끝까지
false였다. 마운트를 27.5 cm 앞으로 당긴 것도 별개로 해가 됐다 — 리드 시간이
줄어 커밋 창을 못 맞춘다.

실측 기하를 쓰려면 커밋 창·리드 시간 튜닝을 새 피딩에 맞춰 손봐야 하는데,
그건 이 작업(jog 스윙 입력 교체)의 범위 밖이다. **`src/` 슈터 기하·기본값은
손대지 않았다.** jog는 슈터 파라미터를 UI로 조절하므로 기본값이 무엇이든
동작한다.

후속 과제: 실측 기하(낮고 가까운 발사구)에서 로봇이 공을 받도록 커밋 창·리드
시간을 재튜닝.

## 예측 경로

```
launch::Settings
  → muzzle_position() / launch_velocity() / launch_angular_velocity()
  → Kinematics::predict_to(HitPlane { y }, PhysicsParams::default())
  → Prediction { time_to_impact_secs, impact_position, incoming_velocity }
```

`predict_hit_plane`은 테이블 바운스를 포함해 적분하고, 네트 미달·테이블 구름·
너무 낮거나 높은 도달·리드 시간 범위 밖이면 `None`을 준다. 실제 파이프라인이
쓰는 예측기 그대로다.

`Kind::Swing` 조합은 기존 `swing_ball_traj`의 계산을 그대로 쓰되 입력만 예측에서
온다:

1. `pred = predict(draft.shooter, draft.hit_plane_y)` — `None`이면 "접수 불가"
2. 목표 = `pred.impact_position`, `v_in = pred.incoming_velocity`
3. 기준 법선 = `−v_in.normalize()`, 여기에 면 기울기 pitch/yaw 적용
4. `inverse_pose_with_rail` → 임팩트 포즈
5. 임팩트 포즈 FK → 실제 법선
6. `Impact::rally_return` → `v_out`, `Impact::required_racket_velocity` → `v_r`
7. `velocities_for_racket_velocity` → 관절·레일 속도
8. `ensure_max_delta` → `build_follow_through_swing`

## Draft 변경

| 제거 | 추가 | 유지 |
|---|---|---|
| `swing_speed` | `shooter: launch::Settings` | `tilt_pitch_deg`, `tilt_yaw_deg` |
| `arrival_xyz` | `hit_plane_y: f64` | `reach_dxyz` (Ik/Pose 전용) |
| `ball_vin` | | `joint_*`, `angles_deg`, `rail_x` |

`hit_plane_y` 범위는 `InterceptWindow::default()`의 `[y_min, y_max]`, 기본값은
`table::DEFAULT_HIT_PLANE_Y`.

## UI

### 공용 슈터 위젯

`src/sim/gui/viewer/panel.rs`의 Shooter 창 본문을
`sim::gui::shooter::ui::draw(ui, &mut Settings) -> Buttons { shoot, random, park }`
로 추출한다. 메인 sim 패널과 jog가 같은 위젯을 쓴다 — 파라미터 목록·범위가
한 곳에서만 정의된다.

노출 파라미터는 메인 sim과 동일한 풀세트: 마운트 오프셋 xyz · yaw/pitch/roll ·
lateral/height · speed · topspin/sidespin/drill, 그리고 Shoot / Random / Park.

### jog 패널

- 별도 "슈터" egui 창에 공용 위젯을 띄운다. 값이 바뀌면 `controls.shooter`에
  push → 월드 슈터 자세·비주얼이 갱신된다.
- Jog 창의 Swing 섹션: 접수평면 y, 면 기울기 pitch/yaw, 예측 결과 표시.
  - 도달점 (x, y, z)
  - 입사 속도 (vx, vy, vz)
  - 리드 시간
  - 접수 가능 / 불가 (+ 사유)
  - IK 가능 / 불가
- 고스트 공이 예측 도달점과 입사속도 화살표를 표시한다 (`sync_arrival_ghost`
  대체). Swing 이외 모션에서는 숨긴다.

Random은 `Settings::randomized()`를 슬라이더에 반영한다. `randomized()`는
네트 통과가 검증된 샷만 반환하므로 "올만한 공"이 자동으로 뽑힌다.

### 슈터 Visual 공용화

`viewer/scene_dynamics.rs`의 큐보이드 생성과 `world.shooter_pose()` 동기화를
`sim::gui::shooter::Visual`로 추출한다. `host/run.rs`의
`let _ = &options.layers.shooter;`를 실제 spawn·sync로 교체해, jog가 쓰는
경량 호스트에서도 슈터가 보이게 한다. 경량 호스트는 월드 대신
`Handle::settings()`로 자세를 맞춘다 (`SimWorld::sync_shooter_pose`와 같은 SSOT).

## 게이트·안전

Sync / Preview / Apply 게이트는 그대로다.

- 예측 실패(접수 불가) 또는 IK 실패면 Preview를 막고 사유를 표시한다.
- 슈터는 sim 전용이다. Apply는 지금과 똑같이 스테이징된 궤적만 실기로 보내고,
  슈터 관련해 하드웨어로 나가는 것은 없다.
- `ensure_max_delta` · `ensure_rail_in_range` 검사는 유지된다.

## 테스트

- `plan::shooter::predict`: 기본 슈터가 기본 접수평면에 도달하고, 로봇 쪽으로
  오며, 테이블 면 위다.
- `plan::shooter::predict`: 낮고 평평한 고속 샷(pitch 0°, height −0.35, 12 m/s)은
  사람이 읽는 사유와 함께 실패한다.
- `plan::shooter::predict`: 유한하지 않은 접수평면 y는 거부된다.
- jog: 기본 슈터 설정 + 기본 접수평면에서 `swing_preview`가 `ik_ok`이고
  `compose(Kind::Swing)`이 궤적을 만든다 (해가 존재한다).
- jog: 도달 불가 슈터 설정에서 `compose`가 사유와 함께 실패하고 `reach_ok`가
  false다.
- jog: `ensure_max_delta` 위반이 여전히 거부된다 (기존 테스트 유지).
- 워크스페이스 전체 회귀가 변경 전과 동일하다 (`src/` 기하를 안 건드렸으므로).
