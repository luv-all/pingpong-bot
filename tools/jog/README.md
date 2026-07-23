# jog

Dynamixel 관절 + AXL 리니어 레일을 **한 세션에서 반복** 조그하는 인터랙티브 REPL.

공 추적·`plan_swing` 같은 planner는 **쓰지 않는다**. 목표만 정하면 FK/IK와 quintic 궤적으로 팔·레일을 같은 시간에 보낸다.

설정 SSOT: [`src/defaults/`](../../src/defaults/) — `dynamixel()` · `rail()` · `robot()` · `control()`.

## 실행

```bash
# 통신 없이 IK·궤적·executor 경로만
cargo run -p jog -- --dry-run

# 실기 (Windows 벤치)
cargo run -p jog -- --port COM8
cargo run -p jog -- --port COM8 --dll-path "C:/path/to/AXL.dll"
```

시작 후 `jog>` 프롬프트에서 명령을 반복 입력한다. `help` / `q`.

## 명령

| 명령 | 설명 |
|------|------|
| `status` | 현재 레일·관절(deg)·FK(라켓 위치/법선) |
| `j <i> <deg>` | 관절 `i`만 목표각 [deg], 나머지 유지 |
| `angles a0,a1,a2,a3` | 전축 목표 [deg] (URDF 관절각) |
| `r <x_m>` | 레일 절대 위치 [m] (소프트 리밋 클램프) |
| `rd <dx>` | 레일 상대 이동 [m] |
| `ik <x> <y> <z>` | 위치 IK → 관절 이동 (레일 x는 현재값 유지) |
| `pose x y z nx ny nz` | 위치+법선 IK → 관절·레일 이동 |
| `swing x y z [nx ny nz] speed <v>` | 임팩트 자세로 가서 **라켓 속도 ≈ v [m/s]** (법선 방향)로 짧게 휘두름 |
| `duration <s>` | 기본 접근 이동 시간 (기본 1.0) |
| `maxdelta <deg>` | 관절 한 번에 허용 Δ (기본 15, 안전) |
| `help` / `q` | 도움말 / 종료 |

### 스윙 세기

`speed`는 토크가 아니라 **임팩트 순간 라켓 선속도 [m/s]** 다.  
법선 방향으로 `velocities_for_racket_velocity` → 관절·레일 속도 → quintic `end_velocity` + 팔로스루(`control().swing_follow_through_secs`).

예:

```text
jog> duration 0.8
jog> maxdelta 30
jog> status
jog> j 2 -10
jog> r 0.05
jog> pose 0.76 0.35 0.85 0 0 1
jog> swing 0.76 0.35 0.85 0 0 1 speed 1.5
```

## 동작 요약

1. 현재 pose 읽기 (`Hardware::read_pose`)
2. 목표 관절·레일 결정 (직접 / IK / pose / swing)
3. `SwingTrajectory` quintic 생성 (관절 + `RailMotion`)
4. `RealHardware::command`가 `stream_hz`로 관절·레일을 **같이** 샘플링
5. `is_busy`가 풀릴 때까지 대기 후 `status`

레일 Live는 tick마다 `command_abs_m`(논블로킹 `AxmMovePos`)으로 목표를 갱신한다.  
단발 절대 이동이 끝날 때까지 기다리려면 예전처럼 `move_abs_m`(대기) 경로를 쓴다 — REPL의 `r`/`rd`는 궤적 샘플링 경로다.

## 안전

- `maxdelta` 초과 관절 점프는 거부
- IK/pose 실패·도달 불가는 에러만 출력하고 하드웨어는 그대로
- 실기는 작은 `j` / `rd`부터
