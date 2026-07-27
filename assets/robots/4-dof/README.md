# 4-DOF 로봇 (`all-4-export`)

CAD(Onshape/Fusion) → URDF 내보내기본. 이 파일이 관절 origin·축·한계,
FK·IK·제어와 mesh 시각화의 단일 모델이다. 로드/변환 실패 시 `competition`
빌더로 대체하지 않고 런타임 시작이 실패한다.

## 레이아웃

```
assets/robots/4-dof/
  urdf/all-4-export.urdf   # 런타임 로드
  meshes/*.stl             # mm STL (scale 0.001)
```

mesh 경로는 `package://4-dof/meshes/...` 이다.  
ROS install의 절대 `file:///Users/...` 경로를 넣지 말 것.

## 실행

```bash
cargo run -p pingpong-bot -- --robot 4-dof
```

## 관절

| 이름 | type | 역할 |
|------|------|------|
| Revolute 6 | continuous | base_pitch |
| Revolute 9 | revolute | pan |
| Revolute 13 | revolute | elbow |
| Revolute 18 | revolute | wrist |
| EE | `pingpong_paddle_v5_1` | 라켓 |

축 (URDF `axis`, 마운트 rpy 전부 0):

- `base_pitch` — `(-1, 0, 0)` 수평. YZ평면 피치로 팔 전체를 앞뒤로 움직인다.
- `pan` — `(0, 0, -1)` 수직. 좌우 선회.
- `elbow` / `wrist` — `(-1, 0, 0)` 수평 피치.

⚠️ `base_pitch`는 예전에 `yaw`라고 불렸으나 **축이 수평(−X)이라 yaw가 아니다** —
실제 yaw(수직축 선회)는 `pan`이다. 2026-07-27에 정정했다. 그 이전 커밋·문서의
"yaw"는 j0을, "shoulder"는 j1을 가리킨다.

## 실물 Dynamixel 매핑

URDF movable joint 순서와 모터 ID 순서는 고정이다.

| URDF joint | Dynamixel ID | sign | notes |
|------------|--------------|------|-------|
| Revolute 6 | 1 (+ slave **2** mirrored) | -1 | base_pitch 듀얼: `slave_ticks = 2·zero − master` |
| Revolute 9 | 3 | +1 | |
| Revolute 13 | 4 | +1 | |
| Revolute 18 | 5 | +1 | |

미러·포트·리밋 SSOT는 [`src/defaults.rs`](../../../src/defaults.rs)의
`dynamixel()`이다.
