# 4-DOF 로봇 (`all-4-export`)

CAD(Onshape/Fusion) → URDF 내보내기본. sim에서는 **mesh 시각화**용이고, 제어·IK는 `competition` 빌더를 쓴다.

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
cargo run -p pingpong-bin -- --robot 4-dof
# 또는
cargo run -p pingpong-bin -- \
  --urdf assets/robots/4-dof/urdf/all-4-export.urdf \
  --ee-link pingpong_paddle_v5_1
```

## 관절

| 이름 | type | 역할 |
|------|------|------|
| Revolute 6 | continuous | yaw |
| Revolute 9 | revolute | shoulder |
| Revolute 13 | revolute | elbow |
| Revolute 18 | revolute | wrist |
| EE | `pingpong_paddle_v5_1` | 라켓 |
