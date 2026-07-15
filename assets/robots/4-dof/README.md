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
cp config/example.toml config/4-dof.toml
# config/4-dof.toml에서 robot = "4-dof"
cargo run -p pingpong-bin -- config/4-dof.toml
```

## 관절

| 이름 | type | 역할 |
|------|------|------|
| Revolute 6 | continuous | yaw |
| Revolute 9 | revolute | shoulder |
| Revolute 13 | revolute | elbow |
| Revolute 18 | revolute | wrist |
| EE | `pingpong_paddle_v5_1` | 라켓 |
