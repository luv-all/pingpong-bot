# urdf-test (실물 로봇 URDF)

Fusion 360 → xacro → `urdf-test.urdf` + STL mesh 패키지입니다.

## 구조

```
urdf-test_description/
  urdf/urdf-test.urdf    # flat URDF (xacro 빌드 결과)
  meshes/*.stl           # mm 단위, URDF scale 0.001
```

## 관절 (actuated 3축)

| 관절 | 타입 | 축 | 비고 |
|------|------|-----|------|
| Revolute 6 | continuous | -X | 베이스 요 |
| Revolute 9 | revolute | -Z | ±50° |
| Revolute 13 | revolute | -X | 라켓 스윙 |

엔드이펙터: `pingpong_paddle_v5_1` (핑퐁 라켓 mesh)

## 실행

```bash
cargo run -p pingpong-bin -- \
  --urdf assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf \
  --ee-link pingpong_paddle_v5_1
```

3축 revolute/continuous 체인이므로 `plan_swing`용 domain `Arm` 변환도 시도합니다.

## 좌표계

Fusion 360 export와 sim 모두 **REP-103 Z-up** (X 앞, Y 왼, Z 위)입니다.

sim 월드는 탁구대 **로봇 쪽 꼭짓점**을 원점으로 씁니다:

| sim 축 | 의미 |
|--------|------|
| +X | 탁구대 너비 (1.525 m) |
| +Y | 탁구대 길이 (2.74 m) |
| +Z | 고도 (테이블 면 z = 0.76 m) |

`urdf-test` 로드 시 `SimRobotMount`가 자동 적용됩니다 (`rpy = 0`, 위치 = 탁구대 y≈0 끝).

참고: [ROS REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)

## 참고

- mesh 경로는 `package://urdf-test_description/meshes/...` (ROS 관례)
- `linear guide v6.stl`은 zip에 포함됐으나 현재 URDF visual에 미사용
- mesh 충돌·xacro 직접 로드는 미지원
