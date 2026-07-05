# 커스텀 로봇 mesh (Fusion 360 → URDF)

스크린샷에 있는 격자(lattice) 팔·원형 라켓을 sim에 그대로 쓰는 절차입니다.

## 1. Fusion 360에서 STL보내기

각 **단일 body(부품)** 마다 mesh를 하나씩 만듭니다.

1. 타임라인에서 body 선택 (또는 우클릭 → **Save as Mesh**)
2. 형식: **STL**
3. **Refinement**: 높을수록 격자 디테일 유지 (파일 크기 증가)
4. 단위: Fusion은 보통 **mm** — URDF에서 `scale="0.001 0.001 0.001"` 사용

권장 파일명 (`meshes/` 폴더):

| STL 파일 | 대응 link |
|----------|-----------|
| `base_link.stl` | 베이스·서보 마운트 |
| `yaw_link.stl` | 하부 회전부 |
| `shoulder_link.stl` | 어깨 링크 |
| `upper_arm_link.stl` | 격자 상완 |
| `forearm_link.stl` | 전완 |
| `wrist_link.stl` | 손목·서보 |
| `racket_link.stl` | 원형 라켓 |

> 한 조립체를 통째로 export하면 URDF link별로 움직이지 않습니다. **조인트마다 body를 분리**한 뒤 export하세요.

## 2. 파일 배치

```
assets/robots/custom/
  robot.urdf
  meshes/
    base_link.stl
    yaw_link.stl
    ...
```

mesh가 없으면 sim에서 **분홍 placeholder cube**가 표시됩니다 (경로 확인용).

## 3. URDF 튜닝

`robot.urdf`의 `<joint><origin>`·`<axis>`는 CAD 조인트 위치/회전축과 일치해야 합니다.

- Fusion **Inspect → Joint** 로 축 방향 확인
- 첫 실행 후 mesh가 어긋나면 origin xyz/rpy를 조금씩 수정

## 4. 실행

```bash
cargo run -p pingpong-bin -- \
  --urdf assets/robots/custom/robot.urdf \
  --ee-link racket_link
```

## 지원 형식

| 형식 | 비고 |
|------|------|
| **STL** | Fusion export 권장 |
| **OBJ** | 동일 폴더 MTL 있으면 색상 반영 |
| box/cylinder/sphere | URDF primitive |

## 제한

- 관절이 3개가 아니면 `plan_swing`은 내장 `competition_arm` 사용 (URDF는 시각화·FK만)
- mesh 충돌은 아직 없음
- CAD 단위가 **m** 이면 `scale="1 1 1"` 로 변경
