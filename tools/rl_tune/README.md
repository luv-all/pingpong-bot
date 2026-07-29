# rl_tune

기존 스윙 플래너의 안전 검사와 접촉 추적은 유지하고, 목표 바운드 위치와
바운드 시간 residual을 CEM(Cross-Entropy Method) 정책 탐색으로 학습한다.
한 에피소드는 공 한 발이며 `contact → return → net clear → opponent bounce`
순으로 보상한다.

저장소 루트에서 실행:

```powershell
cargo run -p rl-tune --release -- --generations 8 --population 16 --shots 4
```

빠른 배선 검증:

```powershell
cargo run -p rl-tune --release -- --generations 1 --population 4 --elite 2 --shots 1
```

결과는 기본적으로 `rl_policy.json`에 저장된다. 이 파일의 `residual` 세 값을
실기 `SimWorld::set_swing_residual` 또는 이후 실기 제어 경로에 전달하면 된다.

이 도구는 신경망이 아니라 3차원 정책의 episodic policy search다. 이 단계에서
보상이 실제로 개선되는지 확인한 뒤, 공 상태별로 다른 액션이 필요할 때 SAC
정책으로 교체하는 것이 목적이다.
