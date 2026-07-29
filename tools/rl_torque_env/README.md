# Residual-torque SAC

기존 플래너는 공까지 라켓을 이동시키고, SAC 정책은 스윙 재생 중 마지막
구간의 4축 signed torque만 보정한다.

## 1. Rust 환경 빌드

```powershell
cargo build -p rl-torque-env --release
```

## 2. Python 환경

DGX Spark에서는 NVIDIA PyTorch 환경/컨테이너에서 `import torch`와 CUDA를
먼저 확인한 뒤:

```bash
python -m pip install -r tools/rl_torque_env/python/requirements.txt
```

먼저 Gym 배선만 검사:

```bash
python tools/rl_torque_env/python/train_sac.py \
  --env-bin target/release/rl_torque_env \
  --check-only
```

1단계는 중앙 고정 공으로 학습:

```bash
python tools/rl_torque_env/python/train_sac.py \
  --env-bin target/release/rl_torque_env \
  --timesteps 500000 \
  --envs 8 \
  --output models/torque_sac
```

2단계는 1단계 정책을 불러와 랜덤 샷으로 확장:

```bash
python tools/rl_torque_env/python/train_sac.py \
  --env-bin target/release/rl_torque_env \
  --resume models/torque_sac.zip \
  --randomize \
  --timesteps 1000000 \
  --envs 8 \
  --output models/torque_sac_random
```

학습 후 반드시 0토크 residual 기준선과 비교:

```bash
python tools/rl_torque_env/python/evaluate.py \
  --env-bin target/release/rl_torque_env \
  --model models/torque_sac_random.zip \
  --randomize \
  --episodes 100
```

Windows에서는 바이너리 경로로 `target/release/rl_torque_env.exe`를 사용한다.

## 환경 계약

- 물리: Rapier 1 kHz
- 정책: 100 Hz (`action_repeat=10`)
- 관측: 공 위치/속도, 관절 위치/속도/기존 궤적 목표, 라켓 위치/속도, TTI,
  활성/접촉 플래그
- 액션: 관절별 `[-1,1] × τ_max`
- 적용 구간: 기존 스윙 재생 중이며 첫 접촉 전
- 안전: 최종 Rapier motor effort는 기존 `τ_max`로 포화
- 핵심 보상: 실제 공의 충돌 후 `+y` 속도, 네트 통과, 상대 코트 착지
