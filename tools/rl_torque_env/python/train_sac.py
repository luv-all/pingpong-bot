"""Train a state-conditioned residual-torque SAC policy."""

from __future__ import annotations

import argparse
from pathlib import Path

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from gym_env import PingPongTorqueEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env-bin", type=Path, required=True)
    parser.add_argument("--timesteps", type=int, default=500_000)
    parser.add_argument("--envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=20260729)
    parser.add_argument("--output", type=Path, default=Path("models/torque_sac"))
    parser.add_argument("--resume", type=Path)
    parser.add_argument(
        "--randomize",
        action="store_true",
        help="좌우 위치·속도를 랜덤화한다. 중앙 고정 공 학습 후 2단계에서 사용",
    )
    parser.add_argument("--check-only", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    probe = PingPongTorqueEnv(
        args.env_bin, seed=args.seed, randomize=args.randomize
    )
    check_env(probe, warn=True)
    probe.close()
    if args.check_only:
        print("Gym environment check passed")
        return

    def make_env(index: int):
        return lambda: PingPongTorqueEnv(
            args.env_bin,
            seed=args.seed + 10_000 * index,
            randomize=args.randomize,
        )

    # 각 Python worker가 독립 Rust/Rapier 프로세스를 소유해 물리 rollout을
    # 실제 CPU 코어들에서 병렬 실행한다.
    env = SubprocVecEnv([make_env(index) for index in range(args.envs)])
    checkpoint = CheckpointCallback(
        save_freq=max(10_000 // args.envs, 1),
        save_path=str(args.output.parent),
        name_prefix="torque_sac_checkpoint",
    )
    if args.resume:
        model = SAC.load(
            str(args.resume),
            env=env,
            device="auto",
            tensorboard_log=str(args.output.parent / "tensorboard"),
        )
    else:
        model = SAC(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            buffer_size=1_000_000,
            learning_starts=10_000,
            batch_size=512,
            tau=0.005,
            gamma=0.99,
            train_freq=1,
            gradient_steps=1,
            policy_kwargs={"net_arch": [256, 256]},
            tensorboard_log=str(args.output.parent / "tensorboard"),
            verbose=1,
            seed=args.seed,
            device="auto",
        )
    try:
        model.learn(total_timesteps=args.timesteps, callback=checkpoint)
        model.save(str(args.output))
    finally:
        env.close()


if __name__ == "__main__":
    main()
