"""Compare the learned SAC policy with zero residual torque."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from stable_baselines3 import SAC

from gym_env import PingPongTorqueEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env-bin", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=20260730)
    parser.add_argument("--randomize", action="store_true")
    return parser.parse_args()


def run(
    env: PingPongTorqueEnv, model: SAC | None, episodes: int
) -> dict[str, float | int]:
    total_reward = 0.0
    successes = 0
    contacts = 0
    outgoing_speeds: list[float] = []
    for _ in range(episodes):
        observation, _ = env.reset()
        terminated = truncated = False
        info: dict = {}
        while not (terminated or truncated):
            if model is None:
                action = np.zeros(env.action_space.shape, dtype=np.float32)
            else:
                action, _ = model.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
        contacts += int(bool(info.get("contact", False)))
        successes += int(bool(info.get("returned_in", False)))
        outgoing_speeds.append(float(info.get("peak_outgoing_y_mps", 0.0)))
    return {
        "episodes": episodes,
        "mean_reward": total_reward / episodes,
        "contacts": contacts,
        "successes": successes,
        "success_rate": successes / episodes,
        "mean_peak_outgoing_y_mps": sum(outgoing_speeds) / episodes,
    }


def main() -> None:
    args = parse_args()
    baseline_env = PingPongTorqueEnv(
        args.env_bin, seed=args.seed, randomize=args.randomize
    )
    learned_env = PingPongTorqueEnv(
        args.env_bin, seed=args.seed, randomize=args.randomize
    )
    try:
        baseline = run(baseline_env, None, args.episodes)
        model = SAC.load(str(args.model), device="auto")
        learned = run(learned_env, model, args.episodes)
    finally:
        baseline_env.close()
        learned_env.close()
    print(json.dumps({"zero_residual": baseline, "learned": learned}, indent=2))


if __name__ == "__main__":
    main()
