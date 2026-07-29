"""Gymnasium wrapper for the persistent Rust JSONL environment."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np


class PingPongTorqueEnv(gym.Env[np.ndarray, np.ndarray]):
    metadata = {"render_modes": []}

    def __init__(
        self, env_bin: str | Path, seed: int = 20260729, randomize: bool = False
    ):
        super().__init__()
        self._randomize = randomize
        self._process = subprocess.Popen(
            [str(env_bin), "--seed", str(seed), "--action-repeat", "10"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            bufsize=1,
        )
        spaces = self._request({"cmd": "spaces"})
        observation_size = int(spaces["observation_size"])
        action_size = int(spaces["action_size"])
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(observation_size,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(action_size,), dtype=np.float32
        )

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        randomize = (
            self._randomize
            if options is None
            else bool(options.get("randomize", self._randomize))
        )
        response = self._request({"cmd": "reset", "randomize": randomize})
        return np.asarray(response["observation"], dtype=np.float32), {}

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        response = self._request(
            {"cmd": "step", "action": np.asarray(action, dtype=float).tolist()}
        )
        observation = np.asarray(response["observation"], dtype=np.float32)
        return (
            observation,
            float(response["reward"]),
            bool(response["terminated"]),
            bool(response["truncated"]),
            dict(response["info"]),
        )

    def close(self) -> None:
        if getattr(self, "_process", None) is None:
            return
        if self._process.poll() is None:
            try:
                self._request({"cmd": "close"})
            except (BrokenPipeError, RuntimeError):
                pass
            self._process.terminate()
        self._process = None

    def _request(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._process.stdin is None or self._process.stdout is None:
            raise RuntimeError("Rust environment process has no pipes")
        self._process.stdin.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self._process.stdin.flush()
        line = self._process.stdout.readline()
        if not line:
            code = self._process.poll()
            raise RuntimeError(f"Rust environment exited unexpectedly (code={code})")
        response = json.loads(line)
        if response.get("kind") == "error":
            raise RuntimeError(response.get("message", "unknown Rust environment error"))
        return response
