"""Policies: whatever turns an observation vector into an action vector.

The deployment path from Isaac Lab is a TorchScript export - the trained actor
saved with ``torch.jit.save`` and loaded here unchanged. Everything else in this
module exists so the surrounding machinery (observations, commands,
terminations, the runner) can be exercised without torch and without a robot.

Actions are normalised: the runner multiplies them by ``ActionCfg.scale`` to get
a body velocity. Keeping the policy in normalised units is what lets the same
export drive a different speed envelope without retraining.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Protocol, runtime_checkable

ACTION_DIM = 3


@runtime_checkable
class Policy(Protocol):
    name: str

    def reset(self) -> None: ...

    def act(self, obs: Sequence[float]) -> Sequence[float]:
        """Map one observation vector to ``ACTION_DIM`` normalised actions."""


class ZeroPolicy:
    """Stands still. The baseline every other policy has to beat."""

    name = "zero"

    def reset(self) -> None:
        return None

    def act(self, obs: Sequence[float]) -> list[float]:  # noqa: ARG002 - constant by design
        return [0.0] * ACTION_DIM


class CommandFollowingPolicy:
    """Open-loop baseline: reproduces the commanded velocity exactly.

    It reads the command straight out of the observation vector and undoes the
    observation and action scales, so the robot tracks the command manager
    perfectly. Useful to validate the whole chain - command generation,
    observation layout, action scaling, terminations - before a learned policy
    is anywhere near the robot.
    """

    name = "command"

    def __init__(
        self,
        command_slice: slice,
        obs_scale: tuple[float, float, float],
        action_scale: tuple[float, float, float],
    ) -> None:
        if any(s == 0 for s in obs_scale) or any(s == 0 for s in action_scale):
            raise ValueError("scales must be non-zero to be invertible")
        self._slice = command_slice
        self._obs_scale = obs_scale
        self._action_scale = action_scale

    def reset(self) -> None:
        return None

    def act(self, obs: Sequence[float]) -> list[float]:
        scaled = list(obs[self._slice])
        if len(scaled) != ACTION_DIM:
            raise ValueError(
                f"command slice yielded {len(scaled)} values, expected {ACTION_DIM}"
            )
        return [
            (value / obs_s) / act_s
            for value, obs_s, act_s in zip(
                scaled, self._obs_scale, self._action_scale, strict=True
            )
        ]


class PatrolPolicy:
    """A deterministic there-and-back gait, ignoring the command.

    Not a learned controller - a repeatable motion for checking that the robot
    walks, turns and comes back without anyone holding a keyboard.
    """

    name = "patrol"

    def __init__(self, period_s: float = 8.0, dt: float = 0.02, speed: float = 0.5) -> None:
        if period_s <= 0 or dt <= 0:
            raise ValueError("period_s and dt must be > 0")
        self._period = period_s
        self._dt = dt
        self._speed = speed
        self._t = 0.0

    def reset(self) -> None:
        self._t = 0.0

    def act(self, obs: Sequence[float]) -> list[float]:  # noqa: ARG002 - open loop
        phase = (self._t % self._period) / self._period
        self._t += self._dt
        # First half walks forward, second half turns in place.
        if phase < 0.5:
            return [self._speed, 0.0, 0.0]
        return [0.0, 0.0, self._speed * math.sin(2 * math.pi * phase)]


class TorchScriptPolicy:
    """A TorchScript actor exported from training, run as-is.

    torch is imported lazily so the rest of the package stays dependency-free.
    """

    def __init__(self, path: str, obs_dim: int | None = None) -> None:
        try:
            import torch  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - needs torch
            raise RuntimeError(
                f"loading {path} needs PyTorch: pip install torch"
            ) from exc
        self._torch = torch
        self._module = torch.jit.load(path, map_location="cpu")
        self._module.eval()
        self._obs_dim = obs_dim
        self.name = f"torchscript:{path}"

    def reset(self) -> None:
        # Recurrent policies expose a reset hook; feed-forward ones do not.
        reset = getattr(self._module, "reset", None)
        if callable(reset):  # pragma: no cover - depends on the export
            reset()

    def act(self, obs: Sequence[float]) -> list[float]:
        if self._obs_dim is not None and len(obs) != self._obs_dim:
            raise ValueError(
                f"policy expects {self._obs_dim} observations, got {len(obs)}; "
                "the observation term layout must match the training config"
            )
        torch = self._torch
        with torch.inference_mode():
            tensor = torch.tensor([list(obs)], dtype=torch.float32)
            output = self._module(tensor)
        action = output.detach().reshape(-1).tolist()
        if len(action) != ACTION_DIM:
            raise ValueError(
                f"policy returned {len(action)} actions, expected {ACTION_DIM}"
            )
        return action
