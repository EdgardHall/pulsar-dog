"""Velocity command generation, mirroring Isaac Lab's ``UniformVelocityCommand``.

During training the command is resampled from uniform ranges every few seconds,
with a fraction of the episodes commanded to stand still so the policy learns to
hold a posture rather than always drifting. Reproducing that on hardware gives a
repeatable autonomous exercise: the robot is driven by the same distribution it
was trained on, instead of by whatever a human happens to press.

Ranges are also the first safety knob: they are clamped to the robot's
:class:`SafetyLimits` on construction, so a command source can never ask for
more than the safety envelope allows.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, replace

from pulsar_dog.config import SafetyLimits
from pulsar_dog.safety import Velocity, clamp


@dataclass(frozen=True)
class VelocityCommandCfg:
    """Sampling ranges, in the robot's own units."""

    lin_vel_x: tuple[float, float] = (-0.3, 0.5)
    lin_vel_y: tuple[float, float] = (-0.2, 0.2)
    ang_vel_z: tuple[float, float] = (-0.5, 0.5)
    # A new command is drawn every ``resample_time_s`` seconds.
    resample_time_s: float = 4.0
    # Fraction of resamples that command a full stop.
    rel_standing: float = 0.2

    def __post_init__(self) -> None:
        for name in ("lin_vel_x", "lin_vel_y", "ang_vel_z"):
            low, high = getattr(self, name)
            if low > high:
                raise ValueError(f"{name} range is inverted: ({low}, {high})")
        if self.resample_time_s <= 0:
            raise ValueError("resample_time_s must be > 0")
        if not 0.0 <= self.rel_standing <= 1.0:
            raise ValueError("rel_standing must be within [0, 1]")

    def bounded_by(self, limits: SafetyLimits) -> VelocityCommandCfg:
        """Shrink the ranges so no sample can exceed the safety envelope."""
        return replace(
            self,
            lin_vel_x=(
                clamp(self.lin_vel_x[0], -limits.max_vx, limits.max_vx),
                clamp(self.lin_vel_x[1], -limits.max_vx, limits.max_vx),
            ),
            lin_vel_y=(
                clamp(self.lin_vel_y[0], -limits.max_vy, limits.max_vy),
                clamp(self.lin_vel_y[1], -limits.max_vy, limits.max_vy),
            ),
            ang_vel_z=(
                clamp(self.ang_vel_z[0], -limits.max_vyaw, limits.max_vyaw),
                clamp(self.ang_vel_z[1], -limits.max_vyaw, limits.max_vyaw),
            ),
        )


class UniformVelocityCommand:
    """Draws velocity commands from uniform ranges, resampling on a timer."""

    def __init__(
        self,
        cfg: VelocityCommandCfg,
        limits: SafetyLimits,
        seed: int | None = None,
    ) -> None:
        self._cfg = cfg.bounded_by(limits)
        self._rng = random.Random(seed)
        self._command = Velocity.zero()
        self._elapsed = 0.0
        self._standing = False
        self._resamples = 0
        self.resample()

    @property
    def cfg(self) -> VelocityCommandCfg:
        return self._cfg

    @property
    def command(self) -> Velocity:
        return self._command

    @property
    def standing(self) -> bool:
        return self._standing

    @property
    def resamples(self) -> int:
        return self._resamples

    def reset(self) -> None:
        self._elapsed = 0.0
        self.resample()

    def resample(self) -> Velocity:
        self._resamples += 1
        self._elapsed = 0.0
        if self._rng.random() < self._cfg.rel_standing:
            self._standing = True
            self._command = Velocity.zero()
        else:
            self._standing = False
            self._command = Velocity(
                self._rng.uniform(*self._cfg.lin_vel_x),
                self._rng.uniform(*self._cfg.lin_vel_y),
                self._rng.uniform(*self._cfg.ang_vel_z),
            )
        return self._command

    def step(self, dt: float) -> Velocity:
        """Advance the timer, resampling when the interval elapses."""
        self._elapsed += dt
        if self._elapsed >= self._cfg.resample_time_s:
            self.resample()
        return self._command


class FixedVelocityCommand:
    """A constant command - the simplest way to check one motion in isolation."""

    def __init__(self, command: Velocity, limits: SafetyLimits) -> None:
        self._command = command.clamped(limits)

    @property
    def command(self) -> Velocity:
        return self._command

    @property
    def standing(self) -> bool:
        return self._command.is_zero()

    def reset(self) -> None:
        return None

    def step(self, dt: float) -> Velocity:  # noqa: ARG002 - fixed by definition
        return self._command
