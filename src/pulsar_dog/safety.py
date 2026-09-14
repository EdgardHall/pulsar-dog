"""Clamping, acceleration limiting and the command watchdog.

Pure functions and small stateful helpers, deliberately free of any robot
import so they can be tested without hardware.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from pulsar_dog.config import SafetyLimits


def clamp(value: float, low: float, high: float) -> float:
    if low > high:
        raise ValueError(f"clamp bounds inverted: {low} > {high}")
    return max(low, min(high, value))


@dataclass(frozen=True)
class Velocity:
    """A body-frame velocity command: forward, lateral, turn rate."""

    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0

    @classmethod
    def zero(cls) -> Velocity:
        return cls(0.0, 0.0, 0.0)

    def is_zero(self, eps: float = 1e-6) -> bool:
        return abs(self.vx) < eps and abs(self.vy) < eps and abs(self.vyaw) < eps

    def clamped(self, limits: SafetyLimits) -> Velocity:
        return Velocity(
            clamp(self.vx, -limits.max_vx, limits.max_vx),
            clamp(self.vy, -limits.max_vy, limits.max_vy),
            clamp(self.vyaw, -limits.max_vyaw, limits.max_vyaw),
        )

    def scaled(self, factor: float) -> Velocity:
        return Velocity(self.vx * factor, self.vy * factor, self.vyaw * factor)

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.vx, self.vy, self.vyaw)


def _approach(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


class RateLimiter:
    """Slew-rate limits a velocity command so the robot never gets a step input.

    A Go2 will happily accept an instant 0 -> 1.5 m/s command and lurch; ramping
    keeps the body settled and gives you time to react.
    """

    def __init__(self, limits: SafetyLimits, initial: Velocity | None = None) -> None:
        self._limits = limits
        self._current = initial or Velocity.zero()

    @property
    def current(self) -> Velocity:
        return self._current

    def reset(self, to: Velocity | None = None) -> None:
        self._current = to or Velocity.zero()

    def step(self, target: Velocity, dt: float) -> Velocity:
        """Move one control tick of ``dt`` seconds towards ``target``."""
        if dt <= 0:
            return self._current
        target = target.clamped(self._limits)
        max_linear = self._limits.max_linear_accel * dt
        max_yaw = self._limits.max_yaw_accel * dt
        self._current = Velocity(
            _approach(self._current.vx, target.vx, max_linear),
            _approach(self._current.vy, target.vy, max_linear),
            _approach(self._current.vyaw, target.vyaw, max_yaw),
        )
        return self._current


class Watchdog:
    """Expires when nobody has poked it recently.

    The control loop polls this every tick: once expired the target velocity is
    forced to zero, so a dead teleop process or a stalled agent stops the robot
    instead of leaving the last command running.
    """

    def __init__(self, timeout_s: float, clock=time.monotonic) -> None:
        if timeout_s <= 0:
            raise ValueError("watchdog timeout must be > 0")
        self._timeout = timeout_s
        self._clock = clock
        self._last_poke = clock()

    @property
    def timeout_s(self) -> float:
        return self._timeout

    def poke(self) -> None:
        self._last_poke = self._clock()

    def age(self) -> float:
        return self._clock() - self._last_poke

    def expired(self) -> bool:
        return self.age() > self._timeout
