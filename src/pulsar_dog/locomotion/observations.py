"""Observation assembly, in the manager style used by Isaac Lab.

An observation is a list of named *terms*. Each term has a function, a scale and
an optional clip, and the manager concatenates them in declaration order into
one flat vector. That ordering is the contract with the policy: a policy trained
in simulation only works on hardware if the terms come out in exactly the same
order, with exactly the same scales - so the manager exposes ``term_slices()``
and ``describe()`` to make that checkable instead of hopeful.

Note on scope: the classic legged locomotion observation also carries joint
positions, joint velocities and the previous joint action (12 each). Those need
low-level joint control, which this project deliberately stays out of. Here the
action space is the body velocity command, so the observation covers the base
state, the command and the previous action.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

from pulsar_dog.safety import Velocity
from pulsar_dog.transport.base import RobotState

GRAVITY_LEVEL = (0.0, 0.0, -1.0)


@dataclass
class ObsContext:
    """Everything an observation term is allowed to look at."""

    state: RobotState | None
    command: Velocity
    last_action: Velocity
    dt: float
    step: int = 0


def projected_gravity(rpy: tuple[float, float, float] | None) -> tuple[float, float, float]:
    """Gravity expressed in the body frame - the policy's sense of "which way is down".

    Level gives ``(0, 0, -1)``; the z component falls towards 0 as the robot
    tips onto its side. Yaw drops out, which is what makes this usable as an
    orientation observation without a heading reference.
    """
    if rpy is None:
        return GRAVITY_LEVEL
    roll, pitch, _yaw = rpy
    return (
        math.sin(pitch),
        -math.cos(pitch) * math.sin(roll),
        -math.cos(pitch) * math.cos(roll),
    )


def tilt_angle(rpy: tuple[float, float, float] | None) -> float:
    """Angle between the body's up axis and vertical, in radians."""
    _, _, gz = projected_gravity(rpy)
    return math.acos(max(-1.0, min(1.0, -gz)))


# --- term functions ----------------------------------------------------
def base_lin_vel(ctx: ObsContext) -> tuple[float, float, float]:
    """Measured body linear velocity. Zeroes when telemetry is missing."""
    if ctx.state is None or ctx.state.velocity is None:
        return (0.0, 0.0, 0.0)
    return (ctx.state.velocity.vx, ctx.state.velocity.vy, 0.0)


def base_ang_vel(ctx: ObsContext) -> tuple[float, float, float]:
    if ctx.state is None or ctx.state.velocity is None:
        return (0.0, 0.0, 0.0)
    return (0.0, 0.0, ctx.state.velocity.vyaw)


def projected_gravity_term(ctx: ObsContext) -> tuple[float, float, float]:
    return projected_gravity(ctx.state.rpy if ctx.state is not None else None)


def velocity_commands(ctx: ObsContext) -> tuple[float, float, float]:
    return ctx.command.as_tuple()


def last_action(ctx: ObsContext) -> tuple[float, float, float]:
    return ctx.last_action.as_tuple()


@dataclass
class ObservationTerm:
    """One named block of the observation vector."""

    name: str
    func: Callable[[ObsContext], Sequence[float]]
    dim: int
    # Per-element scale, or a single float applied to the whole term. Scales
    # normalise the ranges the policy saw during training; they must match.
    scale: float | tuple[float, ...] = 1.0
    clip: tuple[float, float] | None = None

    def compute(self, ctx: ObsContext) -> list[float]:
        raw = list(self.func(ctx))
        if len(raw) != self.dim:
            raise ValueError(
                f"observation term {self.name!r} returned {len(raw)} values, expected {self.dim}"
            )
        scale = self.scale
        scales = (scale,) * self.dim if isinstance(scale, (int, float)) else tuple(scale)
        if len(scales) != self.dim:
            raise ValueError(
                f"observation term {self.name!r} has {len(scales)} scales for {self.dim} values"
            )
        values = [v * s for v, s in zip(raw, scales, strict=True)]
        if self.clip is not None:
            low, high = self.clip
            values = [max(low, min(high, v)) for v in values]
        return values


def default_terms() -> list[ObservationTerm]:
    """The standard term set, with the scales usually used for legged locomotion."""
    return [
        ObservationTerm("base_lin_vel", base_lin_vel, 3, scale=2.0),
        ObservationTerm("base_ang_vel", base_ang_vel, 3, scale=0.25),
        ObservationTerm("projected_gravity", projected_gravity_term, 3, scale=1.0),
        ObservationTerm("velocity_commands", velocity_commands, 3, scale=(2.0, 2.0, 0.25)),
        ObservationTerm("last_action", last_action, 3, scale=1.0),
    ]


@dataclass
class ObservationManager:
    """Concatenates terms into the flat vector a policy consumes."""

    terms: list[ObservationTerm] = field(default_factory=default_terms)
    # Applied after the per-term clips, matching the usual training-side clip.
    clip: tuple[float, float] | None = (-100.0, 100.0)

    def __post_init__(self) -> None:
        names = [t.name for t in self.terms]
        duplicates = {n for n in names if names.count(n) > 1}
        if duplicates:
            raise ValueError(f"duplicate observation terms: {sorted(duplicates)}")

    @property
    def dim(self) -> int:
        return sum(t.dim for t in self.terms)

    def term_slices(self) -> dict[str, slice]:
        """Where each term lands in the vector - use it to sanity-check a policy."""
        slices: dict[str, slice] = {}
        offset = 0
        for term in self.terms:
            slices[term.name] = slice(offset, offset + term.dim)
            offset += term.dim
        return slices

    def describe(self) -> str:
        lines = [f"observation dim={self.dim}"]
        for name, span in self.term_slices().items():
            lines.append(f"  [{span.start:2d}:{span.stop:2d}] {name}")
        return "\n".join(lines)

    def compute(self, ctx: ObsContext) -> list[float]:
        values: list[float] = []
        for term in self.terms:
            values.extend(term.compute(ctx))
        if self.clip is not None:
            low, high = self.clip
            values = [max(low, min(high, v)) for v in values]
        return values
