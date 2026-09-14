"""Termination terms - the simulation idea that matters most on hardware.

In Isaac Lab an episode ends when a termination term fires: the base tipped
over, the height collapsed, the time limit elapsed. In simulation that costs a
reset. Here the same terms are the safety layer: a fired term stops the robot,
and anything that is not a plain timeout latches the emergency stop.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

from pulsar_dog.locomotion.observations import ObsContext, tilt_angle


@dataclass(frozen=True)
class Termination:
    """Why the run stopped."""

    name: str
    detail: str
    # A timeout is a clean end of run; anything else is a fault.
    is_fault: bool = True


@dataclass
class TerminationTerm:
    name: str
    # Returns a reason string when it fires, None otherwise.
    func: Callable[[ObsContext], str | None]
    is_fault: bool = True


def bad_orientation(max_tilt_rad: float = math.radians(35.0)) -> TerminationTerm:
    """Fires when the body has tipped past ``max_tilt_rad`` from vertical."""

    def check(ctx: ObsContext) -> str | None:
        if ctx.state is None or ctx.state.rpy is None:
            return None
        tilt = tilt_angle(ctx.state.rpy)
        if tilt > max_tilt_rad:
            return f"tilt {math.degrees(tilt):.1f}deg > {math.degrees(max_tilt_rad):.1f}deg"
        return None

    return TerminationTerm("bad_orientation", check)


def base_height_below(min_height_m: float = 0.15) -> TerminationTerm:
    """Fires when the body has collapsed towards the ground."""

    def check(ctx: ObsContext) -> str | None:
        if ctx.state is None or ctx.state.body_height is None:
            return None
        if ctx.state.body_height < min_height_m:
            return f"body height {ctx.state.body_height:.3f}m < {min_height_m:.3f}m"
        return None

    return TerminationTerm("base_height", check)


def battery_below(min_soc: int = 20) -> TerminationTerm:
    """A Go2 at a low state of charge lies down without warning. Land it first."""

    def check(ctx: ObsContext) -> str | None:
        if ctx.state is None or ctx.state.battery_soc is None:
            return None
        if ctx.state.battery_soc < min_soc:
            return f"battery {ctx.state.battery_soc}% < {min_soc}%"
        return None

    return TerminationTerm("battery", check)


def telemetry_stale(max_age_s: float = 0.5) -> TerminationTerm:
    """Fires when the robot stopped talking: driving blind is not an option."""

    def check(ctx: ObsContext) -> str | None:
        if ctx.state is None:
            return "no telemetry received"
        age = ctx.state.extra.get("state_age_s")
        if age is None:
            return None
        if age > max_age_s:
            return f"telemetry {age:.2f}s old > {max_age_s:.2f}s"
        return None

    return TerminationTerm("telemetry_stale", check)


def time_limit(duration_s: float) -> TerminationTerm:
    """The clean end of a run, equivalent to a simulation episode length."""
    state = {"elapsed": 0.0}

    def check(ctx: ObsContext) -> str | None:
        state["elapsed"] += ctx.dt
        if state["elapsed"] >= duration_s:
            return f"reached {duration_s:.1f}s"
        return None

    return TerminationTerm("time_limit", check, is_fault=False)


def default_terms(duration_s: float | None = None) -> list[TerminationTerm]:
    terms = [
        bad_orientation(),
        base_height_below(),
        battery_below(),
        telemetry_stale(),
    ]
    if duration_s is not None:
        terms.append(time_limit(duration_s))
    return terms


@dataclass
class TerminationManager:
    """Evaluates every term each control step and reports the first that fires."""

    terms: list[TerminationTerm] = field(default_factory=list)

    def check(self, ctx: ObsContext) -> Termination | None:
        fired: Termination | None = None
        for term in self.terms:
            # Every term is evaluated, even after one fires: stateful terms such
            # as the time limit must keep counting.
            reason = term.func(ctx)
            if reason is not None and fired is None:
                fired = Termination(term.name, reason, term.is_fault)
        return fired
