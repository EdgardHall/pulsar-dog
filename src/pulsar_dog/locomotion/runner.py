"""The control-policy loop: observe, act, check terminations, repeat.

This is the hardware counterpart of a simulation step. Isaac Lab runs the policy
at a decimated rate - the policy decides at 50 Hz while the physics ticks at
200 Hz - and the same split exists here: the policy decides at ``policy_hz``,
while :class:`PulsarDog`'s control thread republishes the ramped command at
``control_hz``. The runner never touches the backend directly; it only sets a
target velocity, so every existing safety guarantee still applies.

What is deliberately different from simulation: a termination does not reset an
episode, it stops a real robot. A fault termination latches the emergency stop.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from pulsar_dog.config import SafetyLimits
from pulsar_dog.locomotion.commands import UniformVelocityCommand, VelocityCommandCfg
from pulsar_dog.locomotion.observations import ObsContext, ObservationManager
from pulsar_dog.locomotion.policy import ACTION_DIM, Policy
from pulsar_dog.locomotion.terminations import (
    Termination,
    TerminationManager,
    default_terms,
)
from pulsar_dog.robot import EmergencyStop, PulsarDog
from pulsar_dog.safety import Velocity, clamp

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class ActionCfg:
    """How a normalised action becomes a body velocity."""

    # Multiplies the (clipped) action to give m/s, m/s and rad/s.
    scale: tuple[float, float, float] = (0.6, 0.4, 0.8)
    clip: tuple[float, float] = (-1.0, 1.0)

    def to_velocity(self, action: list[float]) -> Velocity:
        if len(action) != ACTION_DIM:
            raise ValueError(f"expected {ACTION_DIM} actions, got {len(action)}")
        low, high = self.clip
        clipped = [clamp(a, low, high) for a in action]
        return Velocity(
            clipped[0] * self.scale[0],
            clipped[1] * self.scale[1],
            clipped[2] * self.scale[2],
        )


@dataclass(frozen=True)
class LocomotionCfg:
    policy_hz: float = 50.0
    action: ActionCfg = field(default_factory=ActionCfg)
    command: VelocityCommandCfg = field(default_factory=VelocityCommandCfg)
    # Stand up and settle before the policy takes over.
    stand_first: bool = True
    settle_s: float = 2.0
    # Lie down at the end instead of leaving the robot standing unattended.
    sit_after: bool = False

    def decimation(self, control_hz: float) -> float:
        """Control ticks per policy step - the simulation decimation factor."""
        return control_hz / self.policy_hz


@dataclass
class RunResult:
    steps: int
    duration_s: float
    termination: Termination | None
    mean_rate_hz: float
    late_steps: int

    def summary(self) -> str:
        reason = (
            f"{self.termination.name} ({self.termination.detail})"
            if self.termination is not None
            else "stopped by caller"
        )
        return (
            f"{self.steps} steps in {self.duration_s:.1f}s "
            f"({self.mean_rate_hz:.1f} Hz, {self.late_steps} late) - {reason}"
        )


class LocomotionRunner:
    """Drives a :class:`PulsarDog` from a policy until a termination fires."""

    def __init__(
        self,
        dog: PulsarDog,
        policy: Policy,
        cfg: LocomotionCfg | None = None,
        observations: ObservationManager | None = None,
        terminations: TerminationManager | None = None,
        command: UniformVelocityCommand | None = None,
        seed: int | None = None,
    ) -> None:
        self._dog = dog
        self._policy = policy
        self._cfg = cfg or LocomotionCfg()
        self._obs = observations or ObservationManager()
        self._terminations = terminations or TerminationManager(terms=default_terms())
        self._command = command or UniformVelocityCommand(
            self._cfg.command, dog.config.safety, seed=seed
        )
        self._last_action = Velocity.zero()
        self._step_listener: Callable[[dict], None] | None = None
        self._started = 0.0

    @property
    def observations(self) -> ObservationManager:
        return self._obs

    @property
    def limits(self) -> SafetyLimits:
        return self._dog.config.safety

    def set_step_listener(self, listener: Callable[[dict], None] | None) -> None:
        """Called once per policy step with a flat record - handy for logging."""
        self._step_listener = listener

    def run(self, max_steps: int | None = None) -> RunResult:
        """Run until a termination fires, ``max_steps`` elapses, or Ctrl-C."""
        cfg = self._cfg
        dt = 1.0 / cfg.policy_hz
        self._policy.reset()
        self._command.reset()
        self._last_action = Velocity.zero()

        if cfg.stand_first:
            log.info("standing up before handing over to policy %s", self._policy.name)
            self._dog.stand_up()
            time.sleep(cfg.settle_s)
            self._dog.balance_stand()

        # Telemetry is cached at the control loop's poll rate, so the first step
        # would otherwise judge the robot on a state from before it stood up -
        # and terminate on a body height that is no longer true.
        try:
            self._dog.read_state(fresh=True)
        except Exception as exc:  # noqa: BLE001 - the terminations handle the rest
            log.warning("could not refresh telemetry before the run: %s", exc)

        log.info(
            "policy %s at %.0f Hz (decimation %.1f), action scale %s",
            self._policy.name,
            cfg.policy_hz,
            cfg.decimation(self._dog.config.control_hz),
            cfg.action.scale,
        )

        started = time.monotonic()
        self._started = started
        step = 0
        late = 0
        termination: Termination | None = None
        try:
            while max_steps is None or step < max_steps:
                tick = time.monotonic()
                ctx = ObsContext(
                    state=self._dog.read_state(),
                    command=self._command.command,
                    last_action=self._last_action,
                    dt=dt,
                    step=step,
                )

                termination = self._terminations.check(ctx)
                if termination is not None:
                    break

                self._command.step(dt)
                observation = self._obs.compute(ctx)
                action = list(self._policy.act(observation))
                velocity = cfg.action.to_velocity(action)

                try:
                    self._dog.set_velocity(velocity)
                except EmergencyStop:
                    termination = Termination(
                        "emergency_stop", "e-stop latched outside the runner"
                    )
                    break
                self._last_action = velocity

                if self._step_listener is not None:
                    self._emit(step, ctx, action, velocity)

                step += 1
                sleep_for = dt - (time.monotonic() - tick)
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    # The policy is slower than its own rate; the watchdog covers
                    # us, but it is worth reporting rather than hiding.
                    late += 1
        except KeyboardInterrupt:
            termination = Termination("interrupted", "ctrl-c", is_fault=False)
        finally:
            self._settle(termination)

        duration = time.monotonic() - started
        return RunResult(
            steps=step,
            duration_s=duration,
            termination=termination,
            mean_rate_hz=step / duration if duration > 0 else 0.0,
            late_steps=late,
        )

    # --- internals -----------------------------------------------------
    def _emit(self, step: int, ctx: ObsContext, action: list[float], velocity: Velocity) -> None:
        record = {
            "step": step,
            # Same key the telemetry recorder uses, so one reader covers both.
            "t_rel": round(time.monotonic() - self._started, 4),
            "command": list(ctx.command.as_tuple()),
            "action": action,
            "velocity": list(velocity.as_tuple()),
            "standing_command": self._command.standing,
        }
        if ctx.state is not None:
            record["state"] = ctx.state.to_dict()
        try:
            self._step_listener(record)  # type: ignore[misc]
        except Exception as exc:  # noqa: BLE001 - listener is caller code
            log.warning("step listener raised: %s", exc)

    def _settle(self, termination: Termination | None) -> None:
        """Bring the robot down gently, or hard if something actually went wrong."""
        if termination is not None and termination.is_fault:
            log.error("fault termination: %s (%s)", termination.name, termination.detail)
            try:
                self._dog.emergency_stop()
            except Exception as exc:  # noqa: BLE001 - already in the bad path
                log.error("could not deliver emergency stop: %s", exc)
            return

        try:
            self._dog.stop()
            if self._cfg.sit_after:
                time.sleep(0.5)
                self._dog.stand_down()
        except Exception as exc:  # noqa: BLE001 - teardown is best effort
            log.warning("settling failed: %s", exc)
