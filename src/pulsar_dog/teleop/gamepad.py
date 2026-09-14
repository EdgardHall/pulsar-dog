"""Gamepad teleoperation, with a dead-man switch.

Two things a keyboard cannot give you, and both matter once a real 15 kg robot
is walking:

* **analogue axes** - a stick asks for 0.12 m/s, a key only ever asks for the
  maximum;
* **a dead-man switch** - motion is only authorised while a button is *held*.
  Let go, drop the pad, walk away: the robot stops. That is a property of the
  input device, not of the software noticing something is wrong.

The device itself sits behind a protocol, the same way the robot does, so the
mapping and the safety logic are tested without any hardware plugged in.

Button and axis numbering varies between pads, drivers and platforms. Do not
trust the defaults here - run ``pulsar-dog gamepad-probe`` and read the indices
off your own pad.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from pulsar_dog.robot import EmergencyStop, PulsarDog
from pulsar_dog.safety import Velocity, clamp

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class GamepadState:
    """One sample of every axis and button on the pad."""

    axes: tuple[float, ...] = ()
    buttons: tuple[bool, ...] = ()

    def axis(self, index: int | None) -> float:
        if index is None or index < 0 or index >= len(self.axes):
            return 0.0
        return self.axes[index]

    def button(self, index: int | None) -> bool:
        if index is None or index < 0 or index >= len(self.buttons):
            return False
        return self.buttons[index]


class GamepadUnavailable(RuntimeError):
    """No usable pad: none plugged in, or the driver is missing."""


@runtime_checkable
class GamepadDevice(Protocol):
    name: str

    def open(self) -> None: ...

    def close(self) -> None: ...

    def poll(self) -> GamepadState | None:
        """Current state, or None when the pad has gone away."""


@dataclass(frozen=True)
class GamepadMapping:
    """Which axis and button does what.

    Defaults follow the common Xbox-style layout under SDL/pygame. Sticks report
    -1 at up/left, so the forward and left axes are inverted by default.
    """

    axis_vx: int = 1
    axis_vy: int = 0
    axis_vyaw: int = 3
    invert_vx: bool = True
    invert_vy: bool = True
    invert_vyaw: bool = True
    # Motion is only authorised while this is held. None disables the dead-man
    # switch, which is a deliberate, explicit choice - never a default.
    button_deadman: int | None = 4  # LB
    button_emergency_stop: int | None = 1  # B
    button_clear_estop: int | None = 6  # Back / Select
    button_stand_up: int | None = 0  # A
    button_stand_down: int | None = 2  # X
    button_balance_stand: int | None = 3  # Y
    button_damp: int | None = 7  # Start
    button_slower: int | None = 5  # RB
    # Sticks rest slightly off centre; below this they read as zero.
    deadzone: float = 0.12

    def help_lines(self) -> list[str]:
        return [
            f"  stick gauche      avancer / translater   (axes {self.axis_vx}, {self.axis_vy})",
            f"  stick droit       tourner                (axe {self.axis_vyaw})",
            f"  bouton {self.button_deadman}          HOMME-MORT - a maintenir pour bouger",
            f"  bouton {self.button_emergency_stop}          arret d'urgence"
            f"   ·  bouton {self.button_clear_estop}  rearmement",
            f"  boutons {self.button_stand_up} {self.button_stand_down} "
            f"{self.button_balance_stand}      debout / couche / balance stand",
            f"  bouton {self.button_damp}          damp   ·  bouton {self.button_slower}"
            f"  vitesse reduite",
            "  ctrl-c            quitter (arrete le robot avant)",
        ]


def apply_deadzone(value: float, deadzone: float) -> float:
    """Zero the centre of the axis and rescale the rest to the full range.

    Without the rescale, the first millimetre past the dead zone is a jump to
    ``deadzone`` worth of speed.
    """
    if deadzone < 0 or deadzone >= 1:
        raise ValueError("deadzone must be within [0, 1)")
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * clamp((magnitude - deadzone) / (1.0 - deadzone), 0.0, 1.0)


class FakeGamepad:
    """A scripted pad for tests: hand it the states it should report."""

    name = "fake"

    def __init__(self, states: list[GamepadState | None] | None = None) -> None:
        self.states = list(states or [])
        self.opened = False
        self.closed = False
        self._current = GamepadState()

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.closed = True

    def set_state(self, state: GamepadState) -> None:
        self._current = state

    def poll(self) -> GamepadState | None:
        if self.states:
            return self.states.pop(0)
        return self._current


class PygameGamepad:
    """The real pad, read through pygame's joystick module."""

    def __init__(self, index: int = 0) -> None:
        self._index = index
        self._pygame = None
        self._joystick = None
        self.name = f"gamepad{index}"

    def open(self) -> None:
        try:
            import pygame  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - needs pygame
            raise GamepadUnavailable(
                "gamepad support needs pygame: pip install 'pulsar-dog[gamepad]'"
            ) from exc
        # Only the joystick subsystem: no window, no audio device.
        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count <= self._index:  # pragma: no cover - needs a pad
            raise GamepadUnavailable(
                f"no gamepad at index {self._index} ({count} connected). "
                "Plug it in, or use keyboard teleop."
            )
        joystick = pygame.joystick.Joystick(self._index)
        joystick.init()
        self._pygame = pygame
        self._joystick = joystick
        self.name = joystick.get_name()
        log.info(
            "gamepad %r: %d axes, %d buttons",
            self.name,
            joystick.get_numaxes(),
            joystick.get_numbuttons(),
        )

    def close(self) -> None:  # pragma: no cover - needs pygame
        if self._joystick is not None:
            self._joystick.quit()
            self._joystick = None
        if self._pygame is not None:
            self._pygame.joystick.quit()
            self._pygame.quit()
            self._pygame = None

    def poll(self) -> GamepadState | None:  # pragma: no cover - needs a pad
        if self._pygame is None or self._joystick is None:
            return None
        try:
            self._pygame.event.pump()
            axes = tuple(self._joystick.get_axis(i) for i in range(self._joystick.get_numaxes()))
            buttons = tuple(
                bool(self._joystick.get_button(i))
                for i in range(self._joystick.get_numbuttons())
            )
        except Exception as exc:  # noqa: BLE001 - unplugged mid-run
            log.error("gamepad read failed (unplugged?): %s", exc)
            return None
        return GamepadState(axes=axes, buttons=buttons)


@dataclass
class _Edges:
    """Turns held buttons into one-shot presses."""

    previous: tuple[bool, ...] = field(default_factory=tuple)

    def pressed(self, state: GamepadState, index: int | None) -> bool:
        if index is None:
            return False
        now = state.button(index)
        was = index < len(self.previous) and self.previous[index]
        return now and not was

    def update(self, state: GamepadState) -> None:
        self.previous = state.buttons


class GamepadTeleop:
    """Drives a :class:`PulsarDog` from a pad until Ctrl-C or the pad vanishes."""

    def __init__(
        self,
        dog: PulsarDog,
        device: GamepadDevice,
        mapping: GamepadMapping | None = None,
        rate_hz: float = 50.0,
        stream=None,
    ) -> None:
        self._dog = dog
        self._device = device
        self._map = mapping or GamepadMapping()
        self._rate_hz = rate_hz
        self._out = stream
        self._edges = _Edges()
        self._scale = 1.0
        self._message = "ready"
        self._armed = False

    @property
    def armed(self) -> bool:
        """True while the dead-man switch is held."""
        return self._armed

    def run(self, max_steps: int | None = None) -> str:
        """Returns why the loop ended."""
        self._device.open()
        self._print_header()
        period = 1.0 / self._rate_hz
        reason = "stopped"
        steps = 0
        try:
            while max_steps is None or steps < max_steps:
                tick = time.monotonic()
                state = self._device.poll()
                if state is None:
                    # The pad is the authority on whether motion is allowed; if
                    # it is gone, nothing is allowed.
                    reason = "gamepad disconnected"
                    self._message = reason
                    log.error("%s - stopping", reason)
                    break
                self._step(state)
                self._render()
                steps += 1
                sleep_for = period - (time.monotonic() - tick)
                if sleep_for > 0:
                    time.sleep(sleep_for)
        except KeyboardInterrupt:
            reason = "interrupted"
        finally:
            self._armed = False
            try:
                self._dog.stop()
            except Exception as exc:  # noqa: BLE001 - teardown is best effort
                log.warning("stop on exit failed: %s", exc)
            self._device.close()
            if self._out is not None:
                self._out.write("\n")
                self._out.flush()
        return reason

    # --- internals -----------------------------------------------------
    def _step(self, state: GamepadState) -> None:
        mapping = self._map
        self._handle_buttons(state)

        self._armed = mapping.button_deadman is None or state.button(mapping.button_deadman)
        if not self._armed:
            # Feed zero rather than going silent: the robot ramps down cleanly
            # instead of waiting for the watchdog to notice.
            self._command(Velocity.zero())
            return

        limits = self._dog.config.safety
        scale = self._scale
        vx = apply_deadzone(state.axis(mapping.axis_vx), mapping.deadzone)
        vy = apply_deadzone(state.axis(mapping.axis_vy), mapping.deadzone)
        vyaw = apply_deadzone(state.axis(mapping.axis_vyaw), mapping.deadzone)
        if mapping.invert_vx:
            vx = -vx
        if mapping.invert_vy:
            vy = -vy
        if mapping.invert_vyaw:
            vyaw = -vyaw
        self._command(
            Velocity(
                vx * limits.max_vx * scale,
                vy * limits.max_vy * scale,
                vyaw * limits.max_vyaw * scale,
            )
        )

    def _handle_buttons(self, state: GamepadState) -> None:
        mapping = self._map
        edges = self._edges
        if edges.pressed(state, mapping.button_emergency_stop):
            self._safely(self._dog.emergency_stop, "E-STOP")
        elif edges.pressed(state, mapping.button_clear_estop):
            self._safely(self._dog.clear_emergency_stop, "e-stop released")
        elif edges.pressed(state, mapping.button_stand_up):
            self._safely(self._dog.stand_up, "standing up")
        elif edges.pressed(state, mapping.button_stand_down):
            self._safely(self._dog.stand_down, "lying down")
        elif edges.pressed(state, mapping.button_balance_stand):
            self._safely(self._dog.balance_stand, "balance stand")
        elif edges.pressed(state, mapping.button_damp):
            self._safely(self._dog.damp, "damped")
        elif edges.pressed(state, mapping.button_slower):
            self._scale = 0.4 if self._scale > 0.5 else 1.0
            self._message = f"scale {self._scale:.1f}"
        edges.update(state)

    def _command(self, velocity: Velocity) -> None:
        try:
            self._dog.set_velocity(velocity)
        except EmergencyStop:
            # Expected while latched; the clear button must keep working.
            pass
        except Exception as exc:  # noqa: BLE001 - surfaced on the status line
            self._message = f"command failed: {exc}"

    def _safely(self, action, label: str) -> None:
        try:
            action()
            self._message = label
        except Exception as exc:  # noqa: BLE001 - surfaced on the status line
            self._message = f"{label} failed: {exc}"

    def _print_header(self) -> None:
        if self._out is None:
            return
        limits = self._dog.config.safety
        self._out.write(f"pulsar-dog gamepad teleop - {self._device.name}\n")
        self._out.write(
            f"limits: vx +-{limits.max_vx} m/s, vy +-{limits.max_vy} m/s, "
            f"vyaw +-{limits.max_vyaw} rad/s\n"
        )
        for line in self._map.help_lines():
            self._out.write(line + "\n")
        self._out.write("\n")
        self._out.flush()

    def _render(self) -> None:
        if self._out is None:
            return
        state = self._dog.read_state()
        battery = "--"
        if state is not None and state.battery_soc is not None:
            battery = f"{state.battery_soc}%"
        flag = "ESTOP" if self._dog.emergency_stopped else ("ARMED" if self._armed else "  -  ")
        self._out.write(
            f"\r{flag} x{self._scale:.1f} batt={battery} | {self._message[:44]:<44}"
        )
        self._out.flush()
