"""Keyboard teleoperation.

A terminal reports key *presses*, never releases, so holding a key is inferred
from auto-repeat: each press refreshes a short per-axis deadline and an axis
with an expired deadline decays to zero. Let go of the key and the robot stops
within ~200 ms without you touching anything.

Both QWERTY (WASD) and AZERTY (ZQSD) layouts are bound, plus the arrow keys.
"""

from __future__ import annotations

import select
import sys
import termios
import time
import tty
from dataclasses import dataclass, field

from pulsar_dog.robot import EmergencyStop, PulsarDog
from pulsar_dog.safety import Velocity

# How long a single keypress keeps its axis alive. Must exceed the terminal's
# auto-repeat interval (typically ~30 ms) and stay under the command watchdog.
KEY_HOLD_S = 0.22

ESC = "\x1b"
CTRL_C = "\x03"

ARROWS = {
    "A": "up",
    "B": "down",
    "C": "right",
    "D": "left",
}


@dataclass
class KeyMap:
    """Which keys drive which axis. ``+1``/``-1`` is the sign applied to it."""

    forward: tuple[str, ...] = ("w", "z", "up")
    backward: tuple[str, ...] = ("s", "down")
    strafe_left: tuple[str, ...] = ("a", "q")
    strafe_right: tuple[str, ...] = ("d",)
    yaw_left: tuple[str, ...] = ("j", "left")
    yaw_right: tuple[str, ...] = ("l", "right")
    stop: tuple[str, ...] = (" ",)
    stand_up: tuple[str, ...] = ("1",)
    stand_down: tuple[str, ...] = ("2",)
    balance_stand: tuple[str, ...] = ("3",)
    damp: tuple[str, ...] = ("m",)
    emergency_stop: tuple[str, ...] = ("e",)
    clear_estop: tuple[str, ...] = ("r",)
    faster: tuple[str, ...] = ("+", "=")
    slower: tuple[str, ...] = ("-", "_")
    quit: tuple[str, ...] = (ESC, CTRL_C)

    def help_lines(self) -> list[str]:
        return [
            "  z/w s   forward / backward      a/q d   strafe left / right",
            "  j l     turn left / right       arrows  same, on any layout",
            "  space   stop                    + -     speed scale",
            "  1 2 3   stand up / lie down / balance stand",
            "  m       damp (joints go limp - only from a low posture)",
            "  e       EMERGENCY STOP          r       release e-stop",
            "  esc     quit (stops the robot first)",
        ]


@dataclass
class _Axes:
    """Per-axis magnitudes with the deadline that lets a released key decay to zero."""

    values: dict[str, float] = field(default_factory=lambda: {"vx": 0.0, "vy": 0.0, "vyaw": 0.0})
    deadlines: dict[str, float] = field(
        default_factory=lambda: {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
    )

    def press(self, axis: str, sign: float, now: float) -> None:
        self.values[axis] = sign
        self.deadlines[axis] = now + KEY_HOLD_S

    def expire(self, now: float) -> None:
        for axis, deadline in self.deadlines.items():
            if now > deadline:
                self.values[axis] = 0.0

    def clear(self) -> None:
        for axis in self.values:
            self.values[axis] = 0.0
            self.deadlines[axis] = 0.0


class KeyboardTeleop:
    """Drives a :class:`PulsarDog` from the terminal until the user quits."""

    def __init__(
        self,
        dog: PulsarDog,
        keymap: KeyMap | None = None,
        rate_hz: float = 30.0,
        stream=None,
    ) -> None:
        self._dog = dog
        self._keys = keymap or KeyMap()
        self._rate_hz = rate_hz
        self._out = stream if stream is not None else sys.stdout
        self._axes = _Axes()
        self._scale = 1.0
        self._message = "ready"

    # --- public --------------------------------------------------------
    def run(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError(
                "keyboard teleop needs an interactive terminal (stdin is not a tty)"
            )
        self._print_header()
        fd = sys.stdin.fileno()
        saved = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            self._loop()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
            self._out.write("\n")
            self._out.flush()

    # --- internals -----------------------------------------------------
    def _print_header(self) -> None:
        limits = self._dog.config.safety
        self._out.write(f"pulsar-dog teleop - backend: {self._dog.backend_name}\n")
        self._out.write(
            f"limits: vx +-{limits.max_vx} m/s, vy +-{limits.max_vy} m/s, "
            f"vyaw +-{limits.max_vyaw} rad/s\n"
        )
        for line in self._keys.help_lines():
            self._out.write(line + "\n")
        self._out.write("\n")
        self._out.flush()

    def _loop(self) -> None:
        period = 1.0 / self._rate_hz
        running = True
        while running:
            tick = time.monotonic()
            for key in self._read_keys():
                if not self._handle_key(key):
                    running = False
                    break
            now = time.monotonic()
            self._axes.expire(now)
            self._push_velocity()
            self._render()
            sleep_for = period - (time.monotonic() - tick)
            if sleep_for > 0:
                time.sleep(sleep_for)
        self._dog.stop()

    def _read_keys(self) -> list[str]:
        """Non-blocking drain of stdin, decoding arrow escape sequences."""
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return []
        data = sys.stdin.read(1)
        # Drain whatever else arrived in the same burst (auto-repeat, arrows).
        while select.select([sys.stdin], [], [], 0)[0]:
            data += sys.stdin.read(1)

        keys: list[str] = []
        index = 0
        while index < len(data):
            char = data[index]
            if char == ESC and data[index + 1 : index + 2] == "[":
                final = data[index + 2 : index + 3]
                if final in ARROWS:
                    keys.append(ARROWS[final])
                    index += 3
                    continue
            keys.append(char.lower() if len(char) == 1 else char)
            index += 1
        return keys

    def _handle_key(self, key: str) -> bool:
        """Apply one key. Returns False when the user asked to quit."""
        now = time.monotonic()
        keys = self._keys

        if key in keys.quit:
            self._message = "quitting"
            return False
        if key in keys.forward:
            self._axes.press("vx", 1.0, now)
        elif key in keys.backward:
            self._axes.press("vx", -1.0, now)
        elif key in keys.strafe_left:
            self._axes.press("vy", 1.0, now)
        elif key in keys.strafe_right:
            self._axes.press("vy", -1.0, now)
        elif key in keys.yaw_left:
            self._axes.press("vyaw", 1.0, now)
        elif key in keys.yaw_right:
            self._axes.press("vyaw", -1.0, now)
        elif key in keys.stop:
            self._axes.clear()
            self._dog.stop()
            self._message = "stop"
        elif key in keys.faster:
            self._scale = min(1.0, round(self._scale + 0.1, 2))
            self._message = f"scale {self._scale:.1f}"
        elif key in keys.slower:
            self._scale = max(0.1, round(self._scale - 0.1, 2))
            self._message = f"scale {self._scale:.1f}"
        elif key in keys.emergency_stop:
            self._axes.clear()
            self._safely(self._dog.emergency_stop, "E-STOP")
        elif key in keys.clear_estop:
            self._safely(self._dog.clear_emergency_stop, "e-stop released")
        elif key in keys.stand_up:
            self._axes.clear()
            self._safely(self._dog.stand_up, "standing up")
        elif key in keys.stand_down:
            self._axes.clear()
            self._safely(self._dog.stand_down, "lying down")
        elif key in keys.balance_stand:
            self._axes.clear()
            self._safely(self._dog.balance_stand, "balance stand")
        elif key in keys.damp:
            self._axes.clear()
            self._safely(self._dog.damp, "damped")
        return True

    def _safely(self, action, label: str) -> None:
        """Run a robot command without letting a failure drop the terminal."""
        try:
            action()
            self._message = label
        except Exception as exc:  # noqa: BLE001 - surfaced on the status line
            self._message = f"{label} failed: {exc}"

    def _push_velocity(self) -> None:
        limits = self._dog.config.safety
        axes = self._axes.values
        target = Velocity(
            vx=axes["vx"] * limits.max_vx * self._scale,
            vy=axes["vy"] * limits.max_vy * self._scale,
            vyaw=axes["vyaw"] * limits.max_vyaw * self._scale,
        )
        try:
            self._dog.set_velocity(target)
        except EmergencyStop:
            # Expected while latched: keep the loop alive so 'r' still works.
            pass
        except Exception as exc:  # noqa: BLE001 - surfaced on the status line
            self._message = f"command failed: {exc}"

    def _render(self) -> None:
        state = self._dog.read_state()
        axes = self._axes.values
        limits = self._dog.config.safety
        battery = "--"
        mode = "--"
        if state is not None:
            if state.battery_soc is not None:
                battery = f"{state.battery_soc}%"
            if state.mode is not None:
                mode = str(state.mode)
        flag = "ESTOP" if self._dog.emergency_stopped else "     "
        line = (
            f"\r{flag} x{self._scale:.1f} "
            f"vx={axes['vx'] * limits.max_vx * self._scale:+.2f} "
            f"vy={axes['vy'] * limits.max_vy * self._scale:+.2f} "
            f"vyaw={axes['vyaw'] * limits.max_vyaw * self._scale:+.2f} "
            f"| mode={mode} batt={battery} | {self._message[:40]:<40}"
        )
        self._out.write(line)
        self._out.flush()
