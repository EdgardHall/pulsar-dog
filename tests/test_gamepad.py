"""Gamepad mapping and the dead-man switch, with no pad plugged in."""

import threading
import time

import pytest

from pulsar_dog.config import Config, SafetyLimits
from pulsar_dog.robot import EmergencyStop, PulsarDog
from pulsar_dog.safety import Velocity
from pulsar_dog.teleop.gamepad import (
    FakeGamepad,
    GamepadMapping,
    GamepadState,
    GamepadTeleop,
    apply_deadzone,
)
from pulsar_dog.transport.base import RobotState

LIMITS = SafetyLimits(max_vx=1.0, max_vy=1.0, max_vyaw=1.0)


class FakeDog:
    """Records what the teleop asked for, without any threading."""

    def __init__(self, limits: SafetyLimits = LIMITS) -> None:
        self.config = Config(safety=limits)
        self.commands: list[Velocity] = []
        self.calls: list[str] = []
        self.emergency_stopped = False
        self.raise_on_command = False

    def set_velocity(self, velocity: Velocity) -> None:
        if self.raise_on_command:
            raise EmergencyStop("latched")
        self.commands.append(velocity)

    def stop(self) -> None:
        self.calls.append("stop")

    def emergency_stop(self) -> None:
        self.calls.append("emergency_stop")
        self.emergency_stopped = True

    def clear_emergency_stop(self) -> None:
        self.calls.append("clear_emergency_stop")
        self.emergency_stopped = False

    def stand_up(self) -> None:
        self.calls.append("stand_up")

    def stand_down(self) -> None:
        self.calls.append("stand_down")

    def balance_stand(self) -> None:
        self.calls.append("balance_stand")

    def recovery_stand(self) -> None:
        self.calls.append("recovery_stand")

    def damp(self) -> None:
        self.calls.append("damp")

    def read_state(self):
        return RobotState(timestamp=0.0, battery_soc=80)


def state(axes=(0.0, 0.0, 0.0, 0.0), buttons=(False,) * 8) -> GamepadState:
    return GamepadState(axes=tuple(axes), buttons=tuple(buttons))


def held(*indices: int, count: int = 8) -> tuple[bool, ...]:
    return tuple(i in indices for i in range(count))


# --- deadzone ----------------------------------------------------------
def test_deadzone_zeroes_the_centre():
    assert apply_deadzone(0.05, 0.12) == 0.0
    assert apply_deadzone(-0.05, 0.12) == 0.0


def test_deadzone_rescales_so_there_is_no_jump():
    # Just past the dead zone must be near zero, not 0.12.
    assert apply_deadzone(0.13, 0.12) == pytest.approx(0.0114, abs=1e-3)
    # And the full deflection must still reach 1.0.
    assert apply_deadzone(1.0, 0.12) == pytest.approx(1.0)
    assert apply_deadzone(-1.0, 0.12) == pytest.approx(-1.0)


def test_deadzone_rejects_impossible_values():
    with pytest.raises(ValueError):
        apply_deadzone(0.5, 1.0)
    with pytest.raises(ValueError):
        apply_deadzone(0.5, -0.1)


def test_axis_and_button_lookups_are_bounds_safe():
    sample = state(axes=(0.5,), buttons=(True,))
    assert sample.axis(0) == 0.5
    assert sample.axis(9) == 0.0
    assert sample.axis(None) == 0.0
    assert sample.button(0) is True
    assert sample.button(9) is False
    assert sample.button(None) is False


# --- the dead-man switch ----------------------------------------------
def test_sticks_do_nothing_while_the_deadman_is_released():
    dog = FakeDog()
    teleop = GamepadTeleop(dog, FakeGamepad())
    # Full forward on the stick, dead-man not held.
    teleop._step(state(axes=(0.0, -1.0, 0.0, 0.0)))
    assert not teleop.armed
    # It still commands zero rather than going silent, so the robot ramps down.
    assert dog.commands == [Velocity.zero()]


def test_holding_the_deadman_arms_the_sticks():
    dog = FakeDog()
    teleop = GamepadTeleop(dog, FakeGamepad())
    teleop._step(state(axes=(0.0, -1.0, 0.0, 0.0), buttons=held(4)))
    assert teleop.armed
    assert dog.commands[-1].vx == pytest.approx(1.0)


def test_releasing_the_deadman_mid_motion_stops_immediately():
    dog = FakeDog()
    teleop = GamepadTeleop(dog, FakeGamepad())
    teleop._step(state(axes=(0.0, -1.0, 0.0, 0.0), buttons=held(4)))
    teleop._step(state(axes=(0.0, -1.0, 0.0, 0.0)))
    assert dog.commands[-1] == Velocity.zero()
    assert not teleop.armed


def test_deadman_can_be_disabled_explicitly():
    dog = FakeDog()
    teleop = GamepadTeleop(dog, FakeGamepad(), GamepadMapping(button_deadman=None))
    teleop._step(state(axes=(0.0, -1.0, 0.0, 0.0)))
    assert teleop.armed
    assert dog.commands[-1].vx == pytest.approx(1.0)


# --- axis mapping ------------------------------------------------------
def test_stick_directions_follow_the_mapping():
    dog = FakeDog()
    teleop = GamepadTeleop(dog, FakeGamepad())
    # Stick up is -1 on the pad and must mean forward.
    teleop._step(state(axes=(0.0, -1.0, 0.0, 0.0), buttons=held(4)))
    assert dog.commands[-1].vx > 0
    # Stick left is -1 and must mean strafe left (+vy).
    teleop._step(state(axes=(-1.0, 0.0, 0.0, 0.0), buttons=held(4)))
    assert dog.commands[-1].vy > 0
    # Right stick left is -1 and must mean turn left (+vyaw).
    teleop._step(state(axes=(0.0, 0.0, 0.0, -1.0), buttons=held(4)))
    assert dog.commands[-1].vyaw > 0


def test_inversions_are_configurable():
    dog = FakeDog()
    mapping = GamepadMapping(invert_vx=False)
    teleop = GamepadTeleop(dog, FakeGamepad(), mapping)
    teleop._step(state(axes=(0.0, -1.0, 0.0, 0.0), buttons=held(4)))
    assert dog.commands[-1].vx < 0


def test_commands_are_scaled_by_the_safety_limits():
    dog = FakeDog(SafetyLimits(max_vx=0.3, max_vy=0.2, max_vyaw=0.4))
    teleop = GamepadTeleop(dog, FakeGamepad())
    teleop._step(state(axes=(-1.0, -1.0, 0.0, -1.0), buttons=held(4)))
    command = dog.commands[-1]
    assert command.vx == pytest.approx(0.3)
    assert command.vy == pytest.approx(0.2)
    assert command.vyaw == pytest.approx(0.4)


# --- buttons -----------------------------------------------------------
def test_buttons_fire_once_per_press_not_once_per_frame():
    dog = FakeDog()
    teleop = GamepadTeleop(dog, FakeGamepad())
    for _ in range(5):
        teleop._step(state(buttons=held(0)))  # A held down for five frames
    assert dog.calls.count("stand_up") == 1
    teleop._step(state())  # released
    teleop._step(state(buttons=held(0)))  # pressed again
    assert dog.calls.count("stand_up") == 2


def test_posture_and_estop_buttons():
    dog = FakeDog()
    teleop = GamepadTeleop(dog, FakeGamepad())
    for button, expected in ((1, "emergency_stop"), (2, "stand_down"), (3, "balance_stand")):
        teleop._step(state(buttons=held(button)))
        teleop._step(state())
        assert expected in dog.calls


def test_estop_can_be_cleared_from_the_pad():
    dog = FakeDog()
    teleop = GamepadTeleop(dog, FakeGamepad())
    teleop._step(state(buttons=held(1)))
    assert dog.emergency_stopped
    teleop._step(state())
    teleop._step(state(buttons=held(6)))
    assert not dog.emergency_stopped


def test_a_latched_estop_does_not_break_the_loop():
    dog = FakeDog()
    dog.raise_on_command = True
    teleop = GamepadTeleop(dog, FakeGamepad())
    teleop._step(state(axes=(0.0, -1.0, 0.0, 0.0), buttons=held(4)))  # must not raise
    teleop._step(state(buttons=held(6)))
    assert "clear_emergency_stop" in dog.calls


def test_slow_button_toggles_the_scale():
    dog = FakeDog()
    teleop = GamepadTeleop(dog, FakeGamepad())
    teleop._step(state(buttons=held(5)))
    teleop._step(state(axes=(0.0, -1.0, 0.0, 0.0), buttons=held(4)))
    assert dog.commands[-1].vx == pytest.approx(0.4)


# --- the loop ----------------------------------------------------------
class CountingBackend:
    name = "counting"

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.calls: list[str] = []

    def connect(self) -> None:
        self.calls.append("connect")

    def close(self) -> None:
        self.calls.append("close")

    def stand_up(self) -> None:
        self.calls.append("stand_up")

    def stand_down(self) -> None:
        self.calls.append("stand_down")

    def balance_stand(self) -> None:
        self.calls.append("balance_stand")

    def recovery_stand(self) -> None:
        self.calls.append("recovery_stand")

    def damp(self) -> None:
        self.calls.append("damp")

    def stop_move(self) -> None:
        self.calls.append("stop_move")

    def move(self, velocity: Velocity) -> None:
        with self.lock:
            self.calls.append("move")

    def read_state(self) -> RobotState:
        return RobotState(timestamp=time.time(), mode=1, battery_soc=75)


def test_a_disconnected_pad_ends_the_run_and_stops_the_robot():
    backend = CountingBackend()
    dog = PulsarDog(Config(control_hz=100.0), backend=backend)
    dog.connect()
    try:
        # Two good frames, then the pad vanishes.
        device = FakeGamepad([state(buttons=held(4)), state(buttons=held(4)), None])
        reason = GamepadTeleop(dog, device, rate_hz=200.0).run()
        assert reason == "gamepad disconnected"
        assert device.closed
    finally:
        dog.close()
    assert "stop_move" in backend.calls


def test_the_loop_opens_and_closes_the_device():
    dog = FakeDog()
    device = FakeGamepad()
    teleop = GamepadTeleop(dog, device, rate_hz=500.0)
    assert teleop.run(max_steps=3) == "stopped"
    assert device.opened and device.closed
    assert "stop" in dog.calls
