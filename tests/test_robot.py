"""Control-loop behaviour, exercised against a recording fake backend.

These tests run the real threaded loop, so they wait on conditions with a
deadline rather than sleeping a fixed amount.
"""

import threading
import time
from dataclasses import replace

import pytest

from pulsar_dog.config import Config, SafetyLimits
from pulsar_dog.robot import EmergencyStop, PulsarDog
from pulsar_dog.safety import Velocity
from pulsar_dog.transport.base import RobotState


class FakeBackend:
    """Records every call; can be told to start failing."""

    name = "fake"

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.calls: list[str] = []
        self.moves: list[Velocity] = []
        self.connected = False
        self.closed = False
        self.fail_move = False

    def _record(self, call: str) -> None:
        with self.lock:
            self.calls.append(call)

    def connect(self) -> None:
        self.connected = True
        self._record("connect")

    def close(self) -> None:
        self.closed = True
        self._record("close")

    def stand_up(self) -> None:
        self._record("stand_up")

    def stand_down(self) -> None:
        self._record("stand_down")

    def balance_stand(self) -> None:
        self._record("balance_stand")

    def damp(self) -> None:
        self._record("damp")

    def stop_move(self) -> None:
        self._record("stop_move")

    def move(self, velocity: Velocity) -> None:
        if self.fail_move:
            raise RuntimeError("backend is unhappy")
        with self.lock:
            self.calls.append("move")
            self.moves.append(velocity)

    def read_state(self) -> RobotState:
        with self.lock:
            last = self.moves[-1] if self.moves else Velocity.zero()
        return RobotState(timestamp=time.time(), mode=1, battery_soc=88, velocity=last)

    # --- helpers for assertions ---------------------------------------
    def snapshot_calls(self) -> list[str]:
        with self.lock:
            return list(self.calls)

    def last_move(self) -> Velocity | None:
        with self.lock:
            return self.moves[-1] if self.moves else None


def wait_for(predicate, timeout: float = 2.0, interval: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


@pytest.fixture
def config() -> Config:
    return Config(
        backend="fake",
        control_hz=100.0,
        safety=SafetyLimits(
            max_vx=1.0,
            max_vy=1.0,
            max_vyaw=1.0,
            max_linear_accel=20.0,
            max_yaw_accel=20.0,
            command_timeout_s=0.15,
        ),
    )


@pytest.fixture
def dog(config):
    backend = FakeBackend()
    handle = PulsarDog(config, backend=backend)
    handle.connect()
    try:
        yield handle, backend
    finally:
        handle.close()


def test_connect_starts_loop_and_reports_backend(dog):
    handle, backend = dog
    assert backend.connected
    assert handle.backend_name == "fake"
    assert wait_for(lambda: handle.read_state() is not None)


def test_velocity_reaches_the_backend(dog):
    handle, backend = dog
    handle.set_velocity(Velocity(vx=0.5))
    assert wait_for(lambda: (backend.last_move() or Velocity.zero()).vx > 0.4)


def test_velocity_is_clamped_to_limits(dog):
    handle, backend = dog
    handle.set_velocity(Velocity(vx=99.0))
    assert wait_for(lambda: (backend.last_move() or Velocity.zero()).vx >= 1.0)
    assert (backend.last_move()).vx == pytest.approx(1.0)


def test_watchdog_stops_a_silent_caller(dog):
    handle, backend = dog
    handle.set_velocity(Velocity(vx=0.8))
    assert wait_for(lambda: (backend.last_move() or Velocity.zero()).vx > 0.5)
    # Stop talking to the robot; the loop must wind it down by itself. Once the
    # ramp reaches zero the loop stops sending move and issues a single halt.
    assert wait_for(lambda: backend.snapshot_calls()[-1] == "stop_move", timeout=2.0)


def test_stop_ramps_down_to_zero(dog):
    handle, backend = dog
    handle.set_velocity(Velocity(vx=0.9))
    assert wait_for(lambda: (backend.last_move() or Velocity.zero()).vx > 0.5)
    handle.stop()
    assert wait_for(lambda: backend.snapshot_calls()[-1] == "stop_move")
    # The ramp came down through intermediate speeds rather than jumping.
    assert 0.0 < backend.last_move().vx < 0.9


def test_emergency_stop_latches_and_blocks_commands(dog):
    handle, backend = dog
    handle.set_velocity(Velocity(vx=0.5))
    assert wait_for(lambda: (backend.last_move() or Velocity.zero()).vx > 0.2)
    handle.emergency_stop()
    assert handle.emergency_stopped
    assert "stop_move" in backend.snapshot_calls()
    with pytest.raises(EmergencyStop):
        handle.set_velocity(Velocity(vx=0.5))
    handle.clear_emergency_stop()
    assert not handle.emergency_stopped
    handle.set_velocity(Velocity(vx=0.3))  # accepted again


def test_emergency_stop_damps_when_configured(config):
    backend = FakeBackend()
    handle = PulsarDog(replace(config, damp_on_estop=True), backend=backend)
    handle.connect()
    try:
        handle.emergency_stop()
        assert "damp" in backend.snapshot_calls()
    finally:
        handle.close()


def test_posture_commands_stop_motion_first(dog):
    handle, backend = dog
    handle.set_velocity(Velocity(vx=0.8))
    assert wait_for(lambda: (backend.last_move() or Velocity.zero()).vx > 0.5)
    handle.stand_down()
    calls = backend.snapshot_calls()
    assert "stand_down" in calls
    # The robot must be halted immediately before it is asked to lie down, with
    # no move command slipping in between.
    assert calls[calls.index("stand_down") - 1] == "stop_move"
    assert "move" not in calls[calls.index("stand_down") :]


def test_state_listener_receives_samples(dog):
    handle, _ = dog
    received: list[RobotState] = []
    handle.set_state_listener(received.append)
    assert wait_for(lambda: len(received) >= 2, timeout=2.0)
    handle.set_state_listener(None)
    assert received[0].battery_soc == 88


def test_listener_exception_does_not_kill_the_loop(dog):
    handle, backend = dog

    def angry(_state):
        raise ValueError("no")

    handle.set_state_listener(angry)
    time.sleep(0.3)
    handle.set_state_listener(None)
    handle.set_velocity(Velocity(vx=0.4))
    assert wait_for(lambda: (backend.last_move() or Velocity.zero()).vx > 0.2)


def test_repeated_backend_failures_latch_estop(dog):
    handle, backend = dog
    backend.fail_move = True
    handle.set_velocity(Velocity(vx=0.5))
    assert wait_for(lambda: handle.emergency_stopped, timeout=3.0)
    assert isinstance(handle.last_loop_error, RuntimeError)
    # It gave up rather than retrying forever, and asked the robot to stop.
    assert "stop_move" in backend.snapshot_calls()


def test_close_stops_and_releases(config):
    backend = FakeBackend()
    handle = PulsarDog(config, backend=backend)
    handle.connect()
    handle.set_velocity(Velocity(vx=0.6))
    wait_for(lambda: (backend.last_move() or Velocity.zero()).vx > 0.3)
    handle.close()
    assert backend.closed
    assert backend.snapshot_calls()[-1] == "close"
    assert "stop_move" in backend.snapshot_calls()


def test_close_is_idempotent(config):
    backend = FakeBackend()
    handle = PulsarDog(config, backend=backend)
    handle.connect()
    handle.close()
    handle.close()  # must not raise


def test_context_manager_closes_on_exception(config):
    backend = FakeBackend()
    with pytest.raises(ZeroDivisionError):
        with PulsarDog(config, backend=backend):
            raise ZeroDivisionError
    assert backend.closed
