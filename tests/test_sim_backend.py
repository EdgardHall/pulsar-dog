import math

import pytest

from pulsar_dog.config import Config
from pulsar_dog.safety import Velocity
from pulsar_dog.transport.base import BackendError
from pulsar_dog.transport.sim import MODE_BALANCE_STAND, MODE_WALK, SimBackend


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt


@pytest.fixture
def sim():
    clock = FakeClock()
    backend = SimBackend(Config(backend="sim"), clock=clock)
    backend.connect()
    return backend, clock


def test_commands_before_connect_raise():
    backend = SimBackend(Config(backend="sim"))
    with pytest.raises(BackendError):
        backend.stand_up()


def test_walking_forward_integrates_x(sim):
    backend, clock = sim
    backend.stand_up()
    backend.move(Velocity(vx=0.5))
    clock.advance(2.0)
    state = backend.read_state()
    assert state.position[0] == pytest.approx(1.0)
    assert state.position[1] == pytest.approx(0.0)
    assert state.mode == MODE_WALK


def test_turning_then_walking_changes_heading(sim):
    backend, clock = sim
    backend.stand_up()
    backend.move(Velocity(vyaw=math.pi / 2))
    clock.advance(1.0)
    backend.stop_move()
    state = backend.read_state()
    assert state.rpy[2] == pytest.approx(math.pi / 2, abs=1e-6)
    assert state.mode == MODE_BALANCE_STAND

    backend.move(Velocity(vx=1.0))
    clock.advance(1.0)
    state = backend.read_state()
    # Heading is +90 degrees, so forward motion goes along +y.
    assert state.position[0] == pytest.approx(0.0, abs=1e-6)
    assert state.position[1] == pytest.approx(1.0, abs=1e-6)


def test_move_is_ignored_while_lying_down(sim):
    backend, clock = sim
    backend.stand_down()
    backend.move(Velocity(vx=1.0))
    clock.advance(1.0)
    state = backend.read_state()
    assert state.position[0] == pytest.approx(0.0)
    assert state.velocity.is_zero()


def test_damp_stops_and_drops(sim):
    backend, clock = sim
    backend.stand_up()
    backend.move(Velocity(vx=0.5))
    clock.advance(1.0)
    backend.damp()
    clock.advance(1.0)
    state = backend.read_state()
    # Position froze at the moment of damping, not at zero.
    assert state.position[0] == pytest.approx(0.5)
    assert state.velocity.is_zero()
    assert state.foot_force == (0, 0, 0, 0)


def test_recovery_stand_gets_up_from_a_damped_heap(sim):
    backend, clock = sim
    backend.damp()
    # A damped robot ignores walk commands; recovery must put it back on its feet.
    backend.move(Velocity(vx=0.5))
    clock.advance(1.0)
    assert backend.read_state().position[0] == pytest.approx(0.0)

    backend.recovery_stand()
    backend.move(Velocity(vx=0.5))
    clock.advance(1.0)
    state = backend.read_state()
    assert state.position[0] == pytest.approx(0.5)
    assert state.foot_force == (120, 120, 120, 120)
