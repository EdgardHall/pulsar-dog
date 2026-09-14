import pytest

from pulsar_dog.config import SafetyLimits
from pulsar_dog.safety import RateLimiter, Velocity, Watchdog, clamp


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt


def test_clamp_bounds():
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0
    assert clamp(0.5, -1.0, 1.0) == 0.5


def test_clamp_rejects_inverted_bounds():
    with pytest.raises(ValueError):
        clamp(0.0, 1.0, -1.0)


def test_velocity_clamped_to_limits():
    limits = SafetyLimits(max_vx=0.6, max_vy=0.4, max_vyaw=0.8)
    clamped = Velocity(10.0, -10.0, 10.0).clamped(limits)
    assert clamped == Velocity(0.6, -0.4, 0.8)


def test_safety_limits_reject_non_positive():
    with pytest.raises(ValueError):
        SafetyLimits(max_vx=0.0)


def test_rate_limiter_ramps_instead_of_stepping():
    limits = SafetyLimits(max_vx=2.0, max_linear_accel=1.0)
    limiter = RateLimiter(limits)
    # One 100 ms tick at 1 m/s^2 can only add 0.1 m/s.
    assert limiter.step(Velocity(vx=2.0), dt=0.1).vx == pytest.approx(0.1)
    assert limiter.step(Velocity(vx=2.0), dt=0.1).vx == pytest.approx(0.2)


def test_rate_limiter_reaches_target_without_overshoot():
    limits = SafetyLimits(max_vx=2.0, max_linear_accel=1.0)
    limiter = RateLimiter(limits)
    for _ in range(100):
        limiter.step(Velocity(vx=0.5), dt=0.1)
    assert limiter.current.vx == pytest.approx(0.5)


def test_rate_limiter_clamps_target_before_ramping():
    limits = SafetyLimits(max_vx=0.5, max_linear_accel=100.0)
    limiter = RateLimiter(limits)
    assert limiter.step(Velocity(vx=9.0), dt=0.1).vx == pytest.approx(0.5)


def test_rate_limiter_ignores_non_positive_dt():
    limiter = RateLimiter(SafetyLimits())
    limiter.step(Velocity(vx=0.1), dt=0.1)
    before = limiter.current
    assert limiter.step(Velocity(vx=1.0), dt=0.0) == before


def test_watchdog_expires_then_recovers_on_poke():
    clock = FakeClock()
    watchdog = Watchdog(0.3, clock=clock)
    assert not watchdog.expired()
    clock.advance(0.31)
    assert watchdog.expired()
    watchdog.poke()
    assert not watchdog.expired()


def test_watchdog_rejects_bad_timeout():
    with pytest.raises(ValueError):
        Watchdog(0.0)
