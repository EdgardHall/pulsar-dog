import math

from pulsar_dog.locomotion.observations import ObsContext
from pulsar_dog.locomotion.terminations import (
    TerminationManager,
    bad_orientation,
    base_height_below,
    battery_below,
    default_terms,
    telemetry_stale,
    time_limit,
)
from pulsar_dog.safety import Velocity
from pulsar_dog.transport.base import RobotState


def ctx(dt=0.02, **state_kwargs):
    state = None
    if state_kwargs.pop("no_state", False) is False:
        state = RobotState(timestamp=0.0, **state_kwargs)
    return ObsContext(
        state=state, command=Velocity.zero(), last_action=Velocity.zero(), dt=dt
    )


def test_bad_orientation_fires_past_the_threshold():
    term = bad_orientation(math.radians(30))
    assert term.func(ctx(rpy=(0.0, math.radians(10), 0.0))) is None
    fired = term.func(ctx(rpy=(0.0, math.radians(45), 0.0)))
    assert fired is not None and "tilt" in fired


def test_bad_orientation_stays_quiet_without_orientation():
    assert bad_orientation().func(ctx(rpy=None)) is None


def test_base_height_and_battery():
    assert base_height_below(0.15).func(ctx(body_height=0.30)) is None
    assert base_height_below(0.15).func(ctx(body_height=0.05)) is not None
    assert battery_below(20).func(ctx(battery_soc=55)) is None
    assert battery_below(20).func(ctx(battery_soc=11)) is not None


def test_telemetry_stale_uses_the_reported_age():
    term = telemetry_stale(0.5)
    fresh = ctx()
    fresh.state.extra["state_age_s"] = 0.1
    assert term.func(fresh) is None
    old = ctx()
    old.state.extra["state_age_s"] = 2.0
    assert term.func(old) is not None


def test_telemetry_stale_fires_when_there_is_no_state():
    assert telemetry_stale().func(ctx(no_state=True)) is not None


def test_time_limit_counts_dt_and_is_not_a_fault():
    term = time_limit(0.1)
    assert term.is_fault is False
    assert term.func(ctx(dt=0.04)) is None
    assert term.func(ctx(dt=0.04)) is None
    assert term.func(ctx(dt=0.04)) is not None


def test_manager_reports_the_first_fired_term():
    manager = TerminationManager(terms=default_terms(duration_s=10.0))
    assert manager.check(ctx(rpy=(0.0, 0.0, 0.0), body_height=0.3, battery_soc=90)) is None
    fired = manager.check(ctx(rpy=(0.0, math.radians(80), 0.0), body_height=0.3, battery_soc=90))
    assert fired is not None
    assert fired.name == "bad_orientation"
    assert fired.is_fault


def test_manager_keeps_stateful_terms_counting_after_another_fires():
    # The time limit must still accumulate while a fault term is firing,
    # otherwise a recovered run would never reach its own end.
    manager = TerminationManager(terms=[battery_below(50), time_limit(0.05)])
    first = manager.check(ctx(dt=0.04, battery_soc=10))
    assert first.name == "battery"
    second = manager.check(ctx(dt=0.04, battery_soc=90))
    assert second is not None and second.name == "time_limit"


def test_empty_manager_never_fires():
    assert TerminationManager().check(ctx()) is None
