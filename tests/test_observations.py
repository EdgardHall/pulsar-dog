import math

import pytest

from pulsar_dog.locomotion.observations import (
    ObsContext,
    ObservationManager,
    ObservationTerm,
    projected_gravity,
    tilt_angle,
)
from pulsar_dog.safety import Velocity
from pulsar_dog.transport.base import RobotState


def ctx(rpy=None, velocity=None, command=None, last_action=None):
    return ObsContext(
        state=RobotState(timestamp=0.0, rpy=rpy, velocity=velocity),
        command=command or Velocity.zero(),
        last_action=last_action or Velocity.zero(),
        dt=0.02,
    )


def test_projected_gravity_when_level():
    assert projected_gravity((0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0, -1.0))


def test_projected_gravity_ignores_yaw():
    flat = projected_gravity((0.1, -0.2, 0.0))
    turned = projected_gravity((0.1, -0.2, 2.5))
    assert flat == pytest.approx(turned)


def test_projected_gravity_tips_with_pitch_and_roll():
    # Nose down 90 degrees: gravity now points along the body's +x axis.
    assert projected_gravity((0.0, math.pi / 2, 0.0)) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    # Rolled 90 degrees to the right.
    assert projected_gravity((math.pi / 2, 0.0, 0.0)) == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)


def test_projected_gravity_defaults_to_level_without_telemetry():
    assert projected_gravity(None) == (0.0, 0.0, -1.0)


def test_tilt_angle():
    assert tilt_angle((0.0, 0.0, 0.0)) == pytest.approx(0.0)
    assert tilt_angle((0.0, math.radians(30), 0.0)) == pytest.approx(math.radians(30))
    assert tilt_angle((math.pi / 2, 0.0, 0.0)) == pytest.approx(math.pi / 2)


def test_term_applies_scalar_and_per_element_scales():
    term = ObservationTerm("t", lambda _c: (1.0, 2.0, 3.0), 3, scale=2.0)
    assert term.compute(ctx()) == [2.0, 4.0, 6.0]
    term = ObservationTerm("t", lambda _c: (1.0, 2.0, 3.0), 3, scale=(1.0, 0.5, 0.0))
    assert term.compute(ctx()) == [1.0, 1.0, 0.0]


def test_term_clips_after_scaling():
    term = ObservationTerm("t", lambda _c: (10.0,), 1, scale=2.0, clip=(-5.0, 5.0))
    assert term.compute(ctx()) == [5.0]


def test_term_rejects_wrong_dimension():
    term = ObservationTerm("t", lambda _c: (1.0, 2.0), 3)
    with pytest.raises(ValueError, match="expected 3"):
        term.compute(ctx())


def test_term_rejects_mismatched_scale_count():
    term = ObservationTerm("t", lambda _c: (1.0, 2.0, 3.0), 3, scale=(1.0, 2.0))
    with pytest.raises(ValueError, match="scales"):
        term.compute(ctx())


def test_manager_layout_is_stable():
    manager = ObservationManager()
    slices = manager.term_slices()
    assert manager.dim == 15
    assert slices["velocity_commands"] == slice(9, 12)
    assert list(slices) == [
        "base_lin_vel",
        "base_ang_vel",
        "projected_gravity",
        "velocity_commands",
        "last_action",
    ]


def test_manager_rejects_duplicate_terms():
    term = ObservationTerm("same", lambda _c: (0.0,), 1)
    with pytest.raises(ValueError, match="duplicate"):
        ObservationManager(terms=[term, term])


def test_manager_places_values_in_the_declared_slots():
    manager = ObservationManager()
    obs = manager.compute(
        ctx(
            rpy=(0.0, 0.0, 0.0),
            velocity=Velocity(0.5, 0.1, 0.4),
            command=Velocity(0.3, 0.0, 0.8),
            last_action=Velocity(0.2, 0.0, 0.0),
        )
    )
    slices = manager.term_slices()
    assert len(obs) == manager.dim
    # base_lin_vel is scaled by 2.0, base_ang_vel by 0.25.
    assert obs[slices["base_lin_vel"]] == pytest.approx([1.0, 0.2, 0.0])
    assert obs[slices["base_ang_vel"]] == pytest.approx([0.0, 0.0, 0.1])
    assert obs[slices["projected_gravity"]] == pytest.approx([0.0, 0.0, -1.0])
    # velocity_commands uses per-axis scales (2, 2, 0.25).
    assert obs[slices["velocity_commands"]] == pytest.approx([0.6, 0.0, 0.2])
    assert obs[slices["last_action"]] == pytest.approx([0.2, 0.0, 0.0])


def test_manager_survives_missing_telemetry():
    manager = ObservationManager()
    obs = manager.compute(
        ObsContext(state=None, command=Velocity.zero(), last_action=Velocity.zero(), dt=0.02)
    )
    assert len(obs) == manager.dim
    # Missing orientation reads as level rather than as zeros.
    assert obs[manager.term_slices()["projected_gravity"]] == pytest.approx([0.0, 0.0, -1.0])


def test_manager_applies_the_global_clip():
    term = ObservationTerm("huge", lambda _c: (1e9,), 1)
    manager = ObservationManager(terms=[term], clip=(-100.0, 100.0))
    assert manager.compute(ctx()) == [100.0]
