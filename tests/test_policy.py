import pytest

from pulsar_dog.locomotion.observations import ObservationManager
from pulsar_dog.locomotion.policy import (
    ACTION_DIM,
    CommandFollowingPolicy,
    PatrolPolicy,
    Policy,
    ZeroPolicy,
)


def test_zero_policy_matches_the_protocol():
    policy = ZeroPolicy()
    assert isinstance(policy, Policy)
    policy.reset()
    assert policy.act([0.0] * 15) == [0.0] * ACTION_DIM


def test_command_following_reproduces_the_command():
    manager = ObservationManager()
    obs_scale = (2.0, 2.0, 0.25)
    action_scale = (0.6, 0.4, 0.8)
    policy = CommandFollowingPolicy(
        manager.term_slices()["velocity_commands"], obs_scale, action_scale
    )
    command = (0.3, -0.1, 0.5)
    obs = [0.0] * manager.dim
    obs[9:12] = [c * s for c, s in zip(command, obs_scale, strict=True)]

    action = policy.act(obs)
    # Scaling the action back up must land on the original command.
    assert [a * s for a, s in zip(action, action_scale, strict=True)] == pytest.approx(command)


def test_command_following_rejects_non_invertible_scales():
    with pytest.raises(ValueError, match="non-zero"):
        CommandFollowingPolicy(slice(0, 3), (2.0, 0.0, 1.0), (1.0, 1.0, 1.0))


def test_command_following_rejects_a_wrong_sized_slice():
    policy = CommandFollowingPolicy(slice(0, 2), (1.0, 1.0, 1.0), (1.0, 1.0, 1.0))
    with pytest.raises(ValueError, match="expected 3"):
        policy.act([0.0, 0.0])


def test_patrol_walks_then_turns():
    policy = PatrolPolicy(period_s=1.0, dt=0.1, speed=0.5)
    actions = [policy.act([]) for _ in range(10)]
    forward = actions[:5]
    turning = actions[5:]
    assert all(a[0] == pytest.approx(0.5) and a[2] == 0.0 for a in forward)
    assert all(a[0] == 0.0 for a in turning)
    assert any(abs(a[2]) > 0.1 for a in turning)


def test_patrol_reset_restarts_the_cycle():
    policy = PatrolPolicy(period_s=1.0, dt=0.1)
    first = policy.act([])
    for _ in range(6):
        policy.act([])
    policy.reset()
    assert policy.act([]) == first


def test_patrol_rejects_bad_timing():
    with pytest.raises(ValueError):
        PatrolPolicy(period_s=0.0)
