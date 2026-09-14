import pytest

from pulsar_dog.config import SafetyLimits
from pulsar_dog.locomotion.commands import (
    FixedVelocityCommand,
    UniformVelocityCommand,
    VelocityCommandCfg,
)
from pulsar_dog.safety import Velocity

LIMITS = SafetyLimits(max_vx=0.6, max_vy=0.4, max_vyaw=0.8)


def test_cfg_rejects_inverted_range():
    with pytest.raises(ValueError, match="inverted"):
        VelocityCommandCfg(lin_vel_x=(1.0, -1.0))


def test_cfg_rejects_bad_resample_and_standing_fraction():
    with pytest.raises(ValueError):
        VelocityCommandCfg(resample_time_s=0.0)
    with pytest.raises(ValueError):
        VelocityCommandCfg(rel_standing=1.5)


def test_ranges_are_shrunk_to_the_safety_envelope():
    cfg = VelocityCommandCfg(lin_vel_x=(-5.0, 5.0), ang_vel_z=(-9.0, 9.0))
    bounded = cfg.bounded_by(LIMITS)
    assert bounded.lin_vel_x == (-0.6, 0.6)
    assert bounded.ang_vel_z == (-0.8, 0.8)


def test_samples_stay_inside_the_envelope():
    cfg = VelocityCommandCfg(
        lin_vel_x=(-9.0, 9.0), lin_vel_y=(-9.0, 9.0), ang_vel_z=(-9.0, 9.0), rel_standing=0.0
    )
    command = UniformVelocityCommand(cfg, LIMITS, seed=1)
    for _ in range(200):
        sample = command.resample()
        assert abs(sample.vx) <= LIMITS.max_vx
        assert abs(sample.vy) <= LIMITS.max_vy
        assert abs(sample.vyaw) <= LIMITS.max_vyaw


def test_resamples_only_when_the_interval_elapses():
    cfg = VelocityCommandCfg(resample_time_s=1.0, rel_standing=0.0)
    command = UniformVelocityCommand(cfg, LIMITS, seed=2)
    first = command.command
    for _ in range(49):
        command.step(0.02)
    assert command.command == first
    command.step(0.02)  # crosses 1.0 s
    assert command.command != first


def test_standing_fraction_is_honoured():
    cfg = VelocityCommandCfg(resample_time_s=1.0, rel_standing=0.5)
    command = UniformVelocityCommand(cfg, LIMITS, seed=3)
    standing = sum(1 for _ in range(400) if command.resample().is_zero())
    assert 150 < standing < 250


def test_seeding_makes_a_run_reproducible():
    cfg = VelocityCommandCfg()
    a = UniformVelocityCommand(cfg, LIMITS, seed=7)
    b = UniformVelocityCommand(cfg, LIMITS, seed=7)
    assert [a.resample() for _ in range(10)] == [b.resample() for _ in range(10)]


def test_standing_flag_tracks_the_command():
    cfg = VelocityCommandCfg(rel_standing=1.0)
    command = UniformVelocityCommand(cfg, LIMITS, seed=4)
    assert command.standing
    assert command.command.is_zero()


def test_fixed_command_is_clamped_and_constant():
    command = FixedVelocityCommand(Velocity(vx=99.0), LIMITS)
    assert command.command == Velocity(0.6, 0.0, 0.0)
    assert command.step(10.0) == command.command
    assert not command.standing
