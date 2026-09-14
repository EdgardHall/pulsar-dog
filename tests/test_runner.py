"""The policy loop, against a fake robot whose orientation the test controls."""

import math
import threading
import time

import pytest

from pulsar_dog.config import Config, SafetyLimits
from pulsar_dog.locomotion.commands import FixedVelocityCommand
from pulsar_dog.locomotion.observations import ObservationManager
from pulsar_dog.locomotion.policy import CommandFollowingPolicy, ZeroPolicy
from pulsar_dog.locomotion.runner import ActionCfg, LocomotionCfg, LocomotionRunner
from pulsar_dog.locomotion.terminations import TerminationManager, bad_orientation, time_limit
from pulsar_dog.robot import PulsarDog
from pulsar_dog.safety import Velocity
from pulsar_dog.transport.base import RobotState


class ScriptableBackend:
    """A standing robot whose telemetry the test can rewrite mid-run."""

    name = "scriptable"

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.rpy = (0.0, 0.0, 0.0)
        self.body_height = 0.32
        self.battery_soc = 90
        self.moves: list[Velocity] = []
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
            self.moves.append(velocity)

    def read_state(self) -> RobotState:
        with self.lock:
            return RobotState(
                timestamp=time.time(),
                mode=1,
                rpy=self.rpy,
                body_height=self.body_height,
                battery_soc=self.battery_soc,
                velocity=self.moves[-1] if self.moves else Velocity.zero(),
                extra={"state_age_s": 0.01},
            )

    def tip_over(self) -> None:
        with self.lock:
            self.rpy = (0.0, math.radians(70), 0.0)

    def peak_vx(self) -> float:
        with self.lock:
            return max((m.vx for m in self.moves), default=0.0)


@pytest.fixture
def config() -> Config:
    return Config(
        backend="scriptable",
        control_hz=100.0,
        safety=SafetyLimits(
            max_vx=1.0,
            max_vy=1.0,
            max_vyaw=1.0,
            max_linear_accel=20.0,
            max_yaw_accel=20.0,
            command_timeout_s=0.3,
        ),
    )


@pytest.fixture
def dog(config):
    backend = ScriptableBackend()
    handle = PulsarDog(config, backend=backend)
    handle.connect()
    try:
        yield handle, backend
    finally:
        handle.close()


def fast_cfg(**kwargs) -> LocomotionCfg:
    kwargs.setdefault("policy_hz", 100.0)
    kwargs.setdefault("stand_first", False)
    return LocomotionCfg(**kwargs)


# --- action scaling ----------------------------------------------------
def test_action_scaling_and_clipping():
    cfg = ActionCfg(scale=(0.6, 0.4, 0.8), clip=(-1.0, 1.0))
    assert cfg.to_velocity([0.5, -0.5, 1.0]) == Velocity(0.3, -0.2, 0.8)
    # Out-of-range actions are clipped before scaling, never scaled past the limit.
    assert cfg.to_velocity([9.0, -9.0, 0.0]) == Velocity(0.6, -0.4, 0.0)


def test_action_rejects_wrong_dimension():
    with pytest.raises(ValueError, match="expected 3"):
        ActionCfg().to_velocity([0.0, 0.0])


def test_decimation_is_the_ratio_of_rates():
    assert LocomotionCfg(policy_hz=50.0).decimation(200.0) == 4.0


# --- the loop ----------------------------------------------------------
def test_zero_policy_runs_to_the_time_limit_without_moving(dog):
    handle, backend = dog
    runner = LocomotionRunner(
        handle,
        ZeroPolicy(),
        cfg=fast_cfg(),
        terminations=TerminationManager(terms=[time_limit(0.3)]),
    )
    result = runner.run()
    assert result.termination.name == "time_limit"
    assert not result.termination.is_fault
    assert result.steps > 5
    assert backend.peak_vx() == 0.0
    assert not handle.emergency_stopped


def test_command_policy_drives_the_commanded_velocity(dog):
    handle, backend = dog
    observations = ObservationManager()
    cfg = fast_cfg(action=ActionCfg(scale=(0.6, 0.4, 0.8)))
    policy = CommandFollowingPolicy(
        observations.term_slices()["velocity_commands"], (2.0, 2.0, 0.25), cfg.action.scale
    )
    runner = LocomotionRunner(
        handle,
        policy,
        cfg=cfg,
        observations=observations,
        terminations=TerminationManager(terms=[time_limit(0.4)]),
        command=FixedVelocityCommand(Velocity(vx=0.5), handle.config.safety),
    )
    result = runner.run()
    assert result.termination.name == "time_limit"
    assert backend.peak_vx() == pytest.approx(0.5, abs=0.02)


def test_fault_termination_latches_the_emergency_stop(dog):
    handle, backend = dog
    runner = LocomotionRunner(
        handle,
        ZeroPolicy(),
        cfg=fast_cfg(),
        terminations=TerminationManager(
            terms=[bad_orientation(math.radians(35)), time_limit(5.0)]
        ),
    )
    # Tip the robot over shortly after the run starts.
    threading.Timer(0.15, backend.tip_over).start()
    result = runner.run()
    assert result.termination.name == "bad_orientation"
    assert result.termination.is_fault
    assert handle.emergency_stopped
    assert "stop_move" in backend.calls


def test_max_steps_ends_the_run_without_a_termination(dog):
    handle, _ = dog
    runner = LocomotionRunner(
        handle, ZeroPolicy(), cfg=fast_cfg(), terminations=TerminationManager(terms=[])
    )
    result = runner.run(max_steps=5)
    assert result.steps == 5
    assert result.termination is None
    assert not handle.emergency_stopped


def test_step_listener_records_every_step(dog):
    handle, _ = dog
    records: list[dict] = []
    runner = LocomotionRunner(
        handle,
        ZeroPolicy(),
        cfg=fast_cfg(),
        terminations=TerminationManager(terms=[]),
    )
    runner.set_step_listener(records.append)
    result = runner.run(max_steps=4)
    assert len(records) == result.steps == 4
    assert records[0]["step"] == 0
    assert set(records[0]) >= {"step", "command", "action", "velocity", "state"}


def test_listener_exception_does_not_stop_the_run(dog):
    handle, _ = dog
    runner = LocomotionRunner(
        handle, ZeroPolicy(), cfg=fast_cfg(), terminations=TerminationManager(terms=[])
    )
    runner.set_step_listener(lambda _record: 1 / 0)
    assert runner.run(max_steps=3).steps == 3


def test_a_slow_policy_is_reported_as_late(dog):
    handle, _ = dog

    class SlowPolicy:
        name = "slow"

        def reset(self):
            return None

        def act(self, obs):  # noqa: ARG002 - deliberately slow
            time.sleep(0.03)
            return [0.0, 0.0, 0.0]

    runner = LocomotionRunner(
        handle,
        SlowPolicy(),
        cfg=fast_cfg(policy_hz=100.0),  # 10 ms budget, policy takes 30 ms
        terminations=TerminationManager(terms=[]),
    )
    result = runner.run(max_steps=5)
    assert result.late_steps == 5
    assert result.mean_rate_hz < 60


def test_run_result_summary_mentions_the_reason(dog):
    handle, _ = dog
    runner = LocomotionRunner(
        handle,
        ZeroPolicy(),
        cfg=fast_cfg(),
        terminations=TerminationManager(terms=[time_limit(0.2)]),
    )
    summary = runner.run().summary()
    assert "time_limit" in summary and "Hz" in summary


def test_stand_first_raises_the_robot_before_the_policy(dog):
    handle, backend = dog
    runner = LocomotionRunner(
        handle,
        ZeroPolicy(),
        cfg=LocomotionCfg(policy_hz=100.0, stand_first=True, settle_s=0.05, sit_after=True),
        terminations=TerminationManager(terms=[time_limit(0.2)]),
    )
    runner.run()
    assert backend.calls.index("stand_up") < backend.calls.index("balance_stand")
    assert "stand_down" in backend.calls


def test_command_sampling_is_bounded_by_safety(dog):
    handle, backend = dog
    observations = ObservationManager()
    cfg = fast_cfg()
    policy = CommandFollowingPolicy(
        observations.term_slices()["velocity_commands"], (2.0, 2.0, 0.25), cfg.action.scale
    )
    runner = LocomotionRunner(
        handle,
        policy,
        cfg=cfg,
        observations=observations,
        terminations=TerminationManager(terms=[time_limit(0.5)]),
        seed=11,
    )
    runner.run()
    limits = handle.config.safety
    with backend.lock:
        assert all(abs(m.vx) <= limits.max_vx + 1e-9 for m in backend.moves)
        assert all(abs(m.vyaw) <= limits.max_vyaw + 1e-9 for m in backend.moves)
