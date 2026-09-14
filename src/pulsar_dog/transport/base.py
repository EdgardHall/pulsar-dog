"""The contract every backend implements.

Two backends exist: ``sdk2`` drives a real Go2 EDU through Unitree's DDS SDK,
``sim`` integrates the commands in-process so the whole stack - teleop, safety,
logging - can be exercised with no robot on the bench.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Protocol, runtime_checkable

from pulsar_dog.safety import Velocity

__all__ = ["Velocity", "RobotState", "RobotBackend", "BackendUnavailable", "BackendError"]


class BackendError(RuntimeError):
    """The backend is present but the operation failed."""


class BackendUnavailable(BackendError):
    """The backend cannot be used at all here (SDK missing, no link, ...)."""


@dataclass
class RobotState:
    """A snapshot of the robot, normalised across backends.

    Fields the backend could not read stay ``None`` rather than being faked, so
    a log never claims a battery reading the robot never sent.
    """

    timestamp: float
    # Sport-mode identifier as reported by the robot (0 = idle, 1 = balance
    # stand, ... - the mapping is firmware-dependent, so it is kept raw).
    mode: int | None = None
    gait_type: int | None = None
    body_height: float | None = None
    # Odometry in the robot's world frame, metres.
    position: tuple[float, float, float] | None = None
    # Measured body velocity, same convention as a Velocity command.
    velocity: Velocity | None = None
    # Roll, pitch, yaw in radians.
    rpy: tuple[float, float, float] | None = None
    battery_soc: int | None = None
    foot_force: tuple[int, int, int, int] | None = None
    error_code: int | None = None
    extra: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        data = asdict(self)
        if self.velocity is not None:
            data["velocity"] = list(self.velocity.as_tuple())
        return data


@runtime_checkable
class RobotBackend(Protocol):
    """Minimal surface the control loop needs.

    Implementations must be safe to call from a single control thread; the
    facade in :mod:`pulsar_dog.robot` guarantees that serialisation.
    """

    name: str

    def connect(self) -> None: ...

    def close(self) -> None: ...

    # --- posture -------------------------------------------------------
    def stand_up(self) -> None:
        """Rise to the normal standing posture."""

    def stand_down(self) -> None:
        """Lower the body and lie down, motors still held."""

    def balance_stand(self) -> None:
        """Stand with active balancing - the posture walking starts from."""

    def damp(self) -> None:
        """Release the joints. The robot collapses; use only when it is low."""

    # --- motion --------------------------------------------------------
    def move(self, velocity: Velocity) -> None:
        """Apply a body-frame velocity. Called every control tick."""

    def stop_move(self) -> None:
        """Cancel motion but keep standing."""

    # --- telemetry -----------------------------------------------------
    def read_state(self) -> RobotState:
        """Latest known state. Must not block for long."""
