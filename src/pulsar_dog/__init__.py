"""pulsar-dog: connection, safety envelope and teleoperation for a Unitree Go2 EDU."""

from pulsar_dog.config import Config, NetworkConfig, SafetyLimits
from pulsar_dog.robot import PulsarDog
from pulsar_dog.transport.base import BackendUnavailable, RobotState, Velocity

__all__ = [
    "Config",
    "NetworkConfig",
    "SafetyLimits",
    "PulsarDog",
    "RobotState",
    "Velocity",
    "BackendUnavailable",
]

__version__ = "0.1.0"
