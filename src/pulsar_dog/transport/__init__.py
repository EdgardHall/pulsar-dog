"""Backend selection."""

from __future__ import annotations

from pulsar_dog.config import Config
from pulsar_dog.transport.base import (
    BackendError,
    BackendUnavailable,
    RobotBackend,
    RobotState,
)

__all__ = [
    "BackendError",
    "BackendUnavailable",
    "RobotBackend",
    "RobotState",
    "build_backend",
    "sdk2_available",
]


def sdk2_available() -> bool:
    """True when Unitree's Python SDK can be imported in this interpreter."""
    import importlib.util

    return importlib.util.find_spec("unitree_sdk2py") is not None


def build_backend(config: Config) -> RobotBackend:
    """Instantiate the backend named by ``config.backend``.

    ``auto`` uses the real robot when the SDK is importable and fails loudly
    otherwise - it never silently downgrades to the simulator, because a teleop
    session that quietly drives nothing is worse than one that refuses to start.
    """
    choice = config.backend.strip().lower()
    if choice == "auto":
        choice = "sdk2" if sdk2_available() else "unavailable"

    if choice == "sim":
        from pulsar_dog.transport.sim import SimBackend

        return SimBackend(config)
    if choice == "sdk2":
        from pulsar_dog.transport.sdk2 import Sdk2Backend

        return Sdk2Backend(config)
    if choice == "unavailable":
        raise BackendUnavailable(
            "unitree_sdk2py is not importable, so no real robot can be driven.\n"
            "Install it with:\n"
            "  git clone https://github.com/unitreerobotics/unitree_sdk2_python\n"
            "  pip install -e unitree_sdk2_python\n"
            "Or run against the simulator with --backend sim."
        )
    raise ValueError(f"unknown backend {config.backend!r} (expected auto, sdk2 or sim)")
