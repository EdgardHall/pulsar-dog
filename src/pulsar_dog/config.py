"""Configuration for the Go2 EDU link.

Every field can be overridden through an environment variable prefixed with
``PULSAR_``, which keeps the CLI usable without editing code:

    PULSAR_INTERFACE=enp3s0 PULSAR_MAX_VX=0.4 pulsar-dog teleop
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field, replace

# Default wired addresses of a Go2 EDU. The robot exposes its internal
# 192.168.123.0/24 network on the Ethernet port; the onboard computer answers on
# .161. Verify against your unit before trusting these (see docs/GO2_EDU_NOTES.md).
DEFAULT_ROBOT_IP = "192.168.123.161"
DEFAULT_LOCAL_IP = "192.168.123.99"
DEFAULT_INTERFACE = "eth0"


def _env_float(name: str, fallback: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return fallback
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"{name}={raw!r} is not a number") from exc


def _env_bool(name: str, fallback: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return fallback
    return raw.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class NetworkConfig:
    """Where the robot lives and which NIC talks to it."""

    interface: str = DEFAULT_INTERFACE
    robot_ip: str = DEFAULT_ROBOT_IP
    local_ip: str = DEFAULT_LOCAL_IP

    @classmethod
    def from_env(cls) -> NetworkConfig:
        return cls(
            interface=os.environ.get("PULSAR_INTERFACE", DEFAULT_INTERFACE),
            robot_ip=os.environ.get("PULSAR_ROBOT_IP", DEFAULT_ROBOT_IP),
            local_ip=os.environ.get("PULSAR_LOCAL_IP", DEFAULT_LOCAL_IP),
        )


@dataclass(frozen=True)
class SafetyLimits:
    """The envelope every command is forced through before it reaches the robot.

    Defaults are deliberately timid: a Go2 can walk far faster than this, but a
    first session in a room with furniture should not.
    """

    max_vx: float = 0.6  # m/s, forward/backward
    max_vy: float = 0.4  # m/s, lateral
    max_vyaw: float = 0.8  # rad/s, turn rate
    max_linear_accel: float = 1.5  # m/s^2 applied to vx and vy
    max_yaw_accel: float = 3.0  # rad/s^2
    # If no fresh command arrives within this window the control loop decays the
    # target to zero. This is what protects you from a crashed teleop process.
    command_timeout_s: float = 0.35

    def __post_init__(self) -> None:
        for name in (
            "max_vx",
            "max_vy",
            "max_vyaw",
            "max_linear_accel",
            "max_yaw_accel",
            "command_timeout_s",
        ):
            value = getattr(self, name)
            if value <= 0:
                raise ValueError(f"SafetyLimits.{name} must be > 0, got {value}")

    @classmethod
    def from_env(cls) -> SafetyLimits:
        base = cls()
        return replace(
            base,
            max_vx=_env_float("PULSAR_MAX_VX", base.max_vx),
            max_vy=_env_float("PULSAR_MAX_VY", base.max_vy),
            max_vyaw=_env_float("PULSAR_MAX_VYAW", base.max_vyaw),
            max_linear_accel=_env_float("PULSAR_MAX_ACCEL", base.max_linear_accel),
            max_yaw_accel=_env_float("PULSAR_MAX_YAW_ACCEL", base.max_yaw_accel),
            command_timeout_s=_env_float("PULSAR_CMD_TIMEOUT", base.command_timeout_s),
        )


@dataclass(frozen=True)
class Config:
    network: NetworkConfig = field(default_factory=NetworkConfig)
    safety: SafetyLimits = field(default_factory=SafetyLimits)
    # Rate at which the control thread republishes the current target.
    control_hz: float = 50.0
    # "auto" picks the real SDK when importable, otherwise refuses; "sim" and
    # "sdk2" force one backend.
    backend: str = "auto"
    log_dir: str = "logs"
    # Emergency stop cuts velocity. Setting this to True *also* damps the motors,
    # which makes the robot sit down abruptly - safer when it is about to fall,
    # worse when it is standing on a table.
    damp_on_estop: bool = False

    @classmethod
    def from_env(cls) -> Config:
        base = cls()
        return replace(
            base,
            network=NetworkConfig.from_env(),
            safety=SafetyLimits.from_env(),
            control_hz=_env_float("PULSAR_CONTROL_HZ", base.control_hz),
            backend=os.environ.get("PULSAR_BACKEND", base.backend),
            log_dir=os.environ.get("PULSAR_LOG_DIR", base.log_dir),
            damp_on_estop=_env_bool("PULSAR_DAMP_ON_ESTOP", base.damp_on_estop),
        )
