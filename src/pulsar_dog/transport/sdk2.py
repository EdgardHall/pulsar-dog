"""Real Go2 EDU backend, built on Unitree's DDS SDK (``unitree_sdk2py``).

The SDK is not on PyPI and its API has drifted between firmware releases, so
every optional call is resolved by name at runtime and every telemetry field is
read through ``getattr``. A field this file cannot find stays ``None`` instead
of crashing a running teleop session.

Install:
    git clone https://github.com/unitreerobotics/unitree_sdk2_python
    pip install -e unitree_sdk2_python
"""

from __future__ import annotations

import threading
import time
from typing import Any

from pulsar_dog.config import Config
from pulsar_dog.transport.base import BackendError, BackendUnavailable, RobotState, Velocity

# DDS topics published by the Go2 in sport mode.
TOPIC_SPORT_STATE = "rt/sportmodestate"
TOPIC_LOW_STATE = "rt/lowstate"

# Unitree client calls return 0 on success and a non-zero error code otherwise.
OK = 0


def _import_sdk() -> dict[str, Any]:
    """Import the SDK pieces we need, as a dict, with one clear failure mode."""
    try:
        from unitree_sdk2py.core.channel import (  # type: ignore[import-not-found]
            ChannelFactoryInitialize,
            ChannelSubscriber,
        )
        from unitree_sdk2py.go2.sport.sport_client import (  # type: ignore[import-not-found]
            SportClient,
        )
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import (  # type: ignore[import-not-found]
            LowState_,
            SportModeState_,
        )
    except ImportError as exc:  # pragma: no cover - needs the real SDK
        raise BackendUnavailable(
            f"unitree_sdk2py could not be imported ({exc}). Install it from "
            "https://github.com/unitreerobotics/unitree_sdk2_python, or use "
            "--backend sim."
        ) from exc
    return {
        "ChannelFactoryInitialize": ChannelFactoryInitialize,
        "ChannelSubscriber": ChannelSubscriber,
        "SportClient": SportClient,
        "SportModeState_": SportModeState_,
        "LowState_": LowState_,
    }


def _triple(obj: Any, attr: str) -> tuple[float, float, float] | None:
    values = getattr(obj, attr, None)
    if values is None or len(values) < 3:
        return None
    return (float(values[0]), float(values[1]), float(values[2]))


class Sdk2Backend:
    """Sport-mode (high level) control of a Go2 EDU over DDS.

    High level only: this drives Unitree's own locomotion controller through
    ``SportClient``. Low-level joint control is a different and far more
    dangerous animal - it needs the sport service released first, and is
    deliberately out of scope here.
    """

    name = "sdk2"

    def __init__(self, config: Config) -> None:
        self._config = config
        self._sdk: dict[str, Any] | None = None
        self._sport: Any = None
        self._subscribers: list[Any] = []
        self._lock = threading.Lock()
        self._sport_state: Any = None
        self._low_state: Any = None
        self._last_rx: float | None = None

    # --- lifecycle -----------------------------------------------------
    def connect(self) -> None:
        sdk = _import_sdk()
        self._sdk = sdk
        interface = self._config.network.interface
        try:
            # Domain 0 is what the Go2's own services use.
            sdk["ChannelFactoryInitialize"](0, interface)
        except Exception as exc:  # pragma: no cover - needs the real SDK
            raise BackendUnavailable(
                f"DDS init failed on interface {interface!r}: {exc}. Check the "
                "cable, that the interface name is right (ip -br addr), and that "
                f"this host has an address on {self._config.network.robot_ip}'s subnet."
            ) from exc

        sport = sdk["SportClient"]()
        sport.SetTimeout(10.0)
        sport.Init()
        self._sport = sport

        self._subscribe(TOPIC_SPORT_STATE, sdk["SportModeState_"], self._on_sport_state)
        self._subscribe(TOPIC_LOW_STATE, sdk["LowState_"], self._on_low_state)

    def _subscribe(self, topic: str, msg_type: Any, handler) -> None:
        assert self._sdk is not None
        sub = self._sdk["ChannelSubscriber"](topic, msg_type)
        sub.Init(handler, 10)
        self._subscribers.append(sub)

    def close(self) -> None:
        # Leave the robot standing still rather than trusting the caller.
        if self._sport is not None:
            try:
                self._sport.StopMove()
            except Exception:  # pragma: no cover - best effort on teardown
                pass
        for sub in self._subscribers:
            close = getattr(sub, "Close", None)
            if callable(close):
                try:
                    close()
                except Exception:  # pragma: no cover - best effort on teardown
                    pass
        self._subscribers.clear()
        self._sport = None

    # --- callbacks -----------------------------------------------------
    def _on_sport_state(self, msg: Any) -> None:
        with self._lock:
            self._sport_state = msg
            self._last_rx = time.monotonic()

    def _on_low_state(self, msg: Any) -> None:
        with self._lock:
            self._low_state = msg
            self._last_rx = time.monotonic()

    # --- command plumbing ----------------------------------------------
    def _call(self, method: str, *args) -> None:
        if self._sport is None:
            raise BackendError(f"{method}() called before connect()")
        fn = getattr(self._sport, method, None)
        if not callable(fn):
            raise BackendError(f"this SDK build has no SportClient.{method}()")
        code = fn(*args)
        # Some builds return None instead of a status code.
        if isinstance(code, int) and code != OK:
            raise BackendError(f"SportClient.{method}{args} returned error code {code}")

    # --- posture -------------------------------------------------------
    def stand_up(self) -> None:
        self._call("StandUp")

    def stand_down(self) -> None:
        self._call("StandDown")

    def balance_stand(self) -> None:
        self._call("BalanceStand")

    def recovery_stand(self) -> None:
        self._call("RecoveryStand")

    def damp(self) -> None:
        self._call("Damp")

    # --- motion --------------------------------------------------------
    def move(self, velocity: Velocity) -> None:
        self._call("Move", velocity.vx, velocity.vy, velocity.vyaw)

    def stop_move(self) -> None:
        self._call("StopMove")

    # --- telemetry -----------------------------------------------------
    def read_state(self) -> RobotState:
        with self._lock:
            sport = self._sport_state
            low = self._low_state
            last_rx = self._last_rx

        state = RobotState(timestamp=time.time())
        if last_rx is not None:
            state.extra["state_age_s"] = round(time.monotonic() - last_rx, 4)
        else:
            # No DDS sample yet: the link may be up but the robot silent.
            state.extra["state_age_s"] = None

        if sport is not None:
            state.mode = getattr(sport, "mode", None)
            state.gait_type = getattr(sport, "gait_type", None)
            body_height = getattr(sport, "body_height", None)
            state.body_height = None if body_height is None else float(body_height)
            state.position = _triple(sport, "position")
            measured = _triple(sport, "velocity")
            if measured is not None:
                yaw_speed = getattr(sport, "yaw_speed", 0.0)
                state.velocity = Velocity(measured[0], measured[1], float(yaw_speed))
            imu = getattr(sport, "imu_state", None)
            if imu is not None:
                state.rpy = _triple(imu, "rpy")
            forces = getattr(sport, "foot_force", None)
            if forces is not None and len(forces) >= 4:
                state.foot_force = tuple(int(f) for f in forces[:4])  # type: ignore[assignment]
            state.error_code = getattr(sport, "error_code", None)

        if low is not None:
            bms = getattr(low, "bms_state", None)
            soc = getattr(bms, "soc", None) if bms is not None else None
            state.battery_soc = None if soc is None else int(soc)
            if state.rpy is None:
                imu = getattr(low, "imu_state", None)
                if imu is not None:
                    state.rpy = _triple(imu, "rpy")

        return state
