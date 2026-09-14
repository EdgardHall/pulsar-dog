"""In-process simulator: no robot, no SDK, same interface.

It is a kinematic integrator, not a physics model - it will not tell you whether
a gait is stable. What it does tell you is whether your command path, safety
envelope, watchdog and logging behave, which is the part worth debugging before
the robot is powered on.
"""

from __future__ import annotations

import math
import time

from pulsar_dog.config import Config
from pulsar_dog.transport.base import BackendError, RobotState, Velocity

# Mirrors the raw sport-mode ids the robot reports, kept small on purpose.
MODE_IDLE = 0
MODE_BALANCE_STAND = 1
MODE_WALK = 3
MODE_LYING = 5
MODE_DAMPED = 7


class SimBackend:
    """A Go2-shaped stub that integrates velocity commands into odometry."""

    name = "sim"

    def __init__(self, config: Config, clock=time.monotonic) -> None:
        self._config = config
        self._clock = clock
        self._connected = False
        self._mode = MODE_IDLE
        self._standing = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._velocity = Velocity.zero()
        self._last_update = clock()
        self._start = clock()

    # --- lifecycle -----------------------------------------------------
    def connect(self) -> None:
        self._connected = True
        self._last_update = self._clock()

    def close(self) -> None:
        self._velocity = Velocity.zero()
        self._connected = False

    def _require_link(self) -> None:
        if not self._connected:
            raise BackendError("sim backend used before connect()")

    # --- posture -------------------------------------------------------
    def stand_up(self) -> None:
        self._require_link()
        self._standing = True
        self._mode = MODE_BALANCE_STAND

    def stand_down(self) -> None:
        self._require_link()
        self._integrate()
        self._standing = False
        self._velocity = Velocity.zero()
        self._mode = MODE_LYING

    def balance_stand(self) -> None:
        self._require_link()
        self._standing = True
        self._mode = MODE_BALANCE_STAND

    def recovery_stand(self) -> None:
        # Recovery works from any posture, including a damped heap.
        self._require_link()
        self._integrate()
        self._velocity = Velocity.zero()
        self._standing = True
        self._mode = MODE_BALANCE_STAND

    def damp(self) -> None:
        self._require_link()
        self._integrate()
        self._standing = False
        self._velocity = Velocity.zero()
        self._mode = MODE_DAMPED

    # --- motion --------------------------------------------------------
    def move(self, velocity: Velocity) -> None:
        self._require_link()
        self._integrate()
        if not self._standing:
            # A robot that is lying down ignores walk commands; so do we, rather
            # than sliding the odometry of a dog that is on its belly.
            self._velocity = Velocity.zero()
            return
        self._velocity = velocity
        self._mode = MODE_WALK if not velocity.is_zero() else MODE_BALANCE_STAND

    def stop_move(self) -> None:
        self._require_link()
        self._integrate()
        self._velocity = Velocity.zero()
        if self._standing:
            self._mode = MODE_BALANCE_STAND

    # --- telemetry -----------------------------------------------------
    def read_state(self) -> RobotState:
        self._integrate()
        elapsed = self._clock() - self._start
        return RobotState(
            timestamp=time.time(),
            mode=self._mode,
            gait_type=1 if self._mode == MODE_WALK else 0,
            body_height=0.32 if self._standing else 0.08,
            position=(self._x, self._y, 0.32 if self._standing else 0.08),
            velocity=self._velocity,
            rpy=(0.0, 0.0, self._yaw),
            # Drains ~1% per simulated minute so low-battery paths are reachable.
            battery_soc=max(0, 100 - int(elapsed / 60.0)),
            foot_force=(120, 120, 120, 120) if self._standing else (0, 0, 0, 0),
            error_code=0,
            extra={"simulated": True},
        )

    # --- internals -----------------------------------------------------
    def _integrate(self) -> None:
        now = self._clock()
        dt = now - self._last_update
        self._last_update = now
        if dt <= 0 or self._velocity.is_zero():
            return
        # Body-frame velocity rotated into the world frame, mid-point yaw so a
        # pure turn-and-drive does not accumulate an obvious bias.
        yaw_mid = self._yaw + 0.5 * self._velocity.vyaw * dt
        self._x += (
            self._velocity.vx * math.cos(yaw_mid) - self._velocity.vy * math.sin(yaw_mid)
        ) * dt
        self._y += (
            self._velocity.vx * math.sin(yaw_mid) + self._velocity.vy * math.cos(yaw_mid)
        ) * dt
        self._yaw = (self._yaw + self._velocity.vyaw * dt + math.pi) % (2 * math.pi) - math.pi
