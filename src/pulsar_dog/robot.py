"""The facade every other module talks to.

One control thread owns the backend. Callers never touch it directly: they set a
target velocity or enqueue a posture command, and the loop republishes the
ramped target at a fixed rate. That gives three properties worth having:

* the robot keeps receiving commands even when the caller is busy thinking;
* a caller that dies stops the robot, via the watchdog;
* the backend is only ever used from one thread, so the SDK never sees
  concurrent calls.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from pulsar_dog.config import Config
from pulsar_dog.safety import RateLimiter, Velocity, Watchdog
from pulsar_dog.transport import build_backend
from pulsar_dog.transport.base import RobotBackend, RobotState

log = logging.getLogger(__name__)

# How often the loop refreshes telemetry, independent of the command rate.
STATE_POLL_HZ = 10.0
# Consecutive backend failures tolerated before the loop stops the robot.
MAX_CONSECUTIVE_ERRORS = 10


class EmergencyStop(RuntimeError):
    """Raised when a motion command is issued while the e-stop is latched."""


@dataclass
class _Task:
    """A backend call to run on the control thread."""

    fn: Callable[[RobotBackend], Any]
    done: threading.Event
    result: Any = None
    error: BaseException | None = None


class PulsarDog:
    """High-level, safety-wrapped handle on the robot.

    Typical use::

        with PulsarDog(Config.from_env()) as dog:
            dog.stand_up()
            dog.set_velocity(Velocity(vx=0.3))
            ...
    """

    def __init__(self, config: Config, backend: RobotBackend | None = None) -> None:
        self._config = config
        self._backend = backend if backend is not None else build_backend(config)
        self._limiter = RateLimiter(config.safety)
        self._watchdog = Watchdog(config.safety.command_timeout_s)
        self._tasks: queue.Queue[_Task] = queue.Queue()
        self._lock = threading.Lock()
        self._target = Velocity.zero()
        self._estop = False
        self._state: RobotState | None = None
        self._state_listener: Callable[[RobotState], None] | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        # Owned by the control thread (and by tasks it runs): True while the
        # backend has an outstanding move command.
        self._moving = False
        self._loop_error: BaseException | None = None

    # --- lifecycle -----------------------------------------------------
    def __enter__(self) -> PulsarDog:
        self.connect()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    @property
    def backend_name(self) -> str:
        return self._backend.name

    @property
    def config(self) -> Config:
        return self._config

    def connect(self) -> None:
        """Open the link and start the control loop."""
        if self._thread is not None:
            raise RuntimeError("already connected")
        self._backend.connect()
        self._stop_event.clear()
        self._watchdog.poke()
        self._thread = threading.Thread(
            target=self._run, name="pulsar-control", daemon=True
        )
        self._thread.start()
        log.info("connected via %s backend", self._backend.name)

    def close(self) -> None:
        """Bring the robot to a stop, then tear the link down.

        Safe to call twice, and safe to call from an exception handler - every
        step is best-effort so a failure in one does not skip the next.
        """
        if self._thread is not None:
            try:
                self.stop()
                # Give the ramp a moment to reach zero before cutting the loop.
                self._wait_until_stopped(timeout=self._ramp_down_time() + 0.2)
            except Exception as exc:
                log.warning("stop during close failed: %s", exc)
            self._stop_event.set()
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            self._backend.stop_move()
        except Exception as exc:
            log.warning("final stop_move failed: %s", exc)
        try:
            self._backend.close()
        except Exception as exc:
            log.warning("backend close failed: %s", exc)
        log.info("disconnected")

    # --- motion --------------------------------------------------------
    def set_velocity(self, velocity: Velocity) -> None:
        """Set the target body velocity and pet the watchdog.

        Call this at least once per ``safety.command_timeout_s`` to keep moving;
        stop calling it and the robot coasts to a halt on its own.
        """
        with self._lock:
            if self._estop:
                raise EmergencyStop("e-stop latched; call clear_emergency_stop() first")
            self._target = velocity.clamped(self._config.safety)
        self._watchdog.poke()

    def stop(self) -> None:
        """Ramp to a standstill, staying on the feet."""
        with self._lock:
            self._target = Velocity.zero()
        self._watchdog.poke()

    def emergency_stop(self) -> None:
        """Latch the e-stop: zero the command immediately, no ramp.

        With ``config.damp_on_estop`` the joints are released too, which drops
        the robot where it stands - right when it is toppling, wrong when it is
        up on something.
        """
        with self._lock:
            self._estop = True
            self._target = Velocity.zero()
        try:
            self._submit(self._halt, timeout=2.0)
            if self._config.damp_on_estop:
                self._submit(lambda b: b.damp(), timeout=2.0)
        except Exception as exc:
            # An e-stop that cannot reach the robot must still latch locally.
            log.error("emergency stop could not be delivered: %s", exc)
            raise
        finally:
            log.warning("EMERGENCY STOP latched")

    def clear_emergency_stop(self) -> None:
        with self._lock:
            self._estop = False
        self._watchdog.poke()
        log.info("e-stop cleared")

    @property
    def emergency_stopped(self) -> bool:
        with self._lock:
            return self._estop

    # --- posture -------------------------------------------------------
    def stand_up(self) -> None:
        self._posture(lambda b: b.stand_up())

    def stand_down(self) -> None:
        self._posture(lambda b: b.stand_down())

    def balance_stand(self) -> None:
        self._posture(lambda b: b.balance_stand())

    def recovery_stand(self) -> None:
        """Get back up after a fall.

        Clears the e-stop first: a fall latches it, and the whole point of this
        command is to act once the robot is already down.
        """
        if self.emergency_stopped:
            self.clear_emergency_stop()
        self._posture(lambda b: b.recovery_stand())

    def damp(self) -> None:
        """Release the joints. The robot goes limp - only from a low posture."""
        self._posture(lambda b: b.damp())

    def _posture(self, fn: Callable[[RobotBackend], Any]) -> None:
        # A posture change while walking is how you get a fall. Halting and
        # changing posture go in a single task so the control loop cannot slip a
        # move command between them.
        self.stop()

        def halt_then(backend: RobotBackend) -> Any:
            self._halt(backend)
            return fn(backend)

        self._submit(halt_then, timeout=15.0)

    def _halt(self, backend: RobotBackend) -> None:
        """Zero the ramp and stop the backend. Runs on the control thread."""
        self._limiter.reset()
        backend.stop_move()
        self._moving = False

    # --- telemetry -----------------------------------------------------
    def read_state(self, fresh: bool = False) -> RobotState | None:
        """Latest telemetry; ``fresh=True`` forces a read on the control thread."""
        if fresh:
            state = self._submit(lambda b: b.read_state(), timeout=5.0)
            with self._lock:
                self._state = state
            return state
        with self._lock:
            return self._state

    def set_state_listener(self, listener: Callable[[RobotState], None] | None) -> None:
        """Called from the control thread on every telemetry poll. Keep it quick."""
        with self._lock:
            self._state_listener = listener

    # --- internals -----------------------------------------------------
    def _ramp_down_time(self) -> float:
        limits = self._config.safety
        current = self._limiter.current
        linear = max(abs(current.vx), abs(current.vy)) / limits.max_linear_accel
        yaw = abs(current.vyaw) / limits.max_yaw_accel
        return max(linear, yaw)

    def _wait_until_stopped(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._limiter.current.is_zero(eps=1e-3):
                return True
            time.sleep(0.01)
        return self._limiter.current.is_zero(eps=1e-3)

    def _submit(self, fn: Callable[[RobotBackend], Any], timeout: float) -> Any:
        """Run ``fn`` on the control thread and surface its result or exception."""
        if self._thread is None or not self._thread.is_alive():
            # No loop running: the caller owns the backend, so run inline.
            return fn(self._backend)
        task = _Task(fn=fn, done=threading.Event())
        self._tasks.put(task)
        if not task.done.wait(timeout):
            raise TimeoutError(f"control loop did not run the command within {timeout}s")
        if task.error is not None:
            raise task.error
        return task.result

    def _drain_tasks(self) -> None:
        while True:
            try:
                task = self._tasks.get_nowait()
            except queue.Empty:
                return
            try:
                task.result = task.fn(self._backend)
            except BaseException as exc:  # noqa: BLE001 - reported to the caller
                task.error = exc
            finally:
                task.done.set()

    def _current_target(self) -> Velocity:
        with self._lock:
            if self._estop:
                # Latched: cut the ramp too, so the e-stop is immediate rather
                # than acceleration-limited. Only the control thread gets here.
                self._limiter.reset()
                return Velocity.zero()
            target = self._target
        # A silent caller is a stopped robot.
        if self._watchdog.expired():
            if not target.is_zero():
                log.warning(
                    "command watchdog expired after %.2fs; zeroing target",
                    self._watchdog.age(),
                )
                with self._lock:
                    self._target = Velocity.zero()
            return Velocity.zero()
        return target

    def _run(self) -> None:
        period = 1.0 / self._config.control_hz
        state_period = 1.0 / STATE_POLL_HZ
        last_tick = time.monotonic()
        next_state_poll = last_tick
        errors = 0

        while not self._stop_event.is_set():
            now = time.monotonic()
            dt = now - last_tick
            last_tick = now

            try:
                self._drain_tasks()

                command = self._limiter.step(self._current_target(), dt)
                if not command.is_zero(eps=1e-4):
                    self._backend.move(command)
                    self._moving = True
                elif self._moving:
                    # Send the halt once, then stay quiet instead of spamming.
                    self._backend.stop_move()
                    self._moving = False

                if now >= next_state_poll:
                    next_state_poll = now + state_period
                    state = self._backend.read_state()
                    with self._lock:
                        self._state = state
                        listener = self._state_listener
                    if listener is not None:
                        try:
                            listener(state)
                        except Exception as exc:  # noqa: BLE001 - listener is caller code
                            log.warning("state listener raised: %s", exc)

                errors = 0
            except Exception as exc:  # noqa: BLE001 - the loop must not die silently
                errors += 1
                self._loop_error = exc
                log.error("control loop error (%d/%d): %s", errors, MAX_CONSECUTIVE_ERRORS, exc)
                if errors >= MAX_CONSECUTIVE_ERRORS:
                    log.critical("too many control errors; latching e-stop and exiting loop")
                    with self._lock:
                        self._estop = True
                        self._target = Velocity.zero()
                    self._limiter.reset()
                    try:
                        self._backend.stop_move()
                        self._moving = False
                    except Exception:  # noqa: BLE001 - already failing
                        pass
                    break

            sleep_for = period - (time.monotonic() - now)
            if sleep_for > 0:
                time.sleep(sleep_for)

        # Unblock anyone waiting on a queued command.
        self._drain_tasks()

    @property
    def last_loop_error(self) -> BaseException | None:
        return self._loop_error
