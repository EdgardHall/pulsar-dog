"""Command line entry point: ``pulsar-dog <command>``."""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from dataclasses import replace

from pulsar_dog import net
from pulsar_dog.config import Config
from pulsar_dog.robot import PulsarDog
from pulsar_dog.telemetry import JsonlRecorder, default_log_path
from pulsar_dog.transport.base import BackendUnavailable

log = logging.getLogger("pulsar_dog")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pulsar-dog",
        description="Connection layer, safety envelope and teleoperation for a Unitree Go2 EDU.",
    )
    parser.add_argument(
        "--backend",
        choices=["auto", "sdk2", "sim"],
        help="auto (default) uses the real SDK when installed; sim needs no robot",
    )
    parser.add_argument("--interface", help="network interface facing the robot (e.g. eth0)")
    parser.add_argument("--robot-ip", help="robot address on the wired subnet")
    parser.add_argument("--max-vx", type=float, help="forward speed limit, m/s")
    parser.add_argument("--max-vy", type=float, help="lateral speed limit, m/s")
    parser.add_argument("--max-vyaw", type=float, help="turn rate limit, rad/s")
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("doctor", help="check the link without touching the robot")

    info = sub.add_parser("info", help="connect and print one telemetry sample")
    info.add_argument("--json", action="store_true", help="raw JSON instead of a summary")

    sub.add_parser("stand", help="stand up (balance stand)")
    sub.add_parser("sit", help="lie down, joints still held")
    sub.add_parser("damp", help="release the joints - the robot goes limp")

    teleop = sub.add_parser("teleop", help="drive from the keyboard")
    teleop.add_argument(
        "--no-stand",
        action="store_true",
        help="do not stand up on start (robot is already standing)",
    )
    teleop.add_argument("--record", action="store_true", help="log telemetry to --log-dir")

    record = sub.add_parser("record", help="log telemetry for a while, no motion")
    record.add_argument("--seconds", type=float, default=30.0)
    record.add_argument("--log-dir", default=None)

    walk = sub.add_parser("walk", help="drive the robot from a locomotion policy")
    walk.add_argument(
        "--policy",
        default="command",
        help="zero, command, patrol, or the path to a TorchScript export (.pt)",
    )
    walk.add_argument("--duration", type=float, default=20.0, help="run length in seconds")
    walk.add_argument("--policy-hz", type=float, default=50.0)
    walk.add_argument("--seed", type=int, default=None, help="seed for command sampling")
    walk.add_argument(
        "--no-stand", action="store_true", help="robot is already standing"
    )
    walk.add_argument(
        "--settle", type=float, default=2.0, help="seconds to settle after standing up"
    )
    walk.add_argument("--sit-after", action="store_true", help="lie down when the run ends")
    walk.add_argument("--record", action="store_true", help="log every policy step")
    walk.add_argument("--log-dir", default=None)
    walk.add_argument(
        "--describe", action="store_true", help="print the observation layout and exit"
    )

    return parser


def config_from_args(args: argparse.Namespace) -> Config:
    """Environment first, then anything given on the command line."""
    config = Config.from_env()
    network = config.network
    safety = config.safety
    if args.interface:
        network = replace(network, interface=args.interface)
    if args.robot_ip:
        network = replace(network, robot_ip=args.robot_ip)
    if args.max_vx is not None:
        safety = replace(safety, max_vx=args.max_vx)
    if args.max_vy is not None:
        safety = replace(safety, max_vy=args.max_vy)
    if args.max_vyaw is not None:
        safety = replace(safety, max_vyaw=args.max_vyaw)
    config = replace(config, network=network, safety=safety)
    if args.backend:
        config = replace(config, backend=args.backend)
    if getattr(args, "log_dir", None):
        config = replace(config, log_dir=args.log_dir)
    return config


# --- commands ----------------------------------------------------------
def cmd_doctor(config: Config) -> int:
    checks = net.diagnose(config.network)
    for check in checks:
        print(check.render())
    failed = [c for c in checks if not c.ok]
    if failed:
        print(f"\n{len(failed)} check(s) failed - see docs/GO2_EDU_NOTES.md")
        return 1
    print("\nlink looks healthy")
    return 0


def _summarise(state) -> str:
    if state is None:
        return "no telemetry received yet"
    parts = [f"mode={state.mode}", f"gait={state.gait_type}"]
    if state.battery_soc is not None:
        parts.append(f"battery={state.battery_soc}%")
    if state.body_height is not None:
        parts.append(f"height={state.body_height:.3f}m")
    if state.position is not None:
        x, y, z = state.position
        parts.append(f"pos=({x:.2f}, {y:.2f}, {z:.2f})")
    if state.velocity is not None:
        v = state.velocity
        parts.append(f"vel=({v.vx:.2f}, {v.vy:.2f}, {v.vyaw:.2f})")
    if state.rpy is not None:
        r, p, y = state.rpy
        parts.append(f"rpy=({r:.2f}, {p:.2f}, {y:.2f})")
    if state.error_code:
        parts.append(f"error_code={state.error_code}")
    return "  ".join(parts)


def cmd_info(config: Config, as_json: bool) -> int:
    with PulsarDog(config) as dog:
        # First DDS samples take a moment to arrive after the link opens.
        deadline = time.monotonic() + 3.0
        state = None
        while time.monotonic() < deadline:
            state = dog.read_state(fresh=True)
            if state is not None and state.mode is not None:
                break
            time.sleep(0.1)
        if as_json:
            print(json.dumps(state.to_dict() if state else {}, indent=2, default=str))
        else:
            print(f"backend: {dog.backend_name}")
            print(_summarise(state))
        return 0 if state is not None else 1


def cmd_posture(config: Config, action: str) -> int:
    with PulsarDog(config) as dog:
        getattr(dog, action)()
        # Posture transitions are not instant; hold the link while it settles.
        time.sleep(2.0)
        print(f"{action} done - {_summarise(dog.read_state(fresh=True))}")
    return 0


def cmd_teleop(config: Config, stand_first: bool, record: bool) -> int:
    from pulsar_dog.teleop import KeyboardTeleop

    recorder = None
    with PulsarDog(config) as dog:
        try:
            if record:
                recorder = JsonlRecorder(default_log_path(config.log_dir, "teleop"))
                recorder.open()
                dog.set_state_listener(recorder.write)
                print(f"recording to {recorder.path}")
            if stand_first:
                print("standing up...")
                dog.stand_up()
                time.sleep(2.0)
                dog.balance_stand()
            KeyboardTeleop(dog).run()
        finally:
            # Always land the robot rather than leaving it standing unattended.
            dog.set_state_listener(None)
            if recorder is not None:
                recorder.close()
                print(f"wrote {recorder.count} samples to {recorder.path}")
    return 0


def _build_policy(name: str, runner_cfg, observations):
    """Resolve --policy into a Policy instance."""
    from pulsar_dog.locomotion import (
        CommandFollowingPolicy,
        PatrolPolicy,
        TorchScriptPolicy,
        ZeroPolicy,
    )

    if name == "zero":
        return ZeroPolicy()
    if name == "patrol":
        return PatrolPolicy(dt=1.0 / runner_cfg.policy_hz)
    if name == "command":
        terms = {t.name: t for t in observations.terms}
        command_term = terms.get("velocity_commands")
        if command_term is None:
            raise ValueError(
                "the 'command' policy needs a 'velocity_commands' observation term"
            )
        scale = command_term.scale
        obs_scale = (scale,) * 3 if isinstance(scale, (int, float)) else tuple(scale)
        return CommandFollowingPolicy(
            observations.term_slices()["velocity_commands"],
            obs_scale,  # type: ignore[arg-type]
            runner_cfg.action.scale,
        )
    # Anything else is treated as a path to a TorchScript export.
    return TorchScriptPolicy(name, obs_dim=observations.dim)


def cmd_walk(config: Config, args: argparse.Namespace) -> int:
    from pulsar_dog.locomotion import (
        LocomotionCfg,
        LocomotionRunner,
        ObservationManager,
        TerminationManager,
    )
    from pulsar_dog.locomotion.terminations import default_terms

    observations = ObservationManager()
    if args.describe:
        print(observations.describe())
        return 0

    runner_cfg = LocomotionCfg(
        policy_hz=args.policy_hz,
        stand_first=not args.no_stand,
        settle_s=args.settle,
        sit_after=args.sit_after,
    )
    policy = _build_policy(args.policy, runner_cfg, observations)

    recorder = None
    with PulsarDog(config) as dog:
        runner = LocomotionRunner(
            dog,
            policy,
            cfg=runner_cfg,
            observations=observations,
            terminations=TerminationManager(terms=default_terms(args.duration)),
            seed=args.seed,
        )
        try:
            if args.record:
                path = default_log_path(config.log_dir, "walk")
                recorder = _StepRecorder(path)
                recorder.open()
                runner.set_step_listener(recorder.write)
                print(f"recording policy steps to {path}")
            result = runner.run()
        finally:
            runner.set_step_listener(None)
            if recorder is not None:
                recorder.close()
    print(result.summary())
    # A fault termination is a failed run, not a successful one.
    return 1 if result.termination is not None and result.termination.is_fault else 0


class _StepRecorder:
    """Writes one JSON object per policy step, same format as the telemetry log."""

    def __init__(self, path: str) -> None:
        self.path = path
        self._file = None

    def open(self) -> None:
        import os

        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._file = open(self.path, "a", encoding="utf-8")

    def write(self, record: dict) -> None:
        if self._file is None:
            raise RuntimeError("recorder used before open()")
        self._file.write(json.dumps(record, default=str) + "\n")
        self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None


def cmd_record(config: Config, seconds: float) -> int:
    path = default_log_path(config.log_dir, "telemetry")
    with PulsarDog(config) as dog, JsonlRecorder(path) as recorder:
        dog.set_state_listener(recorder.write)
        print(f"recording {seconds:.0f}s to {path} (ctrl-c to stop early)")
        deadline = time.monotonic() + seconds
        try:
            while time.monotonic() < deadline:
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\ninterrupted")
        dog.set_state_listener(None)
        print(f"wrote {recorder.count} samples")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )
    config = config_from_args(args)

    try:
        if args.command == "doctor":
            return cmd_doctor(config)
        if args.command == "info":
            return cmd_info(config, args.json)
        if args.command == "stand":
            return cmd_posture(config, "stand_up")
        if args.command == "sit":
            return cmd_posture(config, "stand_down")
        if args.command == "damp":
            return cmd_posture(config, "damp")
        if args.command == "teleop":
            return cmd_teleop(config, stand_first=not args.no_stand, record=args.record)
        if args.command == "record":
            return cmd_record(config, args.seconds)
        if args.command == "walk":
            return cmd_walk(config, args)
    except BackendUnavailable as exc:
        print(f"cannot reach the robot:\n{exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        # PulsarDog.close() has already stopped the robot on the way out.
        print("\ninterrupted", file=sys.stderr)
        return 130

    raise AssertionError(f"unhandled command {args.command!r}")


if __name__ == "__main__":
    sys.exit(main())
