"""Link diagnostics: is the wire actually there before we try to walk?"""

from __future__ import annotations

import ipaddress
import os
import shutil
import socket
import subprocess
from dataclasses import dataclass

from pulsar_dog.config import NetworkConfig


@dataclass
class Check:
    name: str
    ok: bool
    detail: str

    def render(self) -> str:
        return f"[{'ok' if self.ok else 'FAIL'}] {self.name}: {self.detail}"


def list_interfaces() -> list[str]:
    try:
        return sorted(os.listdir("/sys/class/net"))
    except OSError:
        return []


def interface_addresses(interface: str) -> list[str]:
    """IPv4 addresses on ``interface``, via ``ip``; empty when it cannot be read."""
    ip_bin = shutil.which("ip")
    if ip_bin is None:
        return []
    try:
        out = subprocess.run(
            [ip_bin, "-4", "-brief", "addr", "show", "dev", interface],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (subprocess.SubprocessError, OSError):
        return []
    if out.returncode != 0:
        return []
    # "eth0  UP  192.168.123.99/24"
    parts = out.stdout.split()
    return [p for p in parts[2:] if "/" in p]


def ping(host: str, timeout_s: int = 2) -> bool:
    ping_bin = shutil.which("ping")
    if ping_bin is None:
        return False
    try:
        result = subprocess.run(
            [ping_bin, "-c", "1", "-W", str(timeout_s), host],
            capture_output=True,
            timeout=timeout_s + 3,
        )
    except (subprocess.SubprocessError, OSError):
        return False
    return result.returncode == 0


def tcp_probe(host: str, port: int, timeout_s: float = 1.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def _same_subnet(address_with_prefix: str, robot_ip: str) -> bool:
    try:
        iface = ipaddress.ip_interface(address_with_prefix)
        return ipaddress.ip_address(robot_ip) in iface.network
    except ValueError:
        return False


def diagnose(network: NetworkConfig) -> list[Check]:
    """Run the checks that explain 90% of 'the SDK just hangs' reports."""
    checks: list[Check] = []

    interfaces = list_interfaces()
    checks.append(
        Check(
            "interface exists",
            network.interface in interfaces,
            f"{network.interface!r} among {', '.join(interfaces) or 'none found'}",
        )
    )

    addresses = interface_addresses(network.interface)
    routable = [a for a in addresses if _same_subnet(a, network.robot_ip)]
    checks.append(
        Check(
            "host address on robot subnet",
            bool(routable),
            (
                f"{', '.join(routable)} can reach {network.robot_ip}"
                if routable
                else (
                    f"{network.interface} has {', '.join(addresses) or 'no IPv4 address'}; "
                    f"set one on {network.robot_ip}'s subnet, e.g. "
                    f"sudo ip addr add {network.local_ip}/24 dev {network.interface}"
                )
            ),
        )
    )

    reachable = ping(network.robot_ip)
    checks.append(
        Check(
            "robot responds to ping",
            reachable,
            f"{network.robot_ip} {'replied' if reachable else 'did not reply'}",
        )
    )

    try:
        from pulsar_dog.transport import sdk2_available

        have_sdk = sdk2_available()
    except Exception:  # noqa: BLE001 - diagnostics must never raise
        have_sdk = False
    checks.append(
        Check(
            "unitree_sdk2py importable",
            have_sdk,
            "installed"
            if have_sdk
            else "missing - pip install -e path/to/unitree_sdk2_python",
        )
    )

    return checks


def setup_commands(network: NetworkConfig) -> list[list[str]]:
    """The privileged commands that put this host on the robot's subnet."""
    return [
        ["sudo", "ip", "addr", "add", f"{network.local_ip}/24", "dev", network.interface],
        ["sudo", "ip", "link", "set", network.interface, "up"],
    ]


def run_command(command: list[str], timeout_s: int = 15) -> tuple[int, str]:
    """Run one command, returning its exit code and combined output."""
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout_s
        )
    except FileNotFoundError:
        return 127, f"{command[0]}: not found"
    except (subprocess.SubprocessError, OSError) as exc:
        return 1, str(exc)
    return result.returncode, (result.stdout + result.stderr).strip()


def configure_interface(network: NetworkConfig, dry_run: bool = False) -> list[Check]:
    """Assign the host address and bring the interface up.

    Idempotent: an address that is already assigned is reported as satisfied
    rather than as a failure, so the command is safe to re-run at the start of
    every session.
    """
    checks: list[Check] = []

    existing = interface_addresses(network.interface)
    if any(_same_subnet(address, network.robot_ip) for address in existing):
        return [
            Check(
                "address already assigned",
                True,
                f"{network.interface} already reaches {network.robot_ip} "
                f"via {', '.join(existing)}",
            )
        ]

    for command in setup_commands(network):
        printable = " ".join(command)
        if dry_run:
            checks.append(Check("would run", True, printable))
            continue
        code, output = run_command(command)
        # "RTNETLINK answers: File exists" means another run already did this.
        already = "file exists" in output.lower()
        checks.append(
            Check(
                printable,
                code == 0 or already,
                "already set" if already else (output or "ok"),
            )
        )

    return checks
