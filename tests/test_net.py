from pulsar_dog import net
from pulsar_dog.config import NetworkConfig


def test_same_subnet_matching_and_not():
    assert net._same_subnet("192.168.123.99/24", "192.168.123.161")
    assert not net._same_subnet("192.168.1.10/24", "192.168.123.161")
    # A /32 host route cannot reach the robot.
    assert not net._same_subnet("192.168.123.99/32", "192.168.123.161")


def test_same_subnet_tolerates_garbage():
    assert not net._same_subnet("not-an-address", "192.168.123.161")
    assert not net._same_subnet("192.168.123.99/24", "nope")


def test_diagnose_reports_every_check_without_a_robot(monkeypatch):
    monkeypatch.setattr(net, "list_interfaces", lambda: ["lo"])
    monkeypatch.setattr(net, "interface_addresses", lambda _iface: [])
    monkeypatch.setattr(net, "ping", lambda _host, timeout_s=2: False)
    checks = net.diagnose(NetworkConfig(interface="eth0"))
    names = [c.name for c in checks]
    assert names == [
        "interface exists",
        "host address on robot subnet",
        "robot responds to ping",
        "unitree_sdk2py importable",
    ]
    assert not any(c.ok for c in checks[:3])
    # The failure message must say how to fix it, not just that it failed.
    assert "ip addr add" in checks[1].detail


def test_diagnose_accepts_a_correctly_addressed_interface(monkeypatch):
    monkeypatch.setattr(net, "list_interfaces", lambda: ["eth0"])
    monkeypatch.setattr(net, "interface_addresses", lambda _iface: ["192.168.123.99/24"])
    monkeypatch.setattr(net, "ping", lambda _host, timeout_s=2: True)
    checks = {c.name: c for c in net.diagnose(NetworkConfig(interface="eth0"))}
    assert checks["interface exists"].ok
    assert checks["host address on robot subnet"].ok
    assert checks["robot responds to ping"].ok


def test_ping_returns_false_when_binary_is_missing(monkeypatch):
    monkeypatch.setattr(net.shutil, "which", lambda _name: None)
    assert net.ping("192.168.123.161") is False
    assert net.interface_addresses("eth0") == []


def test_setup_commands_target_the_configured_interface():
    commands = net.setup_commands(NetworkConfig(interface="enp3s0", local_ip="192.168.123.99"))
    assert commands[0] == [
        "sudo", "ip", "addr", "add", "192.168.123.99/24", "dev", "enp3s0",
    ]
    assert commands[1] == ["sudo", "ip", "link", "set", "enp3s0", "up"]


def test_setup_is_skipped_when_the_address_is_already_there(monkeypatch):
    monkeypatch.setattr(net, "interface_addresses", lambda _i: ["192.168.123.99/24"])
    checks = net.configure_interface(NetworkConfig(interface="eth0"))
    assert len(checks) == 1
    assert checks[0].ok
    assert "already" in checks[0].name


def test_dry_run_only_prints(monkeypatch):
    monkeypatch.setattr(net, "interface_addresses", lambda _i: [])
    ran = []
    monkeypatch.setattr(net, "run_command", lambda cmd, timeout_s=15: ran.append(cmd) or (0, ""))
    checks = net.configure_interface(NetworkConfig(interface="eth0"), dry_run=True)
    assert ran == []
    assert all(c.ok and c.name == "would run" for c in checks)


def test_an_already_assigned_address_is_not_a_failure(monkeypatch):
    monkeypatch.setattr(net, "interface_addresses", lambda _i: [])
    monkeypatch.setattr(
        net,
        "run_command",
        lambda cmd, timeout_s=15: (2, "RTNETLINK answers: File exists"),
    )
    checks = net.configure_interface(NetworkConfig(interface="eth0"))
    assert all(c.ok for c in checks)
    assert "already set" in checks[0].detail


def test_a_real_failure_is_reported(monkeypatch):
    monkeypatch.setattr(net, "interface_addresses", lambda _i: [])
    monkeypatch.setattr(
        net, "run_command", lambda cmd, timeout_s=15: (1, "Cannot find device \"eth0\"")
    )
    checks = net.configure_interface(NetworkConfig(interface="eth0"))
    assert not checks[0].ok
    assert "Cannot find device" in checks[0].detail


def test_run_command_reports_a_missing_binary():
    code, output = net.run_command(["definitely-not-a-binary-xyz"])
    assert code == 127
    assert "not found" in output
