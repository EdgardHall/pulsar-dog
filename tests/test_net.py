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
