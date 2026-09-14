import json

import pytest

from pulsar_dog.cli import build_parser, config_from_args, main


def parse(argv):
    return build_parser().parse_args(argv)


def test_cli_overrides_beat_environment(monkeypatch):
    monkeypatch.setenv("PULSAR_MAX_VX", "0.9")
    monkeypatch.setenv("PULSAR_INTERFACE", "enp0s1")
    config = config_from_args(parse(["--max-vx", "0.2", "info"]))
    assert config.safety.max_vx == 0.2
    # Untouched by the CLI, so the environment still wins.
    assert config.network.interface == "enp0s1"


def test_environment_configures_safety(monkeypatch):
    monkeypatch.setenv("PULSAR_MAX_VYAW", "0.25")
    config = config_from_args(parse(["info"]))
    assert config.safety.max_vyaw == 0.25


def test_bad_env_value_is_reported(monkeypatch):
    monkeypatch.setenv("PULSAR_MAX_VX", "fast")
    with pytest.raises(ValueError, match="PULSAR_MAX_VX"):
        config_from_args(parse(["info"]))


def test_info_against_the_simulator(capsys):
    assert main(["--backend", "sim", "info"]) == 0
    out = capsys.readouterr().out
    assert "backend: sim" in out
    assert "battery=" in out


def test_info_json_is_parsable(capsys):
    assert main(["--backend", "sim", "info", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["extra"]["simulated"] is True


def test_stand_then_sit_against_the_simulator(capsys):
    assert main(["--backend", "sim", "stand"]) == 0
    assert "stand_up done" in capsys.readouterr().out
    assert main(["--backend", "sim", "sit"]) == 0
    assert "stand_down done" in capsys.readouterr().out


def test_record_writes_a_log(tmp_path, capsys):
    assert main(["--backend", "sim", "record", "--seconds", "0.5", "--log-dir", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "wrote" in out
    logs = list(tmp_path.glob("telemetry-*.jsonl"))
    assert len(logs) == 1
    assert logs[0].read_text().strip()


def test_missing_sdk_reports_how_to_fix_it(monkeypatch, capsys):
    monkeypatch.setattr("pulsar_dog.transport.sdk2_available", lambda: False)
    assert main(["--backend", "auto", "info"]) == 2
    err = capsys.readouterr().err
    assert "unitree_sdk2_python" in err
    assert "--backend sim" in err


def test_doctor_runs_and_reports_checks(capsys):
    # No robot here, so it must fail cleanly rather than raise.
    assert main(["--interface", "definitely-not-a-nic", "doctor"]) == 1
    out = capsys.readouterr().out
    assert "interface exists" in out
    assert "check(s) failed" in out


def test_walk_describe_prints_the_observation_layout(capsys):
    assert main(["--backend", "sim", "walk", "--describe"]) == 0
    out = capsys.readouterr().out
    assert "observation dim=15" in out
    assert "velocity_commands" in out


def test_walk_runs_a_short_episode_on_the_simulator(capsys):
    rc = main(
        [
            "--backend", "sim", "walk",
            "--policy", "command",
            "--duration", "0.5",
            "--settle", "0.05",
            "--seed", "5",
        ]
    )
    assert rc == 0
    assert "time_limit" in capsys.readouterr().out


def test_walk_records_every_policy_step(tmp_path, capsys):
    rc = main(
        [
            "--backend", "sim", "walk",
            "--policy", "patrol",
            "--duration", "0.4",
            "--settle", "0.05",
            "--record",
            "--log-dir", str(tmp_path),
        ]
    )
    assert rc == 0
    logs = list(tmp_path.glob("walk-*.jsonl"))
    assert len(logs) == 1
    lines = logs[0].read_text().strip().splitlines()
    assert len(lines) > 5
    assert json.loads(lines[0])["step"] == 0


def test_walk_rejects_an_unknown_policy_path():
    # Without torch installed this is the "install torch" error; with torch it is
    # the load failure. Either way it must surface, not run a silent zero policy.
    with pytest.raises((RuntimeError, OSError, ValueError)):
        main(["--backend", "sim", "walk", "--policy", "/nope/policy.pt", "--duration", "0.2"])


def test_walk_on_a_robot_that_never_stood_up_ends_in_a_fault(capsys):
    # --no-stand on a simulator that starts lying down: the base_height
    # termination must catch it rather than letting the policy drive.
    rc = main(["--backend", "sim", "walk", "--duration", "0.5", "--no-stand"])
    assert rc == 1
    assert "base_height" in capsys.readouterr().out
