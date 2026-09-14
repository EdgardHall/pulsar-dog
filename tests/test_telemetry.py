import json

from pulsar_dog.safety import Velocity
from pulsar_dog.telemetry import JsonlRecorder, default_log_path
from pulsar_dog.transport.base import RobotState


def make_state() -> RobotState:
    return RobotState(
        timestamp=1.0,
        mode=1,
        battery_soc=77,
        position=(0.1, 0.2, 0.3),
        velocity=Velocity(0.4, 0.0, -0.2),
    )


def test_recorder_writes_one_json_object_per_line(tmp_path):
    path = tmp_path / "nested" / "run.jsonl"
    with JsonlRecorder(str(path)) as recorder:
        recorder.write(make_state())
        recorder.write(make_state(), note="second")
    lines = path.read_text().strip().splitlines()
    assert len(lines) == 2
    first = json.loads(lines[0])
    assert first["battery_soc"] == 77
    # Velocity is flattened to a list so the file loads without custom decoding.
    assert first["velocity"] == [0.4, 0.0, -0.2]
    assert "t_rel" in first
    assert json.loads(lines[1])["note"] == "second"


def test_recorder_counts_and_appends(tmp_path):
    path = str(tmp_path / "run.jsonl")
    with JsonlRecorder(path) as recorder:
        recorder.write(make_state())
        assert recorder.count == 1
    with JsonlRecorder(path) as recorder:
        recorder.write(make_state())
    assert len(open(path).read().strip().splitlines()) == 2


def test_default_log_path_is_timestamped():
    path = default_log_path("logs", "teleop")
    assert path.startswith("logs/teleop-")
    assert path.endswith(".jsonl")
