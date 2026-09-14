import json
import math

import pytest

from pulsar_dog import analysis


def write_log(tmp_path, records, name="run.jsonl"):
    path = tmp_path / name
    path.write_text("\n".join(json.dumps(r) for r in records) + "\n")
    return str(path)


def telemetry_record(t, vx=0.0, soc=90, x=0.0):
    return {
        "t_rel": t,
        "mode": 1,
        "battery_soc": soc,
        "velocity": [vx, 0.0, 0.0],
        "position": [x, 0.0, 0.3],
        "rpy": [0.0, 0.0, 0.0],
    }


def walk_record(t, commanded=0.4, measured=0.35):
    return {
        "step": int(t * 50),
        "t_rel": t,
        "command": [commanded, 0.0, 0.0],
        "action": [commanded, 0.0, 0.0],
        "velocity": [commanded, 0.0, 0.0],
        "state": {
            "velocity": [measured, 0.0, 0.0],
            "battery_soc": 80,
            "rpy": [0.0, 0.1, 0.0],
            "position": [0.0, 0.0, 0.3],
        },
    }


def test_telemetry_records_have_no_commanded_velocity(tmp_path):
    path = write_log(tmp_path, [telemetry_record(0.0, vx=0.3)])
    samples, skipped = analysis.load_samples(path)
    assert skipped == 0
    assert samples[0].commanded is None
    assert samples[0].measured == (0.3, 0.0, 0.0)
    assert samples[0].battery_soc == 90


def test_walk_records_separate_commanded_from_measured(tmp_path):
    path = write_log(tmp_path, [walk_record(0.0, commanded=0.4, measured=0.31)])
    samples, _ = analysis.load_samples(path)
    assert samples[0].commanded == (0.4, 0.0, 0.0)
    assert samples[0].measured == (0.31, 0.0, 0.0)


def test_a_truncated_last_line_is_counted_not_fatal(tmp_path):
    path = tmp_path / "crashed.jsonl"
    path.write_text(json.dumps(telemetry_record(0.0)) + "\n" + '{"t_rel": 1.0, "velo')
    samples, skipped = analysis.load_samples(str(path))
    assert len(samples) == 1
    assert skipped == 1


def test_blank_lines_are_ignored(tmp_path):
    path = tmp_path / "spaced.jsonl"
    path.write_text(json.dumps(telemetry_record(0.0)) + "\n\n\n")
    samples, skipped = analysis.load_samples(str(path))
    assert len(samples) == 1 and skipped == 0


def test_malformed_triples_become_none(tmp_path):
    path = write_log(tmp_path, [{"t_rel": 0.0, "velocity": [1.0], "rpy": "nope"}])
    samples, _ = analysis.load_samples(path)
    assert samples[0].measured is None
    assert samples[0].rpy is None


def test_sparkline_shapes():
    assert analysis.sparkline([]) == ""
    # A flat signal has no shape to show.
    assert set(analysis.sparkline([1.0] * 10)) == {analysis.BLOCKS[0]}
    rising = analysis.sparkline(list(range(20)), width=20)
    assert len(rising) == 20
    assert rising[0] == analysis.BLOCKS[0]
    assert rising[-1] == analysis.BLOCKS[-1]


def test_path_length_uses_the_horizontal_plane():
    samples = [
        analysis.Sample(t=0.0, position=(0.0, 0.0, 0.3)),
        analysis.Sample(t=1.0, position=(3.0, 4.0, 0.3)),
    ]
    assert analysis.path_length(samples) == pytest.approx(5.0)


def test_max_tilt_is_reported_in_degrees():
    samples = [analysis.Sample(t=0.0, rpy=(0.0, math.radians(20), 0.0))]
    assert analysis.max_tilt_deg(samples) == pytest.approx(20.0)
    assert analysis.max_tilt_deg([analysis.Sample(t=0.0)]) is None


def test_tracking_error_is_the_mean_absolute_difference():
    samples = [
        analysis.Sample(t=0.0, commanded=(0.5, 0, 0), measured=(0.4, 0, 0)),
        analysis.Sample(t=0.1, commanded=(0.5, 0, 0), measured=(0.3, 0, 0)),
    ]
    assert analysis.tracking_error(samples) == pytest.approx(0.15)
    assert analysis.tracking_error([analysis.Sample(t=0.0)]) is None


def test_gaps_flags_a_stall():
    samples = [analysis.Sample(t=i * 0.02) for i in range(20)]
    # A 1 s hole in the middle of a 50 Hz log.
    samples += [analysis.Sample(t=samples[-1].t + 1.0 + i * 0.02) for i in range(20)]
    found = analysis.gaps(samples)
    assert len(found) == 1
    assert found[0][1] == pytest.approx(1.0, abs=0.01)


def test_no_gaps_in_a_steady_log():
    samples = [analysis.Sample(t=i * 0.02) for i in range(50)]
    assert analysis.gaps(samples) == []


def test_report_covers_a_walk_log(tmp_path):
    records = [walk_record(i * 0.02, commanded=0.4, measured=0.3) for i in range(100)]
    path = write_log(tmp_path, records)
    samples, skipped = analysis.load_samples(path)
    report = analysis.render_report(path, samples, skipped)
    assert "100 samples" in report
    assert "vx cmd" in report and "vx meas" in report
    assert "tracking" in report
    assert "max tilt" in report


def test_report_on_an_empty_log(tmp_path):
    path = tmp_path / "empty.jsonl"
    path.write_text("")
    samples, skipped = analysis.load_samples(str(path))
    assert analysis.render_report(str(path), samples, skipped).endswith("0 samples")
