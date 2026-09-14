"""Reading back a run.

Two record shapes end up in the .jsonl logs:

* telemetry records (``pulsar-dog record``, ``teleop --record``) - one flat
  robot state per line;
* policy step records (``walk --record``) - command, action and the resulting
  velocity, with the state nested under ``state``.

Both are normalised into :class:`Sample` here, so one reader covers every log.
Rendering is plain text on purpose: the first thing you do after a run that went
wrong is read its log over SSH, with nothing installed.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

BLOCKS = "▁▂▃▄▅▆▇█"


@dataclass
class Sample:
    """One moment of a run, whichever log it came from."""

    t: float
    commanded: tuple[float, float, float] | None = None
    measured: tuple[float, float, float] | None = None
    battery_soc: int | None = None
    rpy: tuple[float, float, float] | None = None
    position: tuple[float, float, float] | None = None
    mode: int | None = None


def _triple(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return None


def parse_record(record: dict, index: int) -> Sample:
    """Normalise either record shape into a Sample."""
    state = record.get("state")
    if isinstance(state, dict):
        # A policy step: "velocity" is what was commanded, the measurement is
        # inside the nested state.
        return Sample(
            t=float(record.get("t_rel", record.get("step", index))),
            commanded=_triple(record.get("velocity")),
            measured=_triple(state.get("velocity")),
            battery_soc=state.get("battery_soc"),
            rpy=_triple(state.get("rpy")),
            position=_triple(state.get("position")),
            mode=state.get("mode"),
        )
    return Sample(
        t=float(record.get("t_rel", index)),
        commanded=None,
        measured=_triple(record.get("velocity")),
        battery_soc=record.get("battery_soc"),
        rpy=_triple(record.get("rpy")),
        position=_triple(record.get("position")),
        mode=record.get("mode"),
    )


def load_samples(path: str) -> tuple[list[Sample], int]:
    """Read a .jsonl log. Returns the samples and the number of unreadable lines.

    A truncated last line is normal - the interesting logs are the ones whose
    process died - so a bad line is counted, never fatal.
    """
    samples: list[Sample] = []
    skipped = 0
    with open(path, encoding="utf-8") as handle:
        for index, line in enumerate(handle):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                skipped += 1
                continue
            if not isinstance(record, dict):
                skipped += 1
                continue
            samples.append(parse_record(record, index))
    return samples, skipped


def sparkline(values: list[float], width: int = 60) -> str:
    """A one-line plot, bucketed to ``width`` columns."""
    if not values or width <= 0:
        return ""
    buckets: list[float] = []
    for column in range(min(width, len(values))):
        start = column * len(values) // min(width, len(values))
        stop = (column + 1) * len(values) // min(width, len(values))
        chunk = values[start:stop] or [values[min(start, len(values) - 1)]]
        # Peak per bucket: a brief spike matters more than its average here.
        buckets.append(max(chunk, key=abs))
    low = min(buckets)
    high = max(buckets)
    if math.isclose(high, low):
        return BLOCKS[0] * len(buckets)
    span = high - low
    return "".join(
        BLOCKS[min(len(BLOCKS) - 1, int((value - low) / span * len(BLOCKS)))]
        for value in buckets
    )


def path_length(samples: list[Sample]) -> float:
    """Distance walked according to the robot's own odometry.

    Proprioceptive integration, so it drifts - fine over a run, not a position.
    """
    total = 0.0
    previous: tuple[float, float, float] | None = None
    for sample in samples:
        if sample.position is None:
            continue
        if previous is not None:
            total += math.dist(previous[:2], sample.position[:2])
        previous = sample.position
    return total


def max_tilt_deg(samples: list[Sample]) -> float | None:
    from pulsar_dog.locomotion.observations import tilt_angle

    tilts = [tilt_angle(s.rpy) for s in samples if s.rpy is not None]
    return math.degrees(max(tilts)) if tilts else None


def gaps(samples: list[Sample], factor: float = 4.0) -> list[tuple[float, float]]:
    """Intervals far longer than the median - a stall, a block, a lost link."""
    times = [s.t for s in samples]
    deltas = [b - a for a, b in zip(times, times[1:], strict=False) if b > a]
    if len(deltas) < 4:
        return []
    ordered = sorted(deltas)
    median = ordered[len(ordered) // 2]
    if median <= 0:
        return []
    found = []
    for start, stop in zip(times, times[1:], strict=False):
        if stop - start > median * factor:
            found.append((start, stop - start))
    return found


def render_report(path: str, samples: list[Sample], skipped: int, width: int = 60) -> str:
    """The whole log, as something you can read in a terminal."""
    lines = [f"{path}: {len(samples)} samples"]
    if skipped:
        lines.append(f"  {skipped} unreadable line(s) skipped")
    if not samples:
        return "\n".join(lines)

    duration = samples[-1].t - samples[0].t
    lines.append(f"  {'duration':<12}{duration:.1f}s")
    if duration > 0:
        lines.append(f"  {'rate':<12}{len(samples) / duration:.1f} Hz")

    batteries = [s.battery_soc for s in samples if s.battery_soc is not None]
    if batteries:
        lines.append(f"  {'battery':<12}{batteries[0]}% -> {batteries[-1]}%")

    tilt = max_tilt_deg(samples)
    if tilt is not None:
        lines.append(f"  {'max tilt':<12}{tilt:.1f} deg")

    distance = path_length(samples)
    if distance > 0:
        lines.append(f"  {'odometry':<12}{distance:.2f} m walked (drifts - not a position)")

    for label, extract in (
        ("vx", 0),
        ("vyaw", 2),
    ):
        commanded = [s.commanded[extract] for s in samples if s.commanded is not None]
        measured = [s.measured[extract] for s in samples if s.measured is not None]
        for kind, series in (("cmd", commanded), ("meas", measured)):
            if not series:
                continue
            lines.append(
                f"  {label + ' ' + kind:<12}{min(series):+.2f} .. {max(series):+.2f}  "
                f"|{sparkline(series, width)}|"
            )

    tracking = tracking_error(samples)
    if tracking is not None:
        lines.append(
            f"  {'tracking':<12}mean |cmd-meas| on vx = {tracking:.3f} m/s "
            "(a constant offset is a wrong scale; a growing one is latency)"
        )

    for start, length in gaps(samples):
        lines.append(f"  {'GAP':<12}{length:.2f}s pause at t={start:.1f}s")

    return "\n".join(lines)


def tracking_error(samples: list[Sample]) -> float | None:
    """Mean absolute difference between commanded and measured forward speed."""
    pairs = [
        abs(s.commanded[0] - s.measured[0])
        for s in samples
        if s.commanded is not None and s.measured is not None
    ]
    if not pairs:
        return None
    return sum(pairs) / len(pairs)
