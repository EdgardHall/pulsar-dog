"""Newline-delimited JSON telemetry recording.

One JSON object per line, so a run can be tailed live, grepped, or loaded with
``pandas.read_json(..., lines=True)`` afterwards.
"""

from __future__ import annotations

import json
import os
import time
from datetime import datetime, timezone
from typing import TextIO

from pulsar_dog.transport.base import RobotState


def default_log_path(log_dir: str, prefix: str = "run") -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return os.path.join(log_dir, f"{prefix}-{stamp}.jsonl")


class JsonlRecorder:
    """Append robot states to a .jsonl file.

    Flushes every record: a session that ends in a crash is exactly the one
    whose last few samples you want to read.
    """

    def __init__(self, path: str, flush_each: bool = True) -> None:
        self.path = path
        self._flush_each = flush_each
        self._file: TextIO | None = None
        self._count = 0
        self._started = time.monotonic()

    @property
    def count(self) -> int:
        return self._count

    def __enter__(self) -> JsonlRecorder:
        self.open()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    def open(self) -> None:
        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._file = open(self.path, "a", encoding="utf-8")
        self._started = time.monotonic()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None

    def write(self, state: RobotState, **extra) -> None:
        if self._file is None:
            raise RuntimeError("recorder used before open()")
        record = state.to_dict()
        record["t_rel"] = round(time.monotonic() - self._started, 4)
        if extra:
            record.update(extra)
        self._file.write(json.dumps(record, default=str) + "\n")
        self._count += 1
        if self._flush_each:
            self._file.flush()
