"""run.spin: a crash in the loop closes the writer and ends the process."""

import os
import threading

import pytest

from g1_link_watch.run import spin


class FakeWriter:
    closed = False

    def close(self):
        self.closed = True


class CrashingWatcher:
    mode = "hold"

    def __init__(self):
        self.writer = FakeWriter()

    def step(self, now):
        raise RuntimeError("crash")


def test_a_crash_closes_the_writer_and_exits_1(monkeypatch):
    def exit_(code):
        raise SystemExit(code)

    monkeypatch.setattr(os, "_exit", exit_)
    watcher = CrashingWatcher()
    writer = watcher.writer
    with pytest.raises(SystemExit) as exited:
        spin(watcher, threading.Event())
    assert exited.value.code == 1
    assert writer.closed
