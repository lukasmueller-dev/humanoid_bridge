"""run.spin: a stop in a hold damps first; a crash closes the writer and ends the process."""

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


class HoldingWatcher:
    """In hold; `damp` turns it to damp, 3 steps later it is back in watch."""

    mode = "hold"

    def __init__(self):
        self.calls = []

    def damp(self, now, why):
        self.calls.append("damp")
        if self.mode == "hold":
            self.mode = "damp"

    def step(self, now):
        self.calls.append("step")
        if self.calls.count("step") == 3:
            self.mode = "watch"


def test_a_stop_in_a_hold_damps_until_the_watcher_is_silent():
    watcher = HoldingWatcher()
    stop = threading.Event()
    stop.set()
    spin(watcher, stop)
    assert watcher.mode == "watch"
    assert watcher.calls == ["damp", "step"] * 3
