"""run.poll_mode: a CheckMode that raises skips one report; the poll goes on."""

import json
import threading

from g1_link_watch import run


class Switcher:
    def __init__(self, stop):
        self.stop = stop
        self.calls = 0

    def CheckMode(self):
        self.calls += 1
        if self.calls == 1:
            return 0, json.loads("")  # the SDK's json.loads on an empty reply
        self.stop.set()
        return 0, {"name": ""}


class Watcher:
    def __init__(self):
        self.reports = []

    def on_motion_mode(self, name, now):
        self.reports.append(name)


def test_an_exception_in_check_mode_skips_one_report(monkeypatch):
    monkeypatch.setattr(run, "MODE_POLL_S", 0.001)
    stop = threading.Event()
    watcher = Watcher()
    run.poll_mode(Switcher(stop), watcher, stop)
    assert watcher.reports == [""]
