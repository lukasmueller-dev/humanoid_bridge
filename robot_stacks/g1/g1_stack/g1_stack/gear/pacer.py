"""Publishes goals at a steady rate while the main thread waits on the policy.

Why a thread. One `/act` takes 136 ms on loopback and 354 ms over wifi
(measured on the lab bench; humanoid-locoman-vla, `benches/g1/bench.md`,
"Milestone 0 results"), and a chunk is 400 ms of
motion. A single-threaded loop stops publishing for the whole call, so the
robot stalls and then jumps when the next chunk lands.

What it does instead: drains queued goals at the control rate, and when the
queue runs dry it **republishes the last goal with a fresh `target_time`**.
That holds the pose rather than stalling, and it keeps the controller's
one-second teleop timeout from firing
(`g1_decoupled_whole_body_policy.py:118`), which would inject a stop.
"""

from __future__ import annotations

import collections
import threading
import time


class GoalPacer:
    """Feed it goals; it publishes at `hz` until stopped."""

    def __init__(self, publisher, hz, clock=time.monotonic, sleep=time.sleep):
        self._publisher = publisher
        self._period = 1.0 / hz
        self._clock = clock
        self._sleep = sleep
        self._queue = collections.deque()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._last = None
        self._toggle = False
        self.published = 0
        self.held = 0

    # --- the producing side -------------------------------------------------

    def extend(self, goals):
        """Queue a chunk. Each goal's `target_time` is stamped on the way out,
        not here: queueing time is not publishing time."""
        with self._lock:
            self._queue.extend(goals)

    def request_toggle(self):
        """Put `toggle_policy_action` on the next goal out, and only that one.

        Out of band rather than on a queued goal, because a goal can be
        republished: with an empty queue `_tick` re-sends `_last` every period,
        which would flip the walk policy on and off at `hz`. The key rides the
        copy `_tick` sends, never the stored goal.
        """
        with self._lock:
            self._toggle = True

    def drop_queued(self):
        """Forget anything not yet published. Used on the way out, so a stop
        is not stuck behind a queued chunk."""
        with self._lock:
            self._queue.clear()

    @property
    def queued(self):
        with self._lock:
            return len(self._queue)

    # --- the publishing side ------------------------------------------------

    def _next_goal(self):
        with self._lock:
            if self._queue:
                self._last = self._queue.popleft()
                return self._last, False
        return self._last, True

    def _tick(self):
        goal, is_hold = self._next_goal()
        if goal is None:
            return
        now = self._clock()
        goal = dict(goal)
        goal["target_time"] = now + self._period
        goal["timestamp"] = now
        with self._lock:
            toggle, self._toggle = self._toggle, False
        if toggle:
            goal["toggle_policy_action"] = True
        self._publisher.send(goal)
        self.published += 1
        if is_hold:
            self.held += 1

    def run_until(self, stop_event):
        """The loop. Public so a test can drive it without a thread.

        Each tick is scheduled against an absolute deadline, not
        `period - work`. A sleep overshoots slightly every time and the
        relative form never wins that back, so the drain rate sits a few
        percent under `hz` and the queue grows for the whole run: 802 goals
        published against 840 produced over 28 s, a backlog of 1.3 s of motion
        that only grows. On a commanding run that is how far behind the robot
        executes.
        """
        next_at = self._clock()
        while not stop_event.is_set():
            self._tick()
            next_at += self._period
            now = self._clock()
            if next_at < now - self._period:
                # a long stall: rejoin the present rather than burst to catch up
                next_at = now
            if next_at > now:
                self._sleep(next_at - now)

    def start(self):
        self._thread = threading.Thread(target=self.run_until, args=(self._stop,), daemon=True)
        self._thread.start()
        return self

    def stop(self, timeout=2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
        self._thread = None
