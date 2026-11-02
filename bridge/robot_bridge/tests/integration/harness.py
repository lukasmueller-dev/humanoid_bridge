"""Shared by the drive_*.py drivers: PASS/FAIL lines, spin and wait on one node, ROS setup, exit."""

import contextlib
import sys
import time

import rclpy


class Checks:
    """`check` prints one PASS/FAIL line and records failures; `finish` ends the run."""

    def __init__(self, node=None, width=44, poll=0.005):
        self.node = node
        self.width = width  # name column
        self.poll = poll  # spin_once timeout, s
        self.failures = []

    def check(self, name, ok, detail=""):
        status = "PASS" if ok else "FAIL"
        print(f"{name:<{self.width}} {status}{'  ' + detail if detail else ''}", flush=True)
        if not ok:
            self.failures.append(name)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=self.poll)

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=self.poll)
        return predicate()

    def finish(self):
        """Print the verdict; exit 1 if a check failed."""
        if self.failures:
            print("\n{} check(s) failed: {}".format(len(self.failures), ", ".join(self.failures)))
            sys.exit(1)
        print("\nall checks passed")


@contextlib.contextmanager
def ros_node(name):
    rclpy.init()
    node = rclpy.create_node(name)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()
