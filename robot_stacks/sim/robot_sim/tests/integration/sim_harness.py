"""Shared by the drive_sim*.py drivers: PASS/FAIL lines, spin on one node, ROS setup, exit."""

import contextlib
import sys
import time

import rclpy


class Checks:
    """`check` prints one PASS/FAIL line and records failures; `finish` ends the run."""

    def __init__(self, node):
        self.node = node
        self.failures = []

    def check(self, name, ok, detail=""):
        suffix = f"  ({detail})" if detail else ""
        print(f"{'PASS' if ok else 'FAIL'}  {name}{suffix}", flush=True)
        if not ok:
            self.failures.append(name)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

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
