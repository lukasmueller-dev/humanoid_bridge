"""Reads sources.* from the running /robot_bridge: the build stamp and the params file, as strings.

Usage: drive_sources.py <install/bridge-build.stamp> <params file>
"""

import hashlib
import os
import sys
from pathlib import Path

from harness import Checks, ros_node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

TIMEOUT = 10.0  # s


def main():
    stamp = dict(line.split("=", 1) for line in Path(sys.argv[1]).read_text().splitlines())
    params = Path(sys.argv[2])
    expected = {
        "sources.build_commit": stamp["build_commit"],
        "sources.build_time": stamp["build_time"],
        "sources.params_file": os.path.realpath(params),
        "sources.params_sha1": hashlib.sha1(params.read_bytes()).hexdigest(),
    }
    with ros_node("drive_sources") as node:
        checks = Checks(node, width=24)
        client = node.create_client(GetParameters, "/robot_bridge/get_parameters")
        if not client.wait_for_service(timeout_sec=TIMEOUT):
            checks.check("/robot_bridge answers", False, f"no get_parameters within {TIMEOUT} s")
            checks.finish()
        future = client.call_async(GetParameters.Request(names=list(expected)))
        checks.wait(future.done, TIMEOUT)
        values = future.result().values if future.done() else [None] * len(expected)
        for (name, want), got in zip(expected.items(), values):
            is_string = got is not None and got.type == ParameterType.PARAMETER_STRING
            ok = is_string and got.string_value == want
            checks.check(name, ok, want if ok else f"wanted the string {want!r}, got {got!r}")
        checks.finish()


if __name__ == "__main__":
    main()
