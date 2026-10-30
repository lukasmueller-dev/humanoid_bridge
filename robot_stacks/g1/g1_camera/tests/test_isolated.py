"""The package must import on the Jetson, which has neither ROS nor the rest of
this repo's dependencies.

The package boundary makes this structural rather than conventional, but a
stray `import g1_stack...` would still only show up on the robot. This runs in
a subprocess with the absent modules blocked, because a plain import here would
find them already loaded.
"""

import os
import pathlib
import subprocess
import sys
import textwrap

# What the Jetson does not have.
ABSENT = ("msgpack", "msgpack_numpy", "rclpy", "g1_stack", "robot_bridge")

SCRIPT = """
import sys

class Blocked:
    def find_module(self, name, path=None):
        return self if name.split(".")[0] in {absent!r} else None

    def load_module(self, name):
        raise ImportError("{{}} is not installed on the Jetson".format(name))

sys.meta_path.insert(0, Blocked())

import g1_camera
from g1_camera import CameraClient, CameraServer, wire

assert wire.DEFAULT_PORT
assert CameraClient and CameraServer
print("ok")
"""


def test_the_package_imports_with_nothing_else_installed():
    # The subprocess gets no conftest, so hand it the package's parent directory.
    import g1_camera

    root = pathlib.Path(g1_camera.__file__).parent.parent
    env = dict(os.environ, PYTHONPATH=str(root))
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(SCRIPT.format(absent=ABSENT))],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout
