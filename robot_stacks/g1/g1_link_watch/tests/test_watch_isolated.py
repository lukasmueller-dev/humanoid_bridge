"""The core imports with the standard library alone: no ROS, no SDK, nothing else from this repo."""

import os
import pathlib
import subprocess
import sys

ABSENT = (
    "rclpy",
    "unitree_sdk2py",
    "cyclonedds",
    "numpy",
    "g1_stack",
    "g1_camera",
    "robot_bridge",
    "robot_sim",
)

# A None entry in sys.modules makes its import raise ImportError.
SCRIPT = f"""
import sys
sys.modules.update(dict.fromkeys({ABSENT!r}))
from g1_link_watch import watch
assert watch.Watcher
print("ok")
"""


def test_the_core_imports_with_nothing_else_installed():
    from g1_link_watch import watch

    # The subprocess gets no conftest, so hand it the package's parent directory.
    env = dict(os.environ, PYTHONPATH=str(pathlib.Path(watch.__file__).parents[1]))
    result = subprocess.run(
        [sys.executable, "-c", SCRIPT], capture_output=True, text=True, env=env, check=False
    )
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout
