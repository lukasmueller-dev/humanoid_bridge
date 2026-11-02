"""tools/test.sh: a stage flag alone still gets the stale-build check."""

import os
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent.parent / "test.sh"

# `ros2 pkg prefix <pkg>` names a workspace with no build dir; any other ros2 call fails.
FAKE_ROS2 = """#!/bin/sh
[ "$1 $2" = "pkg prefix" ] && echo "$WS/install/$3" && exit 0
exit 1
"""


@pytest.mark.parametrize("stage", ["--sim", "--sources"])
def test_stage_alone_fails_on_a_stale_build(tmp_path, stage):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    ros2 = bin_dir / "ros2"
    ros2.write_text(FAKE_ROS2)
    ros2.chmod(0o755)
    env = {**os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}", "WS": str(tmp_path / "ws")}
    result = subprocess.run(
        ["bash", str(SCRIPT), stage], capture_output=True, text=True, env=env, timeout=60
    )
    assert result.returncode == 1
    assert "no colcon build dir" in result.stdout
