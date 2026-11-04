"""tools/test.sh: a stage flag alone gets the stale-build check; sources skips without a stamp."""

import os
import subprocess
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent.parent / "test.sh"
REPO = SCRIPT.parent.parent

# `ros2 pkg prefix <pkg>` names a workspace with no build dir; any other ros2 call fails.
FAKE_ROS2 = """#!/bin/sh
[ "$1 $2" = "pkg prefix" ] && echo "$WS/install/$3" && exit 0
exit 1
"""


def run_test_sh(tmp_path, stage):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    ros2 = bin_dir / "ros2"
    ros2.write_text(FAKE_ROS2)
    ros2.chmod(0o755)
    env = {**os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}", "WS": str(tmp_path / "ws")}
    return subprocess.run(
        ["bash", str(SCRIPT), stage], capture_output=True, text=True, env=env, timeout=60
    )


@pytest.mark.parametrize("stage", ["--sim", "--sources"])
def test_stage_alone_fails_on_a_stale_build(tmp_path, stage):
    result = run_test_sh(tmp_path, stage)
    assert result.returncode == 1
    assert "no colcon build dir" in result.stdout


def current_build(tmp_path):
    """A workspace with a robot_bridge build of this clone, newer than every file under bridge/."""
    build = tmp_path / "ws" / "build" / "robot_bridge"
    build.mkdir(parents=True)
    home = REPO / "bridge" / "robot_bridge"
    (build / "CMakeCache.txt").write_text(f"CMAKE_HOME_DIRECTORY:INTERNAL={home}\n")
    rc = build / "colcon_build.rc"
    rc.write_text("0")
    later = time.time() + 1000
    os.utime(rc, (later, later))
    (tmp_path / "ws" / "install").mkdir()
    return rc


def test_sources_skips_without_a_stamp(tmp_path):
    current_build(tmp_path)
    result = run_test_sh(tmp_path, "--sources")
    assert result.returncode == 2
    assert "sources" in result.stdout and "SKIP" in result.stdout
    assert "tools/build.sh writes it" in result.stdout


def test_sources_skips_with_a_stamp_older_than_the_build(tmp_path):
    rc = current_build(tmp_path)
    stamp = tmp_path / "ws" / "install" / "bridge-build.stamp"
    stamp.write_text("build_commit=abc1234\n")
    os.utime(stamp, (rc.stat().st_mtime - 100,) * 2)
    result = run_test_sh(tmp_path, "--sources")
    assert result.returncode == 2
    assert "tools/build.sh writes it" in result.stdout
