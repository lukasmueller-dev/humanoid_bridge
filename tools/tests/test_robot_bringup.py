"""tools/robot-bringup.sh: the --override refusals before ROS; the start command."""

import hashlib
import os
import shlex
import subprocess
from pathlib import Path

import pytest
import yaml

SCRIPT = Path(__file__).resolve().parent.parent / "robot-bringup.sh"
REPO = SCRIPT.parent.parent
JOINT = {"L_LEG_HIP_PITCH": {"dq_trip": 40.0}}


def bringup(tmp_path, *args):
    # A workspace that is not this clone: a start that passes the override check ends there, exit 1.
    command = ["bash", str(SCRIPT), "--workspace", str(tmp_path / "no_ws"), "--dry-run", *args]
    return subprocess.run(command, capture_output=True, text=True)


def override_file(tmp_path, content):
    path = tmp_path / "override.yaml"
    path.write_text(content if isinstance(content, str) else yaml.safe_dump(content))
    return path


def test_options_in_the_usage_text():
    result = subprocess.run(["bash", str(SCRIPT), "--help"], capture_output=True, text=True)
    assert result.returncode == 0
    assert "--override <file>" in result.stdout
    assert "--parallel-ankle" in result.stdout


needs_ros = pytest.mark.skipif(
    not Path("/opt/ros/humble/setup.bash").exists(), reason="sources ROS Humble"
)
T1_PARAMS = REPO / "bridge" / "robot_bridge" / "params" / "T1_config.yaml"


def sim_start(tmp_path, stamp=None, built_after_stamp=None):
    """The T1 start command's arguments after --ros-args; --sim needs no robot.

    built_after_stamp: robot_bridge's build is newer (True) or older (False) than the stamp.
    """
    ws = tmp_path / "ws"
    (ws / "src").mkdir(parents=True)
    (ws / "src" / "humanoid_bridge").symlink_to(REPO)
    (ws / "install").mkdir()
    (ws / "install" / "setup.bash").write_text("")
    if stamp is not None:
        (ws / "install" / "bridge-build.stamp").write_text(stamp)
    if built_after_stamp is not None:
        rc = ws / "build" / "robot_bridge" / "colcon_build.rc"
        rc.parent.mkdir(parents=True)
        rc.write_text("")
        moment = (ws / "install" / "bridge-build.stamp").stat().st_mtime
        os.utime(rc, (moment + (100 if built_after_stamp else -100),) * 2)
    command = ["bash", str(SCRIPT), "--robot", "t1", "--sim", "--parallel-ankle",
               "--workspace", str(ws), "--dry-run"]
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    start = [line for line in result.stdout.splitlines() if "robot_bridge T1_bridge" in line]
    assert len(start) == 1, result.stdout
    arguments = shlex.split(start[0])
    return result, arguments[arguments.index("--ros-args") + 1:]


def parameters(arguments):
    """The -p name:=value pairs, values as the bridge's parser sees them (quotes kept)."""
    pairs = (arguments[i + 1].split(":=", 1) for i, word in enumerate(arguments) if word == "-p")
    return {name: value for name, value in pairs}


@needs_ros
def test_parallel_ankle_reaches_the_bridge_command(tmp_path):
    _, arguments = sim_start(tmp_path)
    assert arguments[:4] == ["--params-file", str(T1_PARAMS), "-p", "parallel_ankle_mapping:=true"]


@needs_ros
@pytest.mark.parametrize("commit", ["abc1234-dirty", "1234567", "12e4567"])
def test_start_passes_the_sources_as_strings(tmp_path, commit):
    stamp = f"build_commit={commit}\nchanged_files=2\nbuild_time=2026-11-02T11:30:49+01:00\n"
    _, arguments = sim_start(tmp_path, stamp)
    assert parameters(arguments) == {
        "parallel_ankle_mapping": "true",
        "sources.build_commit": f'"{commit}"',
        "sources.build_time": '"2026-11-02T11:30:49+01:00"',
        "sources.params_file": f'"{T1_PARAMS.resolve()}"',
        "sources.params_sha1": f'"{hashlib.sha1(T1_PARAMS.read_bytes()).hexdigest()}"',
    }


@needs_ros
def test_start_without_a_stamp_warns_and_passes_unknown(tmp_path):
    result, arguments = sim_start(tmp_path)
    assert "bridge-build.stamp" in result.stderr
    assert parameters(arguments)["sources.build_commit"] == '"unknown"'
    assert parameters(arguments)["sources.build_time"] == '"unknown"'


@pytest.mark.parametrize("robot", ["g1", "go2"])
def test_parallel_ankle_on_another_robot_is_refused(tmp_path, robot):
    result = bringup(tmp_path, "--robot", robot, "--parallel-ankle")
    assert result.returncode == 2
    assert "refusing --parallel-ankle" in result.stderr


def test_override_under_override_only_passes_the_check(tmp_path):
    path = override_file(tmp_path, {"robot_bridge": {"ros__parameters": {"override": JOINT}}})
    result = bringup(tmp_path, "--override", str(path))
    assert result.returncode == 1
    assert "is not this clone" in result.stderr
    assert "--override" not in result.stderr


def test_missing_file_is_refused(tmp_path):
    result = bringup(tmp_path, "--override", str(tmp_path / "missing.yaml"))
    assert result.returncode == 2
    assert "no file at" in result.stderr


@pytest.mark.parametrize(
    "content",
    [
        {"robot_bridge": {"ros__parameters": {"q_rate_limit_default": 99.0, "override": JOINT}}},
        {"robot_bridge": {"ros__parameters": {"override": JOINT}, "other": 1}},
        {"robot_bridge": {"ros__parameters": {"L_LEG_HIP_PITCH": {"dq_limit": 99.0}}}},
        {"other_node": {"ros__parameters": {"override": JOINT}}},
        {"robot_bridge": {"ros__parameters": {"override": JOINT}}, "other_node": {}},
        {"robot_bridge": {"ros__parameters": {"override": 5}}},
        {"robot_bridge": {"ros__parameters": {"override": {}}}},
        {"robot_bridge": {"ros__parameters": {}}},
        "",
        "robot_bridge: [",
    ],
    ids=[
        "other key beside override",
        "other key beside ros__parameters",
        "no override, a joint key",
        "other node",
        "other top-level key",
        "override not a mapping",
        "override empty",
        "ros__parameters empty",
        "empty file",
        "not yaml",
    ],
)
def test_a_key_outside_override_is_refused(tmp_path, content):
    path = override_file(tmp_path, content)
    result = bringup(tmp_path, "--override", str(path))
    assert result.returncode == 2
    assert "refusing --override" in result.stderr


@needs_ros
def test_stamp_older_than_the_bridge_build_is_not_passed(tmp_path):
    stamp = "build_commit=abc1234\nchanged_files=0\nbuild_time=2026-11-02T11:30:49+01:00\n"
    result, arguments = sim_start(tmp_path, stamp, built_after_stamp=True)
    assert "older than the robot_bridge build" in result.stderr
    assert parameters(arguments)["sources.build_commit"] == '"unknown"'
    assert parameters(arguments)["sources.build_time"] == '"unknown"'


@needs_ros
def test_stamp_newer_than_the_bridge_build_is_passed(tmp_path):
    stamp = "build_commit=abc1234\nchanged_files=0\nbuild_time=2026-11-02T11:30:49+01:00\n"
    result, arguments = sim_start(tmp_path, stamp, built_after_stamp=False)
    assert result.stderr == ""
    assert parameters(arguments)["sources.build_commit"] == '"abc1234"'
