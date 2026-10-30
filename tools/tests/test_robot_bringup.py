"""tools/robot-bringup.sh --override: the refusals that come before ROS is touched."""

import subprocess
from pathlib import Path

import pytest
import yaml

SCRIPT = Path(__file__).resolve().parent.parent / "robot-bringup.sh"
JOINT = {"L_LEG_HIP_PITCH": {"dq_trip": 40.0}}


def bringup(tmp_path, *args):
    # A workspace that is not this clone: a start that passes the override check ends there, exit 1.
    command = ["bash", str(SCRIPT), "--workspace", str(tmp_path / "no_ws"), "--dry-run", *args]
    return subprocess.run(command, capture_output=True, text=True)


def override_file(tmp_path, content):
    path = tmp_path / "override.yaml"
    path.write_text(content if isinstance(content, str) else yaml.safe_dump(content))
    return path


def test_override_in_the_usage_text():
    result = subprocess.run(["bash", str(SCRIPT), "--help"], capture_output=True, text=True)
    assert result.returncode == 0
    assert "--override <file>" in result.stdout


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
