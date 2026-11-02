"""tools/build.sh: the build stamp in install/ that robot-bringup.sh passes as sources.*."""

import datetime
import os
import shutil
import subprocess
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent.parent / "build.sh"

# Passes the preflight (colcon, ROS_DISTRO, rosidl_generator_dds_idl) and builds nothing.
FAKE_BIN = "#!/bin/sh\nexit 0\n"
# colcon builds in the workspace: robot_bridge's rc file moves unless SKIP_BRIDGE is set.
FAKE_COLCON = (
    "#!/bin/sh\n"
    '[ -n "$SKIP_BRIDGE" ] && exit 0\n'
    "mkdir -p build/robot_bridge && touch build/robot_bridge/colcon_build.rc\n"
)
GIT = ["git", "-c", "user.name=t", "-c", "user.email=t@t", "-c", "commit.gpgsign=false"]


def clone_with_build_sh(tmp_path):
    """A git repo holding a copy of tools/build.sh, so the script's REPO is a repo of its own."""
    repo = tmp_path / "ws" / "src" / "humanoid_bridge"
    (repo / "tools").mkdir(parents=True)
    shutil.copy(SCRIPT, repo / "tools" / "build.sh")
    subprocess.run(["git", "init", "-q", str(repo)], check=True)
    subprocess.run([*GIT, "-C", str(repo), "add", "."], check=True)
    subprocess.run([*GIT, "-C", str(repo), "commit", "-q", "-m", "x"], check=True)
    return repo


def build(repo, ws, workspace_arg=None, cwd=None, **env):
    bin_dir = repo.parent.parent / "bin"
    bin_dir.mkdir(exist_ok=True)
    for name, text in (("colcon", FAKE_COLCON), ("ros2", FAKE_BIN)):
        (bin_dir / name).write_text(text)
        (bin_dir / name).chmod(0o755)
    env = {**os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}", "ROS_DISTRO": "humble", **env}
    command = ["bash", str(repo / "tools" / "build.sh"), "--workspace", workspace_arg or str(ws)]
    return subprocess.run(command, capture_output=True, text=True, env=env, cwd=cwd, timeout=60)


def stamp(ws):
    lines = (ws / "install" / "bridge-build.stamp").read_text().splitlines()
    return dict(line.split("=", 1) for line in lines)


def test_stamp_holds_commit_count_and_time(tmp_path):
    repo = clone_with_build_sh(tmp_path)
    ws = repo.parent.parent
    (ws / "install").mkdir()
    head = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "--short", "HEAD"], capture_output=True, text=True
    ).stdout.strip()
    result = build(repo, ws)
    assert result.returncode == 0, result.stderr
    values = stamp(ws)
    assert list(values) == ["build_commit", "changed_files", "build_time"]
    assert values["build_commit"] == head
    assert values["changed_files"] == "0"
    datetime.datetime.fromisoformat(values["build_time"])


def test_changed_file_marks_the_commit_dirty(tmp_path):
    repo = clone_with_build_sh(tmp_path)
    ws = repo.parent.parent
    (ws / "install").mkdir()
    (repo / "tools" / "build.sh").write_text("# changed\n" + SCRIPT.read_text())
    (repo / "new.txt").write_text("untracked")
    assert build(repo, ws).returncode == 0
    values = stamp(ws)
    assert values["build_commit"].endswith("-dirty")
    assert values["changed_files"] == "2"


def test_untracked_file_marks_the_commit_dirty(tmp_path):
    repo = clone_with_build_sh(tmp_path)
    ws = repo.parent.parent
    (ws / "install").mkdir()
    (repo / "new.txt").write_text("untracked")
    assert build(repo, ws).returncode == 0
    values = stamp(ws)
    assert values["build_commit"].endswith("-dirty")
    assert values["changed_files"] == "1"


def test_no_git_dir_gives_unknown(tmp_path):
    repo = clone_with_build_sh(tmp_path)
    ws = repo.parent.parent
    (ws / "install").mkdir()
    shutil.rmtree(repo / ".git")
    assert build(repo, ws).returncode == 0
    values = stamp(ws)
    assert values["build_commit"] == "unknown"
    assert values["changed_files"] == "unknown"


def test_no_install_dir_gives_no_stamp(tmp_path):
    repo = clone_with_build_sh(tmp_path)
    ws = repo.parent.parent
    assert build(repo, ws).returncode == 0
    assert not (ws / "install").exists()


def commit_file(repo, name):
    (repo / name).write_text(name)
    subprocess.run([*GIT, "-C", str(repo), "add", name], check=True)
    subprocess.run([*GIT, "-C", str(repo), "commit", "-q", "-m", name], check=True)


def test_partial_build_without_robot_bridge_keeps_the_stamp(tmp_path):
    repo = clone_with_build_sh(tmp_path)
    ws = repo.parent.parent
    (ws / "install").mkdir()
    assert build(repo, ws).returncode == 0
    first = stamp(ws)
    commit_file(repo, "b.txt")
    assert build(repo, ws, SKIP_BRIDGE="1").returncode == 0
    assert stamp(ws) == first
    assert build(repo, ws).returncode == 0
    assert stamp(ws)["build_commit"] != first["build_commit"]


def test_relative_workspace_gets_the_stamp(tmp_path):
    repo = clone_with_build_sh(tmp_path)
    ws = repo.parent.parent
    (ws / "install").mkdir()
    assert build(repo, ws, workspace_arg=ws.name, cwd=ws.parent).returncode == 0
    assert stamp(ws)["build_commit"] != "unknown"
