#!/usr/bin/env bash
# colcon build this workspace with the defaults that actually work.
#
# Needs: ROS 2 Humble sourced, and this clone under <workspace>/src/.
# Run tools/install-build-deps.sh first on a fresh host.
#
# What this exists to get right:
#   - colcon never removes what it installed, so a renamed or deleted node keeps
#     being offered by `ros2 run` until its build/ and install/ trees go.
#     --clean <pkg> is that.
#   - the bridge binaries carry no commit: a build that reaches robot_bridge writes
#     install/bridge-build.stamp (commit, changed-file count, time), and
#     tools/robot-bringup.sh passes it to the bridge as sources.*.
#
# Safe to re-run.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

WORKSPACE=""
CLEAN_PKGS=()
CLEAN_ALL=0

usage() {
    cat <<'EOF'
Usage: tools/build.sh [options] [-- colcon args...]

colcon build this workspace, T1_bridge included (no Booster SDK needed).

Options:
  --workspace <path>  colcon workspace root (default: the dir above src/)
  --t1                accepted and ignored: T1_bridge always builds
  --clean <pkg>       drop build/<pkg> and install/<pkg> first; repeatable
  --clean-all         drop build/, install/ and log/ first
  -h, --help          this text

Anything after -- goes to colcon, e.g.
  tools/build.sh -- --packages-select robot_bridge

Then: tools/test.sh
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --workspace) WORKSPACE="${2:?--workspace needs a path}"; shift ;;
        --t1)        ;;
        --clean)     CLEAN_PKGS+=("${2:?--clean needs a package name}"); shift ;;
        --clean-all) CLEAN_ALL=1 ;;
        -h|--help)   usage; exit 0 ;;
        --)          shift; break ;;
        *)           echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

# --- workspace -------------------------------------------------------------
if [ -z "$WORKSPACE" ]; then
    # Walk up for the dir holding src/, rather than assuming ~/bridge_ws.
    WORKSPACE="$REPO"
    while [ "$WORKSPACE" != "/" ] && [ ! -d "$WORKSPACE/src" ]; do
        WORKSPACE="$(dirname "$WORKSPACE")"
    done
    if [ ! -d "$WORKSPACE/src" ]; then
        echo "no colcon workspace above $REPO: this clone must live under <workspace>/src/." >&2
        echo "    mkdir -p ~/bridge_ws/src && ln -s $REPO ~/bridge_ws/src/humanoid_bridge" >&2
        echo "or pass --workspace <path>." >&2
        exit 1
    fi
fi
[ -d "$WORKSPACE" ] || { echo "no such workspace: $WORKSPACE" >&2; exit 1; }

# --- preflight -------------------------------------------------------------
command -v colcon >/dev/null || {
    echo "colcon not on PATH. Source ROS 2 and install it:" >&2
    echo "    source /opt/ros/humble/setup.bash && tools/install-build-deps.sh" >&2
    exit 1; }

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ROS_DISTRO is not set: source /opt/ros/humble/setup.bash first" >&2
    exit 1
fi

# thirdparty/unitree/*/CMakeLists.txt require this and their package.xml never
# declares it, so rosdep does not pull it and CMake fails with a bare
# "Could not find a package configuration file".
if ! [ -d "/opt/ros/$ROS_DISTRO/share/rosidl_generator_dds_idl" ] \
   && ! ros2 pkg prefix rosidl_generator_dds_idl >/dev/null 2>&1; then
    echo "rosidl_generator_dds_idl is missing; unitree_go/hg/api will not configure." >&2
    echo "    sudo apt-get install -y ros-$ROS_DISTRO-rosidl-generator-dds-idl" >&2
    echo "    (or: tools/install-build-deps.sh)" >&2
    exit 1
fi

# --- clean -----------------------------------------------------------------
if [ "$CLEAN_ALL" = 1 ]; then
    rm -rf "${WORKSPACE:?}/build" "${WORKSPACE:?}/install" "${WORKSPACE:?}/log"
    echo "cleared the whole workspace"
fi
for pkg in ${CLEAN_PKGS+"${CLEAN_PKGS[@]}"}; do
    rm -rf "${WORKSPACE:?}/build/$pkg" "${WORKSPACE:?}/install/$pkg"
    echo "cleared $pkg"
done

# --- build -----------------------------------------------------------------
echo "workspace: $WORKSPACE"
cd "$WORKSPACE"
# ON, not the default: a build dir from before keeps a cached OFF. Last: --cmake-args takes
# what follows it. Your own --cmake-args (the X2 build) replaces it.
T1_ARGS=(--cmake-args -DBUILD_BOOSTER_T1=ON)
for arg in "$@"; do [ "$arg" = --cmake-args ] && T1_ARGS=(); done
BRIDGE_RC=build/robot_bridge/colcon_build.rc   # colcon rewrites it on every robot_bridge build, a no-op one too
rc_before="$(stat -c %y "$BRIDGE_RC" 2>/dev/null || true)"
colcon build "$@" ${T1_ARGS+"${T1_ARGS[@]}"}

# --- stamp -----------------------------------------------------------------
# Only when this build reached robot_bridge: --packages-select example_controller leaves the old binary.
# "unknown" without a git dir (a tarball, or a worktree mounted into a container).
# No stamp when colcon installs elsewhere than install/ (the X2 build).
rc_after="$(stat -c %y "$BRIDGE_RC" 2>/dev/null || true)"
if [ -d install ] && [ -n "$rc_after" ] && [ "$rc_after" != "$rc_before" ]; then
    # -dirty from `status --porcelain`, as the commander's dirty flag: `describe --dirty` skips untracked files.
    commit="$(git -C "$REPO" describe --always 2>/dev/null)" || commit=unknown
    changed="$(git -C "$REPO" status --porcelain 2>/dev/null | wc -l)" || changed=unknown
    [ "$changed" -gt 0 ] 2>/dev/null && commit="$commit-dirty"
    printf 'build_commit=%s\nchanged_files=%s\nbuild_time=%s\n' \
        "$commit" "$changed" "$(date --iso-8601=seconds)" > install/bridge-build.stamp
fi

echo
echo "done. Source it, then test:"
echo "    source $WORKSPACE/install/setup.bash"
echo "    tools/test.sh"
