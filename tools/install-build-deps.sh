#!/usr/bin/env bash
# Install what a fresh ROS 2 Humble host lacks to build and test this repo.
#
# Needs: Ubuntu 22.04 with ROS 2 Humble installed and sourced, and sudo (or
# root). Installs nothing that a desktop ROS install already provides.
#
# Why these are not automatic:
#   rosidl_generator_dds_idl   required by thirdparty/unitree/*/CMakeLists.txt
#                              but declared in none of their package.xml, so
#                              rosdep never pulls it
#   rmw_cyclonedds_cpp         GEAR's sim forces raw CycloneDDS, so every ROS 2
#                              node has to match it. Humble defaults to Fast DDS
#   numpy pytest ruff          the test and lint path; no package declares them
#
# Safe to re-run.
set -euo pipefail

DRY_RUN=0
WITH_PYTHON=1
WITH_CYCLONE=1

usage() {
    cat <<'EOF'
Usage: tools/install-build-deps.sh [options]

Install the build and test dependencies this repo needs and rosdep does not
provide.

Options:
  --dry-run       print the apt and pip commands, install nothing
  --no-python     skip numpy, pytest and ruff (apt packages only)
  --no-cyclone    skip rmw_cyclonedds_cpp (Fast DDS-only host)
  -h, --help      this text

Then: tools/build.sh
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)    DRY_RUN=1 ;;
        --no-python)  WITH_PYTHON=0 ;;
        --no-cyclone) WITH_CYCLONE=0 ;;
        -h|--help)    usage; exit 0 ;;
        *)            echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ROS_DISTRO is not set: source your ROS 2 install first" >&2
    echo "    source /opt/ros/humble/setup.bash" >&2
    exit 1
fi
if [ "$ROS_DISTRO" != "humble" ]; then
    echo "warning: this repo is tested on humble, found '$ROS_DISTRO'." >&2
    echo "         continuing with ros-$ROS_DISTRO-* package names." >&2
fi

APT=(python3-colcon-common-extensions python3-pip
     "ros-$ROS_DISTRO-rosidl-generator-dds-idl")
[ "$WITH_CYCLONE" = 1 ] && APT+=("ros-$ROS_DISTRO-rmw-cyclonedds-cpp")

PIP=(numpy pytest ruff)

SUDO=""
if [ "$(id -u)" != 0 ]; then
    command -v sudo >/dev/null || {
        echo "not root and no sudo on PATH; re-run as root" >&2; exit 1; }
    SUDO="sudo"
fi

run() {
    if [ "$DRY_RUN" = 1 ]; then printf '%q ' "$@"; printf '\n'; else "$@"; fi
}

echo "apt: ${APT[*]}"
run $SUDO apt-get update
run $SUDO apt-get install -y --no-install-recommends "${APT[@]}"

if [ "$WITH_PYTHON" = 1 ]; then
    echo "pip: ${PIP[*]}"
    run python3 -m pip install --no-cache-dir "${PIP[@]}"
fi

[ "$DRY_RUN" = 1 ] && exit 0

echo
echo "done. Next: tools/build.sh"
