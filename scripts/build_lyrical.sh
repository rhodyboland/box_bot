#!/usr/bin/env bash
# Run inside the Isaac ROS development container. No hardware is launched.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export COLCON_DEFAULTS_FILE="${repo_dir}/colcon.defaults.yaml"
cd "${ISAAC_ROS_WS:?Activate the Isaac ROS environment first}"
set +u
source /opt/ros/lyrical/setup.bash
set -u
colcon --log-base log-lyrical build "$@"
