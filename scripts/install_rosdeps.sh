#!/usr/bin/env bash
# Deliberately omit the inherited RealSense wrapper from rosdep discovery.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
rosdep install --rosdistro lyrical --ignore-src -y --from-paths \
  "${repo_dir}/src/box_bot" "${repo_dir}/src/nav2_launcher" \
  "${repo_dir}/src/hoverboard_ros2_control" "${repo_dir}/src/sllidar_ros2" \
  "${repo_dir}/src/bno08x_driver" "${repo_dir}/src/serial" \
  "${repo_dir}/src/ublox_dgnss" "$@"
