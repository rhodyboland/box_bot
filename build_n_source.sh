#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Build the workspace using symlinks to save space and speed up rebuilds
colcon build --symlink-install

# Source the install setup script to overlay the workspace
source install/setup.bash

echo "Build complete and environment sourced."
