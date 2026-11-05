#!/usr/bin/env python3
"""Install Boxbot CLI workspace settings without changing host device setup."""
import os
from pathlib import Path
import sys

import yaml
from isaac_ros_cli.config.loader import ConfigScope, update_config

repo = Path(__file__).resolve().parents[1]
workspace = Path(os.environ['ISAAC_ROS_WS']).resolve()
if repo.parent != workspace / 'src':
    sys.exit('Expected this checkout at ${ISAAC_ROS_WS}/src/box_bot')

# Keep layer lookup and Docker build context local to this checkout.
settings = workspace / 'scripts' / '.isaac_ros_common-config'
content = f'CONFIG_DOCKER_SEARCH_DIRS=("{repo}/docker" "{repo}" /etc/isaac-ros-cli/docker)\n'
previous = f'CONFIG_DOCKER_SEARCH_DIRS=("{repo}" /etc/isaac-ros-cli/docker)\n'
if settings.exists() and settings.read_text() not in (content, previous):
    sys.exit(f'Refusing to overwrite existing layer search settings: {settings}')
settings.parent.mkdir(parents=True, exist_ok=True)
settings.write_text(content)
update_config(yaml.safe_load((repo / 'config/isaac-ros-cli.yaml').read_text()), ConfigScope.WORKSPACE)
print(f'Configured {workspace} for isaac_ros.realsense.box_bot')
