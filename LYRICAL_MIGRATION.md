# Boxbot Lyrical port

Status: 5 November 2026. The software port builds and passes the hardware-free
checks below. The final CLI image built successfully after fixing the missing pip dependency.
A temporary read-only container passed package, ROS-message, and SDK-library checks. Physical robot
operation has not been tested or authorized.

## Branches and baseline

| Repository | Branch | Port revision |
| --- | --- | --- |
| box_bot | `orin_nano_lyrical`, from `orin_nano` at `bb4e158` | Working port |
| hoverboard_ros2_control (Rhody's fork) | `codex/lyrical` | `92069fb` |
| sllidar_ros2 (Rhody's fork) | `codex/lyrical` | `53f0c52` |

Commits are local; branches have not been pushed. The other submodules retain
their original pins. BNO085 builds with a deprecated CMake macro warning, and
serial and u-blox build without source changes, so they did not need new forks.
The old RealSense submodule remains as a historical pin and is excluded from
Lyrical builds and rosdep discovery.

Host inspected: Orin Nano Super, Ubuntu 24.04.4, L4T R39.2.1, official Isaac ROS
CLI `2.6.0-1.20260919010821`, initialized in Docker mode. There is no host
`/opt/ros` installation. NVIDIA's buildfarm supplies Lyrical for Noble; do not
apply upstream Ubuntu 26.04 installation instructions to this Jetson.

Source builds used NVIDIA's Lyrical packages, including ros2_control 6.9.0 and
Nav2 1.5.0. The installed API is the authority: some APIs already removed in
newer upstream documentation remain deprecated in NVIDIA's shipped build.

## What changed

| Area | Implemented change |
| --- | --- |
| Development environment | Replaced the Humble Dockerfile with a CLI layer; configured the workspace for `isaac_ros.realsense.box_bot`. `docker/Dockerfile.isaac_ros` reuses the verified official Jetson image by digest, avoiding a full base-toolchain rebuild. |
| RealSense ownership | NVIDIA's CLI layer supplies SDK v2.56.3 and wrapper r/4.56.3 with its Lyrical patch. The old workspace wrapper is excluded through the build defaults and rosdep wrapper. Camera launch now uses `depth_module.depth_profile` and `rgb_camera.color_profile`. |
| Hardware interface | Updated `on_init` to `HardwareComponentInterfaceParams`; replaced member-pointer exports with framework-managed state and command interfaces. Activation initializes commands to zero. Deactivation sends a stop packet. Nonfinite commands send zero and report an error. Encoder scaling and wheel calibration remain unchanged. |
| Controller/teleop | Uses `TwistStamped` on `/cmd_vel` and the differential-drive controller's `cmd_vel` input. Removed `use_stamped_vel` and obsolete limit-enable booleans; acceleration/deceleration limits use the forward/reverse schema and disabled jerk limits use NaN. The gamepad configuration enables stamped output. Robot description reaches controller manager on its ROS topic. |
| Logging | Motion-chain logger subscribes to `TwistStamped` and reads the nested twist fields. |
| Nav2 | Migrated progress-checker lists, plugin identifiers, explicit stamped-command settings, `error_code_name_prefixes`, local/global behavior costmap topics, and frames. DWB pruning now uses the shared `FeasiblePathHandler`; its transform tolerance comes from the local costmap. Both GPS and indoor configurations pass inactive lifecycle configuration. |
| Nav2 launch | Retains Boxbot's seven-server subset, with installed parameter paths, current components, namespace handling, and both controller/behavior velocity output passing through the smoother. The stock launch also starts servers that Boxbot does not configure. |
| Behavior tree | Added BT.CPP v4 format metadata to the GPS tree. Executing the tree and goal handling still require runtime validation. |
| Build system | Updated the lidar fork to exported CMake targets. Corrected hoverboard library/Boost linkage and declared runtime dependencies. Separate Lyrical output directories and bounded build concurrency avoid Humble artifact reuse. |
| Python/GPIO | Pins Jetson.GPIO 2.1.13 in an isolated Python target directory rather than modifying system Python packages on Noble. GPIO itself has not been exercised. |

Isaac ROS 5 also migrates accelerated transport toward Lyrical `rosidl::Buffer`
and CUDA buffer backends. Boxbot has no direct NITROS API integration to port.
The Python point-cloud serialization regression passes with the installed
Lyrical message representation.

## Working with the CLI

Use the commands in [README.md](README.md). Configuration is tracked in
`config/isaac-ros-cli.yaml`; `scripts/configure_isaac_ros.py` installs the workspace
YAML and Dockerfile search settings. It refuses to overwrite unrelated search
settings. Re-running it is safe for the settings it generated.

The CLI loads packaged, system, user, then workspace YAML settings; invocation
`-c key=value` overrides come last. In the installed loader, mappings merge but
lists are replaced. The workspace image-layer list is therefore complete.

- `isaac-ros activate` replaces `isaac_ros_common/scripts/run_dev.sh`.
- `isaac-ros activate --build-local --build-only` prepares custom images without
  starting or replacing a container.
- `isaac-ros activate` attaches to an existing container first. An already running
  container does not acquire new image dependencies; exit that shell yourself
  before opening the newly built image.
- Container paths use `/workspaces/isaac_ros-dev`; source is bind-mounted there.
- Use `scripts/build_lyrical.sh` from inside the environment. It selects
  `colcon.defaults.yaml`, excludes the old camera packages, and builds into
  `build-lyrical` / `install-lyrical` with logs under `log-lyrical`.
- `scripts/install_rosdeps.sh` omits the old camera source tree. Do not use an
  unrestricted `rosdep --from-paths src` or plain `colcon build` for this branch.
- `~/.isaac_ros_dev-dockerargs` and entrypoint additions remain supported in the
  installed CLI. Existing Boxbot mount/group scripts are retained pending actual
  device validation; they are not made obsolete by their historical filenames.

The software port did not alter host device settings or restart containers.
Subsequent header preparation staged the GPIO overlay for the next boot using
Jetson-IO; see [40-pin header setup](HEADER_SETUP.md). No live GPIO was driven.

## Validation completed

- Full source build: **12 packages**, using the fresh Lyrical output directories.
- Focused CTest run: **25 tests reported by `colcon test-result`, zero errors,
  failures, or skips**. Includes GNSS migration, the four hoverboard
  interface/schema tests, BNO I2C mock tests, and watchdog tests.
- Python suite separately: **9 passed**, including stamped-command logging and
  `PointCloud2` serialization/buffer access.
- Both navigation YAMLs: all seven servers configured in an isolated ROS domain
  and unique namespace, with no activation, hardware driver, or sensor process.
  The test caught the now-rejected `error_code_names` parameter before migration.
- Composed Nav2 bringup also loaded and configured all seven servers successfully
  with autostart disabled in an isolated domain and unique namespace.
- GPS/Nav2 launch argument generation, URDF expansion, Python syntax, shell syntax,
  and Git whitespace checks passed.

Repeat the focused checks inside the development environment:

```bash
cd "${ISAAC_ROS_WS}"
./src/box_bot/scripts/build_lyrical.sh
source install-lyrical/setup.bash
COLCON_DEFAULTS_FILE="${ISAAC_ROS_WS}/src/box_bot/colcon.defaults.yaml" \
  ROS_DOMAIN_ID=137 colcon --log-base log-lyrical test \
  --packages-select box_bot hoverboard_control bno08x_driver \
  --return-code-on-test-failure \
  --ctest-args -R 'gnss_migration|test_lyrical_interface|test_i2c_mock|test_watchdog' \
  --output-on-failure
colcon --log-base log-lyrical test-result --test-result-base build-lyrical --verbose
python3 src/box_bot/scripts/check_nav2_configuration.py
python3 src/box_bot/scripts/check_nav2_configuration.py --composition
python3 src/box_bot/scripts/check_nav2_configuration.py \
  --params src/box_bot/src/nav2_launcher/config/nav2_params.yaml
```

The navigation script configures but never activates its processes, uses domain
138 by default, and cleans them up afterward. It does not validate physical TF,
sensor rates, obstacle response, command timeout, or navigation behavior.

## Camera image build findings

The NVIDIA SDK build recovered from its initial Python-discovery failure and
produced the CUDA-enabled library and Python bindings. The patched ROS wrapper
was packaged into the RealSense image, but eight upstream integration/rosbag
CTest cases timed out. Their cause is not established; camera validation remains
open. These are separate from the 25 passing workspace tests.

The final Boxbot layer then failed with `No module named pip`. Its Dockerfile now
installs `python3-pip` explicitly before installing Jetson.GPIO. The undefined
PYTHONPATH warning is also corrected. The required `BASE_IMAGE` build-argument
lint warning is nonfatal; the CLI supplies that argument.

## Remaining validation

1. The final image is built: `isaac_ros-realsense-box_bot_d1df40f9a9b44b7e900d7a8c9ce9f931-arm64-jetpack`
   in the `nvcr.io/nvidia/isaac/ros` repository. A temporary container with no
   network, GPU, device mounts, or workspace mounts resolved RealSense from
   `/opt/ros/lyrical`, imported ROS messages, loaded `librealsense2.so`, and
   confirmed Jetson.GPIO 2.1.13. Installed versions: RealSense wrapper 4.56.3,
   hardware_interface 6.9.0, navigation2 1.5.0. The 25 workspace tests were run
   previously in the development container; upstream camera test timeouts remain
   unresolved. A fresh development shell can use
   `isaac-ros activate -c docker.run.container_name=boxbot_lyrical`
   to avoid attaching to the older base container.
2. The GPIO overlay is now staged for the next boot; see [header setup](HEADER_SETUP.md).
   Verify physical pinmux/GPIO behavior after installation in the robot.
   The initial context note about differing overlays was stale: the current DTS
   and DTBO both have pin 7 tristate 1 and Schmitt 1, matching the reported old
   installed values. The checked-in DTBO also applied successfully to a temporary
   copy of the running R39 device tree using `fdtoverlay`; no live tree or boot
   configuration was changed. This validates the target symbols, not electrical
   behavior.
3. Recheck device groups, identities, `/dev/boxbot` aliases, and container
   visibility when the sensors are connected. The read-only check found
   `/dev/ttyTHS1`, `/dev/ttyTHS2`, and `/dev/i2c-7`, but no GNSS, lidar, or RealSense
   USB devices and no `/dev/boxbot` aliases. The rules already match programmed
   GNSS serial names `BOXBOT_BASE` / `BOXBOT_ROVER`, rather than USB port order.
4. Check D455 firmware against NVIDIA's documented 5.16.0.1 requirement. No camera
   firmware was read or flashed during this work.
5. With explicit hardware authorization, test wheel feedback and stale-command
   stopping with wheels raised, then IMU, GNSS/RTCM, TF ownership, D455 point clouds,
   costmap marking/clearing, BT execution, goals, cancellation, and floor motion.

Workspace instructions require explicit authorization before physical robot
motion, GPIO toggling, restarting existing containers, or hardware-stack launch.
The inactive tests above did none of those things.

## Upstream references

- [Isaac ROS getting started](https://nvidia-isaac-ros.github.io/getting_started/index.html)
- [Isaac ROS 5 release notes](https://nvidia-isaac-ros.github.io/v/release-5.0/releases/index.html)
- [CLI environment and custom layers](https://nvidia-isaac-ros.github.io/concepts/dev_env/index.html)
- [RealSense pins and setup](https://nvidia-isaac-ros.github.io/getting_started/sensors/realsense_setup.html)
- [Lyrical ros2_control migration](https://control.ros.org/lyrical/doc/ros2_control/doc/migration.html)
- [Lyrical differential-drive controller](https://control.ros.org/lyrical/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [Nav2 Kilted to Lyrical migration](https://docs.nav2.org/rolling/configuration_and_development/migration_guides/kilted/Kilted/)
- [NITROS migration](https://nvidia-isaac-ros.github.io/concepts/rosidl_buffer/nitros_migration.html)
