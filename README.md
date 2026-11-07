# box_bot
Hoverboard powered ros2 robot
```
git clone --recurse-submodules git@github.com:rhodyboland/box_bot.git -b orin_nano_lyrical
```
## JetPack 7.2.1 / Isaac ROS 5 port

Work is on `orin_nano_lyrical`. The hoverboard and lidar forks use
`codex/lyrical` branches. Build and simulated tests precede hardware validation.
See [migration status and assessment](LYRICAL_MIGRATION.md) and
[40-pin header setup / next-boot configuration](HEADER_SETUP.md).

Configure and build the environment on the Jetson host:

```bash
export ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev"
cd "${ISAAC_ROS_WS}/src/box_bot"
python3 scripts/configure_isaac_ros.py
isaac-ros activate --build-local --build-only
isaac-ros activate
```

The configuration pins the verified NVIDIA Jetson base image and adds NVIDIA's
patched RealSense layer, then `Dockerfile.box_bot`. The setup script refuses to
overwrite unrelated Dockerfile search settings. If a container is already
running, the CLI attaches to it; it does not acquire newly built image layers.
Exit that shell yourself before activating the new image.

Inside the container:

```bash
cd "${ISAAC_ROS_WS}"
./src/box_bot/scripts/install_rosdeps.sh
./src/box_bot/scripts/build_lyrical.sh
source install-lyrical/setup.bash
```

The build wrapper excludes the inherited RealSense source packages, which must
not shadow the CLI's patched camera driver, and uses separate Lyrical outputs.
Use the wrapper for builds; plain `colcon build` will discover the old camera.
The rosdep wrapper also excludes that source tree from dependency discovery.

`box_bot_nav_gps.launch.py` enables the ESP32 battery hub by default. It receives
CRC-checked Hub v1 telemetry from `/dev/boxbot/hub` at 115200 baud. Install the
repository's udev rules to create this alias for USB serial `5CBC399477`.
Override with `hub_serial_port:=...`, or disable with `start_hub:=false`.
`/battery/state` (`sensor_msgs/msg/BatteryState`, sensor-data QoS) contains pack
voltage, signed current (negative means discharging), charge fraction, cell
voltages and the higher battery probe temperature. MOS and both probe
temperatures, packet errors and freshness appear under `boxbot_hub/*` on
`/diagnostics`. Missing/stale telemetry produces ERROR diagnostics after five
seconds; cached battery samples are not republished as fresh. Unknown capacity,
chemistry, charging status and health are left unknown. The node only reads
USB; it sends no hub/BMS commands and reconnects if USB is unplugged.

```bash
ros2 topic echo /battery/state --qos-reliability best_effort
```

GPS bringup uses a quiet terminal by default: INFO/DEBUG messages and the
repeated hoverboard `Velocity command timed out. Braking.` warning are hidden
on screen. Other warnings, errors and unstructured output such as tracebacks
remain visible. Full process stdout/stderr (including the hidden messages)
is written to the run's `launch.log`; node logger levels are unchanged.
Use `console_verbose:=true` to show everything in the terminal again.
Inside the container, follow the complete current log with:

```bash
tail -f ~/.ros/log/latest/launch.log
```

If `ROS_LOG_DIR` is configured, look under that directory instead.

Velocity commands now use `geometry_msgs/msg/TwistStamped` on `/cmd_vel`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link
```

Do not run robot bringup or teleop until GPIO/pinmux and device identities are
validated and hardware operation is authorized. See the migration assessment for the read-only overlay checks and remaining
physical validation before installing it.

# packages:

hoverboard driver:
https://github.com/rhodyboland/hoverboard_ros2_control

sllidar_ros2:
https://github.com/Slamtec/sllidar_ros2

realsense2_camera:
https://github.com/realsenseai/realsense-ros/tree/r/4.56.3

The inherited wrapper submodule is excluded from Lyrical builds. NVIDIA's CLI
RealSense layer provides the SDK and patched wrapper for this branch.

gps:
The launch files use the pinned `aussierobots/ublox_dgnss` submodule.
BOXBOT now has two ZED-X20P receivers with custom USB serial names; see
[GNSS setup and driver assessment](GNSS_SETUP.md) for the confirmed front/back
mapping, udev rules, build instructions and validation results.

mag:
https://github.com/rhodyboland/py_qmc6310_driver
