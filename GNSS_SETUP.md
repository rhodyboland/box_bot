# BOXBOT GNSS

BOXBOT uses the pinned [aussierobots/ublox_dgnss](https://github.com/aussierobots/ublox_dgnss)
submodule at `src/ublox_dgnss`, revision
`1cd9ca9e1a959df51d2631e444400bdf0b327c2f` (package version 0.7.6).
The submodule is unmodified; BOXBOT configuration and ROS conversions live in
`src/box_bot`.

## Receiver identities

Positions confirmed by the operator on 4 November 2026.

| Position | USB serial | Role alias | Position alias |
| --- | --- | --- | --- |
| Front | `BOXBOT_BASE` | `/dev/boxbot/gps_moving_base` | `/dev/boxbot/gps_front` |
| Back | `BOXBOT_ROVER` | `/dev/boxbot/gps_rover` | `/dev/boxbot/gps_back` |

Both devices report ZED-X20P, HPG 2.11, protocol 50.11, and USB `1546:01ab`.
The custom USB serials identify receivers across USB port changes. The driver
selects those serial strings directly through libusb, rather than opening tty
aliases. Run only one driver per receiver.

`99-boxbot-devices.rules` grants `0660 dialout` on tty nodes, `0660 plugdev`
on the main USB devices, and disables ModemManager probing. The installed rules,
aliases and permissions were verified on the Jetson. The aliases also survived serial-interface re-enumeration: front/base moved to
`ttyACM0` and back/rover to `ttyACM1` without changing the role names. A physical
USB port swap or reboot has not been tested.

The Isaac ROS container needs `/dev/bus/usb` mounted with device access and the
host USB device groups mapped to its user. `install_boxbot_udev_rules.sh`
maintains the USB mount; `10_add_gpio_groups.sh` preserves each USB device's
udev group and adds the container user to those groups.

## Build and run

Initialize dependencies from the superproject's pinned commits:

```bash
git submodule update --init --recursive
```

Inside the BOXBOT Humble container, from the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to box_bot --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For receiver/fix checks without starting robot motion or navigation nodes:

```bash
ros2 launch box_bot gnss.launch.py
```

For the complete existing robot navigation stack:

```bash
ros2 launch box_bot box_bot_nav_gps.launch.py
```

`ublox_rover.launch.py` remains a compatibility entry point to that stack with
RealSense disabled by default. Driver configuration is now selected through
`gnss_params`, defaulting to `zed_x20p_dual.yaml`. The old F9P parameter file,
serial-device and baud-rate arguments have been retired.

## Configuration and topics

`zed_x20p_dual.yaml` explicitly selects X20P and the two serial strings.
Only USB output settings and the base USB RTCM input are overridden in RAM.
The configured navigation rate (observed at approximately 20 Hz), receiver roles,
constellations and UART correction link are preserved. No flash saves or resets
are issued. NMEA and duplicated RTCM output are disabled on USB; UBX remains
active. This does not disable RTCM on the wired UART link.

NTRIP `/rtcm` feeds only the front/base. The rear's `RTCM_INPUT_ENABLED` is
false; it obtains moving-base corrections through the inter-receiver link.
The base's GGA position remains `/gps_moving_base/fix`. Correction-status output
is sampled one message in 20, rather than reporting every received correction:
the rover was producing approximately 120 RXM-COR status messages per second,
which caused navigation-message loss through the upstream USB receive path.

| Topic | Message / purpose |
| --- | --- |
| `/gps_rover/navpvt`, `/gps_moving_base/navpvt` | `ublox_ubx_msgs/UBXNavPVT` |
| `/gps_rover/hpposllh`, `/gps_moving_base/hpposllh` | `UBXNavHPPosLLH` |
| `/gps_rover/navcov`, `/gps_moving_base/navcov` | `UBXNavCov` |
| `/gps_rover/navstatus`, `/gps_moving_base/navstatus` | `UBXNavStatus` |
| `/gps_rover/navsat`, `/gps_moving_base/navsat` | `UBXNavSat` |
| `/gps_rover/rxmcor`, `/gps_moving_base/rxmcor` | `UBXRxmCor`, replaces RXM-RTCM status |
| `/gps_rover/secsig`, `/gps_moving_base/secsig` | `UBXSecSig`, replaces old MON-HW summary |
| `/gps/relposned` | rear receiver's `UBXNavRelPosNED` |
| `/gps_rover/fix`, `/gps_moving_base/fix` | reliable `sensor_msgs/NavSatFix` |
| `/gps_rover/fix_velocity`, `/gps_moving_base/fix_velocity` | ENU `TwistWithCovarianceStamped` |
| `/gps/heading`, `/gps/heading_corrected` | `sensor_msgs/Imu`, full navigation launch only |

`gnss_navsat.py` pairs PVT and high precision position by GPS time-of-week,
converts their units, rejects invalid fixes, and uses matching valid NED
covariance transformed into ENU when available. Otherwise it derives diagonal
covariance from the high precision accuracy fields. Covariance arriving after
that epoch's publication is not substituted from a different epoch.

`imu_yaw_offset.py` derives raw and corrected headings from the same RELPOSNED
message and rejects invalid headings/baselines. The existing pi-radian offset,
40 cm baseline gate, filters and dynamic covariance remain in the navigation
launch. This compensates for the baseline pointing from the front base to the
rear rover. The existing launch permits float/non-differential baselines when
valid; it does not impose a new fixed-RTK requirement.

Status and motion loggers consume the native messages. Motion CSV correction
fields now use `*_cor_*`, with explicit error status and message usage instead
of treating unknown correction status as a successful CRC check.

## Validation and remaining checks

The driver packages and BOXBOT built in the existing Humble image. Seven ROS
message tests cover precision units, invalid fixes, covariance rotation,
out-of-order epochs, bounded caches, heading direction and validity gates,
and the status/CSV consumers. Run them inside the sourced workspace:

```bash
python3 src/box_bot/test/test_gnss_migration.py
```

A live check ran both receivers as UID 1000 with supplementary group 46
(`plugdev`), using only the two GNSS USB devices. It verified serial/family
selection, native PVT/HPPOSLLH/COV/RELPOSNED output, ROS fixes and frame IDs,
and exactly one `/rtcm` subscriber, belonging to the front/base. Both acquired
3D fixes during that check. The final 30-second run received 580 base PVT and
532 rover PVT messages, plus 531 RELPOSNED and corrected-heading samples. The
last baseline was 0.4109 m with fixed carrier ambiguities. A preceding float
solution around 3.5 m was correctly rejected by the unchanged 40 cm geometry
gate; checksum-verified raw UBX later confirmed a fixed baseline around 39 cm. No NTRIP client or motor/navigation nodes were
started by this validation.

Live checks exposed upstream shutdown failures after SIGINT (`double free or
corruption` and a base-driver segmentation fault). Streaming and subsequent
restart checks passed. This remains a limitation of the unmodified pinned
driver. BOXBOT's Python nodes shut down cleanly. The CDC serial interfaces were
reattached after the checks, and the role aliases were verified again. Following
an upstream crash, restarting the USB driver remains possible; unplug/replug the
receiver if its serial interface is needed and has not returned. Hardware validation logs
are in `log/gnss_hardware/` (ignored build/runtime artifacts).

Confirm corrected heading against physical orientation and turn direction before
relying on the complete localization stack. The operator reports that the NTRIP
base is currently down for resurvey and will also be an all-band X20P. NTRIP
settings have been preserved; end-to-end absolute RTK validation is deferred
until that base returns. A fixed local moving baseline does not establish
centimeter-accurate absolute coordinates while the external base is offline.

## Preserved legacy work

The old KumarRobotics submodule was removed from the active dependency list.
Its exact checkout, including the four pre-existing modified files, is preserved
locally at `.local-backups/ublox-before-x20p`, with a binary patch alongside it.
That ignored directory has `COLCON_IGNORE` and is not part of a fresh checkout.
Existing old build/install products may remain locally, but the current launch
and package dependencies no longer select `ublox_gps` or `ublox_msgs`.
