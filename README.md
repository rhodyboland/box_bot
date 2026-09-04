# box_bot
Hoverboard powered ros2 robot
```
git clone --recurse-submodules git@github.com:rhodyboland/box_bot.git -b orin_nano
```
# packages:

hoverboard driver:
https://github.com/rhodyboland/hoverboard_ros2_control

sllidar_ros2:
https://github.com/Slamtec/sllidar_ros2

realsense2_camera:
https://github.com/NVIDIA-ISAAC-ROS/realsense-ros/tree/release/4.51.1-isaac

The ROS wrapper is kept as a submodule. Dockerfile.box_bot provides the
pinned librealsense SDK used by this wrapper.

gps:
https://github.com/ros-drivers/nmea_navsat_driver/tree/ros2

mag:
https://github.com/rhodyboland/py_qmc6310_driver
