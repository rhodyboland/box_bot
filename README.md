# box_bot
## Micro metal indoor ros2 robot on the orin nano using Isaac ROS 3.1 in a docker container

For a fresh install:
```
sudo systemctl daemon-reload && sudo systemctl restart docker
```
```
sudo apt-get install git-lfs
```
```
git lfs install --skip-repo
```
Create workspace and export location var
```
mkdir -p  ~/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
```
Clone Isaac ros common
```
cd ${ISAAC_ROS_WS}/src && \
   git clone -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
```
Clone this repo
```
cd ${ISAAC_ROS_WS}/src && \
   git clone -b micro-indoor-orin_nano --recurse-submodules git@github.com:rhodyboland/box_bot.git
```
Build (and run) docker container
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

# packages:

sllidar_ros2:
https://github.com/Slamtec/sllidar_ros2

dfrobot motor driver hat interface:
https://github.com/rhodyboland/dfrobot_dc_motor_hardware

# Git shortcuts
After checking out a different branch:
```
git submodule update --init --recursive
```

If git gets angry about a submodule after checking out:
```
git fetch --all
git reset --hard origin/micro-indoor
```
