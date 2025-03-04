# box_bot
## Micro metal indoor ros2 robot on the orin nano using Isaac ROS 3.1 in a docker container

### For a fresh install:
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

To use dockerfile included:

Create a .isaac_ros_common-config file in the home directory and place the below inside. This points to the correct location of dockerfile if above commands used.

```
CONFIG_IMAGE_KEY="ros2_humble.box_bot"
CONFIG_DOCKER_SEARCH_DIRS=("/home/rhody-jetson/workspaces/isaac_ros-dev/src/box_bot")
CONFIG_CONTAINER_NAME_SUFFIX="boxbot"
BASE_DOCKER_REGISTRY_NAMES=("nvcr.io/isaac/ros")
```


Build (and run) docker container. **The first time will take a while, like hours.** This command is what is used to launch the container from now on. Add ```-b/--skip_image_build``` for offline running.
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```
#

If you get this error:

```
docker: Error response from daemon: failed to create task for container: failed to create shim task: OCI runtime create failed: could not apply required modification to OCI specification: error modifying OCI spec: failed to inject CDI devices: unresolvable CDI devices nvidia.com/gpu=all: unknown
```

Try running this:

```
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml
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
