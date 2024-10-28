# box_bot
Micro metal indoor ros2 robot
```
git clone -b micro-indoor --recurse-submodules git@github.com:rhodyboland/box_bot.git
```
After checking out a different branch:
```
git submodule update --init --recursive
```

If git gets angry about a submodule after checking out:
```
git fetch --all
git reset --hard origin/micro-indoor
```
# packages:

sllidar_ros2:
https://github.com/Slamtec/sllidar_ros2

dfrobot motor driver hat interface:
https://github.com/rhodyboland/dfrobot_dc_motor_hardware
