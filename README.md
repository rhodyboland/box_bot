# box_bot
### A ROS2 Humble Learning and Development Platform

This repository serves as a flexible development and experimentation platform for robotics projects using ROS2. It includes configurations for both indoor and outdoor navigation on a Raspberry Pi platform or an NVIDIA Orin Nano.

---

## 🚀 Quick Start

To clone the repository along with its submodules, run:
```bash
git clone --recurse-submodules git@github.com:rhodyboland/box_bot.git
```

---

## 📂 Branches and Configurations

### **1. [Orin Nano Branch](https://github.com/rhodyboland/box_bot/tree/orin_nano)**
This branch is designed for high-performance outdoor navigation using the NVIDIA Orin Nano, leveraging the power of Docker and the Isaac ROS framework.

- **Platform**: NVIDIA Orin Nano (Docker compatible)
- **Motors & Control**:
  - Hoverboard motors controlled using custom [firmware](https://github.com/hoverboard-robotics/hoverboard-firmware-hack-FOC) for odometry.
  - ROS2 communication and control via the [hoverboard_ros2_control](https://github.com/rhodyboland/hoverboard_ros2_control) package (originally developed [here](https://github.com/DataBot-Labs/hoverboard_ros2_control)).
- **Sensors**:
  - **IMU**: ICM20948 sensor with a custom [node](https://github.com/rhodyboland/icm_20948) for accurate heading and orientation.
  - **GPS**: EMLID Reach RTK GPS for high-precision localisation, using an NMEA serial parser.
- **Software Stack**:
  - **EKF** (Extended Kalman Filter) for sensor fusion using `robot_localization`.
  - **SLAM** with `slam_toolbox` for real-time mapping.
  - **Navigation** using `Nav2` for autonomous path planning.

---

### **2. [Micro Indoor Branch](https://github.com/rhodyboland/box_bot/tree/micro-indoor)** (RPI) and [Micro Indoor Orin Nano Branch](https://github.com/rhodyboland/box_bot/tree/micro-indoor-orin_nano) (Orin Nano)
This branch is optimised for indoor navigation, making it suitable for lightweight and compact robot designs.

- **Platform**: Raspberry Pi 4 (ROS2 native) and Orin Nano (Docker)
- **Motors & Control**:
  - **Motor Controller**: DFRobot DC motor driver HAT with encoder inputs.
    - Custom [firmware](https://gitlab.telecom-paris.fr/software/dc-motor-driver-hat).
    - Custom [node](https://github.com/rhodyboland/dfrobot_dc_motor_hardware/tree/custom-firmware) for precise motor control.
  - **Motors**: DFRobot 210:1 N20 encoder motors.
- **Sensors**:
  - **IMU**: ICM20948 sensor using the same [node](https://github.com/rhodyboland/icm_20948) for consistent sensor data.
- **Software Stack**:
  - **EKF** using `robot_localization` for accurate state estimation.
  - **SLAM** with `slam_toolbox` for mapping indoor environments.
  - **Navigation** with `Nav2` for robust path following.

---

## 🔧 Compatibility Notes

- The project is primarily developed for ROS2 Humble but remains **mostly compatible with ROS2 Jazzy**. Some parameter adjustments, such as changes in Nav2 plugin labels, may be required for newer versions.
