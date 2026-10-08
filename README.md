# Ackermann Autonomous Car

Repository for an **Ackermann-steering autonomous car** built around the **NVIDIA Jetson Nano**.

The project currently supports:

- Joystick-based manual control
- LiDAR-based room mapping
- SLAM using `slam_toolbox`
- Experimental autonomous navigation using Nav2

## Components Used

- [Hiwonder Ackermann Steering Chassis](https://www.hiwonder.com/products/ackermann-steering-chassis?variant=40382428348503)
- NVIDIA Jetson Nano
- 3S LiPo Battery
- Step-down Voltage Regulator
- RPLIDAR A1 M8
- Generic USB Joystick

## Stack Used

- [QEngineering Jetson Nano Ubuntu 20 Image with JetPack 4.6.1](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image)
- ROS 2 Foxy
- [SLLIDAR ROS 2 Library](https://github.com/Slamtec/sllidar_ros2)
- Jetson.GPIO for controlling servos
- `slam_toolbox` for maps
- Nav2 for autonomous navigation *(experimental)*

## Current Features

### Manual Control

The car can be controlled using a joystick connected to the Jetson Nano. The joystick commands are translated into steering and drive commands for the Ackermann chassis.

### Mapping

An RPLIDAR A1 M8 is used for 2D LiDAR scanning. The scan data is processed using `slam_toolbox` to generate a map of the environment.

### Autonomous Navigation

Nav2 has been integrated for autonomous navigation using the generated map and LiDAR data. This is still experimental.

## Problems Faced with Nav2

Nav2 is a CPU-heavy process, and the Jetson Nano can be quite sensitive to power limitations when running heavy workloads.

The voltage regulator I initially used had a **peak current rating of 2A**, while the Jetson Nano can draw **up to around 4A under heavy workloads**.

Because of this, I used to get **CPU throttling** errors. Due to the throttling, the Nav2 Planner Server would often get delayed or fail to start properly.

### ROS 2 Foxy / Nav2

I also came across several reports of problems with Nav2 on ROS 2 Foxy. Because of this, I'll need to test the system further with a different power regulator and a newer ROS 2 setup.

Either way, if anyone is planning on implementing Nav2 on a Jetson Nano, I would recommend trying **ROS 2 Humble** instead of Foxy. A containerized setup is also worth considering.

For this, you can look into [jetson-containers](https://github.com/dusty-nv/jetson-containers).

## QEngineering Image and `jetson-io.py`

The QEngineering image is useful because it gives us access to **Ubuntu 20.04** on the Jetson Nano, which makes it possible to use ROS 2 Foxy.

However, I ran into an issue where enabling **PWM/I²C through the `jetson-io.py` interface caused it to crash / stop working properly**.

If the same thing happens to you, try following the guide in [`fix_dtb.md`](./fix_dtb.md).

## Future Work

- Improve Nav2 stability and performance
- Upgrade the Jetson Nano power supply/regulator
- Evaluate ROS 2 Humble
- Improve autonomous navigation
- Add better obstacle avoidance