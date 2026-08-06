#!/usr/bin/env bash
set -e

source /opt/ros/foxy/setup.bash
source /home/jetson/workspaces/car_ws/install/setup.bash

exec ros2 launch motor_control motor_control.launch.py "$@"
