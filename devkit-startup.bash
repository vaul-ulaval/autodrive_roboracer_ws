#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash

screen -wipe > /dev/null 2>&1
killall screen > /dev/null 2>&1

screen -S foxglove -dm bash -c 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
screen -S devkit -dm bash -c 'ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py'

echo "Devkit environment ready"
while true; do sleep 1; done