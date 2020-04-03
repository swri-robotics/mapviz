#!/usr/bin/env bash

# Enable the ros2-testing repository
sed 's#ros2#ros2-testing#' /etc/apt/sources.list.d/ros2-latest.list | tee >> /etc/apt/sources.list.d/ros2-latest.list

# Install colcon, we need it and it's not installed by default
apt-get update
rosdep update
apt-get install -y build-essential python3-colcon-ros

cd /ws

# Use rosdep to install the rest of the package's dependencies, then build it
rosdep install src --from-paths -i -y
colcon build
