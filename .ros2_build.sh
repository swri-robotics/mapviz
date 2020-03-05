#!/usr/bin/env bash

apt-get update
rosdep update
apt-get install -y build-essential python3-colcon-ros

cd /ws

rosdep install src --from-paths -i -y
colcon build
