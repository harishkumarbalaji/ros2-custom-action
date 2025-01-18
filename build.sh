#!/bin/bash

set -e

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
docker compose -f docker-compose.yaml build
