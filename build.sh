#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
docker compose -f docker-compose.yaml build
