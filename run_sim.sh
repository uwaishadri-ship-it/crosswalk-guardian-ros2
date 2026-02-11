#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH="$PWD/models:${GAZEBO_MODEL_PATH}"

killall -9 gzserver gzclient || true
pkill -f cg_ || true

gzserver --verbose \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  "$PWD/worlds/crosswalk_final.world" &

sleep 2
gzclient &

echo ""
echo "Open 2 more terminals and run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  python3 $PWD/cg_nodes/cg_patrol_actuation.py"
echo ""
echo "  source /opt/ros/humble/setup.bash"
echo "  python3 $PWD/cg_nodes/cg_traffic_light_pub.py"
echo ""
