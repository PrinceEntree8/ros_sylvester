#!/bin/bash

# Cloniamo il repository di micro-ROS
git clone --depth 1 -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep install --from-paths src -y --ignore-src # installiamo le dipendenze

colcon build --symlink-install # ricostruiamo il workspace
source install/setup.bash # ricarichiamo l'ambiente

ros2 run micro_ros_setup create_agent_ws.sh # creiamo il ackage per l'agent (necessario se vogliamo modificare l'agent)
ros2 run micro_ros_setup build_agent.sh # costruiamo l'agent
source install/setup.bash # ricarichiamo l'ambiente

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 # creiamo il package per il firmware