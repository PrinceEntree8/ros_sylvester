#!/bin/bash

# Sorgente https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# Installazione repository di ROS
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Installazione di ROS 2 (jazzy)
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop ros-dev-tools -y