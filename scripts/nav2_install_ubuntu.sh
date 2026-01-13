#!/usr/bin/env bash
set -euo pipefail

ros_distro="${1:-jazzy}"

sudo apt update
sudo apt install -y "ros-${ros_distro}-navigation2"
sudo apt install -y "ros-${ros_distro}-nav2-bringup"

case "${ros_distro}" in
  jazzy|kilted|rolling)
    sudo apt install -y "ros-${ros_distro}-nav2-minimal-tb*"
    ;;
  *)
    sudo apt install -y "ros-${ros_distro}-turtlebot3-gazebo"
    ;;
esac

echo "Nav2 + Turtlebot packages installed for ${ros_distro}."
