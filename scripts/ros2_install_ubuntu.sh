#!/usr/bin/env bash
set -euo pipefail

ros_distro="${1:-jazzy}"

sudo rm -f /etc/apt/sources.list.d/ros2-inline-key.list
sudo grep -l "BEGIN PGP PUBLIC KEY BLOCK" /etc/apt/sources.list /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources 2>/dev/null \
  | xargs -r sudo rm -f

sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y "ros-${ros_distro}-desktop"

echo "ROS 2 ${ros_distro} installed."
echo "Run: source /opt/ros/${ros_distro}/setup.bash"
