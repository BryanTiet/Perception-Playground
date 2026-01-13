#!/usr/bin/env bash
set -euo pipefail

ros_distro="${1:-${ROS_DISTRO:-jazzy}}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "Using ROS distro: ${ros_distro}"

bash "${script_dir}/ros2_install_ubuntu.sh" "${ros_distro}"
bash "${script_dir}/nav2_install_ubuntu.sh" "${ros_distro}"
bash "${script_dir}/rosdep_install.sh" "${ros_distro}"
bash "${script_dir}/build.sh" "${ros_distro}"

echo "Setup complete. Run:"
echo "  source /opt/ros/${ros_distro}/setup.bash"
echo "  source install/setup.bash"
