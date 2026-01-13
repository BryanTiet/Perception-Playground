#!/usr/bin/env bash
set -euo pipefail

ros_distro="${1:-${ROS_DISTRO:-jazzy}}"

if ! command -v rosdep >/dev/null 2>&1; then
  echo "rosdep not found. Install ROS 2 + rosdep first." >&2
  exit 1
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  if command -v sudo >/dev/null 2>&1; then
    sudo rosdep init || true
  else
    rosdep init || true
  fi
fi

rosdep update
rosdep install --from-paths src -i -y --rosdistro "${ros_distro}" --skip-keys ament_python
