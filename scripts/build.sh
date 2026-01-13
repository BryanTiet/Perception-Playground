#!/usr/bin/env bash
set -euo pipefail

ros_distro="${1:-${ROS_DISTRO:-}}"
if [[ -n "${ros_distro}" && -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  # setup.bash isn't nounset-safe; disable it while sourcing.
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ros_distro}/setup.bash"
  set -u
fi

colcon build --symlink-install
