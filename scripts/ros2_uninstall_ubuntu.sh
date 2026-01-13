#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "--yes" ]]; then
  cat <<'EOF'
This will remove ROS 2 and related packages installed via the README/scripts:
- All ros-<distro>-* packages
- ROS 2 apt source + keyring
- rosdep sources list + cache

Re-run with:
  ./scripts/ros2_uninstall_ubuntu.sh --yes
EOF
  exit 1
fi

sudo apt remove -y 'ros-*' || true
sudo apt autoremove -y || true

sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
sudo rm -rf /root/.ros/rosdep/sources.cache

echo "ROS 2 uninstall complete."
