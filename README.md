# Perception-Playground

This repo is set up as a ROS 2 `colcon` workspace (packages live in `src/`).

## Quick start (recommended: Docker devcontainer)
If you’re on macOS/Windows (or just want the fastest setup), use the included devcontainer:

1. Install Docker Desktop + VS Code + the “Dev Containers” extension.
2. Open this repo in VS Code → “Dev Containers: Reopen in Container”.
3. In the container terminal, launch Turtlebot3 + Nav2:

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

## Nav2 installation (from the official guide)
The devcontainer image already includes Nav2 packages, but on a native install follow the guide:
https://docs.nav2.org/getting_started/index.html#installation

1. Install ROS 2 binary packages for your distro:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html
2. Install Nav2 packages:

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
sudo apt install "ros-$ROS_DISTRO-navigation2"
sudo apt install "ros-$ROS_DISTRO-nav2-bringup"
```

3. Install the Turtlebot demo for Gazebo:

For Jazzy and newer (Gazebo modern):

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
sudo apt install ros-$ROS_DISTRO-nav2-minimal-tb*
```

For Iron and older (Gazebo Classic):

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
sudo apt install "ros-$ROS_DISTRO-turtlebot3-gazebo"
```

## Running the Turtlebot3 Nav2 example
From a GUI-capable terminal:

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/$ROS_DISTRO/setup.bash
# Iron and older only with Gazebo Classic:
# export TURTLEBOT3_MODEL=waffle
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

If Nav2 does not autostart, click the “Startup” button in RViz.

## macOS GUI setup (XQuartz)
If you're using the devcontainer on macOS, you need X11 forwarding for Gazebo/RViz:

1. Install XQuartz: https://www.xquartz.org/
2. Open XQuartz -> Settings -> Security -> enable "Allow connections from network clients".
3. On macOS (outside the container), allow local X11 connections:

```bash
xhost + 127.0.0.1
```

4. In the devcontainer terminal, set DISPLAY:

```bash
export DISPLAY=host.docker.internal:0
```

5. Launch Nav2 as usual.

## Setup scripts (Ubuntu)
All install steps are scripted for quick onboarding:

```bash
./scripts/setup_all.sh jazzy
```

Individual scripts:
- `scripts/ros2_install_ubuntu.sh` (ROS 2 base install)
- `scripts/nav2_install_ubuntu.sh` (Nav2 + Turtlebot packages)
- `scripts/rosdep_install.sh` (workspace deps)
- `scripts/build.sh` (colcon build)

## Master setup for Turtlebot3 Nav2 (recommended)
Use the master script to install everything needed for the Turtlebot3 Nav2
example described in the Nav2 guide:
https://docs.nav2.org/getting_started/index.html#navigating

```bash
./scripts/setup_all.sh jazzy
```

## Step-by-step: what each part does
1. **Install ROS 2 base** (`scripts/ros2_install_ubuntu.sh`):
   - Adds the ROS 2 apt repo + key.
   - Installs the ROS 2 desktop metapackage for your distro.
2. **Install Nav2 + Turtlebot3** (`scripts/nav2_install_ubuntu.sh`):
   - Installs `navigation2` and `nav2_bringup`.
   - Installs the Turtlebot3 Gazebo packages (modern Gazebo for Jazzy+).
3. **Install workspace dependencies** (`scripts/rosdep_install.sh`):
   - Resolves and installs any dependencies from `src/`.
4. **Build the workspace** (`scripts/build.sh`):
   - Runs `colcon build` so local packages are available.
5. **Launch the example**:
   - Source ROS 2: `source /opt/ros/$ROS_DISTRO/setup.bash`
   - Launch Turtlebot3 + Nav2:
     `ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False`
6. **Navigate in RViz** (from the Nav2 guide):
   - Click “Startup” if Nav2 didn’t autostart.
   - Use “2D Pose Estimate” to set the initial pose.
   - Use “Navigation2 Goal” to send a goal.

After you’re in the devcontainer, verify Nav2 is available:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 pkg list | grep -E '^nav2_|^navigation2$'
```

## Native Ubuntu 24.04 + ROS 2 Jazzy
1. Install ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html
2. Install build tooling + rosdep:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

3. Install package deps + build (only needed if you add local packages):

```bash
source /opt/ros/jazzy/setup.bash
./scripts/rosdep_install.sh jazzy
./scripts/build.sh
source install/setup.bash
```

## Repo layout
- `src/perception_playground/`: minimal ROS 2 package placeholder
- `scripts/`: helper scripts for rosdep + colcon
- `.devcontainer/`: containerized dev environment (ROS 2 Jazzy + Nav2)
