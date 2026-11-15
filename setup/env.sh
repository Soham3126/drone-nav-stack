# env.sh - the two shell shortcuts every script in this repository uses (guidebook 3.2, adapted to Humble).
# Install once:   echo "source ~/drone_ws/src/setup/env.sh" >> ~/.bashrc     then open a new terminal.
#
#   ros_env   ROS 2 / Gazebo terminals: ROS Humble, this workspace, the ArduPilot Gazebo plugin and models
#   ap_env    ArduPilot SITL / MAVProxy terminals
#
# ros_env is the one used on the development laptop (Ubuntu 22.04, RTX GPU). Differences from the book (see
# HUMBLE_NOTES.md): Humble instead of Jazzy; system Python instead of ~/venv-ardupilot; NVIDIA PRIME variables so
# Gazebo renders on the laptop's NVIDIA GPU; ROS_AUTOMATIC_DISCOVERY_RANGE (Jazzy-only) replaced by the optional
# ROS_LOCALHOST_ONLY line.

ap_env() {
  [ -f ~/venv-ardupilot/bin/activate ] && source ~/venv-ardupilot/bin/activate
  export PATH=$HOME/ardupilot/Tools/autotest:$PATH
}

ros_env() {
  [ -n "${VIRTUAL_ENV:-}" ] && deactivate 2>/dev/null || true
  source /opt/ros/humble/setup.bash
  [ -f ~/drone_ws/install/setup.bash ] && source ~/drone_ws/install/setup.bash
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
  export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$HOME/drone_ws/src/drone_sim/models:$HOME/drone_ws/src/drone_sim/worlds:${GZ_SIM_RESOURCE_PATH:-}
  # laptop with hybrid graphics: render Gazebo on the NVIDIA GPU (real-time factor 0.93 -> 0.98).
  # Remove these three lines on a computer with only one GPU.
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  # Optional (guidebook 3.2): keep ROS traffic on this computer, so team members on the same network
  # do not see each other's simulations.
  # export ROS_LOCALHOST_ONLY=1
  # export ROS_DOMAIN_ID=${DRONE_DOMAIN:-0}
}
