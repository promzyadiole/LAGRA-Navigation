#!/usr/bin/env bash
set -e

echo "=============================================="
echo " Setting up LAGRA-Navigation ROS 2 workspace "
echo "=============================================="

# Check ROS 2 installation
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo "ERROR: ROS 2 Humble not found."
  echo "Please install ROS 2 Humble before continuing."
  exit 1
fi

# Source ROS 2
source /opt/ros/humble/setup.bash

# Ensure rosdep is initialized
sudo rosdep init 2>/dev/null || true
rosdep update

# Install workspace dependencies
echo "Installing ROS dependencies via rosdep..."
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
echo "Building workspace with colcon..."
colcon build --symlink-install

# Source workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "=============================================="
echo " LAGRA-Navigation workspace setup complete "
echo "=============================================="
echo ""
echo "To use the workspace in a new terminal, run:"
echo "  source install/setup.bash"

