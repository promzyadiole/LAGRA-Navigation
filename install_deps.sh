#!/usr/bin/env bash
set -e

echo "=============================================="
echo " Installing dependencies for LAGRA-Navigation "
echo "=============================================="

# Update system
sudo apt update && sudo apt upgrade -y

# Basic tools
sudo apt install -y \
  curl \
  git \
  wget \
  build-essential \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# Initialize rosdep (safe if already initialized)
sudo rosdep init 2>/dev/null || true
rosdep update

# ROS 2 Humble packages
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3* \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-rviz2

# Python dependencies for LLM / VLM
pip3 install --upgrade pip
pip3 install \
  numpy \
  scipy \
  opencv-python \
  matplotlib \
  pyyaml \
  requests \
  openai \
  gradio \
  python-dotenv

echo "=============================================="
echo " Dependencies installed successfully "
echo "=============================================="
echo ""
echo "Next steps:"
echo "  source /opt/ros/humble/setup.bash"
echo "  cd <your_workspace>"
echo "  colcon build"

