#!/usr/bin/env bash
# Phase 1: install ROS 2 Jazzy + Swarm-SLAM on Ubuntu 24.04 (x86_64).
# Lean variant of Swarm-SLAM/build-everything.sh: ros-base instead of desktop,
# prebuilt ros-jazzy-gtsam (4.2.0) instead of a source build, CPU-only PyTorch.
# Usage: bash ec2_setup.sh <path-to-swarm.repos>
set -euo pipefail

REPOS_FILE=${1:-$HOME/swarm.repos}
WS=$HOME/swarm_ws
export DEBIAN_FRONTEND=noninteractive

echo "== apt packages"
sudo apt-get update -qq
sudo apt-get install -y -qq \
  build-essential git python3-pip python3-venv \
  python3-rosdep python3-colcon-common-extensions python3-vcstool \
  ros-jazzy-ros-base ros-jazzy-rtabmap-ros ros-jazzy-gtsam ros-jazzy-cv-bridge \
  ros-jazzy-perception-pcl ros-jazzy-pcl-ros ros-jazzy-nav2-msgs ros-jazzy-nav2-common \
  ros-jazzy-image-transport-plugins >/dev/null

echo "== workspace"
mkdir -p "$WS/src"
cd "$WS"
vcs import src < "$REPOS_FILE"

echo "== rosdep"
set +u; source /opt/ros/jazzy/setup.bash; set -u
sudo rosdep init 2>/dev/null || true
rosdep update -q
rosdep install --from-paths src -y --ignore-src --rosdistro jazzy -q

echo "== python deps for cslam (system python, numpy<2 to keep rclpy/cv_bridge ABI)"
PIP="pip install --user --break-system-packages -q"
$PIP --index-url https://download.pytorch.org/whl/cpu torch torchvision
$PIP "numpy<2" networkx numba open3d Pillow scikit_learn scipy sortedcontainers distinctipy numpy-quaternion

echo "== colcon build (low parallelism for 2 vCPU / 8 GB)"
export MAKEFLAGS=-j2
colcon build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release

grep -q swarm_ws/install ~/.bashrc || cat >> ~/.bashrc <<'EOF'
source /opt/ros/jazzy/setup.bash
source $HOME/swarm_ws/install/setup.bash
EOF
echo SETUP_DONE
