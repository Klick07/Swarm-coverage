#!/usr/bin/env bash
# Phase 1: install ROS 2 Jazzy + Swarm-SLAM on Ubuntu 24.04 (x86_64).
# Lean variant of Swarm-SLAM/build-everything.sh: ros-base instead of desktop,
# prebuilt ros-jazzy-gtsam (4.2.0) instead of a source build, CPU-only PyTorch.
# Usage: bash ec2_setup.sh <path-to-swarm.repos>
set -euo pipefail

REPOS_FILE=${1:-$HOME/swarm.repos}
WS=$HOME/swarm_ws
export DEBIAN_FRONTEND=noninteractive

echo "== swap (4 GB: building and running 3 robots on 8 GB RAM)"
if [ ! -f /swapfile ]; then
  sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile && sudo mkswap -q /swapfile && sudo swapon /swapfile
  echo "/swapfile none swap sw 0 0" | sudo tee -a /etc/fstab >/dev/null
fi

echo "== ROS 2 apt source"
sudo apt-get update -qq
sudo apt-get install -y -qq software-properties-common curl >/dev/null
sudo add-apt-repository -y universe >/dev/null
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

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
# Versions verified working on 2026-10-22. coverage>=7.6: numba 0.68 needs a
# newer coverage than Ubuntu's python3-coverage 7.4.
PIP="pip install --user --break-system-packages -q"
$PIP --index-url https://download.pytorch.org/whl/cpu torch==2.14.1 torchvision==0.29.1
$PIP "numpy<2" networkx==3.6.1 numba==0.68.0 open3d==0.20.0 Pillow scikit_learn==1.9.1 \
  scipy==1.17.1 sortedcontainers==2.4.0 distinctipy==1.3.4 numpy-quaternion==2024.0.13 \
  coverage==7.16.2

echo "== colcon build (low parallelism for 2 vCPU / 8 GB)"
export MAKEFLAGS=-j2
colcon build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "== CosPlace ResNet-18 / dim 64 (cslam loads it from its installed share dir)"
MODELS=$WS/install/cslam/share/cslam/models
mkdir -p "$MODELS"
[ -f "$MODELS/resnet18_64.pth" ] || curl -sSfL -o "$MODELS/resnet18_64.pth" \
  https://github.com/gmberton/CosPlace/releases/download/v1.0/ResNet18_64_cosplace.pth
echo "5535824d5314c9fbb75bc4625d9398bcc51ab8b1f448daf0e0abd36c1fffbc48  $MODELS/resnet18_64.pth" | sha256sum -c

grep -q swarm_ws/install ~/.bashrc || cat >> ~/.bashrc <<'EOF'
source /opt/ros/jazzy/setup.bash
source $HOME/swarm_ws/install/setup.bash
EOF
echo SETUP_DONE
