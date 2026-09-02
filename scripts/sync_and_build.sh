#!/usr/bin/env bash
set -e

# Directory setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"

echo "=================================================="
echo ">> UAV NMPC Git Sync & Build"
echo ">> Workspace: ${WS_DIR}"
echo ">> Package:   ${PKG_DIR}"
echo "=================================================="

# 1. Git pull latest changes
cd "${PKG_DIR}"
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
echo ">> Pulling latest updates on branch: ${CURRENT_BRANCH}..."
git pull origin "${CURRENT_BRANCH}"

# 2. Build with colcon
cd "${WS_DIR}"
echo ">> Building uav_mpc package (Release)..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Source environment
source "${WS_DIR}/install/setup.bash"
echo "=================================================="
echo ">> Sync & Build finished successfully!"
echo ">> To run NMPC hover: ros2 launch uav_mpc nmpc.launch.py"
echo ">> To run circle:     ros2 run uav_mpc circle_traj_node"
echo ">> To record CSV log: ros2 run uav_mpc data_logger_node"
echo "=================================================="
