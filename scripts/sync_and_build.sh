#!/usr/bin/env bash
# ==========================================================
# 一键拉取最新代码并编译 (Jetson Nano / 机载端使用)
# ==========================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"

echo "=================================================="
echo ">> [1/3] 从 GitHub 拉取最新代码..."
echo "=================================================="
cd "${PKG_DIR}"
BRANCH=$(git rev-parse --abbrev-ref HEAD)
git pull origin "${BRANCH}"

echo "=================================================="
echo ">> [2/3] 编译 uav_mpc 功能包 (Release)..."
echo "=================================================="
cd "${WS_DIR}"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "=================================================="
echo ">> [3/3] 刷新 ROS 2 环境变量..."
echo "=================================================="
source "${WS_DIR}/install/setup.bash"

echo ">> [完成] 编译成功！"
echo ">> 启动实机 NMPC: ros2 launch uav_mpc nmpc.launch.py"
echo ">> 启动绕圆轨迹:   ros2 run uav_mpc circle_traj_node"
echo ">> 启动数据记录:   ros2 run uav_mpc data_logger_node"
echo "=================================================="
