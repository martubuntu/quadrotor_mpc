#!/usr/bin/env bash
# ==========================================================
# 一键拉取最新代码并编译 (自动适配 ROS 2 Humble / ROS 1 Noetic)
# ==========================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"

echo ">> [1/3] 拉取最新代码..."
cd "${PKG_DIR}"
BRANCH=$(git rev-parse --abbrev-ref HEAD)
git pull origin "${BRANCH}"

echo ">> [2/3] 编译功能包..."
cd "${WS_DIR}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
    source "${WS_DIR}/install/setup.bash"
    echo ">> [3/3] ROS 2 编译完成！"
    echo ">> 启动: ros2 launch uav_mpc nmpc.launch.py"
elif [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    catkin_make -DCMAKE_BUILD_TYPE=Release --pkg uav_mpc
    source "${WS_DIR}/devel/setup.bash"
    echo ">> [3/3] ROS 1 编译完成！"
    echo ">> 启动: roslaunch uav_mpc simulation_mpc.launch"
fi
