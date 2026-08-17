#!/bin/bash
# ==========================================================
# 自动识别 Catkin 工作空间并一键拉取最新代码与增量编译
# ==========================================================

# 1. 切换到包根目录并拉取最新分支
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PKG_DIR="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"
cd "$PKG_DIR"

echo -e "\033[32m[1/3] 拉取最新 dev-simulation 代码...\033[0m"
git pull origin dev-simulation || git pull origin dev-circle-mpc || git pull origin main

# 2. 定位到 catkin 工作空间根目录 (支持 ~/Desktop/catkin_ws 或 ~/catkin_ws)
WORKSPACE_DIR="$( cd "$PKG_DIR/../.." &> /dev/null && pwd )"
cd "$WORKSPACE_DIR"

echo -e "\033[32m[2/3] 在工作空间 $(pwd) 编译 uav_mpc...\033[0m"
source /opt/ros/noetic/setup.bash

if command -v catkin &> /dev/null; then
    catkin build uav_mpc -DCMAKE_BUILD_TYPE=Release
else
    catkin_make -DCMAKE_BUILD_TYPE=Release --pkg uav_mpc
fi

if [ $? -eq 0 ]; then
    echo -e "\033[32m[3/3] 编译成功！加载环境变量...\033[0m"
    source devel/setup.bash
    echo -e "\033[36m--------------------------------------------------\033[0m"
    echo -e "\033[36m仿真启动指令 (NMPC 实验组):\033[0m"
    echo -e "roslaunch uav_mpc simulation_mpc.launch"
    echo -e "\033[36m仿真启动指令 (PID 对照组):\033[0m"
    echo -e "roslaunch uav_mpc simulation_pid_baseline.launch"
    echo -e "\033[36m多维度性能评估与对比图表:\033[0m"
    echo -e "python3 $PKG_DIR/scripts/evaluate_nmpc_vs_pid.py"
    echo -e "\033[36m--------------------------------------------------\033[0m"
else
    echo -e "\033[31m[ERROR] 编译失败，请检查 CMake / C++ 语法错误！\033[0m"
    exit 1
fi
