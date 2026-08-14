#!/bin/bash
# ==========================================================
# 机载电脑端：一键拉取最新代码并增量编译脚本
# ==========================================================

# 1. 切换到包目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR/.."

echo -e "\033[32m[1/3] 拉取最新代码...\033[0m"
git pull origin main || git pull origin master

# 2. 定位到 catkin 工作空间根目录并增量编译
# 假设本仓库在 ~/catkin_ws/src/ 路径下
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$WORKSPACE_DIR"

echo -e "\033[32m[2/3] 增量编译 uav_mpc...\033[0m"
if command -v catkin &> /dev/null; then
    # 如果安装了 catkin-tools (推荐)
    catkin build uav_mpc -DCMAKE_BUILD_TYPE=Release
else
    # 默认 catkin_make (只编译目标包)
    catkin_make -DCMAKE_BUILD_TYPE=Release --pkg uav_mpc
fi

if [ $? -eq 0 ]; then
    echo -e "\033[32m[3/3] 编译成功！加载环境变量...\033[0m"
    source devel/setup.bash
    echo -e "\033[36m可以启动运行:\033[0m"
    echo -e "roslaunch uav_mpc run_circle_traj.launch"
else
    echo -e "\033[31m[ERROR] 编译失败，请检查代码错误！\033[0m"
    exit 1
fi
