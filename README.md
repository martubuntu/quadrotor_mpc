# Quadrotor NMPC 轨迹跟踪与飞行控制系统

本项目基于 **ACADO** 实时非线性模型预测控制（NMPC）与 **MAVROS**，实现多旋翼无人机（支持 1KG~5KG 各类级机型）的高精度轨迹跟踪（如圆轨迹、多项式轨迹）、自适应推力估计与实机安全飞行接管。

---

## 📌 快速启动与指令速查表

### 1. 编译工作空间
```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg uav_mpc
source devel/setup.bash
```

---

### 2. 标准实机 4 步飞行指令流（推荐）

#### 步骤 1 & 2：启动 MPC 控制器与数据记录
```bash
roslaunch uav_mpc run_control.launch
```
> **操作**：
> 1. 飞手使用遥控器手动起飞至空中（如 1.2m）保持定点悬停；
> 2. 拨动遥控器开关切入 **OFFBOARD** 模式；
> 3. 控制器自动锁定当前位置，进入 `AUTO_HOVER` 平稳接管悬停。

#### 步骤 3：按需触发圆轨迹跟踪
确认飞机悬停平稳后，在地面站/新终端执行：
```bash
# 常用参数：center_x, center_y, center_z(圆心坐标), radius(半径), linear_vel(速度), cycles(圈数, -1为无限转)
roslaunch uav_mpc publish_circle_traj.launch center_x:=0.0 center_y:=0.0 center_z:=1.2 radius:=1.0 linear_vel:=0.6 cycles:=3
```

#### 步骤 4：降落与数据绘制
- 飞行完毕或遇异常时，飞手随时拨动遥控器开关切回 **Position（定点）** 或 **Land（降落）** 手动降落。
- 飞行数据已自动保存至 `data/flight_log_YYYYMMDD_HHMMSS.csv`，运行下方脚本一键出图：
```bash
python3 src/quadrotor_mpc/scripts/plot_flight_data.py
```

---

### 3. 一体化启动指令（仿真/一键测试）
```bash
roslaunch uav_mpc run_circle_traj.launch radius:=1.0 linear_vel:=0.8 start_delay:=4.0 cycles:=3
```

---

### 4. 机载电脑代码同步指令
在机载电脑上拉取 PC 端更新并快速增量编译：
```bash
~/catkin_ws/src/quadrotor_mpc/scripts/pull_and_build.sh
```

---

## ⚙️ 核心参数说明 (`config/mpc_para.yaml`)

| 参数项 | 默认值 | 适用机型说明 |
| :--- | :--- | :--- |
| `hover_thrust` | `0.39` | **真实悬停油门 (0.0~1.0)**，请根据实机 QGC 测量值填写 |
| `auto_arm_and_offboard` | `false` | **实机安全开关**（`false`: 遥控器手动切入接管；`true`: 仿真自动解锁） |
| `odomTopicName` | `/mavros/local_position/odom` | 机载里程计话题（支持 VIO / LIO / MAVROS） |
| `T_max` / `T_min` | `25.0` / `2.0` | 最大/最小比推力加速度 ($m/s^2$) |
| `wx_max` / `wy_max` | `2.0` | 滚转/俯仰角速度上限 (rad/s，5KG 大机身建议 `1.0~1.5`) |
| `wz_max` | `2.0` | 偏航角速度上限 (rad/s) |

---

## 📝 版本迭代日志 (Changelog)

> 每次代码更新按时间倒序记录在此处，便于追踪实机更迭。

### [2026-08-17]
- **[新增] 飞行数据记录节点 (`data_logger_node.cpp`)**：
  - 高频记录 3 轴位置/误差、速度、MPC 期望加速度、三轴角速度、下发油门及自适应估计悬停油门。
  - 自动在 `data/` 目录下生成带时间戳的 CSV 表格，并发布实时调试话题 `/mpc_debug/pos_error_3d`。
- **[新增] 离线数据可视化脚本 (`scripts/plot_flight_data.py`)**：
  - 自动抓取最新日志绘制 3D 轨迹对比、位置误差、控制量及推力收敛 4 联曲线。
- **[重构] 解耦任务启动流程 (`publish_circle_traj.launch`)**：
  - 实现 MPC 悬停接管与轨迹下发的完全解耦，支持在飞机悬停稳定后随时人工按需触发轨迹。
- **[修复] MPC 物理约束参数动态绑定**：
  - 修复 `mpc_wrapper.cpp` 中硬编码边界数值的问题，完全打通 YAML 与 ACADO QP 求解器的物理限幅接口。
- **[优化] Git 分支管理**：
  - 建立 `dev-circle-mpc` 开发分支。

### [2026-08-14]
- **[新增] 圆轨迹生成模块 (`circle_traj_node.cpp`)**：
  - 支持参数化配置圆心、半径、速度、高度与圈数，向前预测 20 步平滑动力学参考（$p, v, a$）。
- **[安全] 实机安全起飞接管改造 (`ros_mission.cpp`)**：
  - 移除原代码收到里程计自动解锁起飞的危险逻辑，新增 `auto_arm_and_offboard: false` 安全机制。
  - 支持飞手在定点模式起飞后，由遥控器切入 OFFBOARD 平滑锁定当前点悬停。
- **[修复] 航向角解算 Bug (`ros_mission.cpp`)**：
  - 将原 `acos` 航向计算修改为 `atan2`，彻底解决下半圆航向镜像翻转的问题。
- **[清理] 动捕依赖移除**：
  - 删除 `vrpn_pose_to_odom.launch`，规范化机载里程计接口。
- **[工具] 机载电脑极速构建脚本 (`scripts/pull_and_build.sh`)**：
  - 支持一键 Git 拉取与 1~3 秒增量编译。
