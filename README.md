# UAV Non-linear MPC (ACADO) 实机轨迹跟踪控制系统

本项目是基于 **ACADO Toolkit (qpOASES)** 的四旋翼无人机非线性模型预测控制（NMPC）系统，支持机载定位（VIO / LIO / MAVROS）以及实机安全起飞接管、圆轨迹精准跟踪与飞行数据记录分析。

---

## 🚀 一、 实机飞行 4 步标准操作与启动指令汇总

每次实机实验按照以下顺序操作：

```mermaid
graph LR
    Step1["1. 启动控制器"] --> Step2["2. 手动起飞并切 OFFBOARD"]
    Step2 --> Step3["3. 启动圆轨迹节点"]
    Step3 --> Step4["4. 遥控切回降落并绘图"]
```

### 步骤 1：机载电脑启动 MPC 控制器（含自动数据记录）
```bash
roslaunch uav_mpc run_control.launch
```
> 控制器启动后处于安全监听状态（默认 `auto_arm_and_offboard: false`，不会自动解锁起飞）。

### 步骤 2：手动起飞与遥控接管
1. 飞手使用遥控器在**定点模式（POSCTL）**下手动起飞至预定高度（如 1.2m）悬停；
2. 拨动遥控器开关切入 **OFFBOARD** 模式；
3. 终端打印：`[MPC] Drone is in OFFBOARD & ARMED. Switched to AUTO_HOVER at current position.`，MPC 自动锁定当前位置平稳悬停。

### 步骤 3：启动圆轨迹跟踪（按需触发）
确认悬停平稳后，在终端执行以下指令开始走圆：
```bash
roslaunch uav_mpc publish_circle_traj.launch center_x:=0.0 center_y:=0.0 center_z:=1.2 radius:=1.0 linear_vel:=0.6 cycles:=3
```
| 参数名称 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `center_x`, `center_y`, `center_z` | `0.0, 0.0, 1.2` | 期望圆心世界坐标（米） |
| `radius` | `1.0` | 圆轨迹半径（米） |
| `linear_vel` | `0.6` | 飞行线速度（米/秒），初次试飞建议 0.4~0.6 |
| `cycles` | `3` | 飞行圈数（`-1` 为无限连续转圈，`3` 为跑满 3 圈后停在起点悬停） |

### 步骤 4：手动降落与数据绘制
1. 飞行完成后（或遇突发情况随时），飞手拨动遥控器切回 **Position（定点）** 或 **Land（降落）** 模式手动降落；
2. 在终端运行绘图脚本查看本次飞行的性能分析曲线：
   ```bash
   python3 scripts/plot_flight_data.py
   ```

---

## 📊 二、 数据记录系统位置与说明

### 1. 数据记录源码位置
- **记录节点源码**：`src/data_logger_node.cpp`
- **绘图脚本源码**：`scripts/plot_flight_data.py`
- **控制与自适应推力源码**：`src/ros_mission.cpp`、`include/uav_mpc/thrust_estimator.h`

### 2. 数据保存位置
- **默认存储路径**：`uav_mpc/data/`
- **文件命名格式**：`flight_log_YYYYMMDD_HHMMSS.csv`（每次启动自动按时间戳创建全新文件，退出自动封存）
- **记录的关键字段**：
  - 时间戳（`time_sec`）、飞行模式（`mode`: 0起飞/1悬停/2跟踪）
  - 实际位置与参考位置（`pos_x/y/z`, `ref_x/y/z`）
  - 3 轴位置误差与 3D 欧式总误差（`err_x/y/z`, `err_pos_norm`）
  - 3 轴线速度（`vel_x/y/z`）
  - MPC 期望推力加速度与三轴角速度（`ctrl_acc_z`, `ctrl_wx/wy/wz`）
  - 最终下发油门（`cmd_thrust`）、在线估计悬停油门（`estimated_hover_thrust`）、IMU 垂直加速度（`imu_acc_z`）

---

## 🛠️ 三、 常用编译与环境同步指令

### 机载电脑一键拉取最新代码并增量编译
```bash
~/catkin_ws/src/quadrotor_mpc/scripts/pull_and_build.sh
```

### PC 端推送最新修改至 GitHub
```bash
git push origin dev-circle-mpc
```

---

## 📝 四、 开发更新日志（Changelog）

> *说明：后续对代码的每一次重要修改，请在此处按时间倒序依次记录。*

### [2026-08-17] (实机代码里程碑 v1.0-real-flight)
- **实机版本归档与备份**：
  - 创建实机飞行专用永久备份分支 `real-flight-backup` 与版本标签 `v1.0-real-flight`。
  - 创建并切换至仿真开发分支 `dev-simulation`，用于后续 Gazebo/SITL 仿真开发。
- **新增数据记录节点与可视化工具**：
  - 编写 `src/data_logger_node.cpp`，高频采集位置、速度、跟踪误差、MPC 控制量与推力映射系数，自动保存到 `data/flight_log_*.csv`。
  - 编写 `scripts/plot_flight_data.py`，支持一键绘制 3D 航迹、误差随时间变化、角速度和自适应推力收敛曲线。
- **解耦任务执行流程**：
  - 新建 `launch/publish_circle_traj.launch`，将 MPC 悬停接管与轨迹生成分离，使绕圆启动时机完全可控。
- **修复物理边界约束动态绑定**：
  - 修改 `src/mpc_wrapper.cpp`，消除硬编码数值，将 YAML 中的 `T_max`, `T_min`, `wx_max`, `wy_max`, `wz_max` 动态传递至 ACADO QP 求解器。

### [2026-08-14]
- **新增圆轨迹生成节点**：编写 `src/circle_traj_node.cpp` 与 `launch/run_circle_traj.launch`，支持向前预测 20 步平滑动力学参考。
- **实机安全逻辑改造**：在 `src/ros_mission.cpp` 中新增 `auto_arm_and_offboard: false` 参数，废除收到里程计自动解锁起飞的危险逻辑，实现遥控器平滑切入 OFFBOARD 接管。
- **修复航向角（Yaw）翻折 Bug**：将 `getTrajRef()` 中的 `acos` 修复为 `atan2`，解决下半圆航向镜像翻折问题。
- **清理动捕依赖**：删除 `launch/vrpn_pose_to_odom.launch`，并在 `config/mpc_para.yaml` 中规范化机载里程计（VIO/LIO/MAVROS）接口。
- **初始化 Git 工作流**：添加 `.gitignore` 并编写机载端一键拉取编译脚本 `scripts/pull_and_build.sh`。
