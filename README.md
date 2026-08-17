# Quadrotor NMPC 实机轨迹跟踪控制系统 (uav_mpc)

本项目是一个基于 **ACADO Toolkit (qpOASES)** 实时非线性模型预测控制（NMPC）的四旋翼无人机轨迹跟踪与安全接管控制系统。支持机载里程计（VIO/LIO/MAVROS）、遥控器安全接管、自适应推力估计、圆轨迹跟踪及飞行数据记录分析。

---

## 📑 目录
1. [核心启动指令速查表](#一-核心启动指令速查表)
2. [数据记录机制说明（何时开始记录）](#二-数据记录机制说明)
3. [实机 4 步飞行操作标准流程 (SOP)](#三-实机-4-步飞行操作标准流程-sop)
4. [核心参数配置手册](#四-核心参数配置手册)
5. [代码更新日志 (Changelog)](#五-代码更新日志-changelog)

---

## 一、 核心启动指令速查表

| 操作需求 | 终端启动指令 | 适用场景 |
| :--- | :--- | :--- |
| **步骤 1：启动 MPC 控制器 & 数据记录** | `roslaunch uav_mpc run_control.launch` | 手动起飞后，准备切 OFFBOARD 悬停接管 |
| **步骤 2：启动圆轨迹跟踪** | `roslaunch uav_mpc publish_circle_traj.launch radius:=1.0 linear_vel:=0.6 cycles:=3` | 悬停稳定后，按需触发圆轨迹跟踪 |
| **一键启动（MPC + 延时自动走圆）** | `roslaunch uav_mpc run_circle_traj.launch start_delay:=5.0 radius:=1.0` | 仿真或单人快速实验测试 |
| **离线数据可视化绘图** | `python3 scripts/plot_flight_data.py` | 飞行结束后，一键绘制 4 联分析对比图 |
| **机载电脑一键拉取与增量编译** | `~/catkin_ws/src/quadrotor_mpc/scripts/pull_and_build.sh` | PC 推送代码后，机载电脑秒级编译 |

---

## 二、 数据记录机制说明

### 1. 数据记录是从什么时候开始的？
- **文件创建时机**：当在终端运行 `roslaunch uav_mpc run_control.launch`（或 `run_circle_traj.launch`）时，`data_logger_node` 会在 `uav_mpc/data/` 目录下创建一个以当前系统时间命名的 CSV 文件（例如 `flight_log_20260817_204500.csv`）。
- **开始有效写入数据的时刻**：
  只要 `data_logger_node` 同时接收到 **机载里程计 (`/mavros/local_position/odom`)** 与 **MPC参考状态 (`/mpc_debug/ref_pose`)**，就会立即以设定的频率（默认 **50Hz**）高频记录当前时刻的所有飞行状态。
- **数据区分不同飞行阶段**：
  CSV 表格中包含专门的 **`mode`** 列，可精准区分数据对应的飞行阶段：
  - `mode = 0` (`AUTO_TAKEOFF`)：起飞过渡阶段；
  - `mode = 1` (`AUTO_HOVER`)：遥控器切入 OFFBOARD 后的定点悬停阶段；
  - `mode = 2` (`AUTO_TRACKING`)：**启动圆轨迹后的正式 NMPC 跟踪飞行阶段**。
- **数据保存时机**：按 `Ctrl+C` 退出节点时，缓冲区数据会自动刷新保存到磁盘。

### 2. 记录的核心数据字段
- **位置与误差**：实际坐标 $(x, y, z)$、期望坐标 $(x_{ref}, y_{ref}, z_{ref})$、3轴误差 $(e_x, e_y, e_z)$ 及 3D 总误差 $\|e_{pos}\|$；
- **速度**：实际 3 轴线速度 $(v_x, v_y, v_z)$；
- **控制与动力**：比推力期望加速度 $T$ ($m/s^2$)、三轴角速度指令 $(\omega_x, \omega_y, \omega_z)$、下发油门百分比 ($0\sim1$)、在线自适应估计的悬停油门 ($0\sim1$)、IMU 实测 $z$ 轴加速度。

---

## 三、 实机 4 步飞行操作标准流程 (SOP)

1. **手动起飞**：飞手在定点模式（POSCTL）下手动起飞，操纵无人机在指定高度（如 1.2m）悬停。
2. **切入 OFFBOARD 接管**：机载电脑运行 `run_control.launch`，飞手拨动遥控器开关切入 **OFFBOARD** 模式。
   - 控制器自动抓取当前坐标，平稳接管并原地定点悬停。
3. **触发圆轨迹**：飞手确认悬停平稳后，在地面站/SSH 终端运行 `publish_circle_traj.launch`。
   - 飞机自动切换到 `AUTO_TRACKING` 模式，开始高精度执行圆轨迹跟踪。
4. **降落与保存**：完成指定圈数后飞机自动恢复悬停；飞手随时可拨动遥控器切回定点模式或降落模式手动降落。退出节点保存 CSV 数据并绘图。

---

## 四、 核心参数配置手册 (`config/mpc_para.yaml`)

```yaml
# 1. 悬停油门初值（务必根据实机测量填写，如 5KG 飞机通常在 0.30 ~ 0.45 之间）
hover_thrust: 0.39      

# 2. 安全开关与控制频率
auto_arm_and_offboard: false  # 必须为 false：禁止自动解锁，由遥控器切入 OFFBOARD 接管
takeoff_height: 0.8           # 起飞目标高度(米)
ctrl_hz: 100                  # MPC 控制频率(Hz)

# 3. 机载定位源话题
odomTopicName: /mavros/local_position/odom  # 必须为 nav_msgs/Odometry 格式

# 4. MPC 物理边界约束 (针对大机身可适当放缓角速度限幅)
boundings:
  T_max: 25.0     # 最大比推力加速度 (m/s^2)
  T_min: 2.0      # 最小比推力加速度 (m/s^2)
  wx_max: 2.0     # 滚转角速度上限 (rad/s)
  wx_min: -2.0
  wy_max: 2.0     # 俯仰角速度上限 (rad/s)
  wy_min: -2.0
  wz_max: 2.0     # 偏航角速度上限 (rad/s)
  wz_min: -2.0
```

---

## 五、 代码更新日志 (Changelog)

> *注：后续对代码的任何核心修改均按时间在此处依次归档记录。*

### [2026-08-17]
- **【新增】数据记录与分析模块**：
  - 新建 [src/data_logger_node.cpp](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/src/data_logger_node.cpp)，高频同步记录位置误差、速度、角速度、比推力、归一化油门与 RLS 自适应悬停油门至 `data/` 目录下的 CSV 文件。
  - 新建 [scripts/plot_flight_data.py](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/scripts/plot_flight_data.py) 离线绘图工具，支持一键绘制 4 联对比曲线。
- **【新增】解耦式轨迹触发启动脚本**：
  - 新建 [launch/publish_circle_traj.launch](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/launch/publish_circle_traj.launch)，支持随时手动触发/修改圆轨迹飞行。
- **【修复】MPC 物理约束绑定**：
  - 修复 [src/mpc_wrapper.cpp](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/src/mpc_wrapper.cpp) 中 `acadoVariables.ubValues/lbValues` 硬编码固定数值的问题，现已全面动态绑定 YAML 中的参数。
- **【维护】Git 分支建立**：创建 `dev-circle-mpc` 分支并同步管理全部改动。

### [2026-08-14]
- **【新增】高阶圆轨迹生成节点**：
  - 新建 [src/circle_traj_node.cpp](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/src/circle_traj_node.cpp)，为 MPC 提供 20 步预测时域动力学参考 $(p, v, a)$。
- **【安全】实机遥控接管改造**：
  - 改造 [src/ros_mission.cpp](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/src/ros_mission.cpp)，增加 `auto_arm_and_offboard: false` 安全开关，支持飞手手动起飞后遥控切入 OFFBOARD 平滑接管。
- **【修复】航向角（Yaw）翻折漏洞**：
  - 将 `ros_mission.cpp` 中提取姿态的 `acos` 修复为 `atan2`，解决下半圆航向镜像突变问题。
- **【清理】清理动捕依赖**：删除 `launch/vrpn_pose_to_odom.launch`，规范化机载定位话题接口。
- **【工具】机载自动部署脚本**：新建 [scripts/pull_and_build.sh](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/scripts/pull_and_build.sh)。
