# ROS 2 NMPC Control Stack (Unified Architecture)

This package implements an ACADO/qpOASES NMPC controller for quadrotors under ROS 2 Humble.
The first control input is collective specific thrust $a_T$ in $\text{m/s}^2$; the remaining inputs are body rates $[w_x, w_y, w_z]$ in $\text{rad/s}$.

---

## 1. 架构总览 (Unified Architecture & Complete Decoupling)

全工程严格遵循**“极简统一、双指令解耦”**原则：
* **唯一定名启动文件**：`launch/nmpc.launch.py`
* **唯一统一配置文件**：`config/mpc_para.yaml`
* **两大功能完全解耦（两条指令）**：
  * **指令 1（NMPC 控制器）**：负责底层闭环与高精定点悬停（实机手动切入，仿真自动起飞）。
  * **指令 2（轨迹生成器）**：负责独立开启/停止飞圆轨迹，随时启动、随时 `Ctrl+C` 停掉切回悬停。

---

## 2. 核心参数表 (`config/mpc_para.yaml`)

| 参数名 | 默认值 | 作用说明 |
| :--- | :--- | :--- |
| `is_sim` | `false` | **主模式开关**：`false` 为实机（手动起飞切入，安全第一）；`true` 为 Gazebo 仿真（全自动起飞与模式切换）。 |
| `hover_thrust` | `0.50` | **实机 5 kg 平台悬停推力**（`is_sim=false` 时生效）。 |
| `sim_hover_thrust` | `0.58` | **Gazebo Iris 仿真模型悬停推力**（`is_sim=true` 时自动选用）。 |
| `takeoff_height` | `1.5` | 仿真模式下目标起飞爬升高度（单位 m）。 |
| `sim_start_delay_sec` | `10.0` | 仿真阶段一（起飞至 1.5m 稳定悬停）保持时间，之后自动无缝切入阶段二飞圆轨迹。 |
| `start_trajectory` | `false` | 是否在 Launch 中一并启动轨迹节点（`false` 仅悬停，`true` 飞圆）。 |
| `use_eso` | `false` | 是否接收 `/eso/disturbance` 外部扰动前馈。 |
| `radius` / `linear_speed` | `1.5` / `0.20` | 圆轨迹半径 (m) 与巡航线速度 (m/s)。 |

---

## 3. 实机飞行操作指南 (Real Flight)

### 终端 1：启动 MAVROS 串口通讯
```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```

### 终端 2：【指令 1】启动 NMPC 控制器（实机定点悬停）
```bash
ros2 launch uav_mpc nmpc.launch.py
```
* **实飞流程**：
  1. 飞手遥控器手动起飞至 1.0~1.5m 建立稳定悬停；
  2. 拨动开关切入 **OFFBOARD** 模式，NMPC 瞬时锁定当前空中位姿定点悬停；
  3. 随时切回 Position/Manual 模式可瞬间人工接管。

### 终端 3：【指令 2】按需启动 / 停止飞圆轨迹
```bash
# 启动飞圆轨迹：立即锁定当前圆心，8s平滑切入并以 0.20m/s 飞圆
ros2 run uav_mpc circle_traj_node

# 停止飞圆轨迹：直接在终端 3 按 Ctrl+C
# 控制器 0.5s 判定轨迹流中断，自动锁定当前位姿平稳悬停！
```

---

## 4. Gazebo SITL 仿真操作指南 (Simulation)

### 终端 1：【指令 1】启动 NMPC 控制器（仿真自主起飞与悬停）
```bash
ros2 launch uav_mpc nmpc.launch.py is_sim:=true
```
* **仿真行为**：无人机在 Gazebo 中自动解锁 ARM，自动切入 OFFBOARD，自动从地面平稳爬升至 1.5m 并稳定定点悬停。

### 终端 2：【指令 2】按需启动 / 停止飞圆轨迹
```bash
# 启动飞圆轨迹（带 10s 悬停稳定等待）
ros2 run uav_mpc circle_traj_node --ros-args -p is_sim:=true

# 停止飞圆轨迹：按 Ctrl+C 即可，无人机在空中当前位置平稳锁死悬停
```

> **可选的一键合并单行指令**：若想一条命令跑完全程（起飞+悬停10s+飞圆）：
> ```bash
> ros2 launch uav_mpc nmpc.launch.py is_sim:=true start_trajectory:=true
> ```

---

## 5. 编译与更新指令

```bash
cd ~/Desktop/uav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

或执行一键拉取编译脚本：
```bash
~/Desktop/uav_ws/src/quadrotor_mpc/scripts/sync_and_build.sh
```

---

## 6. 实时性能监控说明

NMPC 节点以 30 Hz 运行，并在终端每 1.0 秒输出一次实时求解耗时与频率：
```text
[NMPC Rate]  30.0 Hz | Solve: 1.65 ms (avg 1.62 ms, max 2.80 ms) | Mode: HOVER
```
* `/mpc_debug/solve_time_ms`：单步 QP 求解耗时（ms）。
* `/mpc_debug/actual_rate_hz`：实际外环执行频率（Hz）。
* `/mpc_debug/mode`：运行状态（`-1`: WAITING 预流，`1`: HOVER 悬停，`2`: TRACKING 轨迹）。
* `/mavros/battery`：动力电池实时电压与放电电流。
