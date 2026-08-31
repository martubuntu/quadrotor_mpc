# ROS 2 NMPC Control Stack (Unified Real-Flight & Gazebo SITL)

This package implements an ACADO/qpOASES NMPC controller for quadrotors under ROS 2 Humble.
The first control input is collective specific thrust $a_T$ in $\text{m/s}^2$; the remaining inputs are body rates $[w_x, w_y, w_z]$ in $\text{rad/s}$.

---

## 1. 单一配置文件与统一启动架构 (Single Config & Unified Launch)

全工程仅使用 **一个统一配置文件 `config/mpc_para.yaml`**。
通过 `is_sim` 主控参数实现**实机物理飞行**与 **Gazebo SITL 自主仿真**的无缝切换。

### 核心参数说明 (`config/mpc_para.yaml`)
| 参数名 | 默认值 | 作用说明 |
| :--- | :--- | :--- |
| `is_sim` | `false` | **主模式开关**：`false` 为实机（手动起飞切入，安全第一）；`true` 为 Gazebo 仿真（全自动起飞与模式切换）。 |
| `hover_thrust` | `0.50` | **实机 5 kg 平台悬停推力**（`is_sim=false` 时生效）。 |
| `sim_hover_thrust` | `0.58` | **Gazebo Iris 仿真模型悬停推力**（`is_sim=true` 时自动选用）。 |
| `takeoff_height` | `1.5` | 仿真模式下目标起飞爬升高度（单位 m）。 |
| `sim_start_delay_sec` | `6.0` | 仿真阶段一（起飞至 1.5m 稳定悬停）保持时间，之后自动无缝切入阶段二飞圆轨迹。 |
| `start_trajectory` | `false` | 是否开启阶段二圆轨迹跟踪（`false` 仅悬停，`true` 飞圆）。 |
| `use_eso` | `false` | 是否接收 `/eso/disturbance` 外部扰动前馈。 |
| `radius` / `linear_speed` | `1.5` / `0.20` | 圆轨迹半径 (m) 与巡航线速度 (m/s)。 |

---

## 2. 编译指南 (Build on Jetson / PC / Ubuntu 22.04)

```bash
cd ~/Desktop/uav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

或直接执行一键同步编译脚本：
```bash
~/Desktop/uav_ws/src/quadrotor_mpc/scripts/sync_and_build.sh
```

---

## 3. 实机飞行操作指南 (`is_sim:=false`)

实机默认禁用自动解锁与自动切模式，采用**最高安全级别的人工起飞闭环**。

### 第 1 步：启动 MAVROS（终端 1）
```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```

### 第 2 步：启动 NMPC 定点悬停测试（终端 2）
```bash
ros2 launch uav_mpc real_nmpc.launch.py
```
* **实飞流程**：
  1. 飞手遥控器手动起飞至 1.0~1.5m 建立稳定悬停；
  2. 拨动开关切入 **OFFBOARD** 模式，NMPC 瞬时锁定当前空中位姿定点悬停；
  3. 随时切回 Position/Manual 模式可瞬间人工接管。

### 第 3 步：实机飞圆轨迹跟踪测试
```bash
ros2 launch uav_mpc real_nmpc.launch.py start_trajectory:=true
```

---

## 4. Gazebo SITL 仿真操作指南 (`is_sim:=true`)

仿真模式下控制器会自动完成：**阶段一（地面自动起飞至 1.5m 悬停 6s） $\to$ 阶段二（自动开始飞圆轨迹）**。

### 方式 A：通过主启动文件传递 `is_sim:=true`（推荐统一语法）
```bash
# 1. 仿真飞圆（起飞悬停 6s 后自动绕圆）
ros2 launch uav_mpc real_nmpc.launch.py is_sim:=true start_trajectory:=true

# 2. 仿真仅悬停（起飞到 1.5m 保持悬停，不飞轨迹）
ros2 launch uav_mpc real_nmpc.launch.py is_sim:=true start_trajectory:=false
```

### 方式 B：通过仿真快捷启动文件
```bash
ros2 launch uav_mpc sim_nmpc.launch.py
```

### 动态自定义仿真轨迹参数示例
```bash
ros2 launch uav_mpc real_nmpc.launch.py is_sim:=true start_trajectory:=true \
  takeoff_height:=2.0 \
  start_delay_sec:=5.0 \
  radius:=2.5 \
  linear_speed:=0.40
```

---

## 5. 实时性能与遥测监控说明

NMPC 节点以 30 Hz 运行，并在终端每 1.0 秒输出一次实时求解耗时与频率：
```text
[NMPC Rate]  30.0 Hz | Solve: 1.65 ms (avg 1.62 ms, max 2.80 ms) | Mode: HOVER
```
* `/mpc_debug/solve_time_ms`：单步 QP 求解耗时（ms）。
* `/mpc_debug/actual_rate_hz`：实际外环执行频率（Hz）。
* `/mpc_debug/mode`：运行状态（`-1`: WAITING 预流，`1`: HOVER 悬停，`2`: TRACKING 轨迹）。
* `/mavros/battery`：动力电池实时电压与放电电流。
