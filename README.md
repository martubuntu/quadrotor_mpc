# UAV Non-linear MPC (ACADO / ROS 2 Humble) 轨迹跟踪控制系统

本项目是基于 **ACADO Toolkit (qpOASES)** 的四旋翼无人机非线性模型预测控制（NMPC）系统，支持 **ROS 2 Humble (Ubuntu 22.04 LTS)**，覆盖 **3.5 kg 实机平台（550轴距 + 6S动力 + 12寸桨 + Jetson Nano + Pixhawk 6C）** 与 **Gazebo SITL 仿真**。系统具备 ESO 扩展状态风扰观测器前馈补偿、毫秒级求解耗时遥测监控、以及悬停与轨迹跟踪解耦的双指令工作流。

当前分支：**`ros2-realflight`**

---

## 📌 最新重构更新日志 (Changelog)

1. **实机动力学标定 (3.5 kg 真实全重)**：
   * 基于实飞 ULog 数据完成精准反演，标定 3.5 kg 机身实测悬停油门 `hover_thrust: 0.72`，稳态悬停功耗基准为 `502.4 W`（~22A @ 22.8V）。
   * 卸载冗余重载传感器后，推重比恢复至健康裕度，彻底解决机动下坠问题。
2. **ROS 2 Humble / Ament 架构移植与 ARM64 编译优化**：
   * 采用 ROS 2 原生 C++ 接口 (`rclcpp`) 与自定义接口消息 (`MpcRefPoint`, `MpcRefTraj`)。
   * 配置 `-fPIC` 独立位置代码，彻底修复 Jetson Nano ARM64 架构链接冲突，实现 1.6ms 极速板载求解。
3. **单一配置文件与单一启动文件（极简架构）**：
   * **单一配置**：全库所有实机与仿真参数合并至 [config/mpc_para.yaml](file:///c:/Users/superglider/Desktop/UAV/CODE/quadrotor_mpc_eso_nmpc/quadrotor_mpc/config/mpc_para.yaml)。
   * **单一启动**：全库统一定名为 [launch/nmpc.launch.py](file:///c:/Users/superglider/Desktop/UAV/CODE/quadrotor_mpc_eso_nmpc/quadrotor_mpc/launch/nmpc.launch.py)，通过 `is_sim` 参数在实机与仿真间自由切换。
4. **悬停与飞圆“双指令物理与时序解耦”**：
   * **指令 1 (NMPC 控制器)**：负责底层闭环与高精度定点悬停。
   * **指令 2 (轨迹生成器)**：独立启停飞圆轨迹，随时按 `Ctrl+C` 停止，控制器 0.5s 内自动切回当前位置平稳悬停。

---

## ⚙️ 核心参数表 (`config/mpc_para.yaml`)

| 参数名 | 默认值 | 作用说明 |
| :--- | :--- | :--- |
| `is_sim` | `false` | **主模式开关**：`false` 为实机（手动起飞切入，安全第一）；`true` 为 Gazebo 仿真（全自动起飞与模式切换）。 |
| `hover_thrust` | `0.72` | **实机 3.5 kg 平台悬停推力**（基于实飞 ULog 标定真值，`is_sim=false` 时生效）。 |
| `sim_hover_thrust` | `0.58` | **Gazebo Iris 仿真模型悬停推力**（`is_sim=true` 时自动选用）。 |
| `thrust_max` | `0.88` | 实机油门上限保护值（留出 ~16% 动态机动裕度）。 |
| `takeoff_height` | `1.5` | 仿真模式下目标起飞爬升高度（单位 m）。 |
| `sim_start_delay_sec` | `10.0` | 仿真阶段一（起飞至 1.5m 稳定悬停）保持时间，之后自动无缝切入阶段二飞圆轨迹。 |
| `start_trajectory` | `false` | 是否在 Launch 中一并启动轨迹节点（`false` 仅悬停，`true` 飞圆）。 |
| `use_eso` | `false` | 是否接收 `/eso/disturbance` 外部扰动前馈。 |
| `radius` / `linear_speed` | `1.5` / `0.20` | 圆轨迹半径 (m) 与巡航线速度 (m/s)。 |

---

## 🚀 一、 编译与环境准备

### 在 Jetson Nano / PC (Ubuntu 22.04 + ROS 2 Humble) 上编译：
```bash
cd ~/Desktop/uav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

或直接执行一键拉取编译脚本：
```bash
~/Desktop/uav_ws/src/quadrotor_mpc/scripts/sync_and_build.sh
```

---

## 🛸 二、 实机飞行操作流程 (Real Flight)

### 步骤 1：终端 1 启动 MAVROS 串口通讯
```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```

### 步骤 2：终端 2【指令 1】启动 NMPC 控制器（实机定点悬停）
```bash
ros2 launch uav_mpc nmpc.launch.py
```
* **实飞流程**：
  1. 飞手遥控器手动起飞至 1.0~1.5m 建立稳定悬停；
  2. 拨动开关切入 **OFFBOARD** 模式，NMPC 瞬时锁定当前空中位姿定点悬停；
  3. 随时切回 Position/Manual 模式可瞬间人工接管。

### 步骤 3：终端 3【指令 2】按需随时启停飞圆轨迹
```bash
# 启动飞圆：锁定当前圆心，8s 平滑切入并以 0.20m/s 飞圆
ros2 run uav_mpc circle_traj_node

# 停止飞圆：在终端 3 按 Ctrl+C 终止，NMPC 自动锁死当前悬停点！
```

### 步骤 4：【可选】轻量化 CSV 飞行数据录制（替代高内存消耗的 rosbag）
若需要保存实验轨迹与能耗数据供论文作图，只需在新终端启动录制节点（每分钟仅占用 ~1.5MB 纯文本 CSV）：
```bash
# 独立启动录制（自动按时间戳保存至 data/jetson_flyrecord/ 文件夹）：
ros2 run uav_mpc data_logger_node

# 或在 launch 启动控制器时一并带上录制：
# ros2 launch uav_mpc nmpc.launch.py record_data:=true
```
* **录制内容**：时间戳、实际位姿(XYZ/RPY)、期望轨迹、3D位置跟踪误差、三维速度、控制器比推力与角速度指令、MAVROS实发油门、电池电压/放电电流/总实时功耗/累计能耗(J)、ESO外部扰动。
* 按 `Ctrl+C` 停止录制时会自动保存并安全关闭文件。

---

## 📁 数据存储与离线分析工具说明 (Data & Analysis Tools)

全工程的飞行日志与分析脚本严格规范划分：

```text
quadrotor_mpc/
├── data/
│   ├── px4_flyrecord/       # 【飞控硬件日志】存放 Pixhawk SD 卡导出的原始 .ulg 格式黑匣子
│   └── jetson_flyrecord/    # 【机载控制器日志】存放 Jetson Nano 上 data_logger_node 实时录制的 .csv 表格
└── scripts/
    ├── parse_px4_ulog.py    # 【PC/Host 离线分析】一键解析 ULog，自动提取真实悬停推力、电机平衡度与平均功耗
    └── sync_and_build.sh    # 【Jetson 一键同步】自动 git pull 并 colcon build 编译
```

### 离线分析 Pixhawk ULog（在电脑端执行）：
```bash
# 自动扫描 data/px4_flyrecord/ 下的所有 .ulg 文件并输出标定汇总报告：
python3 scripts/parse_px4_ulog.py
```

---

## 💻 三、 Gazebo SITL 仿真操作流程 (Simulation)

### 步骤 1：终端 1 启动 PX4 SITL 与 Gazebo 环境
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

### 步骤 2：终端 2 启动 MAVROS 仿真桥接
```bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 步骤 3：终端 3【指令 1】启动 NMPC 控制器（仿真自主起飞与悬停）
```bash
ros2 launch uav_mpc nmpc.launch.py is_sim:=true
```
* **仿真行为**：无人机在 Gazebo 中自动解锁 ARM，自动切入 OFFBOARD，从地面平稳爬升至 1.5m 定点悬停。

### 步骤 4：终端 4【指令 2】按需启动 / 停止飞圆轨迹
```bash
# 启动仿真飞圆（带 10s 悬停稳定等待）
ros2 run uav_mpc circle_traj_node --ros-args -p is_sim:=true

# 停止飞圆：按 Ctrl+C 即可，无人机在空中当前位置平稳锁死悬停
```

> **提示**：若想单行指令直接一键完成仿真全流程（起飞+悬停10s+飞圆）：
> ```bash
> ros2 launch uav_mpc nmpc.launch.py is_sim:=true start_trajectory:=true
> ```

---

## 📊 四、 实时性能与遥测监控

NMPC 节点以 30 Hz 运行，并在终端每 1.0 秒输出一次实时求解耗时与频率：
```text
[NMPC Rate]  30.0 Hz | Solve: 1.65 ms (avg 1.62 ms, max 2.80 ms) | Mode: HOVER
```
* `/mpc_debug/solve_time_ms`：单步 QP 求解耗时（ms）。
* `/mpc_debug/actual_rate_hz`：实际外环执行频率（Hz）。
* `/mpc_debug/mode`：运行状态（`-1`: WAITING 预流，`1`: HOVER 悬停，`2`: TRACKING 轨迹）。
* `/mavros/battery`：动力电池实时电压与放电电流。
