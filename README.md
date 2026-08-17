# UAV Non-linear MPC (ACADO) 轨迹跟踪控制与性能评估系统

本项目是基于 **ACADO Toolkit (qpOASES)** 的四旋翼无人机非线性模型预测控制（NMPC）系统，支持 **Gazebo SITL 仿真（含突发阵风扰动）** 与 **实机自主飞行**，具备动态推力自适应估计、闭环圆轨迹跟踪、PX4 原生级联 PID 对照组以及多维度性能量化评估体系。

---

## 🎯 核心研究目标演进路径

```mermaid
graph LR
    Stage1["【当前】阶段 1: NMPC 闭环仿真与 PID 多维度对比"] --> Stage2["阶段 2: ESO 扩展状态观测器风扰估计与前馈补偿"]
    Stage2 --> Stage3["阶段 3: 偏心载荷与风扰下跟踪-能耗协同优化 NMPC"]
```

---

## 🌪️ 一、 VM Ubuntu 20.04 Gazebo SITL 仿真完整操作指南

本仿真配置基于用户自定义风场 `windy.world`（**Gazebo 仿真时间 90s~120s 施加 +X 方向 3.0 m/s 阶跃突发阵风**），完整对比 NMPC 与 PID 在无风稳态、阵风冲击与风停恢复 3 个阶段的动态响应。

### 步骤 1：终端 1 启动 PX4 SITL 与 Gazebo 风场环境
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris__windy
```

### 步骤 2：终端 2 启动 ROS Master
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

### 步骤 3：终端 3 启动 MAVROS 桥接节点
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 步骤 4：终端 4 启动控制器实验（二选一）

#### 实验组 A：启动 NMPC 轨迹跟踪控制器（全自动起飞与绕圆）
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch uav_mpc simulation_mpc.launch
```
> 无人机将自动解锁并起飞至 1.5m 稳定悬停，15s 后自动切入半径 1.5m、线速度 0.8m/s 的连续绕圆轨迹，自动历经 90~120s 的 3m/s 阵风扰动并自动记录 `flight_log_NMPC_*.csv`。

#### 对照组 B：启动 PX4 原生级联 PID 轨迹跟踪控制器
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch uav_mpc simulation_pid_baseline.launch
```
> 在相同风场与初始条件下运行相同的绕圆轨迹，由 PX4 底层位置-速度 PID 跟踪，自动记录 `flight_log_PID_*.csv`。

### 步骤 5：终端 5 运行多维度定量评估与对比绘图
```bash
cd ~/catkin_ws/src/quadrotor_mpc
python3 scripts/evaluate_nmpc_vs_pid.py
```
> 自动读取最新的 NMPC 与 PID 飞行日志，划分**无风阶段 (20~90s)**、**阵风阶段 (90~120s)** 与**恢复阶段 (120~150s)**，自动输出 Markdown/终端定量指标对比表，并生成高分辨率对比图 `data/simulation_evaluation_report.png`。

---

## 📊 二、 多维度评估指标体系

| 评估维度 | 指标名称 | 计算公式 / 说明 | 物理意义 |
| :--- | :--- | :--- | :--- |
| **1. 轨迹跟踪精度** | **3D Position RMSE** | $\sqrt{\frac{1}{N}\sum (e_x^2 + e_y^2 + e_z^2)}$ | 全程空间几何轨迹贴合精度 |
| | **Max 3D Error** | $\max \|e_{\text{pos}}(t)\|$ | 最劣工况下的最大失轨距离 |
| | **Per-axis RMSE** | $X, Y, Z$ 各轴独立均方根误差 | 评估风向（+X）对各通道的耦合偏差 |
| **2. 抗风扰鲁棒性** | **Gust RMSE** | $t \in [90\text{s}, 120\text{s}]$ 内的 3D RMSE | 阵风扰动下的跟踪恶化程度 |
| | **Gust Amplification Ratio** | $\text{RMSE}_{\text{gust}} / \text{RMSE}_{\text{calm}}$ | 扰动敏感度（越接近 1.0 抗扰越强） |
| | **Gust Onset Peak Error** | $t \in [90\text{s}, 95\text{s}]$ 内的最大瞬态偏差 | 阶跃风扰撞击瞬间的动态恢复能力 |
| **3. 控制平滑度** | **Body Rate RMS** | $\sqrt{\frac{1}{N}\sum (\omega_x^2 + \omega_y^2 + \omega_z^2)}$ | 机体姿态角速度动态激烈程度 |
| | **Throttle Variance** | $\text{Var}(T)$ | 油门输出波动方差（反映执行机构应力） |
| **4. 能耗代理指标** | **Power Proxy Integral** | $\int T(t)^{1.5} dt$ | 旋翼气动机械功率消耗代理积分 |
| | **Control Effort Metric** | $\int (T^2 + 0.05\|\boldsymbol{\omega}\|^2) dt$ | 控制能量总开销 |

---

## 🛠️ 三、 虚拟机桌面工作空间搭建与一键编译指令

### 1. 首次在虚拟机桌面（Desktop）搭建工作空间与克隆代码
在 Ubuntu 20.04 虚拟机终端执行：
```bash
# 创建桌面工作空间
mkdir -p ~/Desktop/catkin_ws/src
cd ~/Desktop/catkin_ws/src

# 克隆 dev-simulation 分支
git clone -b dev-simulation https://github.com/martubuntu/quadrotor_mpc.git

# 编译整个工作空间
cd ~/Desktop/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release

# 写入环境变量配置（开终端自动生效）
echo "source ~/Desktop/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. 后续拉取最新代码与一键增量编译
```bash
cd ~/Desktop/catkin_ws/src/quadrotor_mpc
./scripts/pull_and_build.sh
```

### 3. PC 端推送修改至 GitHub 分支
```bash
git add .
git commit -m "feat(sim): setup SITL simulation with wind gust and NMPC vs PID evaluation framework"
git push origin dev-simulation
```

---

## 📝 四、 开发更新日志（Changelog）

### [2026-08-17] - 阶段一：Gazebo SITL 仿真闭环与多维度性能评估体系
- **仿真配置与风场适配**：
  - 创建 `config/mpc_simulation.yaml`，适配 Gazebo Iris 模型物理参数（`mass: 1.5kg`, `hover_thrust: 0.58`），开启 `auto_arm_and_offboard: true`。
  - 适配 `windy.world`（90s~120s 持续 30s 施加 +X 3m/s 阵风）。
- **PID 对照组基准节点**：
  - 编写 `src/pid_baseline_node.cpp`，实现基于 PX4 原生级联 PID 的轨迹跟踪基准控制器，并发布一致的调试遥测话题。
  - 在 `CMakeLists.txt` 中添加 `pid_baseline_node` 编译目标。
- **仿真一键启动 Launch 体系**：
  - 创建 `launch/simulation_mpc.launch`（NMPC 实验组，自动标记 `log_prefix: NMPC`）。
  - 创建 `launch/simulation_pid_baseline.launch`（PID 对照组，自动标记 `log_prefix: PID`）。
- **多维度性能评估与分析脚本**：
  - 编写 `scripts/evaluate_nmpc_vs_pid.py`，实现多区间划分（无风/阵风/恢复）、4 大维度 14 项指标自动计算、格式化表格输出与综合报告图生成。
- **数据记录器升级**：
  - 更新 `src/data_logger_node.cpp`，支持通过 `log_prefix` 动态命名日志文件（`flight_log_NMPC_*.csv` 与 `flight_log_PID_*.csv`）。
- **分支与脚本同步**：
  - 更新 `scripts/pull_and_build.sh` 适配 `dev-simulation` 分支。
