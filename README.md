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

仿真世界基于 `windy.world`（**Gazebo 仿真时间 90s~120s 施加 +X 方向 3.0 m/s 阶跃突发阵风**），采用**控制器起飞悬停与轨迹触发解耦**的标准流程。

```mermaid
graph LR
    T1["终端 1: Gazebo SITL"] --> T2["终端 2: roscore"]
    T2 --> T3["终端 3: MAVROS"]
    T3 --> T4["终端 4: 启动控制器 (自动起飞并悬停)"]
    T4 --> T5["终端 5: 启动圆轨迹跟踪"]
    T5 --> T6["终端 6: 性能评估与绘图"]
```

### 步骤 1：终端 1 启动 PX4 SITL 与 Gazebo 风场环境
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris__windy
```

### 步骤 2：终端 2 启动 ROS Master
```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 步骤 3：终端 3 启动 MAVROS 桥接节点
```bash
source /opt/ros/noetic/setup.bash
source ~/Desktop/catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 步骤 4：终端 4 启动控制器（起飞至 1.5m 稳定悬停）

#### 实验组 A（NMPC 控制器）：
```bash
source ~/Desktop/catkin_ws/devel/setup.bash
roslaunch uav_mpc simulation_mpc.launch
```
> 控制器自动解锁起飞至 1.5m，随后在 `(0, 0, 1.5)` 稳定悬停等待轨迹指令。

#### 对照组 B（PID 基准控制器）：
```bash
source ~/Desktop/catkin_ws/devel/setup.bash
roslaunch uav_mpc simulation_pid_baseline.launch
```
> 基准控制器自动解锁起飞至 1.5m 稳定悬停。

### 步骤 5：终端 5 发布圆轨迹（按需启动走圆）
确认无人机在 1.5m 高度悬停平稳后，在终端 5 执行：
```bash
source ~/Desktop/catkin_ws/devel/setup.bash
roslaunch uav_mpc publish_circle_traj.launch center_z:=1.5 radius:=1.5 linear_vel:=0.8 cycles:=15
```
> 轨迹节点将在前 3 秒以平滑三次多项式过渡（无速度/加速度冲击）从悬停位置切入圆周，连续飞行 15 圈并历经 90~120s 突发阵风。

### 步骤 6：终端 6 运行多维度定量评估与对比绘图
```bash
cd ~/Desktop/catkin_ws/src/quadrotor_mpc
python3 scripts/evaluate_nmpc_vs_pid.py
```
> 自动输出包含 **3D/各轴跟踪 RMSE**、**阵风抗扰放大率**、**瞬态峰值**、**角速度平滑度** 与 **能耗代理指标** 的对比表格，并在 `data/` 目录下生成高分辨率综合对比图 `simulation_evaluation_report.png`。

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
```bash
mkdir -p ~/Desktop/catkin_ws/src
cd ~/Desktop/catkin_ws/src
git clone -b dev-simulation https://github.com/martubuntu/quadrotor_mpc.git

cd ~/Desktop/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release

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
git commit -m "fix(sim): decouple takeoff and trajectory publisher, enhance thrust estimator and trajectory transition"
git push origin dev-simulation
```
