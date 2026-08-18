# UAV Non-linear MPC (ACADO) 轨迹跟踪控制与性能评估系统

本项目是基于 **ACADO Toolkit (qpOASES)** 的四旋翼无人机非线性模型预测控制（NMPC）系统，支持 **Gazebo SITL 仿真（含突发阵风扰动）** 与 **实机自主飞行**，具备 ESO 扩展状态风扰观测器、动态推力自适应估计、闭环圆轨迹跟踪、PX4 原生级联 PID 对照组以及多维度性能量化评估体系。

当前开发分支：**`NMPC_trajok_simulation`**

---0

## 🎯 核心研究目标演进路径

```mermaid
graph LR
    Stage1["【已完成】阶段 1: NMPC 闭环基线仿真 (RMSE: 0.118m, 相比PID提升 +65.9%)"] --> Stage2["【当前推进】阶段 2: NMPC + ESO 扩展状态观测器风扰估计与前馈补偿"]
    Stage2 --> Stage3["【规划中】阶段 3: 偏心载荷与风扰下跟踪-能耗协同优化 NMPC"]
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

#### 选项 A（【推荐】阶段二 NMPC + ESO 扩展状态观测器实验组）：
```bash
source ~/Desktop/catkin_ws/devel/setup.bash
roslaunch uav_mpc simulation_mpc_eso.launch
```
> ESO 节点将在线估计受到的外部风阻加速度 $\hat{\mathbf{d}}$，并实时注入 ACADO 预测模型进行前馈抗扰补偿。

#### 选项 B（阶段一 纯 NMPC 实验组）：
```bash
source ~/Desktop/catkin_ws/devel/setup.bash
roslaunch uav_mpc simulation_mpc.launch
```

#### 选项 C（PX4 原生级联 PID 基准对照组）：
```bash
source ~/Desktop/catkin_ws/devel/setup.bash
roslaunch uav_mpc simulation_pid_baseline.launch
```

### 步骤 5：终端 5 发布圆轨迹（按需启动走圆）
确认无人机在 1.5m 高度悬停平稳后，在终端 5 执行：
```bash
source ~/Desktop/catkin_ws/devel/setup.bash
roslaunch uav_mpc publish_circle_traj.launch height:=1.5 radius:=1.5 linear_vel:=0.8 cycles:=15
```

### 步骤 6：终端 6 运行多维度定量评估与对比绘图
```bash
cd ~/Desktop/catkin_ws/src/quadrotor_mpc
python3 scripts/evaluate_nmpc_vs_pid.py
```
> 自动加载 `data/` 目录下的实验数据，生成 **NMPC+ESO vs 纯NMPC vs PID** 的三方综合评测表格与高清对比图 `simulation_evaluation_report.png`。

---

## 🛠️ 二、 虚拟机桌面工作空间一键拉取与编译指令

```bash
cd ~/Desktop/catkin_ws/src/quadrotor_mpc
git fetch origin
git checkout NMPC_trajok_simulation
git pull origin NMPC_trajok_simulation

cd ~/Desktop/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```
