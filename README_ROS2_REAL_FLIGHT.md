# ROS 2 NMPC 实机与 Gazebo 仿真使用指南

本工程基于 **ROS 2 Humble / ament_cmake** 重构了四旋翼 ACADO/qpOASES 非线性模型预测控制（NMPC）控制栈。
- 第一控制量统一为比推力加速度 $a_T$（单位：$\text{m/s}^2$）；
- 其余控制量为机体三轴角速度 $[w_x, w_y, w_z]$（单位：$\text{rad/s}$）；
- 具备实时单步求解耗时与控制循环频率统计、动力电池整机功率遥测及多重安全互锁保护。

一套代码同时支持**实机外场飞行**与 **Gazebo SITL 仿真**，通过不同的 Launch 文件与配置文件一键切换。

---

## 一、 控制器核心功能与新增特性

1. **ARM64 (Jetson Nano) 编译与性能优化**：
   - 求解器静态库全面启用 `-fPIC`，完美支持 aarch64 架构。
   - 单步求解耗时平均仅 **1.5 ~ 2.0 ms**（远优于 30 Hz 的 33.3 ms 预算），CPU 负载极低。
2. **实时性能遥测与高负载预警**：
   - 终端每 1 秒输出当前实际控制频率 (Hz)、单步耗时、平均耗时与最大耗时；
   - 求解耗时超过 25 ms 时自动触发高亮警报；
   - 发布高频调试话题：`/mpc_debug/solve_time_ms` 与 `/mpc_debug/actual_rate_hz`。
3. **动力电池功耗遥测闭环**：
   - 订阅 `/mavros/battery`（10 Hz），放电电流取负号 $I_{\text{dis}} = -\text{current}$，实时计算动力总功率 $P = V \cdot I_{\text{dis}}$。
4. **多重安全互锁保护**：
   - **防冲突保护**：检测到 PID 节点发布 `/mavros/setpoint_raw/local` 时自动静默，杜绝双控竞争；
   - **离线回退保护**：轨迹中断超过 0.5 秒，自动在当前瞬时空中位置锁死悬停；
   - **执行器饱和硬约束**：推力与三轴角速度严格限制在安全物理边界内。

---

## 二、 模式一：实机飞行操作指南 (Real Flight)

### 1. 安全逻辑（人工手动闭环）
* **禁用地面自动起飞**（`auto_arm: false`, `auto_offboard: false`）；
* 控制器在地面等待时以 30 Hz 持续向飞控发送水平悬停预流指令；
* 飞手在手动/定点模式起飞至 1.0~1.5m 稳定悬停后，**拨动遥控器开关切入 OFFBOARD**，NMPC 瞬时锁定当前空中三维坐标 $(x_0, y_0, z_0)$ 进行定点悬停。

### 2. 实机启动指令

* **步骤 1：启动 MAVROS 串口连接（终端 1）**
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
  ```

* **步骤 2：启动实机 NMPC 控制器（终端 2）**
  * **仅定点悬停测试（首飞推荐）**：
    ```bash
    source ~/Desktop/uav_ws/install/setup.bash
    ros2 launch uav_mpc real_nmpc.launch.py
    ```
  * **一键开启慢速圆轨迹跟踪**：
    ```bash
    ros2 launch uav_mpc real_nmpc.launch.py start_trajectory:=true
    ```
  * **分步开启圆轨迹**（先悬停稳定后，在终端 3 执行）：
    ```bash
    ros2 run uav_mpc circle_traj_node
    ```

---

## 三、 模式二：Gazebo SITL 仿真指南 (Simulation)

### 1. 自主两阶段切换时序（Phase 1 $\to$ Phase 2）
* 载入 `config/mpc_simulation.yaml`（`hover_thrust: 0.58`, `use_sim_time: true`）；
* **阶段一（自动起飞与定点悬停）**：
  - 启动后自动调用 MAVROS 服务异步切入 OFFBOARD 并 ARM 解锁；
  - NMPC 控制无人机从地面平稳爬升至 `takeoff_height`（默认 1.5 米）并在空中稳定悬停；
* **阶段二（轨迹跟踪自动切入）**：
  - 悬停稳定达到 `start_delay_sec`（默认 6.0 秒）且高度就绪后，`circle_traj_node` 自动锁定空中圆心；
  - 经由 6 秒五次多项式（Quintic）平滑切入圆周巡检（默认半径 1.5m，速度 0.3m/s）；
  - 轨迹结束或中断后，NMPC 自动原地恢复定点悬停。

### 2. 仿真启动指令

* **全自主起飞 + 飞圆仿真**：
  ```bash
  ros2 launch uav_mpc sim_nmpc.launch.py
  ```

* **纯起飞与悬停仿真（不飞轨迹）**：
  ```bash
  ros2 launch uav_mpc sim_nmpc.launch.py start_trajectory:=false
  ```

* **自定义仿真高度、速度与轨迹参数**：
  ```bash
  ros2 launch uav_mpc sim_nmpc.launch.py \
    takeoff_height:=2.0 \
    start_delay_sec:=5.0 \
    radius:=2.5 \
    linear_speed:=0.40
  ```

---

## 四、 编译与 Git 一键同步

### 1. Jetson Nano / PC 工作空间编译
```bash
cd ~/Desktop/uav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Jetson Nano 一键拉取并编译脚本
```bash
~/Desktop/uav_ws/src/quadrotor_mpc/scripts/sync_and_build.sh
```

---

## 五、 常用调试与监控话题

| 话题名称 | 消息类型 | 作用说明 |
| :--- | :--- | :--- |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` | NMPC 输出的推力与三轴角速度控制指令 (30 Hz) |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | 飞控 EKF2 融合后的位姿与机体速度反馈 |
| `/mavros/battery` | `sensor_msgs/msg/BatteryState` | 动力电池电压、放电电流及电量 (10 Hz) |
| `/mpc_debug/solve_time_ms` | `std_msgs/msg/Float64` | NMPC 求解器单步计算耗时（毫秒） |
| `/mpc_debug/actual_rate_hz` | `std_msgs/msg/Float64` | 控制外环实际运行频率（Hz） |
| `/mpc_debug/mode` | `std_msgs/msg/Int8` | 当前控制模式（`-1`: WAITING, `1`: HOVER, `2`: TRACKING） |
| `/mpc_ref_traj` | `uav_mpc/msg/MpcRefTraj` | 轨迹节点发布的未来 20 步预测时域参考点序列 |
