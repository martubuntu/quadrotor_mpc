# UAV NMPC 圆轨迹跟踪与飞行控制系统

本项目基于 **ACADO** 非线性模型预测控制（NMPC）与 **MAVROS**，专为四旋翼无人机实机飞行实验设计，支持**遥控器安全接管悬停**、**高频圆轨迹跟踪**以及**全状态飞行数据自动记录与离线分析**。

---

## 🚀 一、 代码启动指令全流程总结

为确保实机实验安全可控，整个飞行流程遵循 **“先起飞悬停接管，后触发轨迹跟踪”** 的两步操作：

### 1. 编译与环境变量加载
```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg uav_mpc
source devel/setup.bash
```

### 2. 启动 MPC 控制器（含自动数据记录）
```bash
roslaunch uav_mpc run_control.launch
```
> **飞手操作**：
> 1. 用遥控器手动起飞至空中（如 1.2m）并保持定点悬停；
> 2. 拨动遥控器开关切入 **OFFBOARD** 模式；
> 3. 控制器自动将当前位置设为目标点，平稳接管定点悬停。

### 3. 触发圆轨迹跟踪（悬停稳定后按需执行）
在另一个终端中启动圆轨迹节点，开始自主绕圆飞行：
```bash
roslaunch uav_mpc publish_circle_traj.launch center_x:=0.0 center_y:=0.0 center_z:=1.2 radius:=1.0 linear_vel:=0.6 cycles:=3
```
> **常用参数说明**：
> - `center_x`, `center_y`, `center_z`：圆心三维坐标（米）
> - `radius`：圆半径（米）
> - `linear_vel`：切向线速度（m/s，初次试飞建议 0.4~0.6 m/s）
> - `cycles`：飞行圈数（`-1` 为无限连续转圈，`3` 为跑满 3 圈后自动停在起点悬停）

### 4. 数据一键离线绘图与分析
飞行结束降落后，运行分析脚本自动生成图表：
```bash
python3 ~/catkin_ws/src/quadrotor_mpc/scripts/plot_flight_data.py
```

---

## 📊 二、 数据记录机制详解

### 1. 记录数据保存在哪个文件？
- **存储路径**：`uav_mpc/data/` 目录下（若无此文件夹会自动创建）。
- **文件命名规则**：每次启动以系统时间戳自动独立命名，格式为：
  ```text
  uav_mpc/data/flight_log_YYYYMMDD_HHMMSS.csv
  ```
  *(例如：`data/flight_log_20260817_204530.csv`，每次飞行独立建表，不会覆盖历史数据)*

### 2. 数据从什么时候开始记录？
- **触发时机**：只要运行包含记录节点的 launch（`run_control.launch` 默认开启），当节点**收到第一帧有效的里程计数据（Odom）和参考位姿时**，即刻以 **50Hz** 频率高频记录。
- **保存时机**：节点随 launch 关闭（Ctrl+C）或飞行结束后，自动完成数据落盘与文件关闭。

### 3. 记录的数据字段列表：
| 字段类别 | 包含数据 |
| :--- | :--- |
| **时间与模式** | `timestamp`(系统时间戳), `time_sec`(相对秒数), `mode`(0:起飞, 1:悬停, 2:跟踪) |
| **实际位姿与速度** | `pos_x, pos_y, pos_z`(实际坐标), `vel_x, vel_y, vel_z`(实际线速度) |
| **参考轨迹** | `ref_x, ref_y, ref_z`(期望圆轨迹点) |
| **跟踪误差** | `err_x, err_y, err_z`(3轴位置误差), `err_pos_norm`(3D欧式总误差 $\|e_{pos}\|$) |
| **控制输出** | `ctrl_acc_z`(期望比推力 $T$ m/s²), `ctrl_wx, ctrl_wy, ctrl_wz`(期望三轴角速度 rad/s) |
| **推力与油门** | `cmd_thrust`(下发飞控归一化油门 0~1), `estimated_hover_thrust`(在线估计悬停油门), `imu_acc_z`(IMU实测Z轴加速度) |

---

## 🔄 三、 Git 协同与机载电脑快速同步

### PC 端（Antigravity IDE）修改并推送：
```bash
git add .
git commit -m "feat/fix: 说明本次修改内容"
git push origin dev-circle-mpc
```

### 无人机机载电脑端极速更新与增量编译（1~3秒）：
```bash
~/catkin_ws/src/quadrotor_mpc/scripts/pull_and_build.sh
```

---

## 📝 四、 项目开发与版本更迭日志 (Changelog)

> **日志维护规范**：后续对代码进行任何功能增加、Bug修复或参数结构调整，均按时间在下方倒序（最新修改在最上方）追加记录。

### [2026-08-17]
- **新增数据记录节点**：新建 [src/data_logger_node.cpp](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/src/data_logger_node.cpp)，实现 50Hz 高频采集位置/速度误差、三轴角速度、比推力加速度、下发油门及估计悬停油门，自动保存为 CSV。
- **新增离线绘图工具**：新建 [scripts/plot_flight_data.py](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/scripts/plot_flight_data.py)，支持一键绘制 4 联对比图表（3D轨迹、位置误差时程、控制量、推力自适应收敛曲线）。
- **优化控制物理限幅机制**：修复 [src/mpc_wrapper.cpp](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/src/mpc_wrapper.cpp) 中 `acadoVariables` 物理限幅硬编码数值的问题，改为全面动态读取 YAML 约束参数，并修复参数路径容错。
- **解耦轨迹触发脚本**：新建 [launch/publish_circle_traj.launch](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/launch/publish_circle_traj.launch)，使实机悬停接管与圆轨迹开始时机完全可控。
- **建立开发分支**：创建并切换至 `dev-circle-mpc` 分支。

### [2026-08-14]
- **新增圆轨迹生成节点**：新建 [src/circle_traj_node.cpp](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/src/circle_traj_node.cpp)，支持按圆心、半径、速度和圈数参数化生成未来 20 步时域动力学参考轨迹。
- **实机安全接管逻辑改造**：在 [src/ros_mission.cpp](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/src/ros_mission.cpp) 中新增 `auto_arm_and_offboard: false` 参数，关闭收到里程计自动解锁起飞的危险逻辑；实现飞手遥控切入 OFFBOARD 后自动将当前位置设为目标点平滑接管。
- **修复 Yaw 角解算 Bug**：将 `ros_mission.cpp` 中原 `acos` 航向角解算修复为 `atan2`，解决下半圆飞行航向翻折问题。
- **清理动捕依赖**：删除 `launch/vrpn_pose_to_odom.launch`，规范化机载里程计话题接入接口。
- **新增机载端一键编译脚本**：新建 [scripts/pull_and_build.sh](file:///c:/Users/superglider/Downloads/quadrotor_mpc-master/quadrotor_mpc-master/quadrotor_mpc/scripts/pull_and_build.sh) 支持机载电脑端一键 `git pull` 与增量编译。
