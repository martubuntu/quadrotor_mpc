# ROS 2 NMPC (Real-Flight & Gazebo SITL Simulation)

This package contains the ROS 2 Humble NMPC control stack for quadrotors based on ACADO Toolkit and qpOASES.
The first control input is collective specific thrust $a_T$ in $\text{m/s}^2$; the remaining inputs are body angular rates $[\omega_x, \omega_y, \omega_z]$ in $\text{rad/s}$.

---

## 1. Unified Launch Architecture (`real_nmpc.launch.py`)

A single launch file supports both **Real Aircraft** and **Gazebo SITL Simulation** via the `is_sim` switch:

| Mode | Command | Key Behaviors |
| :--- | :--- | :--- |
| **Real Flight** | `ros2 launch uav_mpc real_nmpc.launch.py` | Manual arm & takeoff $\to$ pilot flips switch to OFFBOARD in air $\to$ hover locked. (`is_sim:=false` by default) |
| **Gazebo Sim** | `ros2 launch uav_mpc real_nmpc.launch.py is_sim:=true` | Auto-arm $\to$ auto-OFFBOARD $\to$ Phase 1 auto-takeoff to 1.5m $\to$ Phase 2 circle trajectory. |

*(Shortcut for simulation: `ros2 launch uav_mpc sim_nmpc.launch.py`)*

---

## 2. Real Flight Operation (Jetson Nano / 5 kg Platform)

### 2.1 Build on Jetson (Ubuntu 22.04 / ROS 2 Humble)
```bash
cd ~/Desktop/uav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2.2 Ground Check (Without Propellers)
```bash
# Terminal 1: MAVROS
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600

# Terminal 2: NMPC Controller
source ~/Desktop/uav_ws/install/setup.bash
ros2 launch uav_mpc real_nmpc.launch.py
```
* Expected terminal output: `[NMPC Rate] ~30.0 Hz | Solve: 1.5~2.5 ms | Mode: WAITING`

### 2.3 Phase 1: Real-Flight Hover Test
```bash
ros2 launch uav_mpc real_nmpc.launch.py
```
1. Set `hover_thrust` in `config/mpc_para.yaml` (default `0.50` for 5 kg aircraft).
2. Take off manually in Position/Altitude mode to ~1.5 m.
3. Switch RC flight mode channel to **OFFBOARD**.
4. NMPC locks hover at the current aerial pose.

### 2.4 Phase 2: Real-Flight Circle Trajectory Test
```bash
ros2 launch uav_mpc real_nmpc.launch.py start_trajectory:=true
```
* Optional runtime overrides:
```bash
ros2 launch uav_mpc real_nmpc.launch.py start_trajectory:=true radius:=2.0 linear_speed:=0.30
```

---

## 3. Gazebo SITL Simulation Operation

### 3.1 Start PX4 SITL & MAVROS on PC
```bash
# Terminal 1: PX4 SITL + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic

# Terminal 2: MAVROS for simulation
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 3.2 Run NMPC Simulation (Full Phase 1 $\to$ Phase 2 Sequence)
```bash
ros2 launch uav_mpc real_nmpc.launch.py is_sim:=true start_trajectory:=true
# Or simply:
ros2 launch uav_mpc sim_nmpc.launch.py
```

### 3.3 Simulation Sequence & Mode Transitions
1. **Phase 1 (Auto Takeoff & Hover)**:
   - Node auto-streams setpoints, switches mode to `OFFBOARD`, and sends `ARM` command.
   - NMPC climbs smoothly from ground $(0, 0, 0)$ to `takeoff_height` (default `1.5 m`).
   - Stabilizes and hovers for `start_delay_sec` (default `6.0 s`).
2. **Phase 2 (Circle Trajectory Tracking)**:
   - After `start_delay_sec` and altitude reached, `circle_traj_node` automatically activates.
   - Smooth 5th-order polynomial transition guides the drone into a `1.5 m` radius circle at `0.30 m/s`.
   - On trajectory completion, the drone safely holds hover at the final waypoint.

### 3.4 Simulation Hover-Only Test
```bash
ros2 launch uav_mpc real_nmpc.launch.py is_sim:=true start_trajectory:=false
```

---

## 4. Parameter Overview

* `config/mpc_para.yaml`: Real-flight parameters (`hover_thrust: 0.50`, manual switching).
* `config/mpc_simulation.yaml`: Simulation parameters (`hover_thrust: 0.58`, `takeoff_height: 1.5`, `start_delay_sec: 6.0`, auto arm/offboard).

### Launch Arguments Table

| Argument | Default (Real) | Default (Sim) | Description |
| :--- | :--- | :--- | :--- |
| `is_sim` | `false` | `true` | Toggles simulation mode vs real-flight mode |
| `start_trajectory` | `false` | `true` | Starts circle trajectory generator |
| `takeoff_height` | `1.5` | `1.5` | Target takeoff/hover height in meters |
| `start_delay_sec` | `6.0` | `6.0` | Hover duration before Phase 2 trajectory starts |
| `radius` | `1.5` | `1.5` | Circle trajectory radius in meters |
| `linear_speed` | `0.20` | `0.30` | Trajectory speed in m/s |
| `use_eso` | `false` | `false` | Subscribe to `/eso/disturbance` topic |
