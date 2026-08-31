# ROS 2 NMPC Control Stack (Real-Flight & Gazebo SITL)

This package implements an ACADO/qpOASES NMPC controller for quadrotors under ROS 2 Humble.
The first control input is collective specific thrust $a_T$ in $\text{m/s}^2$; the remaining inputs are body rates $[w_x, w_y, w_z]$ in $\text{rad/s}$.

---

## 1. Unified Launch Architecture & Decoupled Options

A single unified launch structure is used for both **Real Flight** and **Gazebo SITL Simulation**.
The environment and mode-switching logic are decoupled from trajectory and estimator options via the `is_sim` switch.

### Common Options (Identical across Real-Flight & Simulation)
| Parameter | Default (Real) | Default (Sim) | Description |
| :--- | :--- | :--- | :--- |
| `is_sim` | `false` | `true` | **Master Switch**: enables simulation clock (`use_sim_time`), auto-arm, auto-offboard, and auto-takeoff. |
| `start_trajectory` | `false` | `true` | `false`: Hover only; `true`: Phase 2 circle trajectory tracking. |
| `use_eso` | `false` | `false` | Enable subscription to `/eso/disturbance` and online feedforward. |
| `radius` | `1.5` | `1.5` | Circle trajectory radius in meters. |
| `linear_speed` | `0.20` | `0.30` | Circle trajectory cruising speed in m/s. |
| `takeoff_height` | `0.0` (ignored) | `1.5` | Target takeoff/hover altitude in meters (active when `is_sim:=true`). |
| `start_delay_sec` | `0.0` | `6.0` | Hover duration before transitioning from Phase 1 (Takeoff) to Phase 2 (Trajectory). |

---

## 2. Build Instructions (Jetson / Ubuntu 22.04 / ROS 2 Humble)

```bash
cd ~/Desktop/uav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Or run the one-click sync script:
```bash
~/Desktop/uav_ws/src/quadrotor_mpc/scripts/sync_and_build.sh
```

---

## 3. Real-Flight Operation (Manual Safety Mode: `is_sim:=false`)

Real flight uses `config/mpc_para.yaml` (calibrated for the 5 kg platform, `hover_thrust: 0.50`).

### Step 1: Start MAVROS (Terminal 1)
```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```

### Step 2: Hover Flight Validation (Terminal 2)
```bash
ros2 launch uav_mpc real_nmpc.launch.py
```
* **Flight Procedure**:
  1. Manually arm and take off in Position/Manual mode to 1.0~1.5m stable hover.
  2. Switch RC switch to **OFFBOARD**.
  3. NMPC locks hover reference at current pose. Flip switch back to Position/Manual at any time to regain manual control.

### Step 3: Real-Flight Circle Trajectory
```bash
ros2 launch uav_mpc real_nmpc.launch.py start_trajectory:=true
```
* Once in OFFBOARD hover, the aircraft smooths into the 1.5m circle at 0.20 m/s with quintic transition.

---

## 4. Gazebo SITL Simulation (Autonomous Mode: `is_sim:=true`)

Simulation uses `config/mpc_simulation.yaml` (`hover_thrust: 0.58`, `auto_arm: true`, `auto_offboard: true`).

### Autonomous 2-Phase Mission (Takeoff to 1.5m -> Hover 6s -> Circle)
```bash
# Option A (via dedicated sim launch):
ros2 launch uav_mpc sim_nmpc.launch.py

# Option B (via unified launch):
ros2 launch uav_mpc real_nmpc.launch.py is_sim:=true start_trajectory:=true
```

### Simulation Hover Only (Phase 1 Only)
```bash
ros2 launch uav_mpc sim_nmpc.launch.py start_trajectory:=false
```

### Customizing Trajectory Parameters on the Fly
```bash
ros2 launch uav_mpc sim_nmpc.launch.py \
  takeoff_height:=2.0 \
  start_delay_sec:=5.0 \
  radius:=2.0 \
  linear_speed:=0.50
```

---

## 5. Performance & Telemetry Diagnostics

The controller outputs real-time solve performance every 1.0 second:
```text
[NMPC Rate]  30.0 Hz | Solve: 1.65 ms (avg 1.62 ms, max 2.80 ms) | Mode: HOVER
```
Debug topics:
- `/mpc_debug/solve_time_ms`: QP solve duration in milliseconds.
- `/mpc_debug/actual_rate_hz`: Actual control loop frequency.
- `/mpc_debug/mode`: Control mode (`-1`: Waiting/Pre-stream, `1`: Hover, `2`: Tracking).
- `/mavros/battery`: 10 Hz battery voltage and discharge current telemetry.
