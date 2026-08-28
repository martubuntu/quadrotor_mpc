# ROS 2 real-flight NMPC port

This port keeps the ACADO/qpOASES solver and changes the ROS wrapper to ROS 2.
The first control input is collective specific thrust `a_T` in `m/s^2`; the
remaining inputs are body rates `[wx, wy, wz]` in `rad/s`.

## Safety defaults

- Arming and OFFBOARD switching are manual, matching `uav_pid_mavros`.
- The controller pre-streams level hover setpoints while waiting for OFFBOARD.
- The circle trajectory is disabled unless explicitly requested.
- ESO input and online thrust-model adaptation are disabled by default.
- A missing trajectory stream falls back to hover at the current pose.

Only one node may publish a flight-control setpoint. Do not run
`uav_pid_mavros` and `uav_mpc` controllers at the same time.

## Build on Jetson / Ubuntu 22.04 / ROS 2 Humble

```bash
mkdir -p ~/uav_ws/src
cp -r quadrotor_mpc ~/uav_ws/src/
cd ~/uav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Before building, make sure MAVROS for ROS 2 and Eigen3 are installed. The
generated ACADO solver is built from the checked-in C/C++ sources; ACADO itself
is not required on the Jetson unless the model is regenerated.

## Interface check without propellers

```bash
ros2 launch uav_mpc real_nmpc.launch.py
ros2 topic hz /mavros/local_position/odom
ros2 topic hz /mavros/setpoint_raw/attitude
ros2 topic echo /mavros/setpoint_raw/attitude --once
```

Expected before manual OFFBOARD:

- `type_mask` ignores attitude and uses body rates;
- body rates are zero;
- normalized thrust is close to configured `hover_thrust`;
- setpoint rate is close to 30 Hz.

## First hover flight

1. Set `hover_thrust` in `config/mpc_para.yaml` using a restrained/bench-tested
   value for the real aircraft.
2. Keep `start_trajectory:=false`, `use_eso:=false`, and
   `adaptive_thrust_model:=false`.
3. Start MAVROS and confirm FCU connection, odometry, attitude and IMU topics.
4. Start `real_nmpc.launch.py` and confirm setpoint streaming.
5. Arm and take off manually, establish a stable hover, then switch to OFFBOARD.
6. Be ready to switch back to a manual/position mode immediately.

## Low-speed circle test

After hover-only NMPC is verified:

```bash
ros2 launch uav_mpc real_nmpc.launch.py start_trajectory:=true
```

The default circle is deliberately slow (`0.20 m/s`) and uses an 8-second
quintic transition. To stop tracking, stop the circle node; after 0.5 seconds
the controller locks a hover reference at the current pose.

`use_eso:=true` only enables subscription to `/eso/disturbance`; it does not
start an ESO executable in this first-stage port.
