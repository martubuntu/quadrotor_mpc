#include "uav_mpc/ros_mission.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <vector>

namespace
{
constexpr double kGravity = 9.8066;
}

MPCRos::MPCRos(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  is_sim_ = node_->declare_parameter<bool>("is_sim", false);
  control_rate_hz_ = node_->declare_parameter<double>("control_rate_hz", 30.0);
  hover_thrust_ = node_->declare_parameter<double>("hover_thrust", 0.50);
  const double sim_hover_thrust = node_->declare_parameter<double>("sim_hover_thrust", 0.58);
  if (is_sim_) {
    hover_thrust_ = sim_hover_thrust;
  }
  thrust_min_ = node_->declare_parameter<double>("thrust_min", 0.15);
  thrust_max_ = node_->declare_parameter<double>("thrust_max", 0.85);
  body_rate_limit_ = node_->declare_parameter<double>("body_rate_limit", 2.0);
  adaptive_thrust_model_ = node_->declare_parameter<bool>("adaptive_thrust_model", false);
  use_eso_ = node_->declare_parameter<bool>("use_eso", false);
  eso_timeout_sec_ = node_->declare_parameter<double>("eso_timeout_sec", 0.2);
  eso_limit_ = node_->declare_parameter<double>("eso_limit", 3.0);
  trajectory_timeout_sec_ = node_->declare_parameter<double>("trajectory_timeout_sec", 0.5);
  prediction_dt_ = node_->declare_parameter<double>("prediction_dt", 0.1);
  frame_id_ = node_->declare_parameter<std::string>("rviz_frame_id", "map");
  auto_arm_ = node_->declare_parameter<bool>("auto_arm", is_sim_);
  auto_offboard_ = node_->declare_parameter<bool>("auto_offboard", is_sim_);
  takeoff_height_ = node_->declare_parameter<double>("takeoff_height", is_sim_ ? 1.5 : 0.0);

  // 3-Tier Flight Protections parameters
  thrust_rate_limit_step_ = node_->declare_parameter<double>("thrust_rate_limit_step", 0.30);
  max_ref_delta_z_ = node_->declare_parameter<double>("max_ref_delta_z", 0.50);
  max_ref_delta_xy_ = node_->declare_parameter<double>("max_ref_delta_xy", 0.80);
  offboard_hold_time_sec_ = node_->declare_parameter<double>("offboard_hold_time_sec", 2.0);

  const auto arming_service = node_->declare_parameter<std::string>(
    "mavros_arming_service", "/mavros/cmd/arming");
  const auto set_mode_service = node_->declare_parameter<std::string>(
    "mavros_set_mode_service", "/mavros/set_mode");

  if (auto_arm_ || auto_offboard_) {
    arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(arming_service);
    set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(set_mode_service);
    RCLCPP_INFO(
      node_->get_logger(),
      "[Mode: SIMULATION (is_sim=true)] Auto-Arm=%s, Auto-OFFBOARD=%s, Takeoff Height=%.2f m, Hover Thrust=%.2f",
      auto_arm_ ? "true" : "false", auto_offboard_ ? "true" : "false", takeoff_height_, hover_thrust_);
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "[Mode: REAL FLIGHT (is_sim=false)] Manual ARM/OFFBOARD required, Hover Thrust=%.2f",
      hover_thrust_);
  }

  const auto state_topic = node_->declare_parameter<std::string>(
    "mavros_state_topic", "/mavros/state");
  const auto odom_topic = node_->declare_parameter<std::string>(
    "mavros_odom_topic", "/mavros/local_position/odom");
  const auto imu_topic = node_->declare_parameter<std::string>(
    "mavros_imu_topic", "/mavros/imu/data");
  const auto command_topic = node_->declare_parameter<std::string>(
    "mavros_attitude_setpoint_topic", "/mavros/setpoint_raw/attitude");
  conflicting_setpoint_topic_ = node_->declare_parameter<std::string>(
    "conflicting_setpoint_topic", "/mavros/setpoint_raw/local");
  const auto trajectory_topic = node_->declare_parameter<std::string>(
    "trajectory_topic", "/mpc_ref_traj");
  const auto eso_topic = node_->declare_parameter<std::string>(
    "eso_topic", "/eso/disturbance");

  control_rate_hz_ = std::max(10.0, control_rate_hz_);
  thrust_min_ = std::clamp(thrust_min_, 0.0, 1.0);
  thrust_max_ = std::clamp(thrust_max_, thrust_min_, 1.0);
  hover_thrust_ = std::clamp(hover_thrust_, thrust_min_, thrust_max_);
  body_rate_limit_ = std::max(0.1, body_rate_limit_);
  prediction_dt_ = std::max(0.01, prediction_dt_);

  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
    state_topic, rclcpp::QoS(10),
    std::bind(&MPCRos::stateCallback, this, std::placeholders::_1));
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&MPCRos::odomCallback, this, std::placeholders::_1));
  trajectory_sub_ = node_->create_subscription<uav_mpc::msg::MpcRefTraj>(
    trajectory_topic, rclcpp::QoS(1),
    std::bind(&MPCRos::trajectoryCallback, this, std::placeholders::_1));
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&MPCRos::imuCallback, this, std::placeholders::_1));
  if (use_eso_) {
    eso_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      eso_topic, rclcpp::SensorDataQoS(),
      std::bind(&MPCRos::esoCallback, this, std::placeholders::_1));
  }

  command_pub_ = node_->create_publisher<mavros_msgs::msg::AttitudeTarget>(
    command_topic, rclcpp::QoS(10));
  mode_pub_ = node_->create_publisher<std_msgs::msg::Int8>("/mpc_debug/mode", 10);
  reference_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mpc_debug/ref_pose", 10);
  raw_control_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/mpc_debug/raw_control", 10);
  hover_thrust_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/mpc_debug/hover_thrust", 10);
  solve_time_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/mpc_debug/solve_time_ms", 10);
  actual_rate_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/mpc_debug/actual_rate_hz", 10);

  wrapper_ = std::make_unique<MPCWrapper>(node_);
  thrust_estimator_ = std::make_unique<ThrustEstimator>(hover_thrust_, kGravity);
}

void MPCRos::start()
{
  const auto period = std::chrono::milliseconds(
    static_cast<int>(1000.0 / std::max(1.0, control_rate_hz_)));
  control_timer_ = node_->create_wall_timer(period, std::bind(&MPCRos::controlTimer, this));
  RCLCPP_INFO(
    node_->get_logger(),
    "ROS 2 NMPC ready at %.1f Hz. Manual ARM/OFFBOARD required; ESO=%s.",
    control_rate_hz_, use_eso_ ? "enabled" : "disabled");
}

void MPCRos::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = *msg;
  if (!has_odom_) {
    hover_odom_ = *msg;
  }
  has_odom_ = true;
}

void MPCRos::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  current_state_ = *msg;
  has_state_ = true;
}

void MPCRos::trajectoryCallback(const uav_mpc::msg::MpcRefTraj::SharedPtr msg)
{
  trajectory_ = *msg;
  has_trajectory_ = !trajectory_.mpc_ref_points.empty();
  last_trajectory_stamp_ = node_->now();
}

void MPCRos::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!adaptive_thrust_model_ || !has_state_ || !current_state_.armed ||
    current_state_.mode != "OFFBOARD")
  {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp);
  thrust_estimator_->estimateThrustModel(stamp.seconds(), msg->linear_acceleration.z);
}

void MPCRos::esoCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  eso_disturbance_.x() = std::clamp(msg->vector.x, -eso_limit_, eso_limit_);
  eso_disturbance_.y() = std::clamp(msg->vector.y, -eso_limit_, eso_limit_);
  eso_disturbance_.z() = std::clamp(msg->vector.z, -eso_limit_, eso_limit_);
  last_eso_stamp_ = rclcpp::Time(msg->header.stamp);
}

void MPCRos::controlTimer()
{
  const auto now = node_->now();
  if (last_control_cycle_time_.nanoseconds() > 0) {
    const double dt_sec = (now - last_control_cycle_time_).seconds();
    if (dt_sec > 1e-4) {
      actual_loop_rate_hz_ = 1.0 / dt_sec;
    }
  }
  last_control_cycle_time_ = now;

  if (node_->count_publishers(conflicting_setpoint_topic_) > 0) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Another controller is publishing %s; NMPC output is inhibited.",
      conflicting_setpoint_topic_.c_str());
    return;
  }
  if (!has_odom_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "Waiting for MAVROS odometry.");
    return;
  }

  processAutoArmAndOffboard();

  const bool offboard_active =
    has_state_ && current_state_.connected && current_state_.armed &&
    current_state_.mode == "OFFBOARD";

  if (!solver_initialized_) {
    hover_odom_ = current_odom_;
    if (is_sim_ && takeoff_height_ > 0.1 && hover_odom_.pose.pose.position.z < (takeoff_height_ * 0.5)) {
      hover_odom_.pose.pose.position.z = takeoff_height_;
    }
    fillHoverReference();
    solver_initialized_ = wrapper_->initSolver(current_odom_);
    if (!solver_initialized_) {
      return;
    }
  }

  if (!offboard_active) {
    if (mode_ != Mode::WAITING) {
      RCLCPP_WARN(
        node_->get_logger(),
        "OFFBOARD or ARM lost. Stop mission control and wait for manual OFFBOARD again.");
    }
    mode_ = Mode::WAITING;
    has_trajectory_ = false;
    offboard_enter_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_specific_thrust_ = kGravity;
    hover_odom_ = current_odom_;
    if (is_sim_ && takeoff_height_ > 0.1 && hover_odom_.pose.pose.position.z < (takeoff_height_ * 0.5)) {
      hover_odom_.pose.pose.position.z = takeoff_height_;
    }
    fillHoverReference();
    wrapper_->initSolver(current_odom_);
    wrapper_->setReference(reference_);
    wrapper_->setDisturbance(Eigen::Vector3d::Zero(), false);
    publishControl(Eigen::Vector4f(kGravity, 0.0f, 0.0f, 0.0f), false);
    publishDebug();
    return;
  }

  if (mode_ == Mode::WAITING) {
    offboard_enter_stamp_ = now;
    last_specific_thrust_ = kGravity;
    hover_odom_ = current_odom_;
    if (is_sim_ && takeoff_height_ > 0.1 && hover_odom_.pose.pose.position.z < (takeoff_height_ * 0.5)) {
      hover_odom_.pose.pose.position.z = takeoff_height_;
      RCLCPP_INFO(
        node_->get_logger(),
        "Entering OFFBOARD. Simulation Phase 1 Auto-Takeoff initiated: target altitude = %.2f m.",
        takeoff_height_);
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "HOME LOCKED at current ENU position: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad",
        hover_odom_.pose.pose.position.x, hover_odom_.pose.pose.position.y,
        hover_odom_.pose.pose.position.z, currentYaw());
      RCLCPP_INFO(
        node_->get_logger(),
        "[Protection 3] Entering %.1fs level hover grace window (T=mg, rates=0)...",
        offboard_hold_time_sec_);
    }
    fillHoverReference();
    wrapper_->initSolver(current_odom_);
    mode_ = Mode::HOVER;
  }

  // 保护 3: 首次进入 Offboard 保持 2 秒平稳悬停 (T=mg, 零角速度)
  const double time_in_offboard = (now - offboard_enter_stamp_).seconds();
  if (time_in_offboard < offboard_hold_time_sec_) {
    hover_odom_ = current_odom_;
    fillHoverReference();
    wrapper_->initSolver(current_odom_);
    wrapper_->setReference(reference_);
    control_ = Eigen::Vector4f(kGravity, 0.0f, 0.0f, 0.0f);
    publishControl(control_, true);
    publishDebug();
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 500,
      "[Protection 3: Level Hover Hold] Holding T=mg for %.2f / %.2f s...",
      time_in_offboard, offboard_hold_time_sec_);
    return;
  }

  eso_valid_ = false;
  if (use_eso_ && last_eso_stamp_.nanoseconds() > 0) {
    const double age = (node_->now() - last_eso_stamp_).seconds();
    eso_valid_ = age >= 0.0 && age <= eso_timeout_sec_;
  }

  const bool trajectory_fresh = has_trajectory_ &&
    last_trajectory_stamp_.nanoseconds() > 0 &&
    (node_->now() - last_trajectory_stamp_).seconds() <= trajectory_timeout_sec_;
  if (trajectory_fresh) {
    mode_ = Mode::TRACKING;
    fillTrajectoryReference();
  } else {
    if (mode_ == Mode::TRACKING) {
      hover_odom_ = current_odom_;
      RCLCPP_WARN(node_->get_logger(), "Trajectory stream timed out; hover locked at current pose.");
    }
    has_trajectory_ = false;
    mode_ = Mode::HOVER;
    fillHoverReference();
  }
  wrapper_->setReference(reference_);

  wrapper_->setDisturbance(eso_disturbance_, eso_valid_);

  const auto t_start = std::chrono::steady_clock::now();
  const bool solve_ok = wrapper_->getSolution(current_odom_, control_);
  const auto t_end = std::chrono::steady_clock::now();
  last_solve_time_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  solve_time_sum_ms_ += last_solve_time_ms_;
  solve_time_max_ms_ = std::max(solve_time_max_ms_, last_solve_time_ms_);
  stat_cycle_count_++;

  if (last_stat_log_stamp_.nanoseconds() == 0 || (now - last_stat_log_stamp_).seconds() >= 1.0) {
    const double avg_solve_ms =
      stat_cycle_count_ > 0 ? (solve_time_sum_ms_ / stat_cycle_count_) : last_solve_time_ms_;
    const char * mode_str =
      (mode_ == Mode::TRACKING) ? "TRACKING" : (mode_ == Mode::HOVER ? "HOVER" : "WAITING");
    RCLCPP_INFO(
      node_->get_logger(),
      "[NMPC Rate] %5.1f Hz | Solve: %4.2f ms (avg %4.2f ms, max %4.2f ms) | Mode: %s",
      actual_loop_rate_hz_, last_solve_time_ms_, avg_solve_ms, solve_time_max_ms_, mode_str);

    solve_time_sum_ms_ = 0.0;
    solve_time_max_ms_ = 0.0;
    stat_cycle_count_ = 0;
    last_stat_log_stamp_ = now;
  }

  if (last_solve_time_ms_ > 25.0) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[NMPC High Latency] Solve took %.2f ms (budget %.1f ms at %.1f Hz)!",
      last_solve_time_ms_, 1000.0 / control_rate_hz_, control_rate_hz_);
  }

  if (!solve_ok) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "NMPC solve failed; commanding level hover and reinitializing solver.");
    control_ = Eigen::Vector4f(kGravity, 0.0f, 0.0f, 0.0f);
    hover_odom_ = current_odom_;
    has_trajectory_ = false;
    mode_ = Mode::HOVER;
    solver_initialized_ = wrapper_->initSolver(current_odom_);
  }

  publishControl(control_, true);
  publishDebug();
}

void MPCRos::fillHoverReference()
{
  const double curr_x = current_odom_.pose.pose.position.x;
  const double curr_y = current_odom_.pose.pose.position.y;
  const double curr_z = current_odom_.pose.pose.position.z;

  // 保护 2: 严格限制参考目标点相对当前实际位置的最大偏差 (误差限幅)
  const double ref_x = curr_x + std::clamp(hover_odom_.pose.pose.position.x - curr_x, -max_ref_delta_xy_, max_ref_delta_xy_);
  const double ref_y = curr_y + std::clamp(hover_odom_.pose.pose.position.y - curr_y, -max_ref_delta_xy_, max_ref_delta_xy_);
  const double ref_z = curr_z + std::clamp(hover_odom_.pose.pose.position.z - curr_z, -max_ref_delta_z_, max_ref_delta_z_);

  const Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(currentYaw(), Eigen::Vector3d::UnitZ()));
  for (int i = 0; i <= N; ++i) {
    reference_.col(i) <<
      ref_x, ref_y, ref_z,
      q_yaw.w(), q_yaw.x(), q_yaw.y(), q_yaw.z(),
      0.0, 0.0, 0.0, kGravity, 0.0, 0.0, 0.0;
  }
}

void MPCRos::fillTrajectoryReference()
{
  const std::size_t available = trajectory_.mpc_ref_points.size();
  if (available == 0) {
    fillHoverReference();
    return;
  }
  const double yaw = currentYaw();
  std::vector<Eigen::Quaterniond> quaternions(N + 1, Eigen::Quaterniond::Identity());
  Eigen::Quaterniond previous(
    current_odom_.pose.pose.orientation.w, current_odom_.pose.pose.orientation.x,
    current_odom_.pose.pose.orientation.y, current_odom_.pose.pose.orientation.z);
  if (previous.norm() < 1e-6) {
    previous = Eigen::Quaterniond::Identity();
  } else {
    previous.normalize();
  }

  for (int i = 0; i <= N; ++i) {
    const auto & point = trajectory_.mpc_ref_points[std::min<std::size_t>(i, available - 1)];
    const double curr_x = current_odom_.pose.pose.position.x;
    const double curr_y = current_odom_.pose.pose.position.y;
    const double curr_z = current_odom_.pose.pose.position.z;

    // 保护 2: 轨迹参考点误差限幅 (最大允许水平偏差 0.8m, 高度偏差 0.5m)
    const double ref_x = curr_x + std::clamp(point.position.x - curr_x, -max_ref_delta_xy_, max_ref_delta_xy_);
    const double ref_y = curr_y + std::clamp(point.position.y - curr_y, -max_ref_delta_xy_, max_ref_delta_xy_);
    const double ref_z = curr_z + std::clamp(point.position.z - curr_z, -max_ref_delta_z_, max_ref_delta_z_);

    Eigen::Vector3d acceleration(
      point.acceleration.x, point.acceleration.y, point.acceleration.z + kGravity);
    if (use_eso_ && eso_valid_) {
      acceleration -= eso_disturbance_;
    }
    Eigen::Quaterniond q = accelerationToQuaternion(acceleration, yaw);
    if (q.coeffs().dot(previous.coeffs()) < 0.0) {
      q.coeffs() *= -1.0;
    }
    reference_.col(i) <<
      ref_x, ref_y, ref_z,
      q.w(), q.x(), q.y(), q.z(),
      point.velocity.x, point.velocity.y, point.velocity.z,
      acceleration.norm(), 0.0, 0.0, 0.0;
    quaternions[i] = q;
    previous = q;
  }

  for (int i = 0; i < N; ++i) {
    Eigen::Quaterniond delta = quaternions[i].conjugate() * quaternions[i + 1];
    delta.normalize();
    if (delta.w() < 0.0) {
      delta.coeffs() *= -1.0;
    }
    const Eigen::AngleAxisd angle_axis(delta);
    const Eigen::Vector3d body_rate =
      angle_axis.axis() * angle_axis.angle() / std::max(1e-3, prediction_dt_);
    reference_(11, i) = body_rate.x();
    reference_(12, i) = body_rate.y();
    reference_(13, i) = body_rate.z();
  }
  reference_.block<3, 1>(11, N) = reference_.block<3, 1>(11, N - 1);
}

Eigen::Quaterniond MPCRos::accelerationToQuaternion(
  const Eigen::Vector3d & acceleration, double yaw) const
{
  Eigen::Vector3d z_body = acceleration;
  if (z_body.norm() < 1e-6) {
    z_body = Eigen::Vector3d::UnitZ();
  }
  z_body.normalize();
  const Eigen::Vector3d y_heading(-std::sin(yaw), std::cos(yaw), 0.0);
  Eigen::Vector3d x_body = y_heading.cross(z_body);
  if (x_body.norm() < 1e-6) {
    x_body = Eigen::Vector3d::UnitX();
  }
  x_body.normalize();
  const Eigen::Vector3d y_body = z_body.cross(x_body).normalized();
  Eigen::Matrix3d rotation;
  rotation.col(0) = x_body;
  rotation.col(1) = y_body;
  rotation.col(2) = z_body;
  return Eigen::Quaterniond(rotation).normalized();
}

void MPCRos::publishControl(Eigen::Vector4f control, bool active)
{
  if (!control.allFinite()) {
    control = Eigen::Vector4f(kGravity, 0.0f, 0.0f, 0.0f);
  }

  // 保护 1: 每周期推力爬升与变化率限幅 (Slew Rate Limiter, ±0.30 m/s² per cycle at 30Hz)
  if (last_specific_thrust_ > 1e-3) {
    control[0] = std::clamp<float>(
      control[0],
      static_cast<float>(last_specific_thrust_ - thrust_rate_limit_step_),
      static_cast<float>(last_specific_thrust_ + thrust_rate_limit_step_));
  }
  last_specific_thrust_ = control[0];

  const double specific_thrust = std::clamp<double>(control[0], 0.0, 30.0);
  double thrust = hover_thrust_ * specific_thrust / kGravity;
  if (adaptive_thrust_model_) {
    thrust = thrust_estimator_->computeDesiredThrust(specific_thrust);
  }
  thrust = std::clamp(thrust, thrust_min_, thrust_max_);
  if (active) {
    thrust_estimator_->pushThrustRecord(node_->now().seconds(), thrust);
  }

  mavros_msgs::msg::AttitudeTarget command;
  command.header.stamp = node_->now();
  command.header.frame_id = "base_link";
  command.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
  command.body_rate.x = std::clamp<double>(control[1], -body_rate_limit_, body_rate_limit_);
  command.body_rate.y = std::clamp<double>(control[2], -body_rate_limit_, body_rate_limit_);
  command.body_rate.z = std::clamp<double>(control[3], -body_rate_limit_, body_rate_limit_);
  command.thrust = static_cast<float>(thrust);
  command_pub_->publish(command);
  control_ = control;
}

void MPCRos::publishDebug()
{
  std_msgs::msg::Int8 mode;
  mode.data = static_cast<int8_t>(mode_);
  mode_pub_->publish(mode);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node_->now();
  pose.header.frame_id = frame_id_;
  pose.pose.position.x = reference_(0, 0);
  pose.pose.position.y = reference_(1, 0);
  pose.pose.position.z = reference_(2, 0);
  pose.pose.orientation.w = reference_(3, 0);
  pose.pose.orientation.x = reference_(4, 0);
  pose.pose.orientation.y = reference_(5, 0);
  pose.pose.orientation.z = reference_(6, 0);
  reference_pub_->publish(pose);

  std_msgs::msg::Float32MultiArray raw;
  raw.data.assign(control_.data(), control_.data() + control_.size());
  raw_control_pub_->publish(raw);

  std_msgs::msg::Float64 hover;
  hover.data = hover_thrust_;
  hover_thrust_pub_->publish(hover);

  std_msgs::msg::Float64 solve_time_msg;
  solve_time_msg.data = last_solve_time_ms_;
  solve_time_pub_->publish(solve_time_msg);

  std_msgs::msg::Float64 rate_msg;
  rate_msg.data = actual_loop_rate_hz_;
  actual_rate_pub_->publish(rate_msg);
}

double MPCRos::currentYaw() const
{
  const auto & q = hover_odom_.pose.pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void MPCRos::processAutoArmAndOffboard()
{
  if (!auto_arm_ && !auto_offboard_) {
    return;
  }
  if (!has_state_ || !current_state_.connected) {
    return;
  }

  const auto now = node_->now();
  if (node_start_stamp_.nanoseconds() == 0) {
    node_start_stamp_ = now;
    last_service_request_stamp_ = now;
    return;
  }

  // Pre-stream setpoints for at least 1.0 second before requesting OFFBOARD
  if ((now - node_start_stamp_).seconds() < 1.0) {
    return;
  }

  // Limit service requests to every 1.5 seconds
  if ((now - last_service_request_stamp_).seconds() < 1.5) {
    return;
  }
  last_service_request_stamp_ = now;

  if (auto_offboard_ && current_state_.mode != "OFFBOARD") {
    if (set_mode_client_ && set_mode_client_->service_is_ready()) {
      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = "OFFBOARD";
      set_mode_client_->async_send_request(
        req,
        [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
          const auto res = future.get();
          if (res->mode_sent) {
            RCLCPP_INFO(node_->get_logger(), "[Auto Mission] OFFBOARD mode request sent.");
          }
        });
    }
  } else if (auto_arm_ && !current_state_.armed && current_state_.mode == "OFFBOARD") {
    if (arming_client_ && arming_client_->service_is_ready()) {
      auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      req->value = true;
      arming_client_->async_send_request(
        req,
        [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
          const auto res = future.get();
          if (res->success) {
            RCLCPP_INFO(node_->get_logger(), "[Auto Mission] Vehicle armed successfully.");
          }
        });
    }
  }
}
