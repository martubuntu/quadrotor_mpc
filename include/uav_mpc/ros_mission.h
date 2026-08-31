#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>

#include "uav_mpc/mpc_wrapper.h"
#include "uav_mpc/thrust_estimator.h"
#include "uav_mpc/msg/mpc_ref_traj.hpp"

class MPCRos
{
public:
  explicit MPCRos(const rclcpp::Node::SharedPtr & node);
  void start();

private:
  enum class Mode : int8_t {WAITING = -1, HOVER = 1, TRACKING = 2};

  void controlTimer();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
  void trajectoryCallback(const uav_mpc::msg::MpcRefTraj::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void esoCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  void fillHoverReference();
  void fillTrajectoryReference();
  Eigen::Quaterniond accelerationToQuaternion(
    const Eigen::Vector3d & acceleration, double yaw) const;
  void publishControl(Eigen::Vector4f control, bool active);
  void publishDebug();
  void processAutoArmAndOffboard();
  double currentYaw() const;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<MPCWrapper> wrapper_;
  std::unique_ptr<ThrustEstimator> thrust_estimator_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<uav_mpc::msg::MpcRefTraj>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr eso_sub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr raw_control_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hover_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr solve_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr actual_rate_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::Odometry hover_odom_;
  mavros_msgs::msg::State current_state_;
  uav_mpc::msg::MpcRefTraj trajectory_;
  Eigen::MatrixXd reference_{Eigen::MatrixXd::Zero(NY, N + 1)};
  Eigen::Vector4f control_{Eigen::Vector4f::Zero()};
  Eigen::Vector3d eso_disturbance_{Eigen::Vector3d::Zero()};

  bool has_odom_{false};
  bool has_state_{false};
  bool has_trajectory_{false};
  bool solver_initialized_{false};
  bool use_eso_{false};
  bool eso_valid_{false};
  bool adaptive_thrust_model_{false};
  bool is_sim_{false};
  bool auto_arm_{false};
  bool auto_offboard_{false};
  double takeoff_height_{0.0};
  Mode mode_{Mode::WAITING};
  rclcpp::Time last_eso_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trajectory_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time node_start_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_service_request_stamp_{0, 0, RCL_ROS_TIME};

  double control_rate_hz_{30.0};
  double hover_thrust_{0.5};
  double thrust_min_{0.15};
  double thrust_max_{0.80};
  double body_rate_limit_{2.0};
  double eso_timeout_sec_{0.2};
  double eso_limit_{3.0};
  double trajectory_timeout_sec_{0.5};
  double prediction_dt_{0.1};
  std::string frame_id_{"map"};
  std::string conflicting_setpoint_topic_{"/mavros/setpoint_raw/local"};

  // Performance telemetry
  rclcpp::Time last_control_cycle_time_{0, 0, RCL_ROS_TIME};
  double last_solve_time_ms_{0.0};
  double actual_loop_rate_hz_{0.0};
  double solve_time_sum_ms_{0.0};
  double solve_time_max_ms_{0.0};
  int stat_cycle_count_{0};
  rclcpp::Time last_stat_log_stamp_{0, 0, RCL_ROS_TIME};
};
