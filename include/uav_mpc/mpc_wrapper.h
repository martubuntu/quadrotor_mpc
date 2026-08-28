#pragma once

#include <memory>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

#define NX ACADO_NX
#define NU ACADO_NU
#define NOD ACADO_NOD
#define NY ACADO_NY
#define NYN ACADO_NYN
#define N ACADO_N

class MPCWrapper
{
public:
  explicit MPCWrapper(const rclcpp::Node::SharedPtr & node);

  bool initSolver(const nav_msgs::msg::Odometry & odom);
  void setReference(const Eigen::MatrixXd & reference);
  void setDisturbance(const Eigen::Vector3d & disturbance, bool valid);
  bool getSolution(const nav_msgs::msg::Odometry & odom, Eigen::Vector4f & control);

private:
  void updateState(const nav_msgs::msg::Odometry & odom);
  void updateOnlineData();

  rclcpp::Node::SharedPtr node_;

  double cost_px_{};
  double cost_py_{};
  double cost_pz_{};
  double cost_qw_{};
  double cost_qx_{};
  double cost_qy_{};
  double cost_qz_{};
  double cost_vx_{};
  double cost_vy_{};
  double cost_vz_{};
  double cost_at_{};
  double cost_wx_{};
  double cost_wy_{};
  double cost_wz_{};

  double at_max_{};
  double at_min_{};
  double wx_max_{};
  double wx_min_{};
  double wy_max_{};
  double wy_min_{};
  double wz_max_{};
  double wz_min_{};

  Eigen::Vector3d disturbance_{Eigen::Vector3d::Zero()};
  bool disturbance_valid_{false};
};
