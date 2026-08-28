#include "uav_mpc/mpc_wrapper.h"

#include <algorithm>
#include <cstring>

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

MPCWrapper::MPCWrapper(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  cost_px_ = node_->declare_parameter<double>("cost.px", 100.0);
  cost_py_ = node_->declare_parameter<double>("cost.py", 100.0);
  cost_pz_ = node_->declare_parameter<double>("cost.pz", 100.0);
  cost_qw_ = node_->declare_parameter<double>("cost.qw", 100.0);
  cost_qx_ = node_->declare_parameter<double>("cost.qx", 100.0);
  cost_qy_ = node_->declare_parameter<double>("cost.qy", 100.0);
  cost_qz_ = node_->declare_parameter<double>("cost.qz", 100.0);
  cost_vx_ = node_->declare_parameter<double>("cost.vx", 30.0);
  cost_vy_ = node_->declare_parameter<double>("cost.vy", 30.0);
  cost_vz_ = node_->declare_parameter<double>("cost.vz", 30.0);
  cost_at_ = node_->declare_parameter<double>("cost.specific_thrust", 30.0);
  cost_wx_ = node_->declare_parameter<double>("cost.wx", 30.0);
  cost_wy_ = node_->declare_parameter<double>("cost.wy", 30.0);
  cost_wz_ = node_->declare_parameter<double>("cost.wz", 30.0);

  at_max_ = node_->declare_parameter<double>("bounds.specific_thrust_max", 20.0);
  at_min_ = node_->declare_parameter<double>("bounds.specific_thrust_min", 2.0);
  wx_max_ = node_->declare_parameter<double>("bounds.wx_max", 2.0);
  wx_min_ = node_->declare_parameter<double>("bounds.wx_min", -2.0);
  wy_max_ = node_->declare_parameter<double>("bounds.wy_max", 2.0);
  wy_min_ = node_->declare_parameter<double>("bounds.wy_min", -2.0);
  wz_max_ = node_->declare_parameter<double>("bounds.wz_max", 2.0);
  wz_min_ = node_->declare_parameter<double>("bounds.wz_min", -2.0);
}

bool MPCWrapper::initSolver(const nav_msgs::msg::Odometry & odom)
{
  Eigen::Quaterniond q(
    odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  if (q.norm() < 1e-6) {
    return false;
  }
  q.normalize();

  const Eigen::Vector3d v_body(
    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
  const Eigen::Vector3d v_world = q * v_body;

  std::memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  std::memset(&acadoVariables, 0, sizeof(acadoVariables));
  acado_initializeSolver();

  for (int i = 0; i <= N; ++i) {
    real_t * x = &acadoVariables.x[i * NX];
    x[0] = odom.pose.pose.position.x;
    x[1] = odom.pose.pose.position.y;
    x[2] = odom.pose.pose.position.z;
    x[3] = q.w(); x[4] = q.x(); x[5] = q.y(); x[6] = q.z();
    x[7] = v_world.x(); x[8] = v_world.y(); x[9] = v_world.z();
  }
  std::copy_n(acadoVariables.x, NX, acadoVariables.x0);
  for (int i = 0; i < N; ++i) {
    acadoVariables.u[i * NU] = 9.8066;
    acadoVariables.u[i * NU + 1] = 0.0;
    acadoVariables.u[i * NU + 2] = 0.0;
    acadoVariables.u[i * NU + 3] = 0.0;
  }

  const double weights[NY] = {
    cost_px_, cost_py_, cost_pz_, cost_qw_, cost_qx_, cost_qy_, cost_qz_,
    cost_vx_, cost_vy_, cost_vz_, cost_at_, cost_wx_, cost_wy_, cost_wz_};
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < NY; ++j) {
      acadoVariables.W[i * NY * NY + j * NY + j] = weights[j];
    }
  }
  for (int j = 0; j < NYN; ++j) {
    acadoVariables.WN[j * NYN + j] = weights[j];
  }

  // Seed the first RTI preparation with a physically valid hover reference.
  const double hover_reference[NY] = {
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
    q.w(), q.x(), q.y(), q.z(), 0.0, 0.0, 0.0, 9.8066, 0.0, 0.0, 0.0};
  for (int i = 0; i < N; ++i) {
    std::copy_n(hover_reference, NY, &acadoVariables.y[i * NY]);
  }
  std::copy_n(hover_reference, NYN, acadoVariables.yN);

  for (int i = 0; i < N; ++i) {
    acadoVariables.lbValues[i * NU] = at_min_;
    acadoVariables.ubValues[i * NU] = at_max_;
    acadoVariables.lbValues[i * NU + 1] = wx_min_;
    acadoVariables.ubValues[i * NU + 1] = wx_max_;
    acadoVariables.lbValues[i * NU + 2] = wy_min_;
    acadoVariables.ubValues[i * NU + 2] = wy_max_;
    acadoVariables.lbValues[i * NU + 3] = wz_min_;
    acadoVariables.ubValues[i * NU + 3] = wz_max_;
  }

  updateOnlineData();
  acado_preparationStep();
  return true;
}

void MPCWrapper::setReference(const Eigen::MatrixXd & reference)
{
  if (reference.rows() != NY || reference.cols() < N + 1) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Reference must be %d x %d or larger.", NY, N + 1);
    return;
  }
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < NY; ++j) {
      acadoVariables.y[i * NY + j] = reference(j, i);
    }
  }
  for (int j = 0; j < NYN; ++j) {
    acadoVariables.yN[j] = reference(j, N);
  }
}

void MPCWrapper::setDisturbance(const Eigen::Vector3d & disturbance, bool valid)
{
  disturbance_ = valid ? disturbance : Eigen::Vector3d::Zero();
  disturbance_valid_ = valid;
}

void MPCWrapper::updateOnlineData()
{
  const Eigen::Vector3d d = disturbance_valid_ ? disturbance_ : Eigen::Vector3d::Zero();
  for (int i = 0; i <= N; ++i) {
    for (int j = 0; j < NOD; ++j) {
      acadoVariables.od[i * NOD + j] = 0.0;
    }
    if (NOD >= 3) {
      acadoVariables.od[i * NOD] = d.x();
      acadoVariables.od[i * NOD + 1] = d.y();
      acadoVariables.od[i * NOD + 2] = d.z();
    }
  }
}

void MPCWrapper::updateState(const nav_msgs::msg::Odometry & odom)
{
  acadoVariables.x0[0] = odom.pose.pose.position.x;
  acadoVariables.x0[1] = odom.pose.pose.position.y;
  acadoVariables.x0[2] = odom.pose.pose.position.z;

  Eigen::Quaterniond q(
    odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  q.normalize();
  Eigen::Vector4d q_current(q.w(), q.x(), q.y(), q.z());
  const Eigen::Vector4d q_ref(
    acadoVariables.y[3], acadoVariables.y[4], acadoVariables.y[5], acadoVariables.y[6]);
  if (q_current.dot(q_ref) < 0.0) {
    q_current = -q_current;
  }
  for (int i = 0; i < 4; ++i) {
    acadoVariables.x0[3 + i] = q_current[i];
  }

  const Eigen::Vector3d v_body(
    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
  const Eigen::Vector3d v_world = q * v_body;
  acadoVariables.x0[7] = v_world.x();
  acadoVariables.x0[8] = v_world.y();
  acadoVariables.x0[9] = v_world.z();
}

bool MPCWrapper::getSolution(
  const nav_msgs::msg::Odometry & odom, Eigen::Vector4f & control)
{
  updateState(odom);
  updateOnlineData();
  const int status = acado_feedbackStep();
  if (status != 0) {
    return false;
  }
  const real_t * u = acado_getVariablesU();
  for (int i = 0; i < 4; ++i) {
    control[i] = static_cast<float>(u[i]);
  }
  acado_preparationStep();
  return control.allFinite();
}
