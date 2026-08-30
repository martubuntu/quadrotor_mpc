#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "uav_mpc/msg/mpc_ref_point.hpp"
#include "uav_mpc/msg/mpc_ref_traj.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Quintic1D
{
  double c0{}, c1{}, c2{}, c3{}, c4{}, c5{};

  void compute(double x0, double v0, double a0, double xf, double vf, double af, double duration)
  {
    c0 = x0; c1 = v0; c2 = 0.5 * a0;
    const double t2 = duration * duration;
    const double t3 = t2 * duration;
    const double t4 = t3 * duration;
    const double t5 = t4 * duration;
    const double h = xf - x0 - v0 * duration - 0.5 * a0 * t2;
    const double dv = vf - v0 - a0 * duration;
    const double da = af - a0;
    c3 = (10.0 * h - 4.0 * dv * duration + 0.5 * da * t2) / t3;
    c4 = (-15.0 * h + 7.0 * dv * duration - da * t2) / t4;
    c5 = (6.0 * h - 3.0 * dv * duration + 0.5 * da * t2) / t5;
  }

  void evaluate(double t, double & x, double & v, double & a) const
  {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    x = c0 + c1*t + c2*t2 + c3*t3 + c4*t4 + c5*t5;
    v = c1 + 2*c2*t + 3*c3*t2 + 4*c4*t3 + 5*c5*t4;
    a = 2*c2 + 6*c3*t + 12*c4*t2 + 20*c5*t3;
  }
};
}

class CircleTrajectoryNode : public rclcpp::Node
{
public:
  CircleTrajectoryNode()
  : Node("circle_traj_node")
  {
    radius_ = declare_parameter<double>("radius", 1.5);
    speed_ = declare_parameter<double>("linear_speed", 0.3);
    height_ = declare_parameter<double>("height", 0.0);
    cycles_ = declare_parameter<int>("cycles", 1);
    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);
    horizon_steps_ = declare_parameter<int>("horizon_steps", 20);
    horizon_dt_ = declare_parameter<double>("horizon_dt", 0.1);
    transition_time_ = declare_parameter<double>("transition_time", 5.0);
    start_delay_sec_ = declare_parameter<double>("start_delay_sec", 0.0);
    const auto odom_topic = declare_parameter<std::string>(
      "mavros_odom_topic", "/mavros/local_position/odom");
    const auto state_topic = declare_parameter<std::string>(
      "mavros_state_topic", "/mavros/state");
    const auto trajectory_topic = declare_parameter<std::string>(
      "trajectory_topic", "/mpc_ref_traj");

    radius_ = std::max(0.1, radius_);
    speed_ = std::max(0.05, speed_);
    transition_time_ = std::max(1.0, transition_time_);
    omega_ = speed_ / radius_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!center_locked_ && offboard_active_) {
          if (start_delay_sec_ > 0.0) {
            const double elapsed = (now() - offboard_start_stamp_).seconds();
            if (elapsed < start_delay_sec_) {
              return;
            }
          }
          if (height_ > 0.1 && msg->pose.pose.position.z < (height_ - 0.35)) {
            return;
          }
          center_.x() = msg->pose.pose.position.x;
          center_.y() = msg->pose.pose.position.y;
          center_.z() = height_ > 0.0 ? height_ : msg->pose.pose.position.z;
          initializeTransition();
          center_locked_ = true;
          start_time_ = now();
          RCLCPP_INFO(
            get_logger(),
            "Phase 2 Circle Trajectory Activated: Center=(%.2f, %.2f, %.2f), Radius=%.2fm, Speed=%.2fm/s",
            center_.x(), center_.y(), center_.z(), radius_, speed_);
        }
      });
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      state_topic, rclcpp::QoS(10),
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        const bool active = msg->connected && msg->armed && msg->mode == "OFFBOARD";
        if (active && !offboard_active_) {
          offboard_start_stamp_ = now();
        }
        offboard_active_ = active;
      });
    trajectory_pub_ = create_publisher<uav_mpc::msg::MpcRefTraj>(trajectory_topic, 1);
    debug_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/circle_traj/current_ref", 10);

    const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1.0, publish_rate_)));
    timer_ = create_wall_timer(period, std::bind(&CircleTrajectoryNode::publishTrajectory, this));
  }

private:
  void initializeTransition()
  {
    poly_x_.compute(
      center_.x(), 0.0, 0.0, center_.x() + radius_, 0.0,
      -radius_ * omega_ * omega_, transition_time_);
    poly_y_.compute(
      center_.y(), 0.0, 0.0, center_.y(), radius_ * omega_, 0.0, transition_time_);
    poly_z_.compute(center_.z(), 0.0, 0.0, center_.z(), 0.0, 0.0, transition_time_);
  }

  void evaluate(double t, Eigen::Vector3d & p, Eigen::Vector3d & v, Eigen::Vector3d & a) const
  {
    const double circle_duration = cycles_ > 0 ? 2.0 * kPi * cycles_ / omega_ : 1e9;
    if (t < transition_time_) {
      poly_x_.evaluate(t, p.x(), v.x(), a.x());
      poly_y_.evaluate(t, p.y(), v.y(), a.y());
      poly_z_.evaluate(t, p.z(), v.z(), a.z());
      return;
    }
    const double circle_time = std::min(t - transition_time_, circle_duration);
    const double theta = omega_ * circle_time;
    p << center_.x() + radius_ * std::cos(theta),
      center_.y() + radius_ * std::sin(theta), center_.z();
    if (circle_time >= circle_duration) {
      v.setZero(); a.setZero();
      return;
    }
    v << -radius_ * omega_ * std::sin(theta),
      radius_ * omega_ * std::cos(theta), 0.0;
    a << -radius_ * omega_ * omega_ * std::cos(theta),
      -radius_ * omega_ * omega_ * std::sin(theta), 0.0;
  }

  void publishTrajectory()
  {
    if (!center_locked_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for odometry before starting trajectory.");
      return;
    }
    const double active_time = (now() - start_time_).seconds();
    uav_mpc::msg::MpcRefTraj trajectory;
    trajectory.mpc_ref_points.reserve(horizon_steps_ + 1);
    for (int i = 0; i <= horizon_steps_; ++i) {
      Eigen::Vector3d p, v, a;
      evaluate(active_time + i * horizon_dt_, p, v, a);
      uav_mpc::msg::MpcRefPoint point;
      point.position.x = p.x(); point.position.y = p.y(); point.position.z = p.z();
      point.velocity.x = v.x(); point.velocity.y = v.y(); point.velocity.z = v.z();
      point.acceleration.x = a.x(); point.acceleration.y = a.y(); point.acceleration.z = a.z();
      trajectory.mpc_ref_points.push_back(point);
    }
    const double end_time = transition_time_ + 2.0 * kPi * std::max(0, cycles_) / omega_;
    if (cycles_ > 0 && active_time >= end_time) {
      trajectory.goal.x = center_.x() + radius_;
      trajectory.goal.y = center_.y();
      trajectory.goal.z = center_.z();
    } else {
      trajectory.goal.x = trajectory.goal.y = trajectory.goal.z = 9999.0;
    }
    trajectory_pub_->publish(trajectory);

    geometry_msgs::msg::PoseStamped debug;
    debug.header.stamp = now();
    debug.header.frame_id = "map";
    debug.pose.position.x = trajectory.mpc_ref_points.front().position.x;
    debug.pose.position.y = trajectory.mpc_ref_points.front().position.y;
    debug.pose.position.z = trajectory.mpc_ref_points.front().position.z;
    debug.pose.orientation.w = 1.0;
    debug_pub_->publish(debug);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<uav_mpc::msg::MpcRefTraj>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Vector3d center_{Eigen::Vector3d::Zero()};
  Quintic1D poly_x_, poly_y_, poly_z_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time offboard_start_stamp_{0, 0, RCL_ROS_TIME};
  bool center_locked_{false};
  bool offboard_active_{false};
  double radius_{1.5};
  double speed_{0.3};
  double height_{0.0};
  double omega_{0.2};
  double publish_rate_{30.0};
  double horizon_dt_{0.1};
  double transition_time_{5.0};
  double start_delay_sec_{0.0};
  int cycles_{1};
  int horizon_steps_{20};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
