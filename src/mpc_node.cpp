#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "uav_mpc/ros_mission.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("uav_mpc_node");
  auto controller = std::make_shared<MPCRos>(node);
  controller->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
