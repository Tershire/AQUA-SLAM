#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "System.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("aqua_slam_node");
  
  RCLCPP_INFO(node->get_logger(), "[AQUA-SLAM] ROS 2 node started.");
  
  // create a system here later and run a loop.
  
  rclcpp::shutdown();
  return 0;
}
