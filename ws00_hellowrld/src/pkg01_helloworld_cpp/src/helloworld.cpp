#include <cstdio>
#include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("helloworld_cpp");

  RCLCPP_INFO(node->get_logger(), "Hello World, this is INFO!");
  RCLCPP_WARN(node->get_logger(), "Hello World, this is WARN!");


  rclcpp::shutdown();


  return 0;
}
