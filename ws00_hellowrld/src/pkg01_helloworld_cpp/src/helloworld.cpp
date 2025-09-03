#include <cstdio>
#include "rclcpp/rclcpp.hpp"

class MyNode :public rclcpp::Node{
  public:
    MyNode():Node("Class_NOde"){
      RCLCPP_INFO(this->get_logger(),"Hello World MyNode Class");
    }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  // 最好不要混着用，实例化和继承
  // auto node = rclcpp::Node::make_shared("helloworld_cpp");

  auto node_class = std::make_shared<MyNode>();

  // RCLCPP_INFO(node->get_logger(), "Hello World, this is INFO!");
  // RCLCPP_WARN(node->get_logger(), "Hello World, this is WARN!");


  rclcpp::shutdown();


  return 0;
}
