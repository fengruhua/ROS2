#include <cstdio>
#include <string>
#include "rclcpp/rclcpp.hpp"

class Param_demo00 : public rclcpp::Node
{
private:
  /* data */
public:
  Param_demo00(/* args */);
  ~Param_demo00();
};

Param_demo00::Param_demo00(/* args */):Node("Param_demo00")
{
  RCLCPP_INFO(this->get_logger(),"参数服务器api使用");

  rclcpp::Parameter p1("car_name", "Tiger");
  rclcpp::Parameter p2("height", 3.3);
  rclcpp::Parameter p3("wheels", 4);

  RCLCPP_INFO(this->get_logger(), "car_name = %s", p1.as_string().c_str());
  RCLCPP_INFO(this->get_logger(), "height = %f", p2.as_double());
  RCLCPP_INFO(this->get_logger(), "wheels = %ld", p3.as_int());

  RCLCPP_INFO(this->get_logger(), "car_name = %s", p1.get_name().c_str());
  RCLCPP_INFO(this->get_logger(), "height = %s", p2.get_name().c_str());
  RCLCPP_INFO(this->get_logger(), "wheels = %s", p3.get_name().c_str());

  RCLCPP_INFO(this->get_logger(), "car_name = %s", p1.get_type_name().c_str());
  RCLCPP_INFO(this->get_logger(), "height = %s", p2.get_type_name().c_str());
  RCLCPP_INFO(this->get_logger(), "wheels = %s", p3.get_type_name().c_str());

  RCLCPP_INFO(this->get_logger(), "wheels = %d", p3.get_parameter_value().get_type());

  // RCLCPP_INFO(this->get_logger(), "wheels = %d", p3.get_value_message().double_array_value);
}

Param_demo00::~Param_demo00()
{
}


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  std::shared_ptr<Param_demo00> param = std::make_shared<Param_demo00>();
  rclcpp::spin(param);

  rclcpp::shutdown();

  printf("hello world cpp04_param package\n");
  return 0;
}
