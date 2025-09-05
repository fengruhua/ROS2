#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// 设置定时器时间
using namespace std::chrono_literals;

class TopicPublish : public rclcpp::Node
{
private:
  
public:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count;
  TopicPublish();
  ~TopicPublish();
  // 创建定时器
  void TimeCallback();
};

TopicPublish::TopicPublish():Node("TopicPublish"),count(0)
{
  RCLCPP_INFO(this->get_logger(), "发布节点创建");
  publisher_ = this->create_publisher<std_msgs::msg::String>("/demo01", 10);
  timer_ = this->create_wall_timer(1s,std::bind(&TopicPublish::TimeCallback,this));
}

TopicPublish::~TopicPublish()
{
}

// 创建定时器
void TopicPublish::TimeCallback()
{ 
  auto msgs = std_msgs::msg::String();
  msgs.data = "hello world" + std::to_string(count++);
  publisher_->publish(msgs);
  RCLCPP_INFO(this->get_logger(), "发布的消息为：%s ",msgs.data.c_str());
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  // 初始化
  rclcpp::init(argc, argv);
  // 定义话题类智能指针，因为spin()里面只能放智能指针
  std::shared_ptr<TopicPublish> Talker = std::make_shared<TopicPublish>();
  // 回调函数
  rclcpp::spin(Talker);
  // 节点关闭
  rclcpp::shutdown();

  printf("cpp01_topic package shutdown\n");
  return 0;
}
