#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TopicSubscribe : public rclcpp::Node
{
private:
    /* data */
    
public:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Subscribe_ ;

    TopicSubscribe(/* args */);
    ~TopicSubscribe();
    void StringCallBack(const std_msgs::msg::String &msg);
};

TopicSubscribe::TopicSubscribe(/* args */):Node("TopicSubscribe")
{
    RCLCPP_INFO(this->get_logger(),"接收节点创建");
    Subscribe_ = this->create_subscription<std_msgs::msg::String>("/demo01",10, std::bind(&TopicSubscribe::StringCallBack, this, std::placeholders::_1));
}

TopicSubscribe::~TopicSubscribe()
{
}

void TopicSubscribe::StringCallBack(const std_msgs::msg::String &msg)
{
    RCLCPP_INFO(this->get_logger(), " 接收到的消息为：%s \n", msg.data.c_str());
}

int main(int argc, char** argv){
    rclcpp::init(argc,argv);

    std::shared_ptr<TopicSubscribe> Listener = std::make_shared<TopicSubscribe>();
    rclcpp::spin(Listener);


    rclcpp::shutdown();
    printf("cpp01_topic listener package shutdown\n");

}