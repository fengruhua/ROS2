#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"
using namespace std::chrono_literals;

class TopicStuentPub : public rclcpp::Node
{
private:
    /* data */
public:
    rclcpp::Publisher<base_interfaces_demo::msg::Student>::SharedPtr Publish_;
    rclcpp::TimerBase::SharedPtr Timer_;
    base_interfaces_demo::msg::Student Student_;
    size_t count = 0;

    TopicStuentPub(/* args */);
    ~TopicStuentPub();
    void TimeCallBack();
};

TopicStuentPub::TopicStuentPub(/* args */):Node("Student_pub")
{
    RCLCPP_INFO(this->get_logger(), "Student 开始发布");

    Publish_ = this->create_publisher<base_interfaces_demo::msg::Student>("/Student_msgs", 10);

    Timer_ = this->create_wall_timer(1s, std::bind(&TopicStuentPub::TimeCallBack, this));

    Student_.age = 10;
    Student_.name = "张三";
    Student_.height = 190;
}

TopicStuentPub::~TopicStuentPub()
{
}

void TopicStuentPub::TimeCallBack()
{
    Student_.name = "张三" + std::to_string(count++);
    RCLCPP_INFO(this->get_logger(),"学生：%s 年龄：%d 身高：%f",Student_.name.c_str(),Student_.age,Student_.height);
    Publish_->publish(Student_);
}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    std::shared_ptr<TopicStuentPub> Student = std::make_shared<TopicStuentPub>();
    rclcpp::spin(Student);

    rclcpp::shutdown();
}
