#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

class TopicStudentSubscribe : public rclcpp::Node
{
private:
    /* data */
public:

    rclcpp::Subscription<base_interfaces_demo::msg::Student>::SharedPtr Sub_ ; 

    TopicStudentSubscribe(/* args */);
    ~TopicStudentSubscribe();
    void Student_Callback(const base_interfaces_demo::msg::Student Student);
};

TopicStudentSubscribe::TopicStudentSubscribe(/* args */):Node("Student_Sub")
{

    RCLCPP_INFO(this->get_logger(), "Student 开始接收");

    Sub_ = this->create_subscription<base_interfaces_demo::msg::Student>("/Student_msgs", 10, std::bind(&TopicStudentSubscribe::Student_Callback, this, std::placeholders::_1));

}

TopicStudentSubscribe::~TopicStudentSubscribe()
{

}

void TopicStudentSubscribe::Student_Callback(const base_interfaces_demo::msg::Student Student)
{
    RCLCPP_INFO(this->get_logger(),"学生：%s 年龄：%d 身高：%f",Student.name.c_str(),Student.age,Student.height);
}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    std::shared_ptr<TopicStudentSubscribe> Student = std::make_shared<TopicStudentSubscribe>();
    rclcpp::spin(Student);

    rclcpp::shutdown();
}
