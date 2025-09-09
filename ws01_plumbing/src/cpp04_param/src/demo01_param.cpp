#include "rclcpp/rclcpp.hpp"
#include <string.h>

class Param_demo01 : public rclcpp::Node
{
private:
    /* data */
public:
    Param_demo01(/* args */);
    ~Param_demo01();
    template<typename T>
    void declare_param(const std::string &key, const T &default_value);
    void get_param(const std::string &key);
    void update_param();
    void del_param();

};

Param_demo01::Param_demo01(/* args */):Node("Param_demo01", rclcpp::NodeOptions().allow_undeclared_parameters(true))
{
  RCLCPP_INFO(this->get_logger(),"参数服务器服务端使用");


}

Param_demo01::~Param_demo01()
{
}

template<typename T>
void Param_demo01::declare_param(const std::string &key, const T &default_value)
{
    RCLCPP_INFO(this->get_logger(),"------------------------增加-----------------------");
    this->declare_parameter(key, default_value);
    RCLCPP_INFO(this->get_logger(), "声明参数: %s", key.c_str());

    if constexpr (std::is_same_v<T, std::string>)
    {
        RCLCPP_INFO(this->get_logger(), "参数值: %s", default_value.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "参数值: %ld", (long)default_value);
    }

    // if (std::is_same_v<T, std::string>)
    // {
    //     RCLCPP_INFO(this->get_logger(), "参数值: %s", default_value.c_str());
    // } else {
    //     RCLCPP_INFO(this->get_logger(), "参数值: %ld", (long)default_value);
    // }

    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "参数值 : " << default_value);
}

void Param_demo01::get_param(const std::string &key)
{
    RCLCPP_INFO(this->get_logger(),"------------------------查询-----------------------");

    // 判断指定参数是否存在
    if(this->has_parameter(key))
    {
        // 获取指定参数
        rclcpp::Parameter temp = this->get_parameter("tiger");
        RCLCPP_INFO_STREAM(this->get_logger(), "参数 " << key << " 的值: " << temp.value_to_string());
    } else {
        RCLCPP_WARN_STREAM(this->get_logger(), "参数 " << key << " 不存在！");
        return;
    }

    std::vector<rclcpp::Parameter> vals = this->get_parameters({"tiger", "tiger"});

    for (auto &p : vals)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"批量获取 - 参数 " << p.get_name() << " 的值: " << p.value_to_string());
    }
    
    

}

void Param_demo01::update_param()
{
    RCLCPP_INFO(this->get_logger(),"------------------------改动-----------------------");
    Param_demo01::get_param("tiger");

    this->set_parameter(rclcpp::Parameter("tiger", 10));

    Param_demo01::get_param("tiger");
}

void Param_demo01::del_param()
{
    RCLCPP_INFO(this->get_logger(),"------------------------删除-----------------------");

    // this->set_parameter() 设置的才能被删除
    // this->declare_param() 设置的不能被删除

}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    std::shared_ptr<Param_demo01> param = std::make_shared<Param_demo01>();

    param->declare_param("tiger", 1);
    param->get_param("lion");
    param->update_param();
    param->del_param();

    rclcpp::spin(param);

    rclcpp::shutdown();

}



