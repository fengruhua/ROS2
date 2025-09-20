#include "rclcpp/rclcpp.hpp"
#include <string.h>
#include <vector>

class ParamServer : public rclcpp::Node
{
private:
    /* data */
public:
    ParamServer(const std::string &node_name = "Param_Server");
    ~ParamServer();

    void Show_Menu();

    template<typename T>
    bool declare_param(const std::string &key, const T& default_value, bool unallow_delete = false);
    template<typename T>
    bool declare_param(const std::map<std::string, T> &params, bool unallow_delete = true);
    // template<typename T>
    // bool declare_param(const std::string &key, const T& default_value, bool unallow_delete = false, bool unallow_change = false);

    bool find_param(const std::string &key);

    bool show_param(const std::string &key);

    bool show_params();

    template<typename T>
    bool change_param(const std::string &key, const T& value);

    bool delete_param(const std::string &key);

};

ParamServer::ParamServer(const std::string &node_name):Node(node_name, rclcpp::NodeOptions().allow_undeclared_parameters(true))
{
}

ParamServer::~ParamServer()
{
}

void ParamServer::Show_Menu()
{
    std::cout << "1, 添加参数" << std::endl;
    std::cout << "2, 删除参数" << std::endl;
    std::cout << "3, 修改参数" << std::endl;
    std::cout << "4, 查询参数" << std::endl;
    std::cout << "5, 展示参数" << std::endl;
    std::cout << "6, 退出" << std::endl;

    std::cout << "请输入：" << std::endl;
}

// 如果该参数 还没有被声明 → 会用你提供的 default_value 初始化。
// 如果该参数 已经被声明过 → 会抛出 rclcpp::exceptions::ParameterAlreadyDeclaredException，不会覆盖已有值。
// 使用更加好的实现
// template <typename T>
// bool ParamServer::declare_param(const std::string &key, const T &default_value, bool unallow_delete)
// {
//     if (find_param(key))
//     {
//         RCLCPP_INFO_STREAM(this->get_logger(),key << "已经存在" );
//         return false;
//     }

//     if (unallow_delete){
//         this->declare_parameter(key, default_value);
//     } else {
//         this->set_parameter(key, default_value);
//     }
//     if (!find_param(key)){
//         RCLCPP_INFO_STREAM(this->get_logger(),key << "添加成功" );
//         return false
//     }
//     RCLCPP_INFO_STREAM(this->get_logger(),key << "添加失败" );
//     return true;
// }

template <typename T>
bool ParamServer::declare_param(const std::string &key, const T &default_value, bool unallow_delete)
{
    if (!unallow_delete)
    {
        if(find_param(key)){
            RCLCPP_INFO_STREAM(this->get_logger(),key << "已经存在" );
            return false;
        }
        this->set_parameter(rclcpp::Parameter(key, default_value));
        if(find_param(key)){
            RCLCPP_INFO_STREAM(this->get_logger(), key << " 添加成功（可删除参数）");
            return true;
        }
        RCLCPP_INFO_STREAM(this->get_logger(),key << "添加失败" );
        return false;
    }
    // unallow_delete = true
    // 不允许修改的参数
    try
    {
        this->declare_parameter(key, default_value);
        RCLCPP_INFO_STREAM(this->get_logger(), key << " 添加成功（不可删除参数）");
        return true;
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        // std::cerr << e.what() << '\n';
        RCLCPP_INFO_STREAM(this->get_logger(),key << "添加失败" );
        return false;
    }

}

template <typename T>
bool ParamServer::declare_param(const std::map<std::string, T> &params, bool unallow_delete)
{
    for (auto &[key, value] : params)
    {
        declare_param(key, value, unallow_delete);
    }
    return true;
}

// template <typename T>
// bool ParamServer::declare_param(const std::string &key, const T &default_value, bool unallow_delete, bool unallow_change)
// {
//     return false;
// }

bool ParamServer::find_param(const std::string &key)
{
    return this->has_parameter(key);
    // true     key存在
    // false    key不存在
}

template <typename T>
bool ParamServer::change_param(const std::string &key, const T &value)
{
    if(!find_param(key)){
        RCLCPP_INFO_STREAM(this->get_logger(),key << "不存在" );
        return false;
    }
    this->set_parameter(rclcpp::Parameter(key, value));
    RCLCPP_INFO_STREAM(this->get_logger(), "修改成功" );
    show_param(key);
    return true;
}

bool ParamServer::delete_param(const std::string &key)
{
    try
    {
        this->undeclare_parameter(key);
        RCLCPP_INFO_STREAM(this->get_logger(),"删除成功");
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        RCLCPP_INFO_STREAM(this->get_logger(),"删除失败，该参数不可删除");
    }
    
    return false;
}


bool ParamServer::show_param(const std::string &key)
{
    if(!find_param(key)){
        RCLCPP_INFO_STREAM(this->get_logger(),key << "不存在" );
        return false;
    }
    rclcpp::Parameter temp = this->get_parameter(key);
    RCLCPP_INFO_STREAM(this->get_logger(), "key = " << key << " ; " << "value = " << temp.value_to_string());
    return true;
}

bool ParamServer::show_params()
{
    auto results = this->list_parameters({}, 1);

    if (results.names.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No parameters found.");
        return false;
    }

    for (auto &name : results.names)
    {
        if(name == "use_sim_time") continue;
        show_param(name);
    }
    
    return true;
}

int main(int argc , char ** argv){

    rclcpp::init(argc, argv);

    std::shared_ptr Server = std::make_shared<ParamServer>();

    // Server->Show_Menu();

    int choice;
    std::string input;
    bool exit_flag = false;
    bool unallow_delete = false;
    while (!exit_flag && rclcpp::ok())
    {
        Server->Show_Menu();
        std::getline(std::cin, input);
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 清掉菜单输入后的回车

        switch (choice)
        {
        case 1 :
            {
                std::vector<std::string> tokens;
                std::string token;
                std::cout << "1, 添加不变参数" << std::endl;
                std::cout << "2, 添加可变参数" << std::endl;
                std::cin >> choice;

                switch (choice){
                    case 1:
                        unallow_delete = true;
                        break;
                    case 2:
                        unallow_delete = false;
                        break;
                }
                while (true)
                {
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 清掉菜单输入后的回车
                    tokens.clear();
                    token.clear();
                    std::cout << "请输入需要创建的参数和对应的值（格式：key1 value1 key2 value2 ...）" << std::endl;

                    // std::cin.clear();
                    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    input.clear();
                    std::getline(std::cin, input);

                    std::istringstream iss(input);

                    while (iss >> token)
                    {
                        tokens.push_back(token);
                    }

                    // 检查是否成对
                    if (tokens.size() % 2 != 0)
                    {
                        RCLCPP_INFO_STREAM(Server->get_logger(), "输入不成对，请保证每个 key 有对应的 value!");
                        continue; // 重新输入
                    }
                    // if (tokens.size() % 2 == 0) break;

                    // printf("222");
                    // 成对输入，退出循环
                    break;
                }

                for (size_t i = 0; i < tokens.size(); i += 2)
                {
                    const std::string &key = tokens[i];
                    const std::string &value_str = tokens[i + 1];

                    // 尝试转换为整数
                    try {
                        size_t pos;
                        int int_val = std::stoi(value_str, &pos);
                        if (pos == value_str.size()) {
                            Server->declare_param(key, int_val, false);
                            continue;
                        }
                    } catch (...) {}

                    // 尝试转换为浮点数
                    try {
                        size_t pos;
                        double double_val = std::stod(value_str, &pos);
                        if (pos == value_str.size()) {
                            Server->declare_param(key, double_val, false);
                            continue;
                        }
                    } catch (...) {}

                    // 默认当作字符串
                    Server->declare_param(key, value_str, unallow_delete);
                }

                break;

            }
        
        case 2:
            {
                input.clear();
                std::cin >> input;
                Server->delete_param(input);
            }
            break;

        case 3:
            {
                input.clear();
                std::cin >> input;
                auto temp_key = input;
                std::cout << "要改变的数值" << std::endl;
                input.clear();
                std::cin >> input;
                auto temp_value = input;
                Server->change_param(temp_key, temp_value);
            }
            break;

        case 4:
            {
                input.clear();
                std::cin >> input;
                Server->show_param(input);
            }
            break;

        case 5:
            Server->show_params();
            break;

        case 6:
            exit_flag = true;
            break;

        default:
            RCLCPP_INFO_STREAM(Server->get_logger(), "无效选项，请重新输入");
            break;
        }
        
    }
    

    rclcpp::shutdown();
    
}
