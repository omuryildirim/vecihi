#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <string>
using json = nlohmann::json;


class TofNode : public rclcpp::Node
{
private:
json messages;

private:
json read_json_file(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
        return "";
    }

    // Parse the JSON file
    try {
        std::ifstream f(file_path);
        json data = json::parse(f);
        return data;
    } catch (json::parse_error &e) {
        RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
        return "";
    }
};

public:
    TofNode()
    : Node("tofNode")
    {
        RCLCPP_INFO(this->get_logger(), "Hello ROS 2");
        std::string file_path = "/mnt/d/Projects/vecihi/tofSLAM/data/A0_ToF_data_pack.json";  // Update this with the actual file path
        messages = read_json_file(file_path);
    }

private:
void timer_callback()
{
    auto message = std_msgs::msg::String();
    json ex3 = {
        {"x", messages["pos"]["x"][0]},
        {"y", messages["pos"]["y"][0]},
        {"z", messages["pos"]["z"][0]},
        {"roll", messages["pos"]["roll"][0]},
        {"pitch", messages["pos"]["pitch"][0]},
        {"yaw", messages["pos"]["yaw"][0]},
        {"tof", messages["tof"]["distances"][0]}
    };
    message.data = ex3.dump();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TofNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
