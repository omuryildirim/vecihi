#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "../nlohmann/json.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <memory>
#include <string> 

using namespace std::chrono_literals;
using json = nlohmann::json;

class TofNode : public rclcpp::Node
{
private:
    json read_json_file(const std::string& file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
            return "";
        }

        // Parse the JSON file
        try
        {
            std::ifstream f(file_path);
            json data = json::parse(f);
            RCLCPP_INFO(this->get_logger(), "Parsed the JSON!");
            return data;
        }
        catch (json::parse_error& e)
        {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
            return "";
        }
    }

public:
    TofNode()
        : Node("tofNode")
    {
        RCLCPP_INFO(this->get_logger(), "Hello ROS 2");
        std::string file_path = "/home/ubuntu/dev/vecihi/tofSLAM/data/home.json";
        // Update this with the actual file path
        messages = read_json_file(file_path);
        publisher_ = this->create_publisher<std_msgs::msg::String>("tof", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TofNode::timer_callback, this)
        );
        count_ = 0;
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        RCLCPP_INFO(this->get_logger(), "Publishing message: %d", count_);
        json ex3 = {
            {"x", messages["pos"]["x"][count_]},
            {"y", messages["pos"]["y"][count_]},
            {"z", messages["pos"]["z"][count_]},
            {"roll", messages["pos"]["roll"][count_]},
            {"pitch", messages["pos"]["pitch"][count_]},
            {"yaw", messages["pos"]["yaw"][count_]},
            {"tof", messages["tof"]["distances"][count_]}
        };
        message.data = ex3.dump();
        double pitch = messages["pos"]["pitch"][count_];
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        //if (messages["pos"]["vz"][count_] < 5.0 && abs(pitch) < 1.0)
        //{
            publisher_->publish(message);
        //}
        count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    json messages;
    // define int count_ to keep track of the number of messages published
    int count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TofNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
