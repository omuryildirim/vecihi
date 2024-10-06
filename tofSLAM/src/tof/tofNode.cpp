#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "../nlohmann/json.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using json = nlohmann::json;

class TofNode : public rclcpp::Node
{
public:
    TofNode()
        : Node("tof_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello from ToF Node");
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "serial_data", 10, std::bind(&TofNode::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::String>("tof", 10);
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        json data = json::parse(msg->data.c_str());
        double x = 0; // data["x"];
        double y = 0; // data["y"];
        double z = 0; // data["z"];
        double roll = data["roll"];
        double pitch = data["pitch"];
        double yaw = data["yaw"];
        std::vector<float> tof;
        for (auto &cell: data["tof"])
            for (auto &pair: cell)
                tof.push_back(pair[0]);
        // calculate tof distances in x y z considering the position, roll and pitch
        publish_message(x * 100.0, y * 100.0, z * 100.0, roll, pitch, yaw, tof);
    }

private:
    void publish_message(double x, double y, double z, double roll, double pitch, double yaw, std::vector<float> tof)
    {
        auto message = std_msgs::msg::String();
        RCLCPP_INFO(this->get_logger(), "Publishing message");
        json ex3 = {
            {"x", x},
            {"y", y},
            {"z", z},
            {"roll", roll},
            {"pitch", pitch},
            {"yaw", yaw},
            {"tof", tof}
        };
        message.data = ex3.dump();
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        //if (messages["pos"]["vz"][count_] < 5.0 && abs(pitch) < 1.0)
        //{
            publisher_->publish(message);
        //}
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    json messages;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TofNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
