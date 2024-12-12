#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fstream>
#include <iostream>
#include "../nlohmann/json.hpp"
#include <tf2/LinearMath/Quaternion.h>
using json = nlohmann::json;
using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node
{
public:
    IMUPublisher() : Node("imu_publisher")
    {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&IMUPublisher::publish_imu_data, this));
        load_imu_data();
    }

private:
    void load_imu_data()
    {
        std::ifstream file("src/test_imu_publisher/testIMUData.json");
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open IMU data file");
            return;
        }
        file >> imu_data_;
        file.close();
        data_index_ = 0;
    }

    void publish_imu_data()
    {
        if (data_index_ >= imu_data_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All IMU data published");
            rclcpp::shutdown();
            return;
        }

        const auto &data = imu_data_[data_index_];
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

        imu_msg->header.stamp = this->now();
        imu_msg->header.frame_id = "imu_frame";

        imu_msg->orientation.x = data["orientation"]["x"];
        imu_msg->orientation.y = data["orientation"]["y"];
        imu_msg->orientation.z = data["orientation"]["z"];
        imu_msg->orientation.w = data["orientation"]["w"];

        imu_msg->angular_velocity.x = data["angular_velocity"]["x"];
        imu_msg->angular_velocity.y = data["angular_velocity"]["y"];
        imu_msg->angular_velocity.z = data["angular_velocity"]["z"];

        imu_msg->linear_acceleration.x = data["linear_acceleration"]["x"];
        imu_msg->linear_acceleration.y = data["linear_acceleration"]["y"];
        imu_msg->linear_acceleration.z = data["linear_acceleration"]["z"];

        imu_publisher_->publish(std::move(imu_msg));
        data_index_++;
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    json imu_data_;
    size_t data_index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}