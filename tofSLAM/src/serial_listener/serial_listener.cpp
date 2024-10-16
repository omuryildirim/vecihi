#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <sensor_msgs/sensor_msgs/msg/imu.hpp>
#include "../nlohmann/json.hpp"
#include <tf2/tf2/LinearMath/Quaternion.h>

using namespace boost::asio;
using namespace std::chrono_literals;
using json = nlohmann::json;

class SerialListener : public rclcpp::Node
{
public:
    SerialListener() : Node("serial_listener"), io_(), serial_(io_)
    {
        // Open the serial port
        serial_.open("/dev/ttyACM0");

        // Set the baud rate
        serial_.set_option(serial_port_base::baud_rate(115200));

        // Create a publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("serial_data", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // Create a timer to periodically read from the serial port
        timer_ = this->create_wall_timer(10ms, std::bind(&SerialListener::read_serial_data, this));
    }

private:
    void read_serial_data()
    {
        if (serial_.is_open())
        {
            char c;
            std::string data;

            // Read until a newline character
            while (true)
            {
                boost::asio::read(serial_, buffer(&c, 1));
                if (c == '\n') break;
                data += c;
            }

            RCLCPP_INFO(this->get_logger(), "Received: '%s'", data.c_str());

            // Process the data
            process_serial_data(data);

            // Publish the data to the ROS 2 topic
            // std_msgs::msg::String msg;
            // msg.data = data;
            // publisher_->publish(msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        }
    }

    void process_serial_data(const std::string &data)
    {
        rclcpp::Time now = this->now();

        json j = json::parse(data);

        auto imu_msg = get_imu_message_from_serial_data(j, now);
        std::vector<float> tof = get_tof_data_from_serial_data(j);
        imu_publisher_->publish(std::move(imu_msg));
    }

    static std::unique_ptr<sensor_msgs::msg::Imu_<std::allocator<void>>> get_imu_message_from_serial_data(
        const json &data, const rclcpp::Time &now) {
        double roll = data["roll"];
        double pitch = data["pitch"];
        double yaw = data["yaw"];

        double accel_x = data["ax"];
        double accel_y = data["ay"];
        double accel_z = data["az"];

        double gyro_x = data["gx"];
        double gyro_y = data["gy"];
        double gyro_z = data["gz"];

        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
        imu_msg->header.stamp = now;
        imu_msg->header.frame_id = "imu_frame";

        // Set linear acceleration (assumed to be in m/s²)
        imu_msg->linear_acceleration.x = accel_x;
        imu_msg->linear_acceleration.y = accel_y;
        imu_msg->linear_acceleration.z = accel_z;

        // Set angular velocity (assumed to be in rad/s)
        imu_msg->angular_velocity.x = gyro_x;
        imu_msg->angular_velocity.y = gyro_y;
        imu_msg->angular_velocity.z = gyro_z;

        // Set orientation (convert roll, pitch, yaw to quaternion)
        // For simplicity, assuming the magnetometer/madgwick filter gives orientation in terms of roll, pitch, yaw
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        imu_msg->orientation.x = q.x();
        imu_msg->orientation.y = q.y();
        imu_msg->orientation.z = q.z();
        imu_msg->orientation.w = q.w();

        imu_msg->orientation.x = roll;
        imu_msg->orientation.y = pitch;
        imu_msg->orientation.z = yaw;
        return imu_msg;
    }

    static std::vector<float> get_tof_data_from_serial_data(const json &data) {
        std::vector<float> tof;
        for (auto &cell : data["tof"])
            for (auto &pair : cell)
                tof.push_back(pair[0]);
        return tof;
    }

    io_service io_;
    serial_port serial_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialListener>());
    rclcpp::shutdown();
    return 0;
}
