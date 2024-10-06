#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <string>

using namespace boost::asio;
using namespace std::chrono_literals;

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

        // Create a timer to periodically read from the serial port
        timer_ = this->create_wall_timer(100ms, std::bind(&SerialListener::read_serial_data, this));
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

            // Publish the data to the ROS 2 topic
            std_msgs::msg::String msg;
            msg.data = data;
            publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Received: '%s'", data.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        }
    }

    io_service io_;
    serial_port serial_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialListener>());
    rclcpp::shutdown();
    return 0;
}
