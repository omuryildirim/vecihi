#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserialport.h>
#include <iostream>
#include <memory>

class SerialListener : public rclcpp::Node
{
public:
    SerialListener() : Node("serial_listener")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("serial_data", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SerialListener::read_serial_data, this));

        sp_return error = sp_get_port_by_name("/dev/ttyV0", &serial_port_);
        if (error != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get serial port by name ttyV0.");
            struct sp_port **ports;
            sp_return result = sp_list_ports(&ports);
            for (int i = 0; ports[i] != nullptr; i++) {
                struct sp_port *port = ports[i];

                // Get the name of the port
                RCLCPP_INFO(this->get_logger(), "Port: '%s'", sp_get_port_name(port));

                std::cout << std::endl;
            }
            rclcpp::shutdown();
        }

        error = sp_open(serial_port_, SP_MODE_READ);
        if (error != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            rclcpp::shutdown();
        }

        sp_set_baudrate(serial_port_, 460800);  // Adjust baudrate as needed
        sp_set_bits(serial_port_, 8);
        sp_set_parity(serial_port_, SP_PARITY_NONE);
        sp_set_stopbits(serial_port_, 1);
    }

    ~SerialListener()
    {
        sp_close(serial_port_);
        sp_free_port(serial_port_);
    }

private:
    void read_serial_data()
    {
        char buffer[256];
        int bytes_read = sp_nonblocking_read(serial_port_, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0'; // Null terminate the string
            std::string data(buffer);
            auto message = std_msgs::msg::String();
            message.data = data;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Received: '%s'", data.c_str());
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct sp_port* serial_port_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialListener>());
    rclcpp::shutdown();
    return 0;
}
