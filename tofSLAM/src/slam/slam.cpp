#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <nlohmann/json.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using json = nlohmann::json;


class SlamNode : public rclcpp::Node
{
public:
    SlamNode()
    : Node("subscriber_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "tof", 10, std::bind(&SlamNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscriber node has been started.");

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        json data = json::parse(msg->data.c_str());
        double x = data["x"];
        double y = data["y"];
        double z = data["z"];
        double roll = data["roll"];
        double pitch = data["pitch"];
        double yaw = data["yaw"];
        std::vector< float > tof;
        for (auto& elem : data["tof"])
            tof.push_back(elem);
        // calculate tof distances in x y z considering the position, roll and pitch
        laser_callback(x, y, z, roll, pitch, yaw, tof);
    }


private:
    void laser_callback(double x_s, double y_s, double z_s, double roll, double pitch, double yaw, std::vector< float > tof)
    {
        // For simplicity, we'll use the first laser measurement

        // loog the tof vector
        int size = tof.size();

        for (int i = 0; i < size; i++)
        {
            double d = tof[i]; // Distance measurement

            if (d == -1.0)
            {
                continue;
            }
            // Calculate the elevation and azimuth angles
            std::pair<double, double> angles = calculate_elavation_and_azimuth(i);
            double alpha = angles.first;
            double beta = angles.second;

            // Calculate the 3D position in world coordinates
            Vector3d world_coords = calculate3DPosition(d, alpha, beta, roll, pitch, yaw, x_s, y_s, z_s);

            // log the x,y,z information
            // RCLCPP_INFO(this->get_logger(), "point - x: %f, y: %f, z: %f", world_coords.x(), world_coords.y(), world_coords.z());
            // RCLCPP_INFO(this->get_logger(), "sensor - x: %f, y: %f, z: %f", x_s, y_s, z_s);

            x_values.push_back(world_coords.x());
            y_values.push_back(world_coords.y());
            z_values.push_back(world_coords.z());
        }
        count++;

        // RCLCPP_INFO(this->get_logger(), "count: %f", (double) count);

        if (count % 25 == 0)
        {
            // Create a PointCloud2 message
            auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

            // Set the header
            point_cloud_msg->header.stamp = this->now();
            point_cloud_msg->header.frame_id = "map"; // Set the frame ID as needed

            // Set point cloud properties
            point_cloud_msg->height = 1; // Single row point cloud
            point_cloud_msg->width = x_values.size();  // 3 points
            point_cloud_msg->is_dense = true; // No invalid points

            // Set the point cloud fields
            sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
            modifier.setPointCloud2Fields(3, // x, y, z
                                           "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                           "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                           "z", 1, sensor_msgs::msg::PointField::FLOAT32);

            // Create iterators
            sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");

            for (size_t i = 0; i < x_values.size(); ++i)
            {
                *iter_x = x_values[i];
                *iter_y = y_values[i];
                *iter_z = z_values[i];
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }

            publisher_->publish(std::move(point_cloud_msg));
        }

        // double alpha = 0.0; // Elevation angle (assuming 0 for simplicity)
        // double beta = 0.0; // Azimuth angle
    }

    std::pair<double, double> calculate_elavation_and_azimuth(int index)
    {
        int num_zones_vertical_ = 8;
        int num_zones_horizontal_ = 8;
        double vertical_fov_ = 45.0;
        double horizontal_fov_ = 45.0;

        // Calculate elevation and azimuth angles for each zone
        double elevation_increment = vertical_fov_ / num_zones_vertical_;
        double azimuth_increment = horizontal_fov_ / num_zones_horizontal_;

        int vertical_zone = index / num_zones_vertical_;
        int horizontal_zone = index % num_zones_horizontal_;

        double elevation = vertical_zone * elevation_increment;
        double azimuth = horizontal_zone * azimuth_increment;

        return std::make_pair(elevation, azimuth);
    }

    Vector3d calculate3DPosition(double d, double alpha, double beta, double roll, double pitch, double yaw, double x_s, double y_s, double z_s)
    {
        // Assume the point is directly in front of the camera along its optical axis
        double x_l = d * cos(alpha) * sin(beta);
        double y_l = d * cos(alpha) * cos(beta);
        double z_l = d * sin(alpha);

        Vector3d local_coords(x_l, y_l, z_l);
        // Get the rotation matrix
        Matrix3d R = getRotationMatrix(roll, pitch, yaw);

        // Calculate the world coordinates
        Vector3d sensor_position(x_s, y_s, z_s);
        Vector3d world_coords = R * local_coords + sensor_position;

        return world_coords;
    }
    Matrix3d getRotationMatrix(double roll, double pitch, double yaw)
    {
        Matrix3d R_x, R_y, R_z;

        // Roll (X-axis rotation)
        R_x << 1, 0, 0,
               0, cos(roll), -sin(roll),
               0, sin(roll), cos(roll);

        // Pitch (Y-axis rotation)
        R_y << cos(pitch), 0, sin(pitch),
               0, 1, 0,
               -sin(pitch), 0, cos(pitch);

        // Yaw (Z-axis rotation)
        R_z << cos(yaw), -sin(yaw), 0,
               sin(yaw), cos(yaw), 0,
               0, 0, 1;

        // Combined rotation matrix
        Matrix3d R = R_z * R_y * R_x;

        return R;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    // Populate point cloud data
    std::vector<float> x_values = {};
    std::vector<float> y_values = {};
    std::vector<float> z_values = {};
    int count = 0;
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
