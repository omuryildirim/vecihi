# PointCloud generation from sample flights with VL53L5CX sensor
This early SLAM code creates a point cloud from a flight of a UAV which is equipped with a VL53L5CX sensor. The collected
data includes VL53L5CX sensor, and UAV IMU data x,y,z, vx, vy, vz, pitch, roll and yaw values. Each flight data is stored
in a JSON file.

System designed as two nodes; ToF sensor node and SLAM node. ToF sensor node imitates the UAV and outputs ToF and pose 
data. ToF sensor node reads the sensor data from a sample JSON file and publishes the data to `tof` topic. SLAM node
subscribes to the `tof` topic and generates a point cloud from the sensor data. SLAM node publishes generated point cloud
to `pointcloud` topic as well as publishes pose and path data to relevant topics.

Then all generated data is visualized in Rviz with the configuration file `rviz-config/map-visualizer.rviz`.

Here a sample point cloud produced from A6 sample data:
![img](../docs/images/rviz-a6-pointcloud.png)

## Installation

In order to have Eigen library installed, you can use the following command:

```bash 
sudo apt-get install libeigen3-dev
```
