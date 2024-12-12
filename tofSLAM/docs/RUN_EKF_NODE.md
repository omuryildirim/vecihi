# Visualizing robot_localization Output in RViz

#### 1. Run the EKF node with the following commands:
```bash
cd D:\Projects\ignore-the-dark\tofSlam
wsl
source install/setup.bash
ros2 run robot_localization ekf_node --ros-args --params-file ./config/ekf.yaml
```

#### 2. Run publisher:
```bash
cd D:\Projects\ignore-the-dark\tofSlam
wsl
ros2 run tofSlam test_imu_publisher
```
#### 3. Provide the Missing Transform
```bash
cd D:\Projects\ignore-the-dark\tofSlam
wsl
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 imu_frame base_link
```

#### 4. Ensure your IMU is publishing data on the /imu/data topic with the correct frame_id.
Check the topic:

```bash
ros2 topic echo /imu/data
```

Example output:

```bash
header:
frame_id: imu_frame
```

#### 6. Check if the EKF node is publishing data:
```bash
ros2 topic echo /odometry/filtered
```

#### 7. Launch RViz:
```bash
cd D:\Projects\ignore-the-dark\tofSlam
wsl
ros2 run rviz2 rviz2
```

In RViz, add an Odometry display and set the Topic to /odometry/filtered.

Set the Fixed Frame to odom (or map).