cmake_minimum_required(VERSION 3.8)
project(tofSlam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED QUIET)
find_package(tf2 REQUIRED QUIET)
# serial
find_package(Boost REQUIRED COMPONENTS system)
# robot localization
find_package(robot_localization REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

# tof
add_executable(tofNode src/tof/tofNode.cpp)
add_executable(slamNode src/slam/slam.cpp)
add_executable(serial_listener src/serial_listener/serial_listener.cpp)
ament_target_dependencies(tofNode rclcpp std_msgs)
ament_target_dependencies(slamNode rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs PCL pcl_conversions tf2 tf2_geometry_msgs)
ament_target_dependencies(serial_listener rclcpp std_msgs)
target_link_libraries(serial_listener Boost::system)

install(TARGETS
        tofNode
        slamNode
        serial_listener
        DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
